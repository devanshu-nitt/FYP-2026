// =============================================================================
// directory_controller.v
// Centralised MESI directory controller.
//
// Directory states:
//   DIR_U  – line is uncached (memory holds authoritative copy)
//   DIR_S  – one or more cores hold clean (shared) copies
//   DIR_M  – exactly one core holds the dirty (modified) copy
//
// ID scheme (must match cache_controller.v):
//   ID_BITS = $clog2(NUM_CORES+1)
//   Core i → id i,   Directory → id NUM_CORES
//   Requester id is embedded in the low ID_BITS of the data field of
//   FWD_GETS / FWD_GETM messages so the recipient knows where to send data.
//
// Memory interface: 1-cycle registered read (data available the cycle after
//   rd_en is asserted), synchronous write.
// =============================================================================
`timescale 1ns/1ps

module directory_controller #(
    parameter NUM_CORES  = 2,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 8
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // ── Network → Directory ───────────────────────────────────────────────
    input  wire                         net_valid,
    input  wire [3:0]                   net_msg_type,
    input  wire [ADDR_WIDTH-1:0]        net_addr,
    input  wire [DATA_WIDTH-1:0]        net_data,
    input  wire [$clog2(NUM_CORES+1)-1:0] net_src_id,

    // ── Directory → Network ───────────────────────────────────────────────
    output reg                          out_valid,
    output reg  [3:0]                   out_msg_type,
    output reg  [ADDR_WIDTH-1:0]        out_addr,
    output reg  [DATA_WIDTH-1:0]        out_data,
    output reg  [$clog2(NUM_CORES+1)-1:0] out_dst_id,

    // ── Backing memory interface ──────────────────────────────────────────
    output reg                          mem_rd_en,
    output reg                          mem_wr_en,
    output reg  [ADDR_WIDTH-1:0]        mem_addr,
    output reg  [DATA_WIDTH-1:0]        mem_wr_data,
    input  wire [DATA_WIDTH-1:0]        mem_rd_data
);

// ── Local parameters ─────────────────────────────────────────────────────────
localparam ID_BITS = $clog2(NUM_CORES+1);

// ── Message type encoding ─────────────────────────────────────────────────────
localparam MSG_GETS          = 4'd1;
localparam MSG_GETM          = 4'd2;
localparam MSG_DATA          = 4'd3;
localparam MSG_DATA_EXCL     = 4'd4;
localparam MSG_INV           = 4'd5;
localparam MSG_FWD_GETS      = 4'd6;
localparam MSG_FWD_GETM      = 4'd7;
localparam MSG_DATA_TRANSFER = 4'd8;
localparam MSG_FLUSH         = 4'd9;
localparam MSG_ACK           = 4'd10;

// ── Directory state encoding ─────────────────────────────────────────────────
localparam DIR_U = 2'd0;
localparam DIR_S = 2'd1;
localparam DIR_M = 2'd2;

// ── Directory FSM states ──────────────────────────────────────────────────────
localparam DC_IDLE           = 4'd0;
localparam DC_MEM_RD         = 4'd1;   // waiting 1 cycle for memory read data
localparam DC_SEND_GETS_DATA = 4'd2;   // reply DATA / DATA_EXCL to GETS requester
localparam DC_SEND_GETM_DATA = 4'd3;   // reply DATA_EXCL to GETM requester (U case)
localparam DC_INV_SEND       = 4'd4;   // send INV messages one by one (S→M case)
localparam DC_INV_WAIT       = 4'd5;   // wait for all ACKs then grant M
localparam DC_FWD_WAIT_GETS  = 4'd6;   // wait for M-owner ACK after FWD_GETS
localparam DC_FWD_WAIT_GETM  = 4'd7;   // wait for old-owner FLUSH after FWD_GETM

// ── Directory entry ───────────────────────────────────────────────────────────
reg [1:0]            dir_state;            // DIR_U / DIR_S / DIR_M
reg [NUM_CORES-1:0]  sharers;             // bitmask; scalable to N cores
reg [ID_BITS-1:0]    owner;               // valid only in DIR_M

// ── Pending transaction ───────────────────────────────────────────────────────
reg [3:0]            dc_state;
reg [ID_BITS-1:0]    pend_req;            // requester core id
reg [ADDR_WIDTH-1:0] pend_addr;
reg                  pend_is_getm;        // 1 = pending request is GETM

// INV tracking
reg [NUM_CORES-1:0]  ack_mask;            // bit per core: 1 = ACK still expected
reg [DATA_WIDTH-1:0] mem_data_latch;      // latched memory read data

// Loop variable (declared here for Verilog-2001 / iVerilog compatibility)
integer k;

// ── Directory FSM ─────────────────────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dir_state      <= DIR_U;
        sharers        <= {NUM_CORES{1'b0}};
        owner          <= {ID_BITS{1'b0}};
        dc_state       <= DC_IDLE;
        pend_req       <= {ID_BITS{1'b0}};
        pend_addr      <= {ADDR_WIDTH{1'b0}};
        pend_is_getm   <= 1'b0;
        ack_mask       <= {NUM_CORES{1'b0}};
        mem_data_latch <= {DATA_WIDTH{1'b0}};
        out_valid      <= 1'b0;
        mem_rd_en      <= 1'b0;
        mem_wr_en      <= 1'b0;
        mem_addr       <= {ADDR_WIDTH{1'b0}};
        mem_wr_data    <= {DATA_WIDTH{1'b0}};
    end else begin
        // Default: clear one-cycle strobes
        out_valid <= 1'b0;
        mem_rd_en <= 1'b0;
        mem_wr_en <= 1'b0;

        case (dc_state)

            // ──────────────────────────────────────────────────────────────────
            // IDLE: accept a new request from any core
            // ──────────────────────────────────────────────────────────────────
            DC_IDLE: begin
                if (net_valid) begin
                    case (net_msg_type)

                        // ── GETS ──────────────────────────────────────────────
                        MSG_GETS: begin
                            pend_req     <= net_src_id;
                            pend_addr    <= net_addr;
                            pend_is_getm <= 1'b0;
                            case (dir_state)
                                DIR_U, DIR_S: begin
                                    // Fetch clean data from memory
                                    mem_rd_en <= 1'b1;
                                    mem_addr  <= net_addr;
                                    dc_state  <= DC_MEM_RD;
                                end
                                DIR_M: begin
                                    // Forward to M-owner; embed requester id in data
                                    out_valid    <= 1'b1;
                                    out_msg_type <= MSG_FWD_GETS;
                                    out_addr     <= net_addr;
                                    out_data     <= {{(DATA_WIDTH-ID_BITS){1'b0}}, net_src_id};
                                    out_dst_id   <= owner;
                                    dc_state     <= DC_FWD_WAIT_GETS;
                                end
                                default: ;
                            endcase
                        end

                        // ── GETM ──────────────────────────────────────────────
                        MSG_GETM: begin
                            pend_req     <= net_src_id;
                            pend_addr    <= net_addr;
                            pend_is_getm <= 1'b1;
                            case (dir_state)
                                DIR_U: begin
                                    // Fetch from memory; will give exclusive write grant
                                    mem_rd_en <= 1'b1;
                                    mem_addr  <= net_addr;
                                    dc_state  <= DC_MEM_RD;
                                end
                                DIR_S: begin
                                    // Build INV mask: all sharers except requester
                                    begin : build_ack_mask
                                        reg [NUM_CORES-1:0] mask;
                                        integer m;
                                        mask = {NUM_CORES{1'b0}};
                                        for (m = 0; m < NUM_CORES; m = m + 1) begin
                                            if (sharers[m] &&
                                                (m[ID_BITS-1:0] != net_src_id[ID_BITS-1:0]))
                                                mask[m] = 1'b1;
                                        end
                                        ack_mask <= mask;
                                    end
                                    // Also read memory (holds clean copy in S state)
                                    mem_rd_en <= 1'b1;
                                    mem_addr  <= net_addr;
                                    // Clear sharers; will be re-set to requester on grant
                                    sharers   <= {NUM_CORES{1'b0}};
                                    dc_state  <= DC_INV_SEND;
                                end
                                DIR_M: begin
                                    // Forward to current M-owner; embed requester id
                                    out_valid    <= 1'b1;
                                    out_msg_type <= MSG_FWD_GETM;
                                    out_addr     <= net_addr;
                                    out_data     <= {{(DATA_WIDTH-ID_BITS){1'b0}}, net_src_id};
                                    out_dst_id   <= owner;
                                    dc_state     <= DC_FWD_WAIT_GETM;
                                end
                                default: ;
                            endcase
                        end

                        // ── FLUSH (owner writing back without a forward) ───────
                        MSG_FLUSH: begin
                            mem_wr_en   <= 1'b1;
                            mem_addr    <= net_addr;
                            mem_wr_data <= net_data;
                            dir_state   <= DIR_U;
                            sharers     <= {NUM_CORES{1'b0}};
                        end

                        // ── ACK (stray, already handled in sub-states) ─────────
                        MSG_ACK: ; // consume silently

                        default: ;
                    endcase
                end
            end // DC_IDLE

            // ──────────────────────────────────────────────────────────────────
            // DC_MEM_RD: one-cycle memory latency; data available now
            // ──────────────────────────────────────────────────────────────────
            DC_MEM_RD: begin
                mem_data_latch <= mem_rd_data;
                dc_state <= pend_is_getm ? DC_SEND_GETM_DATA : DC_SEND_GETS_DATA;
            end

            // ──────────────────────────────────────────────────────────────────
            // DC_SEND_GETS_DATA: reply to GETS
            //   DIR_U  → DATA_EXCL (cache goes I→E)
            //   DIR_S  → DATA      (cache goes I→S)
            // ──────────────────────────────────────────────────────────────────
            DC_SEND_GETS_DATA: begin
                out_valid <= 1'b1;
                out_addr  <= pend_addr;
                out_data  <= mem_data_latch;
                out_dst_id <= pend_req;
                if (dir_state == DIR_U) begin
                    out_msg_type        <= MSG_DATA_EXCL; // only reader → E
                    dir_state           <= DIR_S;
                    sharers             <= {NUM_CORES{1'b0}};
                    sharers[pend_req[($clog2(NUM_CORES)>0 ? $clog2(NUM_CORES)-1 : 0):0]] <= 1'b1;
                end else begin
                    out_msg_type        <= MSG_DATA;       // additional reader → S
                    sharers[pend_req[($clog2(NUM_CORES)>0 ? $clog2(NUM_CORES)-1 : 0):0]] <= 1'b1;
                end
                dc_state <= DC_IDLE;
            end

            // ──────────────────────────────────────────────────────────────────
            // DC_SEND_GETM_DATA: reply to GETM on U state with DATA_EXCL
            // ──────────────────────────────────────────────────────────────────
            DC_SEND_GETM_DATA: begin
                out_valid    <= 1'b1;
                out_msg_type <= MSG_DATA_EXCL;
                out_addr     <= pend_addr;
                out_data     <= mem_data_latch;
                out_dst_id   <= pend_req;
                // U → M
                dir_state    <= DIR_M;
                owner        <= pend_req;
                sharers      <= {NUM_CORES{1'b0}};
                dc_state     <= DC_IDLE;
            end

            // ──────────────────────────────────────────────────────────────────
            // DC_INV_SEND: GETM on S state – send INV to each sharer that needs
            //   it, one per cycle (sequential, simple; parallel possible for
            //   higher performance)
            // ──────────────────────────────────────────────────────────────────
            DC_INV_SEND: begin
                begin : send_one_inv
                    integer m;
                    reg inv_sent;
                    inv_sent = 0;
                    for (m = 0; m < NUM_CORES && !inv_sent; m = m + 1) begin
                        if (ack_mask[m]) begin
                            out_valid    <= 1'b1;
                            out_msg_type <= MSG_INV;
                            out_addr     <= pend_addr;
                            out_data     <= {DATA_WIDTH{1'b0}};
                            out_dst_id   <= m[ID_BITS-1:0];
                            ack_mask[m]  <= 1'b0;  // clear – ACK will confirm
                            inv_sent = 1;
                        end
                    end
                    if (!inv_sent) begin
                        // All INVs have been sent; wait for ACKs
                        dc_state <= DC_INV_WAIT;
                    end
                    // If we sent, stay in DC_INV_SEND to emit remaining INVs
                end
            end

            // ──────────────────────────────────────────────────────────────────
            // DC_INV_WAIT: wait until ack_mask is fully cleared, then grant M
            // ──────────────────────────────────────────────────────────────────
            DC_INV_WAIT: begin
                if (net_valid && net_msg_type == MSG_ACK
                        && net_addr == pend_addr) begin
                    ack_mask[net_src_id[$clog2(NUM_CORES)-1:0]] <= 1'b0;
                end

                // Combinational look-ahead to check if the last ACK just arrived
                begin : check_all_acks
                    reg [NUM_CORES-1:0] remaining;
                    remaining = ack_mask;
                    if (net_valid && net_msg_type == MSG_ACK
                            && net_addr == pend_addr)
                        remaining[net_src_id[$clog2(NUM_CORES)-1:0]] = 1'b0;

                    if (remaining == {NUM_CORES{1'b0}}) begin
                        // All sharers ACK'd; grant exclusive ownership
                        dir_state    <= DIR_M;
                        owner        <= pend_req;
                        sharers      <= {NUM_CORES{1'b0}};
                        out_valid    <= 1'b1;
                        out_msg_type <= MSG_DATA_EXCL;
                        out_addr     <= pend_addr;
                        out_data     <= mem_data_latch;  // clean copy from earlier read
                        out_dst_id   <= pend_req;
                        dc_state     <= DC_IDLE;
                    end
                end
            end

            // ──────────────────────────────────────────────────────────────────
            // DC_FWD_WAIT_GETS: sent FWD_GETS to M-owner; wait for owner's ACK
            //   (owner sends DATA_TRANSFER directly to requester, then ACKs us)
            // ──────────────────────────────────────────────────────────────────
            DC_FWD_WAIT_GETS: begin
                if (net_valid && net_addr == pend_addr
                        && net_msg_type == MSG_ACK) begin
                    // Owner confirmed downgrade M→S
                    dir_state <= DIR_S;
                    sharers   <= {NUM_CORES{1'b0}};
                    // Both old owner and requester are now S
                    sharers[owner[$clog2(NUM_CORES)-1:0]]    <= 1'b1;
                    sharers[pend_req[$clog2(NUM_CORES)-1:0]] <= 1'b1;
                    dc_state  <= DC_IDLE;
                end
            end

            // ──────────────────────────────────────────────────────────────────
            // DC_FWD_WAIT_GETM: sent FWD_GETM to M-owner; wait for FLUSH
            //   (owner sends DATA_TRANSFER to requester then FLUSHes us)
            // ──────────────────────────────────────────────────────────────────
            DC_FWD_WAIT_GETM: begin
                if (net_valid && net_addr == pend_addr
                        && net_msg_type == MSG_FLUSH) begin
                    // Write back old owner's data to memory
                    mem_wr_en   <= 1'b1;
                    mem_addr    <= pend_addr;
                    mem_wr_data <= net_data;
                    // Transfer M ownership to requester
                    dir_state   <= DIR_M;
                    owner       <= pend_req;
                    sharers     <= {NUM_CORES{1'b0}};
                    dc_state    <= DC_IDLE;
                end
            end

            default: dc_state <= DC_IDLE;
        endcase
    end
end

endmodule
