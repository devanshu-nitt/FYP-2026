// =============================================================================
// cache_controller.v
// Per-core MESI cache controller (FSM-based, blocking, single cache line).
//
// KEY DESIGN CHOICE – ID width:
//   All destination/source fields use ID_BITS = $clog2(NUM_CORES+1) bits.
//   Core i  → id = i           (0 .. NUM_CORES-1)
//   Directory → id = NUM_CORES
//   This avoids the collision that occurs when NUM_CORES is a power-of-2
//   and "all-ones" aliases to a valid core id.
//
// Parameters:
//   NUM_CORES  – total cores in system (for ID sizing)
//   CORE_ID    – this core's numeric id
//   DATA_WIDTH – cache-line data width in bits
//   ADDR_WIDTH – address bus width in bits
// =============================================================================
`timescale 1ns/1ps

module cache_controller #(
    parameter NUM_CORES  = 2,
    parameter CORE_ID    = 0,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 8
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // ── CPU interface ─────────────────────────────────────────────────────
    input  wire                         cpu_req,
    input  wire                         cpu_wr,
    input  wire [ADDR_WIDTH-1:0]        cpu_addr,
    input  wire [DATA_WIDTH-1:0]        cpu_wr_data,
    output reg  [DATA_WIDTH-1:0]        cpu_rd_data,
    output reg                          cpu_ack,

    // ── Network → Cache ───────────────────────────────────────────────────
    input  wire                         net_valid,
    input  wire [3:0]                   net_msg_type,
    input  wire [ADDR_WIDTH-1:0]        net_addr,
    input  wire [DATA_WIDTH-1:0]        net_data,
    input  wire [$clog2(NUM_CORES+1)-1:0] net_src_id,

    // ── Cache → Network ───────────────────────────────────────────────────
    output reg                          out_valid,
    output reg  [3:0]                   out_msg_type,
    output reg  [ADDR_WIDTH-1:0]        out_addr,
    output reg  [DATA_WIDTH-1:0]        out_data,
    output reg  [$clog2(NUM_CORES+1)-1:0] out_dst_id
);

// ── Local parameters ─────────────────────────────────────────────────────────
localparam ID_BITS = $clog2(NUM_CORES+1);
localparam DIR_ID  = NUM_CORES[ID_BITS-1:0];   // directory node id

// ── Message type encoding (must match directory_controller.v) ────────────────
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

// ── MESI state encoding ──────────────────────────────────────────────────────
localparam ST_I = 2'b00;   // Invalid
localparam ST_S = 2'b01;   // Shared
localparam ST_E = 2'b10;   // Exclusive
localparam ST_M = 2'b11;   // Modified

// ── FSM state encoding ───────────────────────────────────────────────────────
localparam CC_IDLE       = 4'd0;
localparam CC_SEND_GETS  = 4'd1;   // about to send GETS
localparam CC_SEND_GETM  = 4'd2;   // about to send GETM
localparam CC_WAIT_DATA  = 4'd3;   // GETS in-flight; waiting for data
localparam CC_WAIT_EXCL  = 4'd4;   // GETM in-flight; waiting for excl data
localparam CC_SILENT_M   = 4'd5;   // E→M silent upgrade (no network msg)
localparam CC_SEND_ACK   = 4'd6;   // send ACK to dir after FWD_GETS served
localparam CC_SEND_FLUSH = 4'd7;   // send FLUSH to dir after FWD_GETM served

// ── Internal registers ───────────────────────────────────────────────────────
reg [1:0]            mesi_state;
reg [DATA_WIDTH-1:0] cache_data;
reg [ADDR_WIDTH-1:0] cache_addr_r;
reg                  cache_valid;
reg [3:0]            cc_state;

// Pending CPU request fields
reg                  pend_wr;
reg [ADDR_WIDTH-1:0] pend_addr;
reg [DATA_WIDTH-1:0] pend_wr_data;

// FWD bookkeeping
reg [ID_BITS-1:0]    fwd_target;    // requester id extracted from FWD message
reg [ADDR_WIDTH-1:0] fwd_addr;      // address of the FWD'ed line
reg [DATA_WIDTH-1:0] flush_data;    // data copy to FLUSH to directory

// ── Main FSM ─────────────────────────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mesi_state   <= ST_I;
        cache_data   <= {DATA_WIDTH{1'b0}};
        cache_addr_r <= {ADDR_WIDTH{1'b0}};
        cache_valid  <= 1'b0;
        cc_state     <= CC_IDLE;
        cpu_ack      <= 1'b0;
        cpu_rd_data  <= {DATA_WIDTH{1'b0}};
        out_valid    <= 1'b0;
        out_msg_type <= 4'd0;
        out_addr     <= {ADDR_WIDTH{1'b0}};
        out_data     <= {DATA_WIDTH{1'b0}};
        out_dst_id   <= {ID_BITS{1'b0}};
        pend_wr      <= 1'b0;
        pend_addr    <= {ADDR_WIDTH{1'b0}};
        pend_wr_data <= {DATA_WIDTH{1'b0}};
        fwd_target   <= {ID_BITS{1'b0}};
        fwd_addr     <= {ADDR_WIDTH{1'b0}};
        flush_data   <= {DATA_WIDTH{1'b0}};
    end else begin
        // Default: clear one-cycle strobes
        cpu_ack   <= 1'b0;
        out_valid <= 1'b0;

        case (cc_state)

            // ─────────────────────────────────────────────────────────────────
            // IDLE: accept incoming coherence messages first, then CPU requests
            // ─────────────────────────────────────────────────────────────────
            CC_IDLE: begin
                if (net_valid) begin
                    case (net_msg_type)

                        // ── INV: directory orders invalidation ───────────────
                        MSG_INV: begin
                            if (mesi_state == ST_S || mesi_state == ST_E) begin
                                mesi_state  <= ST_I;
                                cache_valid <= 1'b0;
                            end
                            // Always ACK (even if already Invalid)
                            out_valid    <= 1'b1;
                            out_msg_type <= MSG_ACK;
                            out_addr     <= net_addr;
                            out_data     <= {DATA_WIDTH{1'b0}};
                            out_dst_id   <= DIR_ID;
                        end

                        // ── FWD_GETS: directory forwards read to M-owner ─────
                        // net_data[ID_BITS-1:0] = requester core id
                        MSG_FWD_GETS: begin
                            if (mesi_state == ST_M && cache_valid
                                    && cache_addr_r == net_addr) begin
                                fwd_target <= net_data[ID_BITS-1:0];
                                fwd_addr   <= net_addr;
                                // Send our data to the requester this cycle
                                out_valid    <= 1'b1;
                                out_msg_type <= MSG_DATA_TRANSFER;
                                out_addr     <= net_addr;
                                out_data     <= cache_data;
                                out_dst_id   <= net_data[ID_BITS-1:0];
                                // Downgrade to S
                                mesi_state   <= ST_S;
                                // Next cycle send ACK to directory
                                cc_state     <= CC_SEND_ACK;
                            end
                        end

                        // ── FWD_GETM: directory forwards write to M-owner ────
                        // net_data[ID_BITS-1:0] = requester core id
                        MSG_FWD_GETM: begin
                            if (mesi_state == ST_M && cache_valid
                                    && cache_addr_r == net_addr) begin
                                fwd_target <= net_data[ID_BITS-1:0];
                                fwd_addr   <= net_addr;
                                flush_data <= cache_data;
                                // Send data to new owner this cycle
                                out_valid    <= 1'b1;
                                out_msg_type <= MSG_DATA_TRANSFER;
                                out_addr     <= net_addr;
                                out_data     <= cache_data;
                                out_dst_id   <= net_data[ID_BITS-1:0];
                                // Invalidate ourselves
                                mesi_state   <= ST_I;
                                cache_valid  <= 1'b0;
                                // Next cycle FLUSH to directory
                                cc_state     <= CC_SEND_FLUSH;
                            end
                        end

                        default: ; // ignore unexpected messages in IDLE
                    endcase

                end else if (cpu_req) begin
                    // Latch CPU request
                    pend_addr    <= cpu_addr;
                    pend_wr      <= cpu_wr;
                    pend_wr_data <= cpu_wr_data;

                    if (!cpu_wr) begin
                        // ── READ ──────────────────────────────────────────────
                        if (cache_valid && cache_addr_r == cpu_addr
                                && mesi_state != ST_I) begin
                            // Hit in any valid MESI state
                            cpu_rd_data <= cache_data;
                            cpu_ack     <= 1'b1;
                        end else begin
                            // Miss → GETS
                            cc_state <= CC_SEND_GETS;
                        end
                    end else begin
                        // ── WRITE ─────────────────────────────────────────────
                        if (cache_valid && cache_addr_r == cpu_addr) begin
                            case (mesi_state)
                                ST_M: begin
                                    // Hit M: write in place, no network msg
                                    cache_data <= cpu_wr_data;
                                    cpu_ack    <= 1'b1;
                                end
                                ST_E: begin
                                    // Silent E→M upgrade
                                    cc_state <= CC_SILENT_M;
                                end
                                ST_S: begin
                                    // Upgrade request
                                    cc_state <= CC_SEND_GETM;
                                end
                                default: begin
                                    // ST_I: miss
                                    cc_state <= CC_SEND_GETM;
                                end
                            endcase
                        end else begin
                            // Different address or invalid → GETM
                            cc_state <= CC_SEND_GETM;
                        end
                    end
                end
            end // CC_IDLE

            // ─────────────────────────────────────────────────────────────────
            // CC_SEND_GETS: emit GETS to directory
            // ─────────────────────────────────────────────────────────────────
            CC_SEND_GETS: begin
                out_valid    <= 1'b1;
                out_msg_type <= MSG_GETS;
                out_addr     <= pend_addr;
                out_data     <= {DATA_WIDTH{1'b0}};
                out_dst_id   <= DIR_ID;
                cc_state     <= CC_WAIT_DATA;
            end

            // ─────────────────────────────────────────────────────────────────
            // CC_SEND_GETM: emit GETM to directory
            // ─────────────────────────────────────────────────────────────────
            CC_SEND_GETM: begin
                out_valid    <= 1'b1;
                out_msg_type <= MSG_GETM;
                out_addr     <= pend_addr;
                out_data     <= {DATA_WIDTH{1'b0}};
                out_dst_id   <= DIR_ID;
                cc_state     <= CC_WAIT_EXCL;
            end

            // ─────────────────────────────────────────────────────────────────
            // CC_WAIT_DATA: GETS in-flight; wait for DATA / DATA_EXCL
            //   Also handles spurious INV or peer DATA_TRANSFER
            // ─────────────────────────────────────────────────────────────────
            CC_WAIT_DATA: begin
                if (net_valid) begin
                    case (net_msg_type)
                        // Directory: first reader → Exclusive
                        MSG_DATA_EXCL: begin
                            cache_data   <= net_data;
                            cache_addr_r <= pend_addr;
                            cache_valid  <= 1'b1;
                            mesi_state   <= ST_E;
                            cpu_rd_data  <= net_data;
                            cpu_ack      <= 1'b1;
                            cc_state     <= CC_IDLE;
                        end
                        // Directory: another sharer exists → Shared
                        MSG_DATA: begin
                            cache_data   <= net_data;
                            cache_addr_r <= pend_addr;
                            cache_valid  <= 1'b1;
                            mesi_state   <= ST_S;
                            cpu_rd_data  <= net_data;
                            cpu_ack      <= 1'b1;
                            cc_state     <= CC_IDLE;
                        end
                        // Peer (M-owner) forwarded data directly (FWD_GETS path)
                        MSG_DATA_TRANSFER: begin
                            cache_data   <= net_data;
                            cache_addr_r <= pend_addr;
                            cache_valid  <= 1'b1;
                            mesi_state   <= ST_S;
                            cpu_rd_data  <= net_data;
                            cpu_ack      <= 1'b1;
                            cc_state     <= CC_IDLE;
                        end
                        // Interleaved INV while waiting for data
                        MSG_INV: begin
                            mesi_state  <= ST_I;
                            cache_valid <= 1'b0;
                            out_valid    <= 1'b1;
                            out_msg_type <= MSG_ACK;
                            out_addr     <= net_addr;
                            out_data     <= {DATA_WIDTH{1'b0}};
                            out_dst_id   <= DIR_ID;
                            // Remain in CC_WAIT_DATA; data is still coming
                        end
                        default: ;
                    endcase
                end
            end // CC_WAIT_DATA

            // ─────────────────────────────────────────────────────────────────
            // CC_WAIT_EXCL: GETM in-flight; wait for DATA_EXCL
            // ─────────────────────────────────────────────────────────────────
            CC_WAIT_EXCL: begin
                if (net_valid) begin
                    case (net_msg_type)
                        // Grant from directory (U→M or S→M path)
                        MSG_DATA_EXCL: begin
                            // For writes: use pend_wr_data; for ownership-only: keep net_data
                            cache_data   <= pend_wr ? pend_wr_data : net_data;
                            cache_addr_r <= pend_addr;
                            cache_valid  <= 1'b1;
                            mesi_state   <= ST_M;
                            cpu_ack      <= 1'b1;
                            cc_state     <= CC_IDLE;
                        end
                        // Old M-owner forwarded data (M→M transfer path)
                        MSG_DATA_TRANSFER: begin
                            cache_data   <= pend_wr ? pend_wr_data : net_data;
                            cache_addr_r <= pend_addr;
                            cache_valid  <= 1'b1;
                            mesi_state   <= ST_M;
                            cpu_ack      <= 1'b1;
                            cc_state     <= CC_IDLE;
                        end
                        // Spurious INV
                        MSG_INV: begin
                            mesi_state  <= ST_I;
                            cache_valid <= 1'b0;
                            out_valid    <= 1'b1;
                            out_msg_type <= MSG_ACK;
                            out_addr     <= net_addr;
                            out_data     <= {DATA_WIDTH{1'b0}};
                            out_dst_id   <= DIR_ID;
                        end
                        default: ;
                    endcase
                end
            end // CC_WAIT_EXCL

            // ─────────────────────────────────────────────────────────────────
            // CC_SILENT_M: E→M silent upgrade, no network message needed
            // ─────────────────────────────────────────────────────────────────
            CC_SILENT_M: begin
                mesi_state <= ST_M;
                cache_data <= pend_wr_data;
                cpu_ack    <= 1'b1;
                cc_state   <= CC_IDLE;
            end

            // ─────────────────────────────────────────────────────────────────
            // CC_SEND_ACK: DATA_TRANSFER to requester sent last cycle (FWD_GETS)
            //   Now inform directory the owner has downgraded to S
            // ─────────────────────────────────────────────────────────────────
            CC_SEND_ACK: begin
                out_valid    <= 1'b1;
                out_msg_type <= MSG_ACK;
                out_addr     <= fwd_addr;
                out_data     <= {DATA_WIDTH{1'b0}};
                out_dst_id   <= DIR_ID;
                cc_state     <= CC_IDLE;
            end

            // ─────────────────────────────────────────────────────────────────
            // CC_SEND_FLUSH: DATA_TRANSFER to new owner sent last cycle (FWD_GETM)
            //   Now write-back dirty data to directory
            // ─────────────────────────────────────────────────────────────────
            CC_SEND_FLUSH: begin
                out_valid    <= 1'b1;
                out_msg_type <= MSG_FLUSH;
                out_addr     <= fwd_addr;
                out_data     <= flush_data;
                out_dst_id   <= DIR_ID;
                cc_state     <= CC_IDLE;
            end

            default: cc_state <= CC_IDLE;
        endcase
    end
end

endmodule
