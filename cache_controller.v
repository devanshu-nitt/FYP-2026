// ============================================================
//  Write-Back Write-Allocate Cache Controller
//  Matches FSM: IDLE → LOOKUP → READ/WRITE/EVICTION/WRITE_ALLOC → UPDATE_META
//
//  NOTE — one small addition is required in cache_memory:
//    Add  output reg [TAG_BITS-1:0] victim_tag_out
//    and in the combinational miss path:
//        victim_tag_out = tag_array[way][index];
//    (needed so the controller knows the victim block's MM address during eviction)
// ============================================================

module cache_controller #(
    parameter ADDR_WIDTH  = 16,
    parameter DATA_WIDTH  = 8,
    parameter INDEX_BITS  = 6,
    parameter OFFSET_BITS = 2,
    parameter WAYS        = 4
)(
    input  wire                   clk,
    input  wire                   rst,

    // ── CPU Interface ──────────────────────────────────────────
    input  wire                   cpu_read,
    input  wire                   cpu_write,
    input  wire [ADDR_WIDTH-1:0]  cpu_addr,
    input  wire [DATA_WIDTH-1:0]  cpu_wdata,
    output reg  [DATA_WIDTH-1:0]  cpu_rdata,
    output reg                    cpu_ready,

    // ── Cache Memory Interface ─────────────────────────────────
    // Outputs to cache
    output reg                    cm_read,
    output reg                    cm_write,
    output reg                    cm_lookup,
    output reg                    cm_update_meta,
    output reg                    cm_reset_dirty,
    output reg  [ADDR_WIDTH-1:0]  cm_addr,
    output reg  [DATA_WIDTH-1:0]  cm_wdata,
    output reg  [1:0]             cm_input_way,
    // Inputs from cache
    input  wire [DATA_WIDTH-1:0]  cm_rdata,
    input  wire                   cm_hit,
    input  wire                   cm_dirty,
    input  wire                   cm_write_ack,
    input  wire                   cm_read_ack,
    input  wire [1:0]             cm_way,
    input  wire                   cm_miss,
    input  wire [ADDR_WIDTH-OFFSET_BITS-INDEX_BITS-1:0] cm_victim_tag, // added port

    // ── Main Memory Interface ──────────────────────────────────
    output reg                    mm_read,
    output reg                    mm_write,
    output reg  [ADDR_WIDTH-1:0]  mm_addr,
    output reg  [DATA_WIDTH-1:0]  mm_wdata,
    input  wire [DATA_WIDTH-1:0]  mm_rdata,
    input  wire                   mm_ready
);

    // ── Derived constants ──────────────────────────────────────
    localparam TAG_BITS   = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    localparam BLOCK_LAST = {OFFSET_BITS{1'b1}};   // = BLOCK_SIZE-1, all 1s

    // ── Top-level FSM state encoding ───────────────────────────
    localparam [2:0]
        S_IDLE   = 3'd0,
        S_LOOKUP = 3'd1,
        S_READ   = 3'd2,
        S_WRITE  = 3'd3,
        S_WALLOC = 3'd4,
        S_EVICT  = 3'd5,
        S_META   = 3'd6;

    // ── Eviction sub-phase encoding ────────────────────────────
    // For each offset word: read from cache → buffer → write to MM
    localparam [1:0]
        EV_RD  = 2'd0,   // assert cm_read (issue cache read)
        EV_WCM = 2'd1,   // wait cm_read_ack, buffer word
        EV_WR  = 2'd2,   // assert mm_write (issue MM write)
        EV_WMM = 2'd3;   // wait mm_ready; advance or finish

    // ── Write-Allocate sub-phase encoding ─────────────────────
    // For each offset word: read from MM → write to cache
    localparam [1:0]
        WA_RD  = 2'd0,   // assert mm_read (issue MM read)
        WA_WMM = 2'd1,   // wait mm_ready, data valid
        WA_WR  = 2'd2,   // assert cm_write (install into cache)
        WA_WCM = 2'd3;   // wait cm_write_ack; advance or finish

    // ── Registers ─────────────────────────────────────────────
    reg [2:0]              state;

    // Latched CPU request
    reg                    op_read;
	reg			op_write;
	reg			read_miss;
    reg [ADDR_WIDTH-1:0]   req_addr;
    reg [DATA_WIDTH-1:0]   req_wdata;

    // Victim info (latched at LOOKUP on miss)
    reg [1:0]              victim_way;
    reg [TAG_BITS-1:0]     victim_tag_r;   // tag of the line being evicted
    reg                    was_miss;        // 1 = came through WRITE_ALLOC path

    // Block-fill loop counter
    reg [OFFSET_BITS-1:0]  off_cnt;

    // Single-word pipeline buffer used during eviction
    reg [DATA_WIDTH-1:0]   ev_buf;

    // Sub-phase registers
    reg [1:0]              ev_ph;           // eviction sub-phase
    reg [1:0]              wa_ph;           // write-alloc sub-phase
    reg                    um_ph;           // update-meta sub-phase

    // LOOKUP needs two clock cycles:
    //   cycle 1 – drive cm_lookup/cm_addr so cache comb block evaluates
    //   cycle 2 – sample cm_hit/cm_miss (now stable) and branch
    reg                    lk_sent;

    // ── Address field wires ────────────────────────────────────
    wire [OFFSET_BITS-1:0] req_off = req_addr[OFFSET_BITS-1 : 0];
    wire [INDEX_BITS-1:0]  req_idx = req_addr[OFFSET_BITS +: INDEX_BITS];
    wire [TAG_BITS-1:0]    req_tag = req_addr[OFFSET_BITS+INDEX_BITS +: TAG_BITS];

    // ══════════════════════════════════════════════════════════
    //  Sequential FSM
    // ══════════════════════════════════════════════════════════
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state         <= S_IDLE;
            op_read       <= 0;  op_write	<=0; req_addr    <= 0;  req_wdata <= 0;
            victim_way    <= 0;  victim_tag_r <= 0; was_miss  <= 0;
            off_cnt       <= 0;  ev_buf      <= 0;
            ev_ph         <= EV_RD;
            wa_ph         <= WA_RD;
            um_ph         <= 1'b0;
            lk_sent       <= 0;
            // All output strobes low
            cm_read       <= 0;  cm_write       <= 0;  cm_lookup      <= 0;
            cm_update_meta <= 0; cm_reset_dirty <= 0;
            cm_addr       <= 0;  cm_wdata       <= 0;  cm_input_way   <= 0;
            mm_read       <= 0;  mm_write       <= 0;
            mm_addr       <= 0;  mm_wdata       <= 0;
            cpu_rdata     <= 0;  cpu_ready      <= 0;
        end else begin

            // ── Default: de-assert all one-cycle strobes ───────
            cm_read        <= 0;
            cm_write       <= 0;
            cm_lookup      <= 0;
            cm_update_meta <= 0;
            cm_reset_dirty <= 0;
            mm_read        <= 0;
            mm_write       <= 0;
            cpu_ready      <= 0;

            case (state)

            // ──────────────────────────────────────────────────
            //  IDLE
            //  Poll CPU bus; latch request and move to LOOKUP.
            // ──────────────────────────────────────────────────
            S_IDLE: begin
                if (cpu_read || cpu_write) begin
                    op_read   <= cpu_read;
			op_write	<= cpu_write;
                    req_addr  <= cpu_addr;
                    req_wdata <= cpu_wdata;
                    lk_sent   <= 1'b0;
                    state     <= S_LOOKUP;
			if(cpu_read) read_miss <= 1;
			else if (cpu_write) read_miss <=0;
                end
            end

            // ──────────────────────────────────────────────────
            //  LOOKUP  (2-cycle handshake)
            //  Cycle 1 (lk_sent=0): assert cm_lookup + cm_addr
            //                       cache combinational block fires
            //  Cycle 2 (lk_sent=1): cm_hit / cm_miss are stable → branch
            //
            //  Hit  → S_READ or S_WRITE
            //  Miss, clean victim → S_WALLOC
            //  Miss, dirty victim → S_EVICT
            // ──────────────────────────────────────────────────
            S_LOOKUP: begin
                cm_lookup <= 1'b1;
                cm_addr   <= req_addr;

                if (!lk_sent) begin
                    lk_sent <= 1'b1;          // wait one cycle for comb to settle
                end else begin
                    lk_sent <= 1'b0;
                    if (cm_hit) begin
                        victim_way <= cm_way;
                        was_miss   <= 1'b0;
			
                        state      <= read_miss ? S_READ : S_WRITE;
                    end else if (cm_miss) begin            // miss
                        victim_way   <= cm_way;
                        victim_tag_r <= cm_victim_tag;
                        was_miss     <= 1'b1;
                        state        <= cm_dirty ? S_EVICT : S_WALLOC;
                    end
                end
            end

            // ──────────────────────────────────────────────────
            //  READ
            //  Drive cm_read; wait for cm_read_ack (one cycle
            //  after cm_read is asserted).  Forward data to CPU.
            //  Next: S_META
            // ──────────────────────────────────────────────────
            S_READ: begin
                cm_read      <= 1'b1;
                cm_addr      <= req_addr;
                cm_input_way <= victim_way;
                if (cm_read_ack) begin
                    cpu_rdata <= cm_rdata;
                    state     <= S_META;
                end
            end

            // ──────────────────────────────────────────────────
            //  WRITE
            //  Drive cm_write at req_addr/victim_way with CPU
            //  data; cache sets dirty=1 internally.
            //  Wait for cm_write_ack.  Next: S_META
            // ──────────────────────────────────────────────────
            S_WRITE: begin
                cm_write     <= 1'b1;
                cm_addr      <= req_addr;
                cm_input_way <= victim_way;
                cm_wdata     <= req_wdata;
                if (cm_write_ack)
                    state <= S_META;
            end

            // ──────────────────────────────────────────────────
            //  EVICTION
            //  Write every word of the dirty victim line to MM,
            //  one word per iteration.
            //
            //  EV_RD  → assert cm_read at {victim_tag,idx,off}
            //  EV_WCM → wait cm_read_ack; buffer in ev_buf
            //  EV_WR  → assert mm_write with ev_buf
            //  EV_WMM → wait mm_ready; off_cnt++ or finish → S_WALLOC
            // ──────────────────────────────────────────────────
            S_EVICT: begin
                case (ev_ph)

                    EV_RD: begin
                        cm_read      <= 1'b1;
                        cm_addr      <= {victim_tag_r, req_idx, off_cnt};
                        cm_input_way <= victim_way;
                        ev_ph        <= EV_WCM;
                    end

                    EV_WCM: begin
                        if (cm_read_ack) begin
                            ev_buf <= cm_rdata;
                            ev_ph  <= EV_WR;
                        end
                    end

                    EV_WR: begin
                        mm_write <= 1'b1;
                        mm_addr  <= {victim_tag_r, req_idx, off_cnt};
                        mm_wdata <= ev_buf;
                        ev_ph    <= EV_WMM;
                    end

                    EV_WMM: begin
                        if (mm_ready) begin
                            if (off_cnt == BLOCK_LAST) begin
                                off_cnt <= 0;
                                ev_ph   <= EV_RD;
                                state   <= S_WALLOC;   // eviction complete
                            end else begin
                                off_cnt <= off_cnt + 1'b1;
                                ev_ph   <= EV_RD;
                            end
                        end
                    end

                endcase
            end

            // ──────────────────────────────────────────────────
            //  WRITE_ALLOC
            //  Fetch every word of the requested block from MM
            //  and install into the victim cache way.
            //
            //  WA_RD  → assert mm_read at {req_tag,idx,off}
            //  WA_WMM → wait mm_ready (data is then in mm_rdata)
            //  WA_WR  → assert cm_write with mm_rdata
            //  WA_WCM → wait cm_write_ack; off_cnt++ or finish
            //           if done: op_read → S_READ, else → S_WRITE
            //
            //  Note: cache writes here set dirty=1 transiently.
            //  For a read-miss this is cleared in S_META (um_ph=1).
            //  For a write-miss, S_WRITE immediately follows and
            //  re-asserts dirty=1 (correct final state).
            // ──────────────────────────────────────────────────
            S_WALLOC: begin
                case (wa_ph)

                    WA_RD: begin
                        mm_read  <= 1'b1;
                        mm_addr  <= {req_tag, req_idx, off_cnt};
                        wa_ph    <= WA_WMM;
                    end

                    WA_WMM: begin
                        if (mm_ready)
                            wa_ph <= WA_WR;
                    end

                    WA_WR: begin
                        cm_write     <= 1'b1;
                        cm_addr      <= {req_tag, req_idx, off_cnt};
                        cm_input_way <= victim_way;
                        cm_wdata     <= mm_rdata;
                        wa_ph        <= WA_WCM;
                    end

                    WA_WCM: begin
                        if (cm_write_ack) begin
                            if (off_cnt == BLOCK_LAST) begin
                                off_cnt <= 0;
                                wa_ph   <= WA_RD;
                                // Serve the original CPU request
                                state   <= read_miss ? S_READ : S_WRITE;
                            end else begin
                                off_cnt <= off_cnt + 1'b1;
                                wa_ph   <= WA_RD;
                            end
                        end
                    end

                endcase
            end

            // ──────────────────────────────────────────────────
            //  UPDATE_META  (2-cycle, split to avoid cm_update_meta
            //               and cm_reset_dirty being asserted
            //               simultaneously — cache uses if/else if)
            //
            //  um_ph = 0: cm_update_meta → updates LRU rank
            //  um_ph = 1: cm_reset_dirty (read-miss only)
            //             + assert cpu_ready → return to S_IDLE
            // ──────────────────────────────────────────────────
            S_META: begin
                case (um_ph)

                    1'b0: begin   // Phase 0: update LRU
                        cm_update_meta <= 1'b1;
                        cm_input_way   <= victim_way;
                        cm_addr        <= req_addr;
                        um_ph          <= 1'b1;
                    end

                    1'b1: begin   // Phase 1: clean dirty if read-miss; notify CPU
                        // A read-miss block was loaded from MM (clean) but
                        // WRITE_ALLOC's cache writes set dirty=1 transiently.
                        // Reset it here so the line correctly reflects MM state.
                        if (read_miss && was_miss) begin
                            cm_reset_dirty <= 1'b1;
                            cm_input_way   <= victim_way;
                            cm_addr        <= req_addr;
                        end
                        cpu_ready <= 1'b1;
                        um_ph     <= 1'b0;
                        state     <= S_IDLE;
                    end

                endcase
            end

            default: state <= S_IDLE;

            endcase
        end
    end

endmodule



//     end
