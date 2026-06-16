// =============================================================================
// coherence_network.v  – final corrected version
// Scalable point-to-point MESI coherence message fabric.
//
// Three routing paths:
//   PATH-1  Core → Directory   (round-robin arbitration among cores)
//   PATH-2  Core → Core        (peer forwarding, e.g. DATA_TRANSFER)
//   PATH-3  Directory → Core   (no arbitration; overrides PATH-2 same dst)
//
// Two pipeline stages:
//   Stage-1 (latch): captures all inputs on posedge (eliminates combo loops)
//   Stage-2 (route): computes and drives outputs one cycle later
//
// ID scheme:  ID_BITS = $clog2(NUM_CORES+1)
//   Core i → id i  (0..NUM_CORES-1)    Directory → id NUM_CORES
// =============================================================================
`timescale 1ns/1ps

module coherence_network #(
    parameter NUM_CORES  = 2,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 8
)(
    input  wire clk,
    input  wire rst_n,

    // Core → Network
    input  wire [NUM_CORES-1:0]                       core_out_valid,
    input  wire [NUM_CORES*4-1:0]                     core_out_msg_type,
    input  wire [NUM_CORES*ADDR_WIDTH-1:0]            core_out_addr,
    input  wire [NUM_CORES*DATA_WIDTH-1:0]            core_out_data,
    input  wire [NUM_CORES*$clog2(NUM_CORES+1)-1:0]   core_out_dst_id,

    // Network → Core
    output reg  [NUM_CORES-1:0]                       core_in_valid,
    output reg  [NUM_CORES*4-1:0]                     core_in_msg_type,
    output reg  [NUM_CORES*ADDR_WIDTH-1:0]            core_in_addr,
    output reg  [NUM_CORES*DATA_WIDTH-1:0]            core_in_data,
    output reg  [NUM_CORES*$clog2(NUM_CORES+1)-1:0]   core_in_src_id,

    // Network → Directory
    output reg                                        dir_in_valid,
    output reg  [3:0]                                 dir_in_msg_type,
    output reg  [ADDR_WIDTH-1:0]                      dir_in_addr,
    output reg  [DATA_WIDTH-1:0]                      dir_in_data,
    output reg  [$clog2(NUM_CORES+1)-1:0]             dir_in_src_id,

    // Directory → Network
    input  wire                                       dir_out_valid,
    input  wire [3:0]                                 dir_out_msg_type,
    input  wire [ADDR_WIDTH-1:0]                      dir_out_addr,
    input  wire [DATA_WIDTH-1:0]                      dir_out_data,
    input  wire [$clog2(NUM_CORES+1)-1:0]             dir_out_dst_id
);

localparam ID_BITS = $clog2(NUM_CORES+1);
localparam DIR_ID  = NUM_CORES[ID_BITS-1:0];

// ── Stage-1 input latches ──────────────────────────────────────────────────────
reg [NUM_CORES-1:0]            s1_cv;
reg [NUM_CORES*4-1:0]          s1_cm;
reg [NUM_CORES*ADDR_WIDTH-1:0] s1_ca;
reg [NUM_CORES*DATA_WIDTH-1:0] s1_cd;
reg [NUM_CORES*ID_BITS-1:0]    s1_dst;

reg                            s1_dv;
reg [3:0]                      s1_dm;
reg [ADDR_WIDTH-1:0]           s1_da;
reg [DATA_WIDTH-1:0]           s1_dd;
reg [ID_BITS-1:0]              s1_ddst;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s1_cv   <= {NUM_CORES{1'b0}};
        s1_cm   <= {NUM_CORES*4{1'b0}};
        s1_ca   <= {NUM_CORES*ADDR_WIDTH{1'b0}};
        s1_cd   <= {NUM_CORES*DATA_WIDTH{1'b0}};
        s1_dst  <= {NUM_CORES*ID_BITS{1'b0}};
        s1_dv   <= 1'b0;
        s1_dm   <= 4'd0;
        s1_da   <= {ADDR_WIDTH{1'b0}};
        s1_dd   <= {DATA_WIDTH{1'b0}};
        s1_ddst <= {ID_BITS{1'b0}};
    end else begin
        s1_cv   <= core_out_valid;
        s1_cm   <= core_out_msg_type;
        s1_ca   <= core_out_addr;
        s1_cd   <= core_out_data;
        s1_dst  <= core_out_dst_id;
        s1_dv   <= dir_out_valid;
        s1_dm   <= dir_out_msg_type;
        s1_da   <= dir_out_addr;
        s1_dd   <= dir_out_data;
        s1_ddst <= dir_out_dst_id;
    end
end

// ── Stage-2 routing ────────────────────────────────────────────────────────────
// Loop variables declared at module scope for Verilog-2001 compatibility
integer k, idx, dst, fwd;
reg [ID_BITS-1:0] rr_ptr;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rr_ptr           <= {ID_BITS{1'b0}};
        dir_in_valid     <= 1'b0;
        dir_in_msg_type  <= 4'd0;
        dir_in_addr      <= {ADDR_WIDTH{1'b0}};
        dir_in_data      <= {DATA_WIDTH{1'b0}};
        dir_in_src_id    <= {ID_BITS{1'b0}};
        core_in_valid    <= {NUM_CORES{1'b0}};
        core_in_msg_type <= {NUM_CORES*4{1'b0}};
        core_in_addr     <= {NUM_CORES*ADDR_WIDTH{1'b0}};
        core_in_data     <= {NUM_CORES*DATA_WIDTH{1'b0}};
        core_in_src_id   <= {NUM_CORES*ID_BITS{1'b0}};
    end else begin
        // Default: clear valid strobes only; hold data (cleaner debug)
        dir_in_valid  <= 1'b0;
        core_in_valid <= {NUM_CORES{1'b0}};

        // ── PATH-1: Core → Directory (round-robin) ────────────────────────────
        fwd = 0;
        for (k = 0; k < NUM_CORES; k = k + 1) begin
            if (!fwd) begin
                idx = rr_ptr + k;
                if (idx >= NUM_CORES) idx = idx - NUM_CORES;
                if (s1_cv[idx] &&
                    (s1_dst[idx*ID_BITS +: ID_BITS] == DIR_ID)) begin
                    dir_in_valid    <= 1'b1;
                    dir_in_msg_type <= s1_cm [idx*4        +: 4];
                    dir_in_addr     <= s1_ca [idx*ADDR_WIDTH +: ADDR_WIDTH];
                    dir_in_data     <= s1_cd [idx*DATA_WIDTH +: DATA_WIDTH];
                    dir_in_src_id   <= idx[ID_BITS-1:0];
                    rr_ptr <= (idx + 1 >= NUM_CORES) ?
                              {ID_BITS{1'b0}} : idx[ID_BITS-1:0] + 1'b1;
                    fwd = 1;
                end
            end
        end

        // ── PATH-2: Core → Core (lower priority than PATH-3) ─────────────────
        for (k = 0; k < NUM_CORES; k = k + 1) begin
            if (s1_cv[k]) begin
                dst = s1_dst[k*ID_BITS +: ID_BITS];
                if ((dst != DIR_ID) && (dst < NUM_CORES)) begin
                    core_in_valid   [dst]                           <= 1'b1;
                    core_in_msg_type[dst*4        +: 4]             <= s1_cm [k*4        +: 4];
                    core_in_addr    [dst*ADDR_WIDTH +: ADDR_WIDTH]  <= s1_ca [k*ADDR_WIDTH +: ADDR_WIDTH];
                    core_in_data    [dst*DATA_WIDTH +: DATA_WIDTH]  <= s1_cd [k*DATA_WIDTH +: DATA_WIDTH];
                    core_in_src_id  [dst*ID_BITS   +: ID_BITS]     <= k[ID_BITS-1:0];
                end
            end
        end

        // ── PATH-3: Directory → Core (highest priority for core_in) ──────────
        if (s1_dv) begin
            dst = s1_ddst;
            if (dst < NUM_CORES) begin
                core_in_valid   [dst]                          <= 1'b1;
                core_in_msg_type[dst*4        +: 4]            <= s1_dm;
                core_in_addr    [dst*ADDR_WIDTH +: ADDR_WIDTH]  <= s1_da;
                core_in_data    [dst*DATA_WIDTH +: DATA_WIDTH]  <= s1_dd;
                core_in_src_id  [dst*ID_BITS   +: ID_BITS]     <= DIR_ID;
            end
        end

    end
end

endmodule
