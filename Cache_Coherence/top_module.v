// =============================================================================
// top_module.v
// Top-level integration for the MESI directory-based coherence system.
//
// Scalability: change NUM_CORES parameter only.
//   All sub-module port widths, generate loops, and wire dimensions adapt
//   automatically via localparams and parameterised instances.
//
// ID scheme:
//   ID_BITS = $clog2(NUM_CORES+1)
//   Core i → id i,   Directory → id NUM_CORES
// =============================================================================
`timescale 1ns/1ps

module top_module #(
    parameter NUM_CORES  = 2,
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 8
)(
    input  wire clk,
    input  wire rst_n,

    // ── CPU interfaces (flat arrays, indexed [core * width +: width]) ─────
    input  wire [NUM_CORES-1:0]             cpu_req,
    input  wire [NUM_CORES-1:0]             cpu_wr,
    input  wire [NUM_CORES*ADDR_WIDTH-1:0]  cpu_addr,
    input  wire [NUM_CORES*DATA_WIDTH-1:0]  cpu_wr_data,
    output wire [NUM_CORES*DATA_WIDTH-1:0]  cpu_rd_data,
    output wire [NUM_CORES-1:0]             cpu_ack
);

localparam ID_BITS = $clog2(NUM_CORES+1);

// ── Core → Network wires ──────────────────────────────────────────────────────
wire [NUM_CORES-1:0]              core_out_valid;
wire [NUM_CORES*4-1:0]            core_out_msg_type;
wire [NUM_CORES*ADDR_WIDTH-1:0]   core_out_addr;
wire [NUM_CORES*DATA_WIDTH-1:0]   core_out_data;
wire [NUM_CORES*ID_BITS-1:0]      core_out_dst_id;

// ── Network → Core wires ──────────────────────────────────────────────────────
wire [NUM_CORES-1:0]              core_in_valid;
wire [NUM_CORES*4-1:0]            core_in_msg_type;
wire [NUM_CORES*ADDR_WIDTH-1:0]   core_in_addr;
wire [NUM_CORES*DATA_WIDTH-1:0]   core_in_data;
wire [NUM_CORES*ID_BITS-1:0]      core_in_src_id;

// ── Network ↔ Directory wires ─────────────────────────────────────────────────
wire              dir_in_valid;
wire [3:0]        dir_in_msg_type;
wire [ADDR_WIDTH-1:0] dir_in_addr;
wire [DATA_WIDTH-1:0] dir_in_data;
wire [ID_BITS-1:0]    dir_in_src_id;

wire              dir_out_valid;
wire [3:0]        dir_out_msg_type;
wire [ADDR_WIDTH-1:0] dir_out_addr;
wire [DATA_WIDTH-1:0] dir_out_data;
wire [ID_BITS-1:0]    dir_out_dst_id;

// ── Directory ↔ Memory wires ──────────────────────────────────────────────────
wire              mem_rd_en;
wire              mem_wr_en;
wire [ADDR_WIDTH-1:0] mem_addr;
wire [DATA_WIDTH-1:0] mem_wr_data;
wire [DATA_WIDTH-1:0] mem_rd_data;

// ── Cache controller instances ────────────────────────────────────────────────
generate
    genvar c;
    for (c = 0; c < NUM_CORES; c = c + 1) begin : gen_cores
        cache_controller #(
            .NUM_CORES (NUM_CORES),
            .CORE_ID   (c),
            .DATA_WIDTH(DATA_WIDTH),
            .ADDR_WIDTH(ADDR_WIDTH)
        ) u_cc (
            .clk          (clk),
            .rst_n        (rst_n),
            // CPU
            .cpu_req      (cpu_req[c]),
            .cpu_wr       (cpu_wr[c]),
            .cpu_addr     (cpu_addr    [c*ADDR_WIDTH +: ADDR_WIDTH]),
            .cpu_wr_data  (cpu_wr_data [c*DATA_WIDTH +: DATA_WIDTH]),
            .cpu_rd_data  (cpu_rd_data [c*DATA_WIDTH +: DATA_WIDTH]),
            .cpu_ack      (cpu_ack[c]),
            // Network in
            .net_valid    (core_in_valid[c]),
            .net_msg_type (core_in_msg_type[c*4 +: 4]),
            .net_addr     (core_in_addr    [c*ADDR_WIDTH +: ADDR_WIDTH]),
            .net_data     (core_in_data    [c*DATA_WIDTH +: DATA_WIDTH]),
            .net_src_id   (core_in_src_id  [c*ID_BITS   +: ID_BITS]),
            // Network out
            .out_valid    (core_out_valid[c]),
            .out_msg_type (core_out_msg_type[c*4        +: 4]),
            .out_addr     (core_out_addr    [c*ADDR_WIDTH +: ADDR_WIDTH]),
            .out_data     (core_out_data    [c*DATA_WIDTH +: DATA_WIDTH]),
            .out_dst_id   (core_out_dst_id  [c*ID_BITS   +: ID_BITS])
        );
    end
endgenerate

// ── Coherence network ─────────────────────────────────────────────────────────
coherence_network #(
    .NUM_CORES (NUM_CORES),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) u_net (
    .clk              (clk),
    .rst_n            (rst_n),
    .core_out_valid   (core_out_valid),
    .core_out_msg_type(core_out_msg_type),
    .core_out_addr    (core_out_addr),
    .core_out_data    (core_out_data),
    .core_out_dst_id  (core_out_dst_id),
    .core_in_valid    (core_in_valid),
    .core_in_msg_type (core_in_msg_type),
    .core_in_addr     (core_in_addr),
    .core_in_data     (core_in_data),
    .core_in_src_id   (core_in_src_id),
    .dir_in_valid     (dir_in_valid),
    .dir_in_msg_type  (dir_in_msg_type),
    .dir_in_addr      (dir_in_addr),
    .dir_in_data      (dir_in_data),
    .dir_in_src_id    (dir_in_src_id),
    .dir_out_valid    (dir_out_valid),
    .dir_out_msg_type (dir_out_msg_type),
    .dir_out_addr     (dir_out_addr),
    .dir_out_data     (dir_out_data),
    .dir_out_dst_id   (dir_out_dst_id)
);

// ── Directory controller ──────────────────────────────────────────────────────
directory_controller #(
    .NUM_CORES (NUM_CORES),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) u_dir (
    .clk          (clk),
    .rst_n        (rst_n),
    .net_valid    (dir_in_valid),
    .net_msg_type (dir_in_msg_type),
    .net_addr     (dir_in_addr),
    .net_data     (dir_in_data),
    .net_src_id   (dir_in_src_id),
    .out_valid    (dir_out_valid),
    .out_msg_type (dir_out_msg_type),
    .out_addr     (dir_out_addr),
    .out_data     (dir_out_data),
    .out_dst_id   (dir_out_dst_id),
    .mem_rd_en    (mem_rd_en),
    .mem_wr_en    (mem_wr_en),
    .mem_addr     (mem_addr),
    .mem_wr_data  (mem_wr_data),
    .mem_rd_data  (mem_rd_data)
);

// ── Shared backing memory ─────────────────────────────────────────────────────
shared_memory #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) u_mem (
    .clk    (clk),
    .rst_n  (rst_n),
    .rd_en  (mem_rd_en),
    .wr_en  (mem_wr_en),
    .addr   (mem_addr),
    .wr_data(mem_wr_data),
    .rd_data(mem_rd_data)
);

endmodule
