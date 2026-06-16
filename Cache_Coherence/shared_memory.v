// =============================================================================
// shared_memory.v
// Backing store for the directory coherence system.
//
// Read:  combinational (rd_data valid same cycle as rd_en & addr).
//        This avoids the 2-cycle latency issue in the directory FSM.
// Write: synchronous on posedge clk when wr_en=1.
//
// Parameters:
//   DATA_WIDTH – word width in bits
//   ADDR_WIDTH – address bits (depth = 2^ADDR_WIDTH words)
// =============================================================================
`timescale 1ns/1ps

module shared_memory #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  rd_en,
    input  wire                  wr_en,
    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire [DATA_WIDTH-1:0] rd_data    // combinational read
);

localparam DEPTH = 1 << ADDR_WIDTH;
reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

integer j;
initial begin
    for (j = 0; j < DEPTH; j = j + 1)
        mem[j] = j * 32'h11111111;
end

// Synchronous write
always @(posedge clk) begin
    if (wr_en)
        mem[addr] <= wr_data;
end

// Combinational read (same-cycle, gated by rd_en)
assign rd_data = rd_en ? mem[addr] : {DATA_WIDTH{1'b0}};

endmodule
