module main_memory #(
    parameter ADDR_WIDTH = 16,        // MUST match cache/controller
    parameter DATA_WIDTH = 8,
    parameter MEM_DEPTH  = 1024,
    parameter LATENCY    = 5
    //parameter VALID_ADDR_WIDTH = $clog2(MEM_DEPTH)
)(
    input clk,
    input reset,

    // Request interface
    input                  read,
    input                  write,
    input [ADDR_WIDTH-1:0] addr,
    input [DATA_WIDTH-1:0] write_data,

    output reg [DATA_WIDTH-1:0] read_data,
    output reg                 ready
);

    // =========================================================
    // 🔷 BRAM STORAGE (FORCED)
    // =========================================================
    (* ram_style = "block" *) 
    reg [DATA_WIDTH-1:0] memory [0:MEM_DEPTH-1];

    // =========================================================
    // 🔷 INTERNAL REGISTERS
    // =========================================================
    reg [ADDR_WIDTH-1:0] addr_reg;
    reg [DATA_WIDTH-1:0] write_data_reg;
    reg write_reg;

    reg [$clog2(LATENCY+1):0] counter;
    reg busy;

    integer i;

    // =========================================================
    // 🔷 INITIALIZATION (SYNTH OK FOR FPGA)
    // =========================================================
    initial begin
        for (i = 0; i < MEM_DEPTH; i = i + 1)
            memory[i] = 0;
    end

    // =========================================================
    // 🔷 CONTROL FSM (NO MEMORY ACCESS HERE)
    // =========================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ready   <= 0;
            busy    <= 0;
            counter <= 0;
        end else begin

            // NEW REQUEST
            if (!busy && (read || write)) begin
                addr_reg       <= addr[ADDR_WIDTH-1:0];
                write_data_reg <= write_data;
                write_reg      <= write;

                busy    <= 1;
                counter <= LATENCY;
                ready   <= 0;
            end

            // LATENCY HANDLING
            else if (busy) begin
                if (counter > 0) begin
                    counter <= counter - 1;
                end else begin
                    ready <= 1;
                    busy  <= 0;
                end
            end

            // IDLE
            else begin
                ready <= 0;
            end
        end
    end

    // =========================================================
    // 🔷 BRAM ACCESS (CRITICAL FIX)
    // =========================================================
    always @(posedge clk) begin
        // WRITE
        if (write_reg)
            memory[addr_reg] <= write_data_reg;

        // READ (ALWAYS ACTIVE → REQUIRED FOR BRAM)
        read_data <= memory[addr_reg];
    end

endmodule
