module main_memory #(
    parameter ADDR_WIDTH = 16,        // Address bits
    parameter DATA_WIDTH = 8,        // Word size
    parameter MEM_DEPTH  = (1<<ADDR_WIDTH),   // Number of words
    parameter LATENCY    = 5          // Read latency cycles
)(
    input clk,
    input reset,

    // Request interface
    input                  read,
    input                  write,        // write enable
    input [ADDR_WIDTH-1:0] addr,
    input [DATA_WIDTH-1:0] write_data,

    output reg [DATA_WIDTH-1:0] read_data,
    output reg                 ready
);

    reg [DATA_WIDTH-1:0] memory [0:MEM_DEPTH-1];

    reg [ADDR_WIDTH-1:0] addr_reg;
    reg [DATA_WIDTH-1:0] write_data_reg;
    reg write_reg;
	reg read_reg;

    reg [$clog2(LATENCY+1):0] counter;
    reg busy;
	//assign addr_reg= addr[ $clog2(MEM_DEPTH)-1:0]
    integer i;

    // Initialize memory
    initial begin
        for (i = 0; i < MEM_DEPTH; i = i + 1)
            memory[i] = 0;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ready   <= 0;
            busy    <= 0;
            counter <= 0;
        end else begin
            if (!busy && (read || write) ) begin
                // Latch request
                addr_reg<= addr[ ADDR_WIDTH-1:0];
                write_data_reg <= write_data;
                write_reg    <= write;
		read_reg	<=read;
                busy    <= 1;
                counter <= LATENCY;
                ready   <= 0;
            end else if (busy) begin
                if (counter > 0) begin
                    counter <= counter - 1;
                end 
		else if (counter==0) begin
                    // Perform operation
                    if (write_reg) begin
                        memory[addr_reg] <= write_data_reg;
                    end else begin
                        read_data <= memory[addr_reg];
                    end

                    ready <= 1;
                    busy  <= 0;
                end
            end else begin
                ready <= 0;
            end
        end
    end

endmodule
