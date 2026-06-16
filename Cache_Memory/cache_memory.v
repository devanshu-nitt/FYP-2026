module cache_memory #(
	parameter ADDR_WIDTH  = 16,
	parameter DATA_WIDTH  = 8,
    	parameter INDEX_BITS  = 6,
    	parameter OFFSET_BITS = 2,
    	parameter WAYS        = 4,
	parameter TAG_BITS   = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS
	)(
    	input  wire                   clk,
	input  wire			rst,
    	input  wire [ADDR_WIDTH-1:0]  addr,
    	input  wire                   read,
    	input  wire                   write,
    	input  wire [DATA_WIDTH-1:0]  write_data,
    	input  wire                   update_meta,
    	input  wire [1:0]             input_way,
	input  wire                   lookup,
    	input  wire                   reset_dirty,
	
    	output reg  [DATA_WIDTH-1:0]  read_data,
    	output reg                    cache_hit,
    	output reg                    dirty,
    	output reg                    write_ack,
    	output reg                    read_ack,
    	output reg  [1:0]             way,
	output reg			miss,
	output reg [TAG_BITS-1:0] victim_tag_out
	);

    	// ---------------------------------------------------------
    	// Parameters and Internal Signals
    	// ---------------------------------------------------------
    	//localparam TAG_BITS   = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    	localparam SETS       = 1 << INDEX_BITS;
    	localparam BLOCK_SIZE = 1 << OFFSET_BITS;

    	// Address breakdown
   	wire [OFFSET_BITS-1:0] offset = addr[OFFSET_BITS-1:0];
    	wire [INDEX_BITS-1:0]  index  = addr[OFFSET_BITS + INDEX_BITS - 1 : OFFSET_BITS];
    	wire [TAG_BITS-1:0]    tag    = addr[ADDR_WIDTH-1 : OFFSET_BITS + INDEX_BITS];

    	// ---------------------------------------------------------
    	// Memory Arrays
    	// ---------------------------------------------------------
    	reg [DATA_WIDTH-1:0] data_array  [0:WAYS-1][0:SETS-1][0:BLOCK_SIZE-1];
    	reg [TAG_BITS-1:0]   tag_array   [0:WAYS-1][0:SETS-1];
    	reg                  valid_array [0:WAYS-1][0:SETS-1];
    	reg                  dirty_array [0:WAYS-1][0:SETS-1];

    	// LRU: 2-bit rank per way
    	reg [1:0]            lru         [0:SETS-1][0:WAYS-1];

    	

    	// ---------------------------------------------------------
    	// Hit Logic (Combinational)
    	// ---------------------------------------------------------
    	reg [1:0] hit_way;
    	reg       hit;
	integer i, j, k;
    	always @(*) begin
        	hit       = 0;
        	hit_way   = 0;
        	cache_hit = 0;
        	way       = 0;
        	dirty     = 0;
		miss      = 0;
		victim_tag_out = 0;
        	if (lookup) begin 
        	    for (i = 0; i < WAYS; i = i + 1) begin
                	if (valid_array[i][index] && (tag_array[i][index] == tag)) begin
                    	hit       = 1;
                    	cache_hit = 1;
                    	hit_way   = i[1:0];
                	end
            	end

            	if (hit) begin 
                	way = hit_way;
            	end else begin
                	way   = get_lru_way(index);
                	dirty = dirty_array[way][index];
			victim_tag_out = tag_array[way][index];
			miss = 1;
            	end
        	end
    	end

    // ---------------------------------------------------------
    // LRU Management Functions/Tasks
    // ---------------------------------------------------------
    
    // Task to update LRU ranks
    	task update_lru;
        	input [1:0] used_way;
        	//integer k;
        	begin
            	for (k = 0; k < WAYS; k = k + 1) begin
                	if (k == used_way)
                    	lru[index][k] <= 0;
                	else if (lru[index][k] < 3)
                    	lru[index][k] <= lru[index][k] + 1;
            	end
        	end
    	endtask

    // Function to find the least recently used way (victim)
	function [1:0] get_lru_way;
        	input [INDEX_BITS-1:0] index; // Fixed bit-width to match index
        	//integer k;
        	begin
            		get_lru_way = 0;
            		for (k = 0; k < WAYS; k = k + 1) begin
                		if (lru[index][k] == 3)
                    		get_lru_way = k[1:0];
			end
            	end
	endfunction
	// ---------------------------------------------------------
    	// Main Sequential Logic
    	// ---------------------------------------------------------
    	always @(posedge clk or posedge rst) begin
		write_ack <= 0;
            	read_ack <= 0;

		if (rst) begin
            	read_data <= 0;
            	write_ack <= 0;
            	read_ack <= 0;

            	// FULL RESET (IMPORTANT)
            	for (i = 0; i < WAYS; i = i + 1) begin
                	for (j = 0; j < SETS; j = j + 1) begin
                    	valid_array[i][j] <= 0;
                    	dirty_array[i][j] <= 0;
                    	tag_array[i][j] <= 0;

                    	for (k = 0; k < BLOCK_SIZE; k = k + 1)
                        	data_array[i][j][k] <= 0;

                   	 lru[j][i] <= i; // initial ranking
                	end
		
            	end
		end
		else begin

        	if (read) begin
            		read_data <= data_array[input_way][index][offset];
			read_ack <= 1;
            		// Note: 'dirty' is an output reg, but is also assigned in 
            		// the combinational block above. Be careful of multiple drivers.
        		end 
        	else if (write) begin
            		data_array[input_way][index][offset] <= write_data;
            		dirty_array[input_way][index]        <= 1;
			tag_array[input_way][index] <= tag;
			valid_array[input_way][index] <=1;
			write_ack <= 1;
        		end 
        	else if (reset_dirty) begin
            		dirty_array[input_way][index]        <= 0;
        		end 
        	else if (update_meta) begin
            		update_lru(input_way);
        	end
		end
    	end

endmodule
