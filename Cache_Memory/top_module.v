module cache_system_top #(
    parameter ADDR_WIDTH  = 16,
    parameter DATA_WIDTH  = 8,
    parameter INDEX_BITS  = 6,
    parameter OFFSET_BITS = 2
)(
    input clk,
    input rst,

    // CPU Interface
    input cpu_read,
    input cpu_write,
    input [ADDR_WIDTH-1:0] cpu_addr,
    input [DATA_WIDTH-1:0] cpu_wdata,

    output [DATA_WIDTH-1:0] cpu_rdata,
    output cpu_ready
);

    // ============================
    // CACHE ↔ CONTROLLER SIGNALS
    // ============================
    wire cm_read, cm_write, cm_lookup;
    wire cm_update_meta, cm_reset_dirty;
    wire [ADDR_WIDTH-1:0] cm_addr;
    wire [DATA_WIDTH-1:0] cm_wdata;
    wire [1:0] cm_input_way;

    wire [DATA_WIDTH-1:0] cm_rdata;
    wire cm_hit, cm_dirty, cm_write_ack, cm_read_ack;
    wire [1:0] cm_way;
    wire cm_miss;
    wire [ADDR_WIDTH-OFFSET_BITS-INDEX_BITS-1:0] cm_victim_tag;

    // ============================
    // MEMORY ↔ CONTROLLER SIGNALS
    // ============================
    wire mm_read, mm_write;
    wire [ADDR_WIDTH-1:0] mm_addr;
    wire [DATA_WIDTH-1:0] mm_wdata;
    wire [DATA_WIDTH-1:0] mm_rdata;
    wire mm_ready;

    // ============================
    // CACHE MEMORY INSTANCE
    // ============================
    cache_memory cache_inst (
        .clk(clk),
        .rst(rst),

        .addr(cm_addr),
        .read(cm_read),
        .write(cm_write),
        .write_data(cm_wdata),

        .update_meta(cm_update_meta),
        .input_way(cm_input_way),
        .lookup(cm_lookup),
        .reset_dirty(cm_reset_dirty),

        .read_data(cm_rdata),
        .cache_hit(cm_hit),
        .dirty(cm_dirty),
        .write_ack(cm_write_ack),
        .read_ack(cm_read_ack),
        .way(cm_way),
        .miss(cm_miss),
        .victim_tag_out(cm_victim_tag)
    );

    // ============================
    // MAIN MEMORY INSTANCE
    // ============================
    main_memory mem_inst (
        .clk(clk),
        .reset(rst),

        .read(mm_read),
        .write(mm_write),
        .addr(mm_addr),
        .write_data(mm_wdata),

        .read_data(mm_rdata),
        .ready(mm_ready)
    );

    // ============================
    // CACHE CONTROLLER INSTANCE
    // ============================
    cache_controller ctrl_inst (
        .clk(clk),
        .rst(rst),

        // CPU
        .cpu_read(cpu_read),
        .cpu_write(cpu_write),
        .cpu_addr(cpu_addr),
        .cpu_wdata(cpu_wdata),
        .cpu_rdata(cpu_rdata),
        .cpu_ready(cpu_ready),

        // CACHE
        .cm_read(cm_read),
        .cm_write(cm_write),
        .cm_lookup(cm_lookup),
        .cm_update_meta(cm_update_meta),
        .cm_reset_dirty(cm_reset_dirty),
        .cm_addr(cm_addr),
        .cm_wdata(cm_wdata),
        .cm_input_way(cm_input_way),

        .cm_rdata(cm_rdata),
        .cm_hit(cm_hit),
        .cm_dirty(cm_dirty),
        .cm_write_ack(cm_write_ack),
        .cm_read_ack(cm_read_ack),
        .cm_way(cm_way),
        .cm_miss(cm_miss),
        .cm_victim_tag(cm_victim_tag),

        // MEMORY
        .mm_read(mm_read),
        .mm_write(mm_write),
        .mm_addr(mm_addr),
        .mm_wdata(mm_wdata),
        .mm_rdata(mm_rdata),
        .mm_ready(mm_ready)
    );

endmodule
