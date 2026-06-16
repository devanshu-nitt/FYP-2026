// =============================================================================
// tb_mesi_coherence.v
// Testbench for the 2-core MESI directory-based coherence system.
//
// Tests:
//   1. Shared Read       – both cores read same address → both land in S
//   2. Read-After-Write  – Core0 writes; Core1 reads same address
//   3. Write-Write       – Core0 writes; Core1 writes; Core0 reads (sees C1's val)
//   4. Silent E→M        – Core0 reads (→E); Core0 writes (→M silent); Core1 reads
//
// Clock: 10 ns period.  Timeout: 10 000 ns.
// =============================================================================
`timescale 1ns/1ps

module tb_mesi_coherence;

localparam NUM_CORES  = 2;
localparam DATA_WIDTH = 32;
localparam ADDR_WIDTH = 8;
localparam ID_BITS    = $clog2(NUM_CORES+1);  // = 2 for NUM_CORES=2

// ── DUT signals ───────────────────────────────────────────────────────────────
reg                             clk, rst_n;
reg  [NUM_CORES-1:0]            cpu_req, cpu_wr;
reg  [NUM_CORES*ADDR_WIDTH-1:0] cpu_addr;
reg  [NUM_CORES*DATA_WIDTH-1:0] cpu_wr_data;
wire [NUM_CORES*DATA_WIDTH-1:0] cpu_rd_data;
wire [NUM_CORES-1:0]            cpu_ack;

// ── DUT ───────────────────────────────────────────────────────────────────────
top_module #(
    .NUM_CORES (NUM_CORES),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) dut (
    .clk(clk), .rst_n(rst_n),
    .cpu_req(cpu_req), .cpu_wr(cpu_wr),
    .cpu_addr(cpu_addr), .cpu_wr_data(cpu_wr_data),
    .cpu_rd_data(cpu_rd_data), .cpu_ack(cpu_ack)
);

// ── Clock ─────────────────────────────────────────────────────────────────────
initial clk = 0;
always  #5 clk = ~clk;

// ── Timeout ───────────────────────────────────────────────────────────────────
initial begin
    #10000;
    $display("[TB] TIMEOUT – simulation exceeded 10 000 ns.");
    $finish;
end

// ── VCD waveform dump ────────────────────────────────────────────────────────
initial begin
    $dumpfile("mesi_waves.vcd");
    $dumpvars(0, tb_mesi_coherence);
end

// ── Helper tasks ──────────────────────────────────────────────────────────────

// issue_read: drive a read on core `cid`, wait for ack, return data
task issue_read;
    input  integer          cid;
    input  [ADDR_WIDTH-1:0] addr;
    output [DATA_WIDTH-1:0] rdata;
    integer t;
begin
    @(negedge clk);
    cpu_req[cid]                              = 1'b1;
    cpu_wr [cid]                              = 1'b0;
    cpu_addr    [cid*ADDR_WIDTH +: ADDR_WIDTH] = addr;
    cpu_wr_data [cid*DATA_WIDTH +: DATA_WIDTH] = {DATA_WIDTH{1'b0}};
    // Hold until ack
    t = 0;
    @(posedge clk); #1;
    while (!cpu_ack[cid] && t < 200) begin
        @(posedge clk); #1;
        t = t + 1;
    end
    rdata = cpu_rd_data[cid*DATA_WIDTH +: DATA_WIDTH];
    @(negedge clk);
    cpu_req[cid] = 1'b0;
    @(posedge clk); #1;  // let ack de-assert
end
endtask

// issue_write: drive a write on core `cid`, wait for ack
task issue_write;
    input  integer          cid;
    input  [ADDR_WIDTH-1:0] addr;
    input  [DATA_WIDTH-1:0] wdata;
    integer t;
begin
    @(negedge clk);
    cpu_req[cid]                              = 1'b1;
    cpu_wr [cid]                              = 1'b1;
    cpu_addr    [cid*ADDR_WIDTH +: ADDR_WIDTH] = addr;
    cpu_wr_data [cid*DATA_WIDTH +: DATA_WIDTH] = wdata;
    t = 0;
    @(posedge clk); #1;
    while (!cpu_ack[cid] && t < 200) begin
        @(posedge clk); #1;
        t = t + 1;
    end
    @(negedge clk);
    cpu_req[cid] = 1'b0;
    cpu_wr [cid] = 1'b0;
    @(posedge clk); #1;
end
endtask

// ── Test variables ────────────────────────────────────────────────────────────
reg [DATA_WIDTH-1:0] rd0, rd1;
integer              pass_cnt, fail_cnt;

// ── Stimulus ──────────────────────────────────────────────────────────────────
initial begin
    $display("================================================");
    $display("  MESI Directory Coherence TB  (%0d cores)", NUM_CORES);
    $display("================================================");
    pass_cnt = 0;
    fail_cnt = 0;

    rst_n      = 1'b0;
    cpu_req    = {NUM_CORES{1'b0}};
    cpu_wr     = {NUM_CORES{1'b0}};
    cpu_addr   = {NUM_CORES*ADDR_WIDTH{1'b0}};
    cpu_wr_data= {NUM_CORES*DATA_WIDTH{1'b0}};
    repeat(6) @(posedge clk);
    rst_n = 1'b1;
    repeat(4) @(posedge clk);

    // =========================================================================
    // TEST 1 – Shared Read
    //   Both cores read address 0x10.
    //   Core0 gets DATA_EXCL (→ E). Core1's request hits the directory while
    //   Core0 is in E; directory responds with DATA (→ S for both).
    //   Both cores must read the same value.
    // =========================================================================
    $display("\n[TEST 1] Shared Read (addr=0x10)");
    issue_read(0, 8'h10, rd0);
    $display("  Core0 read → 0x%08X", rd0);
    issue_read(1, 8'h10, rd1);
    $display("  Core1 read → 0x%08X", rd1);

    if (rd0 === rd1) begin
        $display("  PASS: both cores read coherent value 0x%08X", rd0);
        pass_cnt = pass_cnt + 1;
    end else begin
        $display("  FAIL: rd0=0x%08X  rd1=0x%08X", rd0, rd1);
        fail_cnt = fail_cnt + 1;
    end
    repeat(4) @(posedge clk);

    // =========================================================================
    // TEST 2 – Read After Write
    //   Core0 writes 0xDEADBEEF to 0x20.
    //   Core1 reads 0x20 → must observe Core0's write.
    // =========================================================================
    $display("\n[TEST 2] Read-After-Write (addr=0x20)");
    issue_write(0, 8'h20, 32'hDEAD_BEEF);
    $display("  Core0 wrote 0xDEADBEEF");
    issue_read(1, 8'h20, rd1);
    $display("  Core1 read  → 0x%08X", rd1);

    if (rd1 === 32'hDEAD_BEEF) begin
        $display("  PASS: Core1 sees Core0's write.");
        pass_cnt = pass_cnt + 1;
    end else begin
        $display("  FAIL: expected 0xDEADBEEF, got 0x%08X", rd1);
        fail_cnt = fail_cnt + 1;
    end
    repeat(4) @(posedge clk);

    // =========================================================================
    // TEST 3 – Write-Write Conflict
    //   Core0 writes 0xAAAAAAAA to 0x30.
    //   Core1 writes 0xBBBBBBBB to 0x30 (takes ownership away from Core0).
    //   Core0 reads 0x30 → must see Core1's later write.
    // =========================================================================
    $display("\n[TEST 3] Write-Write Conflict (addr=0x30)");
    issue_write(0, 8'h30, 32'hAAAA_AAAA);
    $display("  Core0 wrote 0xAAAAAAAA");
    issue_write(1, 8'h30, 32'hBBBB_BBBB);
    $display("  Core1 wrote 0xBBBBBBBB");
    issue_read(0, 8'h30, rd0);
    $display("  Core0 read  → 0x%08X", rd0);

    if (rd0 === 32'hBBBB_BBBB) begin
        $display("  PASS: Core0 sees Core1's latest write.");
        pass_cnt = pass_cnt + 1;
    end else begin
        $display("  FAIL: expected 0xBBBBBBBB, got 0x%08X", rd0);
        fail_cnt = fail_cnt + 1;
    end
    repeat(4) @(posedge clk);

    // =========================================================================
    // TEST 4 – Silent E→M Upgrade
    //   Core0 reads 0x40 (gets DATA_EXCL → E, no other sharers).
    //   Core0 writes 0x40 → silent E→M (no network message).
    //   Core1 reads 0x40 → must see Core0's written value 0xCAFEBABE.
    // =========================================================================
    $display("\n[TEST 4] Silent E->M Upgrade (addr=0x40)");
    issue_read(0, 8'h40, rd0);
    $display("  Core0 read  → 0x%08X  (expects E state)", rd0);
    issue_write(0, 8'h40, 32'hCAFE_BABE);
    $display("  Core0 wrote 0xCAFEBABE  (silent E->M)");
    issue_read(1, 8'h40, rd1);
    $display("  Core1 read  → 0x%08X", rd1);

    if (rd1 === 32'hCAFE_BABE) begin
        $display("  PASS: Core1 sees Core0's silently-upgraded write.");
        pass_cnt = pass_cnt + 1;
    end else begin
        $display("  FAIL: expected 0xCAFEBABE, got 0x%08X", rd1);
        fail_cnt = fail_cnt + 1;
    end
    repeat(4) @(posedge clk);

    // =========================================================================
    // SUMMARY
    // =========================================================================
    $display("\n================================================");
    $display("  Results:  %0d PASS   %0d FAIL", pass_cnt, fail_cnt);
    $display("================================================");
    if (fail_cnt == 0)
        $display("  *** ALL TESTS PASSED ***\n");
    else
        $display("  *** FAILURES DETECTED – check waveforms ***\n");

    $finish;
end

endmodule
