//========================================================================
// mem_1r1w_fifo - Memory-backed FIFO (1R1W)
//========================================================================
// Connects to mem_1r1w_if; RVHS on input/output with ECC. Input: pipes_rv
// (0 or 1 stage per FLOP_INPUT). Output: circular_fifo (READ_LATENCY+1
// entries, 1-cycle fall-through). Read issue gated by token counter.
// When BYPASS=1 and memory empty with no reads in-flight, input can bypass
// into circular_fifo to reduce fall-through latency.
//========================================================================


module mem_1r1w_fifo
  import edac_pkg::*;
  #(
    parameter int MEM_DEPTH    = 256,    // Depth of backing 1R1W memory
    parameter int W_DATA       = 32,     // Data width (bits)
    parameter int N_ECC        = 1,      // Number of ECC lanes
    parameter int FLOP_INPUT   = 0,      // 0 or 1: add flop stages to input
    parameter int READ_LATENCY = 1,      // Memory read latency (cycles); must be >= 1
    parameter int BYPASS       = 1,      // 0: disable or 1: enable empty-FIFO bypass path
    localparam int CAPACITY    = MEM_DEPTH + READ_LATENCY + 1 + FLOP_INPUT,
    localparam int W_LEVEL     = (CAPACITY > 0) ? $clog2(CAPACITY + 1) : 1
) (
    input  logic                 clk,
    input  logic                 rst_n,

    // RVHS input
    input  logic                 i_in_v,
    input  logic [W_DATA-1:0]    i_in_d,
    input  ecc_err_e [N_ECC-1:0] i_ecc_err,
    output logic                 o_in_r,

    // RVHS output
    output logic                 o_out_v,
    output logic [W_DATA-1:0]    o_out_d,
    output ecc_err_e [N_ECC-1:0] o_ecc_err,
    input  logic                 i_out_r,

    // Fill level (0 .. capacity); capacity = MEM_DEPTH + READ_LATENCY + 1 + FLOP_INPUT
    output logic [W_LEVEL-1:0]   o_level,

    // Memory interface (user side)
    mem_1r1w_if.user             if_mem
);

    //----------------------------------------------------------------------
    // Parameters
    //----------------------------------------------------------------------
    localparam int W_PTR    = (MEM_DEPTH > 1) ? $clog2(MEM_DEPTH) : 1;
    localparam int W_ECC    = $bits(ecc_err_e);
    localparam int W_PIPE   = W_DATA + N_ECC * W_ECC;
    localparam int W_RD_CNT = $clog2(READ_LATENCY + 2);

    // synthesis translate_off
    initial begin
        assert(MEM_DEPTH >= 1) else $fatal(1, "mem_1r1w_fifo: MEM_DEPTH >= 1, got %0d", MEM_DEPTH);
        assert(READ_LATENCY >= 1) else $fatal(1, "mem_1r1w_fifo: READ_LATENCY >= 1, got %0d", READ_LATENCY);
        assert(FLOP_INPUT == 0 || FLOP_INPUT == 1) else $fatal(1, "mem_1r1w_fifo: FLOP_INPUT 0 or 1, got %0d", FLOP_INPUT);
        assert(BYPASS == 0 || BYPASS == 1) else $fatal(1, "mem_1r1w_fifo: BYPASS 0 or 1, got %0d", BYPASS);
        assert(W_PTR == if_mem.W_ADDR) else $fatal(1, "mem_1r1w_fifo: W_PTR %0d != if_mem.W_ADDR %0d", W_PTR, if_mem.W_ADDR);
    end
    // synthesis translate_on

    //----------------------------------------------------------------------
    // Pointers (W_PTR+1 bits: [W_PTR] = wrap, [W_PTR-1:0] = address)
    //----------------------------------------------------------------------
    logic [W_PTR:0] wr_ptr_q, rd_ptr_q;
    logic [W_PTR:0] wr_ptr_d, rd_ptr_d;

    logic empty, full;
    assign empty = (wr_ptr_q == rd_ptr_q);
    assign full  = (wr_ptr_q[W_PTR-1:0] == rd_ptr_q[W_PTR-1:0]) && (wr_ptr_q[W_PTR] != rd_ptr_q[W_PTR]);

    function automatic logic [W_PTR:0] ptr_incr(logic [W_PTR:0] ptr);
        if (ptr[W_PTR-1:0] == W_PTR'(MEM_DEPTH - 1))
            return {~ptr[W_PTR], {(W_PTR){1'b0}}};
        else
            return ptr + 1'b1;
    endfunction

    assign wr_ptr_d = ptr_incr(wr_ptr_q);
    assign rd_ptr_d = ptr_incr(rd_ptr_q);

    //----------------------------------------------------------------------
    // Input pipe: STAGES=0 or 1 (FLOP_INPUT), payload = {data, ecc_err}
    //----------------------------------------------------------------------
    logic in_pipe_v;
    logic [W_PIPE-1:0] in_pipe_d, in_pipe_out_d;
    ecc_err_e [N_ECC-1:0] werr_unpacked;

    always_comb begin
        in_pipe_d[W_DATA-1:0] = i_in_d;
        for (int i = 0; i < N_ECC; i++)
            in_pipe_d[W_DATA + i*W_ECC +: W_ECC] = W_ECC'(i_ecc_err[i]);
    end
    assign if_mem.wdata = in_pipe_out_d[W_DATA-1:0];
    always_comb
        for (int i = 0; i < N_ECC; i++)
            werr_unpacked[i] = ecc_err_e'(in_pipe_out_d[W_DATA + i*W_ECC +: W_ECC]);
    assign if_mem.werr = werr_unpacked;

    pipes_rv #(
        .STAGES             (FLOP_INPUT),
        .W_DATA             (W_PIPE),
        .FLOP_FWD_LAST      (1),
        .FLOP_BKWD_FIRST    (0),
        .FLOP_FWD_INTERNAL  (1),
        .FLOP_BKWD_INTERNAL (0)
    ) u_in_pipe (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_in_v  (i_in_v),
        .i_in_d  (in_pipe_d),
        .o_in_r  (o_in_r),
        .o_out_v (in_pipe_v),
        .o_out_d (in_pipe_out_d),
        .i_out_r (~full)
    );

    //----------------------------------------------------------------------
    // Read token counter: +1 when if_mem.ren or bypass_push, -1 when o_out_v & i_out_r.
    // Issue when ~empty and ~out_fifo_full. rd_inflight tracks when rdata will be
    // valid (READ_LATENCY cycles after ren).
    //----------------------------------------------------------------------
    logic [W_RD_CNT-1:0]     rd_count_q, rd_count_d;
    logic [READ_LATENCY-1:0] rd_inflight_q, rd_inflight_d;
    logic                    do_consume, do_bypass, out_fifo_full;

    assign out_fifo_full = (rd_count_q >= (READ_LATENCY + 1));
    assign do_consume    = o_out_v && i_out_r;
    assign if_mem.ren    = ~empty && (~out_fifo_full || do_consume);
    assign if_mem.raddr  = rd_ptr_q[W_PTR-1:0];
    assign rd_count_d    = rd_count_q + if_mem.ren + do_bypass - do_consume;
    assign rd_inflight_d = (rd_inflight_q << 1) | if_mem.ren;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_count_q    <= '0;
            rd_ptr_q      <= '0;
            rd_inflight_q <= '0;
        end else begin
            rd_count_q    <= rd_count_d;
            rd_ptr_q      <= if_mem.ren ? rd_ptr_d : rd_ptr_q;
            rd_inflight_q <= rd_inflight_d;
        end
    end

    //----------------------------------------------------------------------
    // Write: consume from input pipe when memory not full
    // Bypass: when empty, no reads in-flight, and output buffer has room, cancel write and push
    // input-pipe data straight to output FIFO instead of memory (no wen, no wr_ptr advance).
    //----------------------------------------------------------------------
    assign do_bypass = (BYPASS != 0) && empty && (~|rd_inflight_q) && ~out_fifo_full && in_pipe_v;

    always_ff @(posedge clk)
        if (rst_n && do_bypass)
            $display("%0t mem_1r1w_fifo: bypass used", $time);

    assign if_mem.wen   = in_pipe_v && ~full && !do_bypass;
    assign if_mem.waddr = wr_ptr_q[W_PTR-1:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_ptr_q <= '0;
        else if (if_mem.wen)
            wr_ptr_q <= wr_ptr_d;
    end

    //----------------------------------------------------------------------
    // Output buffer: circular_fifo (READ_LATENCY+1 entries, 1-cycle fall-through).
    // Needed to absord inflight reads if output stalls.
    // Push when rdata valid (READ_LATENCY cycles after ren), or on bypass.
    //----------------------------------------------------------------------
    logic fifo_in_r, fifo_in_v;
    logic [W_PIPE-1:0] in_fifo_d, out_fifo_d;
    ecc_err_e [N_ECC-1:0] o_ecc_err_unpacked;

    assign fifo_in_v = rd_inflight_q[READ_LATENCY-1] || do_bypass;
    always_comb begin
        if (do_bypass) begin
            in_fifo_d = in_pipe_out_d;
        end else begin
            in_fifo_d[W_DATA-1:0] = if_mem.rdata;
            for (int i = 0; i < N_ECC; i++)
                in_fifo_d[W_DATA + i*W_ECC +: W_ECC] = W_ECC'(if_mem.rerr[i]);
        end
    end
    assign o_out_d = out_fifo_d[W_DATA-1:0];
    always_comb
        for (int i = 0; i < N_ECC; i++)
            o_ecc_err_unpacked[i] = ecc_err_e'(out_fifo_d[W_DATA + i*W_ECC +: W_ECC]);
    assign o_ecc_err = o_ecc_err_unpacked;

    circular_fifo #(
        .DEPTH  (READ_LATENCY + 1),
        .W_DATA (W_PIPE)
    ) u_out_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_in_v  (fifo_in_v),
        .i_in_d  (in_fifo_d),
        .o_in_r  (fifo_in_r),
        .o_out_v (o_out_v),
        .o_out_d (out_fifo_d),
        .i_out_r (i_out_r),
        .o_level ()
    );

    //----------------------------------------------------------------------
    // Fill level: +1 on push (i_in_v & o_in_r), -1 on pop (o_out_v & i_out_r)
    //----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            o_level <= '0;
        else begin
            if (i_in_v && o_in_r && !(o_out_v && i_out_r))
                o_level <= o_level + 1'b1;
            else if (o_out_v && i_out_r && !(i_in_v && o_in_r))
                o_level <= o_level - 1'b1;
            else
                o_level <= o_level;
        end
    end

    //----------------------------------------------------------------------
    // Assertions (disable when not in active run: rst_n X or 0)
    //----------------------------------------------------------------------
    a_bypass_no_wen      : assert property (@(posedge clk) disable iff (rst_n !== 1'b1)
    (do_bypass) |-> !if_mem.wen)
        else $error("mem_1r1w_fifo: bypass active but if_mem.wen asserted");
    a_rd_count_bounded   : assert property (@(posedge clk) disable iff (rst_n !== 1'b1)
    (rd_count_q <= READ_LATENCY + 1) && (rd_count_q >= 0))
        else $error("mem_1r1w_fifo: rd_count_q > READ_LATENCY+1");
    a_out_fifo_never_drop: assert property (@(posedge clk) disable iff (rst_n !== 1'b1)
    !(fifo_in_v && !fifo_in_r))
        else $error("mem_1r1w_fifo: output FIFO full on push");
    a_level_incr_ok      : assert property (@(posedge clk) disable iff (rst_n !== 1'b1)
    (i_in_v && o_in_r && !(o_out_v && i_out_r)) |-> (o_level < CAPACITY))
        else $error("mem_1r1w_fifo: increment when o_level >= CAPACITY");
    a_level_decr_ok      : assert property (@(posedge clk) disable iff (rst_n !== 1'b1)
    (o_out_v && i_out_r && !(i_in_v && o_in_r)) |-> (o_level > 0))
        else $error("mem_1r1w_fifo: decrement when o_level == 0");

endmodule
