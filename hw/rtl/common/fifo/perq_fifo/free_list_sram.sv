//==============================================================================
// free_list_sram.sv -- RAM-backed free pointer list (EN-2695)
//==============================================================================
// Drop-in replacement for free_list.sv (same I/O minus N_PARTITIONS).
// Backed by a 1R1W RAM via mem_1r1w_fifo; the RAM stores free pointer values
// (N-deep x log2(N)-wide) rather than a used/free bitmap.
//
// Pseudo-initialization: the first N_PTRS allocs are served by an incrementing
// counter (0..N_PTRS-1) -- no RAM writes required at reset.  Freed pointers
// push into the FIFO during this phase.  Once the counter wraps (all N initial
// entries issued), NORMAL mode begins and the FIFO drives all subsequent allocs.
//
// Full-rate: simultaneous alloc + free every cycle is supported in both phases.
//   INIT:   alloc = counter read (combinational); free = FIFO push
//   NORMAL: alloc = FIFO pop;                     free = FIFO push
//==============================================================================

module free_list_sram
  import edac_pkg::*;
#(
    parameter int N_PTRS = 1024
) (
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         i_alloc_r,   // consumer taking o_alloc_addr
    input  logic                         i_free_v,    // return address valid
    input  logic [$clog2(N_PTRS)-1:0]    i_free_addr, // returned pointer
    output logic                         o_alloc_v,   // alloc address is valid
    output logic [$clog2(N_PTRS)-1:0]    o_alloc_addr, // free pointer
    mem_1r1w_if.user                     if_mem       // DEPTH=N_PTRS, W_DATA=$clog2(N_PTRS)
);

    localparam int W_ADDR = $clog2(N_PTRS);

    initial begin
        assert(N_PTRS >= 2)
            else $fatal(1, "free_list_sram: N_PTRS must be >= 2, got %0d", N_PTRS);
    end

    //--------------------------------------------------------------------------
    // Init-phase counter: counts 0..N_PTRS-1.
    // init_done_q goes high after the last init entry is consumed (i_alloc_r
    // while init_ctr_q == N_PTRS-1).
    //--------------------------------------------------------------------------
    logic              init_done_q;
    logic [W_ADDR-1:0] init_ctr_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_done_q <= 1'b0;
            init_ctr_q  <= '0;
        end else if (!init_done_q && i_alloc_r) begin
            if (init_ctr_q == W_ADDR'(N_PTRS - 1))
                init_done_q <= 1'b1;
            init_ctr_q <= init_ctr_q + 1'b1;
        end
    end

    //--------------------------------------------------------------------------
    // FIFO: freed pointers always push here.
    // In NORMAL mode, alloc pops here (i_out_r gated by init_done_q).
    // In INIT mode, the FIFO queues freed entries but does not drain.
    //--------------------------------------------------------------------------
    logic              fifo_out_v;
    logic [W_ADDR-1:0] fifo_out_d;
    logic              fifo_in_r;
    ecc_err_e [0:0]    unused_ecc_err;

    mem_1r1w_fifo #(
        .MEM_DEPTH    (N_PTRS),
        .W_DATA       (W_ADDR),
        .N_ECC        (1),
        .FLOP_INPUT   (0),
        .READ_LATENCY (1),
        .BYPASS       (1)
    ) u_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .i_in_v   (i_free_v),
        .i_in_d   (i_free_addr),
        .i_ecc_err('0),
        .o_in_r   (fifo_in_r),
        .o_out_v  (fifo_out_v),
        .o_out_d  (fifo_out_d),
        .o_ecc_err(unused_ecc_err),
        .i_out_r  (init_done_q && i_alloc_r),
        .o_level  (),
        .if_mem   (if_mem)
    );

    //--------------------------------------------------------------------------
    // Output mux: counter during INIT, FIFO during NORMAL.
    //--------------------------------------------------------------------------
    assign o_alloc_v    = init_done_q ? fifo_out_v   : 1'b1;
    assign o_alloc_addr = init_done_q ? fifo_out_d   : init_ctr_q;

    //--------------------------------------------------------------------------
    // Assertions
    //--------------------------------------------------------------------------
`ifndef ASSERTS_OFF
    // The FIFO has N_PTRS capacity and at most N_PTRS pointers exist, so it
    // must never backpressure a free.
    a_free_no_backpressure : assert property (
        @(posedge clk) disable iff (!rst_n)
        i_free_v |-> fifo_in_r
    ) else $error("free_list_sram: FIFO full on free -- capacity exceeded");
`endif

endmodule : free_list_sram
