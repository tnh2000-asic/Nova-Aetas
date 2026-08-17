//==============================================================================
// perq_fifo_sram.sv - Per-Queue FIFO with SRAM-backed qstate
//==============================================================================
// Drop-in replacement for perq_fifo intended for large N_QUEUE (e.g. 4096).
// Uses qstate_sram instead of qstate to keep per-queue state in SRAM, with
// an extra cycle of read-response latency (o_rd_resp_v at i_rd_req_v + 2
// cycles instead of +1).
//
// Differences from perq_fifo:
//   - Two additional mem_1r1w_if.user ports for the mirrored qstate SRAMs
//     (push-side and pop-side); the integrator instantiates the actual
//     storage (e.g. mem_1r1w in sim, hard-macro in syn).
//   - The free list is RAM-backed (free_list_sram, EN-2695) instead of the
//     flop bitmap: one more mem_1r1w_if.user port (if_mem_fl,
//     MEM_DEPTH x $clog2(MEM_DEPTH)).
//   - Read-response latency is +1 cycle vs perq_fifo.
//   - On rare same-cycle qstate write-port conflicts (push S1 + LL response
//     to different qids), the push is stalled by deasserting o_wr_req_r.
//==============================================================================

module perq_fifo_sram
#(
    parameter int N_QUEUE        = 64,
    parameter int MEM_DEPTH      = 1024,
    parameter int W_DATA         = 1024
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // Write interface
    output logic                       o_wr_req_r,
    input  logic                       i_wr_req_v,
    input  logic [$clog2(N_QUEUE)-1:0] i_wr_req_qnum,
    input  logic [W_DATA-1:0]          i_wr_req_data,

    // Read interface (response is +2 cycles vs +1 for perq_fifo)
    input  logic                       i_rd_req_v,
    input  logic [$clog2(N_QUEUE)-1:0] i_rd_req_qnum,
    input  logic                       i_rd_req_peek,
    output logic                       o_rd_resp_v,
    output logic [W_DATA-1:0]          o_rd_resp_data,

    // Queue status
    output logic [N_QUEUE-1:0]         o_status_qne,
    output logic                       o_rd_has_more,      // Read's queue still non-empty after the op (one cycle after the request)

    // Memory interfaces
    mem_1r1w_if.user                   if_mem_data,        // Main data memory
    mem_1r1w_if.user                   if_mem_ll,          // Linked-list memory
    mem_1r1w_if.user                   if_mem_qstate_p,    // Qstate SRAM (push-side read)
    mem_1r1w_if.user                   if_mem_qstate_q,    // Qstate SRAM (pop-side read)
    mem_1r1w_if.user                   if_mem_fl           // Free-list pointer RAM (MEM_DEPTH x $clog2(MEM_DEPTH))
);

    localparam int W_PTR = $clog2(MEM_DEPTH);

    //--------------------------------------------------------------------------
    // Free list
    //--------------------------------------------------------------------------
    logic             fl_alloc_v;
    logic [W_PTR-1:0] fl_alloc_addr;
    logic             fl_free_v;
    logic [W_PTR-1:0] fl_free_addr;
    logic             fl_alloc_r;        // S0 ack to free_list

    //--------------------------------------------------------------------------
    // Qstate SRAM controller signals
    //--------------------------------------------------------------------------
    logic             qs_wr_stall;
    // S1 control to data/LL memory interfaces
    logic             mem_ll_wen;
    logic             mem_ll_ren;
    logic [W_PTR-1:0] mem_ll_waddr;
    logic [W_PTR-1:0] mem_ll_raddr;
    logic [W_PTR-1:0] mem_ll_rdata;
    logic             mem_data_wen;
    logic             mem_data_ren;
    logic [W_PTR-1:0] mem_data_raddr;
    logic [W_PTR-1:0] mem_data_waddr;

    //--------------------------------------------------------------------------
    // S0 push transaction
    //--------------------------------------------------------------------------
    logic wr_xact;
    assign o_wr_req_r = fl_alloc_v && !qs_wr_stall;
    assign wr_xact    = i_wr_req_v && o_wr_req_r;
    assign fl_alloc_r = wr_xact;

    //--------------------------------------------------------------------------
    // Pipeline write data S0 → S1 (used as data memory wdata at S1)
    //--------------------------------------------------------------------------
    logic [W_DATA-1:0] wr_data_s1_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) wr_data_s1_q <= '0;
        else if (wr_xact) wr_data_s1_q <= i_wr_req_data;
    end

    //--------------------------------------------------------------------------
    // QSTATE_SRAM
    //--------------------------------------------------------------------------
    qstate_sram #(
        .N_QUEUE(N_QUEUE),
        .W_PTR  (W_PTR)
    ) qstate_sram_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        // Push (S0)
        .i_wr_req_v     (wr_xact),
        .i_wr_req_qnum  (i_wr_req_qnum),
        .o_wr_req_stall (qs_wr_stall),
        // Pop (S0)
        .i_rd_req_v     (i_rd_req_v),
        .i_rd_req_qnum  (i_rd_req_qnum),
        .i_rd_req_peek  (i_rd_req_peek),
        // Free-list pointer in (S0)
        .i_fl_alloc_addr(fl_alloc_addr),
        // Memory interfaces (S1 control)
        .o_mem_ll_wen   (mem_ll_wen),
        .o_mem_ll_ren   (mem_ll_ren),
        .o_mem_ll_waddr (mem_ll_waddr),
        .o_mem_ll_raddr (mem_ll_raddr),
        .i_mem_ll_rdata (mem_ll_rdata),
        .o_mem_data_wen (mem_data_wen),
        .o_mem_data_ren (mem_data_ren),
        .o_mem_data_raddr(mem_data_raddr),
        .o_mem_data_waddr(mem_data_waddr),
        // S1 metadata
        .o_wr_v_s1      (),
        .o_rd_v_s1      (),
        .o_fl_free_v    (fl_free_v),
        .o_fl_free_addr (fl_free_addr),
        // Status
        .o_status_qne   (o_status_qne),
        .o_rd_has_more  (o_rd_has_more),
        // SRAM ports
        .if_mem_qstate_p(if_mem_qstate_p),
        .if_mem_qstate_q(if_mem_qstate_q)
    );

    //--------------------------------------------------------------------------
    // FREE LIST
    //--------------------------------------------------------------------------
    free_list_sram #(
        .N_PTRS(MEM_DEPTH)
    ) free_list_inst (
        .clk          (clk),
        .rst_n        (rst_n),
        .i_free_addr  (fl_free_addr),
        .i_free_v     (fl_free_v),
        .i_alloc_r    (fl_alloc_r),
        .o_alloc_v    (fl_alloc_v),
        .o_alloc_addr (fl_alloc_addr),
        .if_mem       (if_mem_fl)
    );

    //--------------------------------------------------------------------------
    // LINK LIST MEMORY INTERFACE (S1 control)
    //--------------------------------------------------------------------------
    // LL wdata is the just-allocated pointer registered through to S1; the
    // free list output is registered, so the same alloc_addr is presented at
    // S1 cycle for the push that consumed it at S0... but free_list advances
    // immediately on i_alloc_r=1, so we must register the consumed address
    // ourselves.
    logic [W_PTR-1:0] fl_alloc_addr_s1_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) fl_alloc_addr_s1_q <= '0;
        else if (wr_xact) fl_alloc_addr_s1_q <= fl_alloc_addr;
    end

    assign if_mem_ll.wen   = mem_ll_wen;
    assign if_mem_ll.ren   = mem_ll_ren;
    assign if_mem_ll.waddr = mem_ll_waddr;
    assign if_mem_ll.raddr = mem_ll_raddr;
    assign if_mem_ll.wdata = fl_alloc_addr_s1_q;
    assign if_mem_ll.werr  = '0;
    assign mem_ll_rdata    = if_mem_ll.rdata;

    //--------------------------------------------------------------------------
    // DATA MEMORY INTERFACE (S1 control)
    //
    // The data SRAM can see same-cycle ren+wen to the same address when a
    // pop's bypass head (off a prior pop's LL response) lands on the
    // just-allocated push pointer in the very cycle push S1 writes that
    // pointer. mem_1r1w is read-before-write, so wrap with a hazard guard
    // (WBR=1) to bypass the new write data to the read response.
    //--------------------------------------------------------------------------
    mem_1r1w_if #(.DEPTH(MEM_DEPTH), .W_DATA(W_DATA)) data_ctrl ();

    assign data_ctrl.wen   = mem_data_wen;
    assign data_ctrl.ren   = mem_data_ren;
    assign data_ctrl.waddr = mem_data_waddr;
    assign data_ctrl.raddr = mem_data_raddr;
    assign data_ctrl.wdata = wr_data_s1_q;
    assign data_ctrl.werr  = '0;
    assign o_rd_resp_data  = data_ctrl.rdata;

    mem_1r1w_hazard_guard #(.READ_LATENCY(1), .WBR(1'b1)) u_hg_data (
        .clk    (clk),
        .rst_n  (rst_n),
        .if_ctrl(data_ctrl),
        .if_mem (if_mem_data)
    );

    //--------------------------------------------------------------------------
    // FLOW CONTROL
    //--------------------------------------------------------------------------
    // Read response valid: data memory ren (S1) → response one cycle later (S2)
    pipes #(.STAGES(1), .W_DATA(1)) u_pipes_rd_resp_v (
        .clk    (clk),
        .rst_n  (rst_n),
        .i_data (mem_data_ren),
        .o_data (o_rd_resp_v)
    );

    //==========================================================================
    // ASSERTIONS - Protocol Checks
    //==========================================================================
`ifndef ASSERTS_OFF

    // Read requests must target non-empty queues
    a_rd_from_nonempty: assert property (
        @(posedge clk) disable iff (!rst_n)
        i_rd_req_v |-> o_status_qne[i_rd_req_qnum]
    ) else $error("perq_fifo_sram: read request to empty queue %0d", i_rd_req_qnum);

    // Read response valid must follow a read request (2-cycle pipeline now)
    a_rd_resp_follows_req: assert property (
        @(posedge clk) disable iff (!rst_n)
        o_rd_resp_v |-> $past(i_rd_req_v, 2)
    ) else $error("perq_fifo_sram: read response without preceding read request");

    //==========================================================================
    // ASSERTIONS - Data Integrity (Model-Based, Simulation Only)
    //==========================================================================
    // synthesis translate_off

    logic [W_DATA-1:0] expected_data[N_QUEUE][$];
    logic [$clog2(N_QUEUE)-1:0] rd_qnum_s1_q;
    logic [$clog2(N_QUEUE)-1:0] rd_qnum_s2_q;
    logic                        rd_peek_s1_q;
    logic                        rd_peek_s2_q;

    // Pop request side (S0 -> S1 -> S2 = response cycle)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_qnum_s1_q <= '0;
            rd_qnum_s2_q <= '0;
            rd_peek_s1_q <= 1'b0;
            rd_peek_s2_q <= 1'b0;
        end else begin
            rd_qnum_s1_q <= i_rd_req_qnum;
            rd_qnum_s2_q <= rd_qnum_s1_q;
            rd_peek_s1_q <= i_rd_req_peek;
            rd_peek_s2_q <= rd_peek_s1_q;
        end
    end

    // Track writes
    always @(posedge clk) begin
        if (rst_n && wr_xact) begin
            expected_data[i_wr_req_qnum].push_back(i_wr_req_data);
        end
    end

    // Verify reads
    always @(posedge clk) begin
        if (rst_n && o_rd_resp_v) begin
            automatic logic [W_DATA-1:0] exp_data;

            a_model_not_empty: assert (expected_data[rd_qnum_s2_q].size() > 0)
                else $error("perq_fifo_sram: read response for queue %0d but model is empty",
                            rd_qnum_s2_q);

            if (expected_data[rd_qnum_s2_q].size() > 0) begin
                if (rd_peek_s2_q) begin
                    exp_data = expected_data[rd_qnum_s2_q][0];
                end else begin
                    exp_data = expected_data[rd_qnum_s2_q].pop_front();
                end

                a_data_integrity: assert (o_rd_resp_data == exp_data)
                    else $error("perq_fifo_sram: data mismatch on queue %0d: expected 0x%h, got 0x%h",
                                rd_qnum_s2_q, exp_data, o_rd_resp_data);
            end
        end
    end
    // synthesis translate_on

`endif // ASSERTS_OFF

endmodule : perq_fifo_sram
