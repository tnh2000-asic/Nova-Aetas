//========================================================================
// Per-Queue FIFO Module
//========================================================================
// This module implements a per-queue FIFO where the memory is shared between
// N_QUEUE independent queues. Each queue can be pushed/popped independently,
// but all queues share the same physical memory space.
//========================================================================

module perq_fifo
#(
    parameter int N_QUEUE        = 64,    // Number of packet buffer queues
    parameter int MEM_DEPTH      = 1024,  // Memory depth
    parameter int NUM_PARTITIONS = 32,    // Number of partitions for free list (FL_SRAM=0 only)
    parameter int W_DATA         = 1024,  // Data width in bits
    parameter bit FL_SRAM        = 0     // 1: RAM-backed free list (free_list_sram); the
                                          // flop bitmap's find-first-zero does not scale
                                          // to large MEM_DEPTH (timing)
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // Write interface
    output logic                       o_wr_req_r,        // Space available (stall when deasserted)
    input  logic                       i_wr_req_v,        // Valid write request
    input  logic [$clog2(N_QUEUE)-1:0] i_wr_req_qnum,     // Queue number to write to
    input  logic [W_DATA-1:0]          i_wr_req_data,     // Data to write

    // Read interface
    input  logic                       i_rd_req_v,        // Valid read request
    input  logic [$clog2(N_QUEUE)-1:0] i_rd_req_qnum,     // Queue number to read from
    input  logic                       i_rd_req_peek,     // Peek mode (0: pop, 1: peek)
    output logic                       o_rd_resp_v,       // Valid read response
    output logic [W_DATA-1:0]          o_rd_resp_data,    // Data that was read

    // Queue status interface
    output logic [N_QUEUE-1:0]         o_status_qne,      // Queue not empty
    output logic                       o_rd_has_more,     // Read's queue still non-empty after the op (one cycle after the request)

    // Memory interfaces
    mem_1r1w_if.user                   if_mem_data,       // Main data memory interface
    mem_1r1w_if.user                   if_mem_ll,         // Link list memory interface
    mem_1r1w_if.user                   if_mem_fl          // Free-list pointer RAM, MEM_DEPTH x
                                                          // $clog2(MEM_DEPTH) (FL_SRAM=1; tied
                                                          // off otherwise)
);

    localparam int W_PTR = $clog2(MEM_DEPTH);

    // Free list signals
    logic             fl_alloc_v;
    logic [W_PTR-1:0] fl_alloc_addr;
    // Linked list memory signals
    logic             mem_ll_wen;
    logic             mem_ll_ren;
    logic [W_PTR-1:0] mem_ll_waddr;
    logic [W_PTR-1:0] mem_ll_raddr;
    logic [W_PTR-1:0] mem_ll_rdata;
    // Data memory signals
    logic             mem_data_wen;
    logic             mem_data_ren;
    logic [W_PTR-1:0] mem_data_raddr;
    // Status Interface
    logic [N_QUEUE-1:0] status_qne;
    logic qs_rd_has_more;
    // ------------------------------------------------------------
    // QUEUE STATE
    // ------------------------------------------------------------
    qstate #(
        .N_QUEUE(N_QUEUE),
        .W_PTR(W_PTR)
    ) qstate_inst (
        .clk(clk),
        .rst_n(rst_n),
        // Write interface
        .i_wr_req_v(i_wr_req_v),
        .i_wr_req_qnum(i_wr_req_qnum),
        // Read interface
        .i_rd_req_v(i_rd_req_v),
        .i_rd_req_qnum(i_rd_req_qnum),
        .i_rd_req_peek(i_rd_req_peek),
        // Free List Interface
        .i_fl_alloc_v(fl_alloc_v),
        .i_fl_alloc_addr(fl_alloc_addr),
        // Linked List Memory Interface
        .o_mem_ll_wen(mem_ll_wen),
        .o_mem_ll_ren(mem_ll_ren),
        .o_mem_ll_waddr(mem_ll_waddr),
        .o_mem_ll_raddr(mem_ll_raddr),
        .i_mem_ll_rdata(mem_ll_rdata),
        // Data Memory Interface
        .o_mem_data_wen(mem_data_wen),
        .o_mem_data_ren(mem_data_ren),
        .o_mem_data_raddr(mem_data_raddr),
        // Status Interface
        .o_status_qne(o_status_qne),
        .o_rd_has_more(qs_rd_has_more)
    );

    // Align has_more with the cycle after the request (matches perq_fifo_sram,
    // which can only know post-pop occupancy at its qstate S1).
    logic rd_has_more_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_has_more_q <= 1'b0;
        else        rd_has_more_q <= qs_rd_has_more;
    end
    assign o_rd_has_more = rd_has_more_q;

    // ------------------------------------------------------------
    // FREE LIST
    // ------------------------------------------------------------
    if (FL_SRAM) begin : gen_fl_sram
        free_list_sram #(
            .N_PTRS(MEM_DEPTH)
        ) free_list_inst (
            .clk          (clk),
            .rst_n        (rst_n),
            .i_free_addr  (mem_data_raddr),  // Return to free list after a read
            .i_free_v     (mem_data_ren && !i_rd_req_peek),  // Only free on pop, not peek
            .i_alloc_r    (mem_data_wen),    // Acknowledge pointers if doing write
            .o_alloc_v    (fl_alloc_v),
            .o_alloc_addr (fl_alloc_addr),
            .if_mem       (if_mem_fl)
        );
    end else begin : gen_fl_flop
        free_list #(
            .N_PTRS(MEM_DEPTH),
            .N_PARTITIONS(NUM_PARTITIONS)
        ) free_list_inst (
            .clk          (clk),
            .rst_n        (rst_n),
            .i_free_addr  (mem_data_raddr),  // Return to free list after a read
            .i_free_v     (mem_data_ren && !i_rd_req_peek),  // Only free on pop, not peek
            .i_alloc_r    (mem_data_wen),    // Acknowledge pointers if doing write
            .o_alloc_v    (fl_alloc_v),
            .o_alloc_addr (fl_alloc_addr)
        );

        // Free-list RAM unused; tie off the driven side of the interface
        assign if_mem_fl.wen   = 1'b0;
        assign if_mem_fl.ren   = 1'b0;
        assign if_mem_fl.waddr = '0;
        assign if_mem_fl.raddr = '0;
        assign if_mem_fl.wdata = '0;
        assign if_mem_fl.werr  = '0;
    end

    // ------------------------------------------------------------
    // LINK LIST MEMORY INTERFACE
    // ------------------------------------------------------------
    assign if_mem_ll.wen   = mem_ll_wen;
    assign if_mem_ll.ren   = mem_ll_ren;
    assign if_mem_ll.waddr = mem_ll_waddr;
    assign if_mem_ll.raddr = mem_ll_raddr;
    assign if_mem_ll.wdata = fl_alloc_addr;
    assign if_mem_ll.werr  = '0; // FIXME: eventually drive from debug register
    assign mem_ll_rdata    = if_mem_ll.rdata;

    // ------------------------------------------------------------
    // DATA MEMORY INTERFACE
    // ------------------------------------------------------------
    assign if_mem_data.wen   = mem_data_wen;
    assign if_mem_data.ren   = mem_data_ren;
    assign if_mem_data.waddr = fl_alloc_addr;
    assign if_mem_data.raddr = mem_data_raddr;
    assign if_mem_data.wdata = i_wr_req_data;
    assign if_mem_data.werr  = '0; // FIXME: eventually drive from debug register
    assign o_rd_resp_data    = if_mem_data.rdata;

    // ------------------------------------------------------------
    // FLOW CONTROL
    // ------------------------------------------------------------
    assign o_wr_req_r = fl_alloc_v;
    pipes #(.STAGES(1),.W_DATA(1)) u_pipes_rd_resp_v (
      .clk      (clk),
      .rst_n    (rst_n),
      .i_data   (mem_data_ren),
      .o_data   (o_rd_resp_v)
    );

    // ============================================================
    // ASSERTIONS - Protocol Checks
    // ============================================================
`ifndef ASSERTS_OFF

    // Read requests must target non-empty queues
    a_rd_from_nonempty: assert property (
        @(posedge clk) disable iff (!rst_n)
        i_rd_req_v |-> o_status_qne[i_rd_req_qnum]
    ) else $error("Read request to empty queue %0d", i_rd_req_qnum);

    // Read response valid must follow a read request (1 cycle pipeline)
    a_rd_resp_follows_req: assert property (
        @(posedge clk) disable iff (!rst_n)
        o_rd_resp_v |-> $past(i_rd_req_v)
    ) else $error("Read response without preceding read request");

    // ============================================================
    // ASSERTIONS - Data Integrity (Model-Based, Simulation Only)
    // ============================================================
    // Uses SV queues to model expected FIFO contents per queue.
    // Verifies data read matches data written in FIFO order.
    // Peek operations verify data but don't pop from model.
    // synthesis translate_off

    logic [W_DATA-1:0] expected_data[N_QUEUE][$];
    logic [$clog2(N_QUEUE)-1:0] rd_qnum_q;
    logic rd_peek_q;

    // Delay read request signals to align with response (1 cycle pipeline)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_qnum_q <= '0;
            rd_peek_q <= 1'b0;
        end else begin
            rd_qnum_q <= i_rd_req_qnum;
            rd_peek_q <= i_rd_req_peek;
        end
    end

    // Track writes: push data when write completes
    always @(posedge clk) begin
        if (rst_n && i_wr_req_v && o_wr_req_r) begin
            expected_data[i_wr_req_qnum].push_back(i_wr_req_data);
        end
    end

    // Verify data integrity; pop from model only on pop (not peek)
    always @(posedge clk) begin
        if (rst_n && o_rd_resp_v) begin
            automatic logic [W_DATA-1:0] exp_data;

            a_model_not_empty: assert (expected_data[rd_qnum_q].size() > 0)
                else $error("Read response for queue %0d but model is empty", rd_qnum_q);

            if (expected_data[rd_qnum_q].size() > 0) begin
                // Peek: check front without removing; Pop: check and remove
                if (rd_peek_q) begin
                    exp_data = expected_data[rd_qnum_q][0];
                end else begin
                    exp_data = expected_data[rd_qnum_q].pop_front();
                end

                a_data_integrity: assert (o_rd_resp_data == exp_data)
                    else $error("Data mismatch on queue %0d: expected 0x%h, got 0x%h",
                               rd_qnum_q, exp_data, o_rd_resp_data);
            end
        end
    end
    // synthesis translate_on

`endif // ASSERTS_OFF

endmodule
