module qstate
#(
    parameter int N_QUEUE = 64,      // Number of queues
    parameter int W_PTR   = 10       // Pointer width in bits
) (
    input  logic                       clk,
    input  logic                       rst_n,
    // Write Interface
    input  logic                       i_wr_req_v,       // Valid write request
    input  logic [$clog2(N_QUEUE)-1:0] i_wr_req_qnum,    // Queue number to write to
    // Read Interface
    input  logic                       i_rd_req_v,       // Valid read request
    input  logic [$clog2(N_QUEUE)-1:0] i_rd_req_qnum,    // Queue number to read from
    input  logic                       i_rd_req_peek,    // Peek mode (0: pop, 1: peek)
    // Free List Interface
    input  logic                       i_fl_alloc_v,
    input  logic [W_PTR-1:0]           i_fl_alloc_addr,
    // Linked List Memory Interface
    output logic                       o_mem_ll_wen,
    output logic                       o_mem_ll_ren,
    output logic [W_PTR-1:0]           o_mem_ll_waddr,
    output logic [W_PTR-1:0]           o_mem_ll_raddr,
    input  logic [W_PTR-1:0]           i_mem_ll_rdata,
    // Data Memory Interface
    output logic                       o_mem_data_wen,
    output logic                       o_mem_data_ren,
    output logic [W_PTR-1:0]           o_mem_data_raddr,
    // Status Interface
    output logic [N_QUEUE-1:0]         o_status_qne,     // Queue not empty (stall when deasserted)
    output logic                       o_rd_has_more     // Read's queue still non-empty after the op (combinational with the request)
);

typedef struct packed {
    logic [W_PTR-1:0] head;            // Head pointer
    logic [W_PTR-1:0] tail;            // Tail pointer
    logic             valid;           // Valid bit
    logic             head_eq_tail;    // Head equals tail flag
} qstate_t;

// Queue state arrays
qstate_t qstate_d [N_QUEUE-1:0];
qstate_t qstate_q [N_QUEUE-1:0];

//================================================================================
// On a link_list read, save the qnum and validity to use when the read response
// is received. This is also used for the bypass logic that is needed for
// back-to-back reads to the same queue.
//================================================================================
logic ll_rdata_v;
logic [$clog2(N_QUEUE)-1:0] ll_rdata_qnum;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ll_rdata_v    <= 1'b0;
        ll_rdata_qnum <= '0;
    end else begin
        ll_rdata_v    <= o_mem_ll_ren;
        ll_rdata_qnum <= i_rd_req_qnum;
    end
end

//================================================================================
// Memory Interfaces with Bypass for Back-to-Back Reads
//================================================================================
// Bypass: use LL read data directly when reading same queue that has ll_rdata_v
logic bypass_head_v;
logic [W_PTR-1:0] head_eff;
logic head_eq_tail_eff;

assign bypass_head_v     = ll_rdata_v && (ll_rdata_qnum == i_rd_req_qnum) && i_rd_req_v;
assign head_eff          = bypass_head_v ? i_mem_ll_rdata : qstate_q[i_rd_req_qnum].head;
assign head_eq_tail_eff  = bypass_head_v ? (i_mem_ll_rdata == qstate_q[i_rd_req_qnum].tail)
                                         : qstate_q[i_rd_req_qnum].head_eq_tail;

// Data Memory Interface
assign o_mem_data_wen   = i_wr_req_v && i_fl_alloc_v;
assign o_mem_data_ren   = i_rd_req_v && qstate_q[i_rd_req_qnum].valid;
assign o_mem_data_raddr = head_eff;

// Linked List Memory Interface
// LL read only needed for pop (not peek) to get next head pointer
assign o_mem_ll_wen   = o_mem_data_wen && qstate_q[i_wr_req_qnum].valid;
assign o_mem_ll_ren   = o_mem_data_ren && !head_eq_tail_eff && !i_rd_req_peek;
assign o_mem_ll_waddr = qstate_q[i_wr_req_qnum].tail;
assign o_mem_ll_raddr = head_eff;

//================================================================================
// Queue State Update Logic
//================================================================================
// Event signals (using effective/bypassed values for read)
logic wr_v;                    // Valid write with allocation
logic rd_last_v;               // Pop that empties the queue (peek does not count as emptying)
logic same_q_wr_rd_last_v;     // Write and read-last to same queue
logic skip_ll_head_update;     // Skip LL head update when LL response queue matches write+read-last queue

assign wr_v              = o_mem_data_wen;
assign rd_last_v         = o_mem_data_ren && head_eq_tail_eff && !i_rd_req_peek;
assign same_q_wr_rd_last_v = wr_v && rd_last_v && (i_wr_req_qnum == i_rd_req_qnum);

always_comb begin
    qstate_d = qstate_q;

    // WRITE: Update tail, and head/valid if first write or same-queue read+write
    if (wr_v) begin
        qstate_d[i_wr_req_qnum].tail = i_fl_alloc_addr;
        if (!qstate_q[i_wr_req_qnum].valid || same_q_wr_rd_last_v) begin
            qstate_d[i_wr_req_qnum].head         = i_fl_alloc_addr;
            qstate_d[i_wr_req_qnum].valid        = 1'b1;
            qstate_d[i_wr_req_qnum].head_eq_tail = 1'b1;
        end else begin
            qstate_d[i_wr_req_qnum].head_eq_tail = 1'b0;
        end
    end

    // READ-LAST: Invalidate queue (unless same-queue write is happening)
    if (rd_last_v && !same_q_wr_rd_last_v) begin
        qstate_d[i_rd_req_qnum].valid = 1'b0;
    end

    // LL-READ-COMPLETE: Advance head to next pointer from linked list
    // Skip only if the LL response queue (ll_rdata_qnum, from previous cycle) matches
    // the current cycle's write+read-last queue (write path already set that queue's head)
    skip_ll_head_update = same_q_wr_rd_last_v && (ll_rdata_qnum == i_rd_req_qnum);
    if (ll_rdata_v && !skip_ll_head_update) begin
        qstate_d[ll_rdata_qnum].head         = i_mem_ll_rdata;
        qstate_d[ll_rdata_qnum].head_eq_tail = (i_mem_ll_rdata == qstate_d[ll_rdata_qnum].tail);
    end
end

//================================================================================
// Sequential Update
//================================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < N_QUEUE; i++) begin
            qstate_q[i] <= '0;
        end
    end else begin
        qstate_q <= qstate_d;
    end
end

// Drive status interface
always_comb begin
  for (int i = 0; i < N_QUEUE; i++) begin
      o_status_qne[i] = qstate_q[i].valid;
  end
end

// Post-op occupancy for the read presented this cycle: qstate_d already folds
// in this pop's drain (rd_last_v) and any same-cycle push refill; a peek
// leaves the queue intact. Consumers use this to decide whether another entry
// is queued behind the one being read.
assign o_rd_has_more = i_rd_req_v && qstate_d[i_rd_req_qnum].valid;

//================================================================================
// SystemVerilog Assertions
//================================================================================
`ifndef ASSERTS_OFF

// Assert that read requests are only made to valid queues
a_read_to_valid_queue: assert property (
    @(posedge clk) disable iff (!rst_n)
    i_rd_req_v |-> o_status_qne[i_rd_req_qnum]
) else $error("Read request to invalid queue %0d", i_rd_req_qnum);

// Assert that LL writes only happen to already-valid queues (subsequent writes)
a_ll_wen_to_valid_queue: assert property (
    @(posedge clk) disable iff (!rst_n)
    o_mem_ll_wen |-> o_status_qne[i_wr_req_qnum]
) else $error("LL write to invalid queue %0d", i_wr_req_qnum);

// Assert head_eq_tail consistency: when true, head must equal tail
generate
    for (genvar i = 0; i < N_QUEUE; i++) begin : gen_head_eq_tail_check
        a_head_eq_tail_consistent: assert property (
            @(posedge clk) disable iff (!rst_n)
            (qstate_q[i].valid && qstate_q[i].head_eq_tail) |->
            (qstate_q[i].head == qstate_q[i].tail)
        ) else $error("Queue %0d: head_eq_tail is set but head=%0d != tail=%0d",
            i, qstate_q[i].head, qstate_q[i].tail);
        end
endgenerate

`endif // ASSERTS_OFF

endmodule
