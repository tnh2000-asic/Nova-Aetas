//==============================================================================
// qstate_sram.sv - SRAM-backed per-queue state for perq_fifo_sram
//==============================================================================
// SRAM-backed equivalent of qstate.sv intended for large N_QUEUE (e.g. 4096),
// where the flop-based qstate exhibits an explosion in flop count.
//
// Storage:
//   - {head, tail, head_eq_tail} per queue → mirrored 1R1W qstate SRAMs
//     (one read by the push path, one read by the pop path; both written
//     identically). SRAM ports are exposed at the top of this module, so the
//     integrator picks the technology mapping (mirrors data/LL pattern).
//   - valid bit per queue → flop array. Drives o_status_qne combinationally
//     and is used at S0 to detect "initial write" / "rd_last" conditions.
//
// Pipeline (1 extra cycle of latency vs. flop-based qstate):
//   - S0 (request cycle): accept request, issue qstate SRAM reads.
//                         Free-list alloc is consumed at S0 and registered
//                         through to S1.
//   - S1 (action cycle):  qstate SRAM rdata available; drive data/LL memory
//                         interfaces and write back next-state to qstate
//                         SRAMs + valid flop array.
//   - S2 (LL-resp cycle): for pops that issued an LL read in S1, the LL
//                         response data (i_mem_ll_rdata) is registered here
//                         and used to advance head/het in qstate SRAMs.
//
// Hazards:
//   - Back-to-back same-qid push/pop RAW on the qstate SRAMs is handled by
//     mem_1r1w_hazard_guard wrappers (WBR=1) on each SRAM, instantiated
//     internally so the exposed top-level ports are post-guard.
//   - Back-to-back same-qid pop where the LL response hasn't yet committed
//     to head_sram is handled by an explicit S1 head/het bypass from the
//     in-flight LL response (same idea as the original qstate).
//   - Push S1 and LL-response writes to qstate SRAM in the same cycle would
//     collide on the single write port. They are merged when they target
//     the same qid (push updates tail, LL updates head, het is recomputed);
//     when they target different qids, push is stalled at S0 by deasserting
//     o_wr_req_stall_n upstream (back-pressuring the push request).
//==============================================================================

module qstate_sram
#(
    parameter int N_QUEUE = 64,
    parameter int W_PTR   = 10
) (
    input  logic                       clk,
    input  logic                       rst_n,

    //--------------------------------------------------------------------------
    // S0 inputs
    //--------------------------------------------------------------------------
    // Push request
    input  logic                       i_wr_req_v,
    input  logic [$clog2(N_QUEUE)-1:0] i_wr_req_qnum,
    output logic                       o_wr_req_stall,    // S0: must back off this cycle
    // Pop request
    input  logic                       i_rd_req_v,
    input  logic [$clog2(N_QUEUE)-1:0] i_rd_req_qnum,
    input  logic                       i_rd_req_peek,
    // Free-list pointer (consumed combinationally at S0 when push enters)
    input  logic [W_PTR-1:0]           i_fl_alloc_addr,

    //--------------------------------------------------------------------------
    // S1 outputs (data / LL memory interfaces) — fire one cycle after S0 inputs
    //--------------------------------------------------------------------------
    // Linked-list memory
    output logic                       o_mem_ll_wen,
    output logic                       o_mem_ll_ren,
    output logic [W_PTR-1:0]           o_mem_ll_waddr,
    output logic [W_PTR-1:0]           o_mem_ll_raddr,
    input  logic [W_PTR-1:0]           i_mem_ll_rdata,
    // Data memory
    output logic                       o_mem_data_wen,
    output logic                       o_mem_data_ren,
    output logic [W_PTR-1:0]           o_mem_data_raddr,
    output logic [W_PTR-1:0]           o_mem_data_waddr,

    //--------------------------------------------------------------------------
    // S1 metadata to wrapper (perq_fifo_sram)
    //--------------------------------------------------------------------------
    output logic                       o_wr_v_s1,         // push committed at S1 this cycle
    output logic                       o_rd_v_s1,         // pop committed at S1 this cycle
    output logic                       o_fl_free_v,       // S1: free a popped pointer
    output logic [W_PTR-1:0]           o_fl_free_addr,

    //--------------------------------------------------------------------------
    // Status (combinational from flop array)
    //--------------------------------------------------------------------------
    output logic [N_QUEUE-1:0]         o_status_qne,
    output logic                       o_rd_has_more,     // Read's queue still non-empty after the op (S1, one cycle after the request)

    //--------------------------------------------------------------------------
    // Qstate SRAM ports (post-hazard-guard, exposed to integrator)
    //--------------------------------------------------------------------------
    mem_1r1w_if.user                   if_mem_qstate_p,   // push-side read
    mem_1r1w_if.user                   if_mem_qstate_q    // pop-side read
);

    //--------------------------------------------------------------------------
    // Local types / params
    //--------------------------------------------------------------------------
    localparam int W_QID = $clog2(N_QUEUE);

    typedef struct packed {
        logic [W_PTR-1:0] head;
        logic [W_PTR-1:0] tail;
        logic             head_eq_tail;
    } qstate_t;

    localparam int W_QSTATE = $bits(qstate_t);

    //--------------------------------------------------------------------------
    // Per-queue valid (flop array). N_QUEUE flops; cheap and used to drive
    // o_status_qne combinationally with no SRAM latency.
    //--------------------------------------------------------------------------
    logic [N_QUEUE-1:0] valid_q;
    logic [N_QUEUE-1:0] valid_d;

    // Next-cycle valid: includes the effect of the in-flight S1 pop draining
    // the queue and any S1 push validating it. Exposing this (rather than
    // valid_q) prevents a TB race where a new pop S0 is issued in the same
    // cycle that the prior pop S1 is draining the queue. That new pop would
    // land in S1 next cycle on a queue that just became empty.
    assign o_status_qne = valid_d;

    //--------------------------------------------------------------------------
    // Internal qstate SRAM controller-side interfaces (pre-hazard-guard)
    //--------------------------------------------------------------------------
    mem_1r1w_if #(.DEPTH(N_QUEUE), .W_DATA(W_QSTATE)) qsp_ctrl ();
    mem_1r1w_if #(.DEPTH(N_QUEUE), .W_DATA(W_QSTATE)) qsq_ctrl ();

    mem_1r1w_hazard_guard #(.READ_LATENCY(1), .WBR(1'b1)) u_hg_qsp (
        .clk    (clk),
        .rst_n  (rst_n),
        .if_ctrl(qsp_ctrl),
        .if_mem (if_mem_qstate_p)
    );

    mem_1r1w_hazard_guard #(.READ_LATENCY(1), .WBR(1'b1)) u_hg_qsq (
        .clk    (clk),
        .rst_n  (rst_n),
        .if_ctrl(qsq_ctrl),
        .if_mem (if_mem_qstate_q)
    );

    //==========================================================================
    // S0: predict next-cycle write-port conflict + drive SRAM reads
    //==========================================================================

    //--------------------------------------------------------------------------
    // S1 pipeline registers (visible at S1 = current cycle for cycle-T-1's S0)
    //--------------------------------------------------------------------------
    logic              wr_v_s1_q;           // push reached S1 this cycle
    logic [W_QID-1:0]  wr_qnum_s1_q;
    logic [W_PTR-1:0]  wr_alloc_addr_s1_q;  // pointer registered from S0
    logic              wr_initial_s1_q;     // valid[wr_qid] was 0 at S0
    logic              wr_same_q_rd_s1_q;   // S0 had push & pop to same qid

    logic              rd_v_s1_q;
    logic [W_QID-1:0]  rd_qnum_s1_q;
    logic              rd_peek_s1_q;

    //--------------------------------------------------------------------------
    // Pop-side qstate read data at S1 (current cycle)
    //--------------------------------------------------------------------------
    qstate_t qsq_rdata_s1;
    qstate_t qsp_rdata_s1;
    assign qsq_rdata_s1 = qstate_t'(qsq_ctrl.rdata);
    assign qsp_rdata_s1 = qstate_t'(qsp_ctrl.rdata);

    //--------------------------------------------------------------------------
    // S2 LL-response state (current cycle = S2 of a pop in S1 last cycle)
    //--------------------------------------------------------------------------
    logic              ll_rdata_v_q;
    logic [W_QID-1:0]  ll_rdata_qnum_q;
    logic [W_PTR-1:0]  ll_tail_snap_q;      // tail at pop S1, used for het recompute

    //--------------------------------------------------------------------------
    // S1 pop bypass logic (S1 head/het sourced from in-flight LL response when
    // a back-to-back same-qid pop is in flight)
    //--------------------------------------------------------------------------
    logic              bypass_pop_head_v;
    logic [W_PTR-1:0]  pop_head_eff;
    logic              pop_het_eff;

    // Effective tail at this S1 for the popping qid. ll_tail_snap_q carries
    // the tail that was current when the prior pop entered S1; it does NOT
    // see any push to the same qid that lands in S1 *this* cycle. If such a
    // push exists, the live tail is the push's just-allocated pointer, so
    // pop_het_eff must compare against that instead — otherwise a non-empty
    // queue can be misclassified as rd_last and the merge writeback collapses
    // qstate to (K,K,1), corrupting head/tail for all subsequent pops.
    logic [W_PTR-1:0]  pop_eff_tail;
    assign pop_eff_tail = (wr_v_s1_q && (wr_qnum_s1_q == rd_qnum_s1_q))
                        ? wr_alloc_addr_s1_q
                        : ll_tail_snap_q;

    // Bypass A: prior pop's LL response carries the new head for a back-to-back
    //           same-qid pop (queue had >=2 entries on prior pop).
    assign bypass_pop_head_v = ll_rdata_v_q && rd_v_s1_q && (ll_rdata_qnum_q == rd_qnum_s1_q);

    // Bypass B: same-cycle push to the popping qid creates the queue's only
    //           entry (push wr_initial_s1_q=1). qsq_rdata is stale (queue was
    //           empty / just emptied), so the live head is the push's alloc.
    //           Applies to peek and pop alike — both need to read K's data.
    logic              bypass_pop_from_push_v;
    assign bypass_pop_from_push_v = wr_v_s1_q && wr_initial_s1_q && rd_v_s1_q
                                 && (wr_qnum_s1_q == rd_qnum_s1_q);

    // Net-empty case: like bypass_pop_from_push_v but NOT a peek — pop
    // consumes the push's only entry, leaving the queue empty. Suppress
    // qstate/valid update so the queue is recognized as empty next cycle.
    logic              push_pop_net_empty;
    assign push_pop_net_empty = bypass_pop_from_push_v && !rd_peek_s1_q;

    assign pop_head_eff = bypass_pop_from_push_v ? wr_alloc_addr_s1_q
                        : bypass_pop_head_v      ? i_mem_ll_rdata
                                                 : qsq_rdata_s1.head;
    assign pop_het_eff  = bypass_pop_from_push_v ? 1'b1
                        : bypass_pop_head_v      ? (i_mem_ll_rdata == pop_eff_tail)
                                                 : qsq_rdata_s1.head_eq_tail;

    //--------------------------------------------------------------------------
    // S1 pop classification
    //--------------------------------------------------------------------------
    logic pop_will_ll_ren;     // pop in S1 will issue LL read this cycle
    logic pop_rd_last;         // pop in S1 will empty the queue
    assign pop_will_ll_ren = rd_v_s1_q && !rd_peek_s1_q && !pop_het_eff;
    assign pop_rd_last     = rd_v_s1_q && !rd_peek_s1_q &&  pop_het_eff;

    // Post-op occupancy for the read in S1 this cycle: valid_d folds in this
    // pop's drain (pop_rd_last) and any S1 push refill; a peek leaves the
    // queue intact. One cycle after the accepted read request, one cycle
    // before o_rd_resp_v at the perq_fifo_sram boundary.
    assign o_rd_has_more = rd_v_s1_q && valid_d[rd_qnum_s1_q];

    //--------------------------------------------------------------------------
    // Predict next-cycle qstate write-port conflict
    //   - push at this S0 will be in S1 next cycle and ALWAYS writes qstate
    //     (to update tail).
    //   - LL response will land next cycle iff a pop in S1 this cycle issues
    //     an LL read (pop_will_ll_ren).
    //   - If they target the same qid, the writes are merged in S1 (push
    //     updates tail, LL response updates head, het recomputed). Otherwise
    //     stall the push at S0.
    //--------------------------------------------------------------------------
    logic ll_resp_will_land_next;
    assign ll_resp_will_land_next = pop_will_ll_ren;

    // Stall the push whenever an LL response is incoming next cycle, so S1
    // only ever sees a single qstate write source.
    //
    // We deliberately do NOT qualify on (rd_qnum_s1_q != i_wr_req_qnum):
    // that would make o_wr_req_r combinationally dependent on i_wr_req_qnum.
    // A driver that samples o_wr_req_r before driving i_wr_req_qnum could
    // see ready=1 with a stale qnum, commit the push to its model, then
    // drive the real qnum and have ready collapse to 0 mid-cycle -- silently
    // dropping the push.
    //
    // o_wr_req_stall must also not depend on i_wr_req_v: the wrapper feeds
    // i_wr_req_v = wr_xact = (user.v && fl_alloc_v && !o_wr_req_stall), so
    // such a dependency would close a combinational loop. wr_accept_s0
    // below re-gates on i_wr_req_v, so a spuriously-asserted stall when the
    // user isn't pushing is harmless.
    //
    // TODO(EN-2252): unconditional stall is overly conservative.
    //   Today we backpressure ANY push whenever a same-cycle pop in S1 will
    //   issue an LL ren, regardless of whether the push targets the same
    //   qid (in which case it could merge in S1) or a different qid (in
    //   which case the only conflict is the single qstate write port one
    //   cycle later). Realistic workloads see this stall fire only on the
    //   one cycle per pop-with-LL, but a high-throughput multi-queue
    //   sender will measurably lose pushes.
    //   Fixes (separate MR):
    //     1. Add a small skid buffer at the push S0 boundary so we can
    //        accept the push this cycle and replay it next cycle when the
    //        write port is free. Cuts the credit penalty to ~0.
    //     2. Add an arbiter on the qstate write port that gives push S1
    //        priority and lets the LL-response write retry one cycle later
    //        (requires a 1-deep LL-response holding register).
    //     3. Resolve the comb-loop concern via a registered o_wr_req_r ack
    //        so the qid-match qualification becomes safe.
    //   Acceptable for the v1 EN-2252 MR; revisit if desc_pool credit
    //   throughput becomes a bottleneck.
    assign o_wr_req_stall = ll_resp_will_land_next;

    //--------------------------------------------------------------------------
    // S0 same-cycle push+pop classification (used to decide "initial-write"
    // vs. "rd_last" interactions). Computed combinationally at S0 and
    // captured into S1 pipeline registers.
    //--------------------------------------------------------------------------
    logic wr_accept_s0;
    logic rd_accept_s0;
    logic same_q_wr_rd_s0;
    logic wr_initial_s0;

    assign wr_accept_s0   = i_wr_req_v && !o_wr_req_stall;
    assign rd_accept_s0   = i_rd_req_v;
    assign same_q_wr_rd_s0= wr_accept_s0 && rd_accept_s0 &&
                            (i_wr_req_qnum == i_rd_req_qnum);
    // wr_initial means "this push S1 will land into an empty queue", which
    // determines whether the writeback resets head=tail=K. valid_q lags by
    // one cycle, so to predict the queue's state at this push's S1 time we
    // use valid_d -- the combinational next-state, which already folds in
    // any push at S1 making the qid valid and any pop_rd_last clearing it.
    assign wr_initial_s0  = wr_accept_s0 && !valid_d[i_wr_req_qnum];

    //--------------------------------------------------------------------------
    // S0: drive qstate SRAM reads
    //--------------------------------------------------------------------------
    assign qsp_ctrl.ren   = wr_accept_s0;
    assign qsp_ctrl.raddr = i_wr_req_qnum;
    assign qsq_ctrl.ren   = rd_accept_s0;
    assign qsq_ctrl.raddr = i_rd_req_qnum;

    //--------------------------------------------------------------------------
    // S0 → S1 pipeline registers
    //--------------------------------------------------------------------------
    // het_q flop array: snapshot used for predicting pop's LL ren behavior at
    // S1 (only the qid we read is used, but easier to keep the full flop array
    // updated coherently with the SRAM-backed het field).
    logic [N_QUEUE-1:0] het_q;
    logic [N_QUEUE-1:0] het_d;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_v_s1_q          <= 1'b0;
            wr_qnum_s1_q       <= '0;
            wr_alloc_addr_s1_q <= '0;
            wr_initial_s1_q    <= 1'b0;
            wr_same_q_rd_s1_q  <= 1'b0;
            rd_v_s1_q          <= 1'b0;
            rd_qnum_s1_q       <= '0;
            rd_peek_s1_q       <= 1'b0;
        end else begin
            wr_v_s1_q          <= wr_accept_s0;
            wr_qnum_s1_q       <= i_wr_req_qnum;
            wr_alloc_addr_s1_q <= i_fl_alloc_addr;
            wr_initial_s1_q    <= wr_initial_s0;
            wr_same_q_rd_s1_q  <= same_q_wr_rd_s0;
            rd_v_s1_q          <= rd_accept_s0;
            rd_qnum_s1_q       <= i_rd_req_qnum;
            rd_peek_s1_q       <= i_rd_req_peek;
        end
    end

    //==========================================================================
    // S1: drive memory interfaces and compute next-state
    //==========================================================================

    //--------------------------------------------------------------------------
    // Same-q wr+rd-last handling at S1 (covers the push+pop-to-same-qid case
    // where pop empties the queue and push refills it).
    //--------------------------------------------------------------------------
    logic same_q_wr_rd_last_s1;
    assign same_q_wr_rd_last_s1 = wr_same_q_rd_s1_q && pop_rd_last;

    //--------------------------------------------------------------------------
    // Data memory: write on push S1; read on pop S1 (when valid)
    //--------------------------------------------------------------------------
    assign o_mem_data_wen   = wr_v_s1_q;
    assign o_mem_data_waddr = wr_alloc_addr_s1_q;
    assign o_mem_data_ren   = rd_v_s1_q;            // pop must target a valid queue (asserted upstream)
    assign o_mem_data_raddr = pop_head_eff;

    //--------------------------------------------------------------------------
    // Linked-list memory:
    //   - Write on a push that's NOT initial-write (LL stitches old tail → new alloc).
    //   - Read on a pop that's NOT rd_last and NOT peek (advance head pointer).
    //--------------------------------------------------------------------------
    assign o_mem_ll_wen   = wr_v_s1_q && !wr_initial_s1_q;
    assign o_mem_ll_waddr = qsp_rdata_s1.tail;
    assign o_mem_ll_ren   = pop_will_ll_ren;
    assign o_mem_ll_raddr = pop_head_eff;

    //--------------------------------------------------------------------------
    // S1 metadata out
    //--------------------------------------------------------------------------
    assign o_wr_v_s1     = wr_v_s1_q;
    assign o_rd_v_s1     = rd_v_s1_q;
    assign o_fl_free_v   = rd_v_s1_q && !rd_peek_s1_q;
    assign o_fl_free_addr= pop_head_eff;

    //==========================================================================
    // S2: register LL response and recompute head/het
    //==========================================================================
    // Snapshot tail of the popped queue so the het update at S2 can compute
    // (new_head == tail). If a push to the same qid lands at S2 same cycle,
    // the merged-write logic below uses the new (post-push) tail instead.
    //
    // Special case: if a push to the SAME qid happened at the same S0 cycle
    // as this pop (wr_same_q_rd_s1_q=1) AND the pop will issue an LL read
    // (pop_will_ll_ren=1, i.e. queue has >=2 elements), then the push S1 is
    // updating tail to wr_alloc_addr_s1_q THIS cycle. But our qsq_rdata_s1
    // was read at S0 (one cycle before that write) so it carries the OLD
    // tail. We must capture the NEW tail (push's alloc) so the LL response
    // at S2 doesn't revert qstate.tail back to the pre-push value. The
    // hazard guard would have bypassed if the read and write were in the
    // SAME cycle, but here the write is one cycle after the read.
    logic [W_PTR-1:0] ll_tail_snap_d;
    assign ll_tail_snap_d = (wr_same_q_rd_s1_q && pop_will_ll_ren)
                          ? wr_alloc_addr_s1_q
                          : qsq_rdata_s1.tail;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ll_rdata_v_q    <= 1'b0;
            ll_rdata_qnum_q <= '0;
            ll_tail_snap_q  <= '0;
        end else begin
            ll_rdata_v_q    <= o_mem_ll_ren;
            ll_rdata_qnum_q <= rd_qnum_s1_q;
            ll_tail_snap_q  <= ll_tail_snap_d;
        end
    end

    //==========================================================================
    // Qstate SRAM write-back at S1 (push) and S2 (LL response), merged when
    // both target the same qid.
    //==========================================================================
    logic            wr_qstate_v;
    logic [W_QID-1:0]wr_qstate_qid;
    qstate_t         wr_qstate_data;

    // LL-response qstate update value (only the head/het fields matter).
    logic [W_PTR-1:0] ll_new_head;
    logic             ll_new_het;
    assign ll_new_head = i_mem_ll_rdata;

    // Push qstate update value (tail, plus head/het when initial / drain-fill).
    logic [W_PTR-1:0] push_new_head;
    logic [W_PTR-1:0] push_new_tail;
    logic             push_new_het;
    assign push_new_tail = wr_alloc_addr_s1_q;
    assign push_new_head = (wr_initial_s1_q || same_q_wr_rd_last_s1) ? wr_alloc_addr_s1_q
                                                                     : qsp_rdata_s1.head;
    assign push_new_het  = (wr_initial_s1_q || same_q_wr_rd_last_s1) ? 1'b1
                                                                     : 1'b0;

    // Merge case: push and LL response both touch the same qid this cycle.
    // - push provides the tail (always)
    // - LL response provides the head
    // - het = (new_head == new_tail)
    logic merge_push_ll;
    assign merge_push_ll = wr_v_s1_q && ll_rdata_v_q && (wr_qnum_s1_q == ll_rdata_qnum_q);

    // For the merged case wr_initial cannot be true (LL response implies the
    // queue was previously valid with >1 entries), so push_new_head/tail apply
    // unmodified except we override head with the LL result.
    assign ll_new_het  = merge_push_ll ? (ll_new_head == push_new_tail)
                                       : (ll_new_head == ll_tail_snap_q);

    always_comb begin
        wr_qstate_v    = 1'b0;
        wr_qstate_qid  = '0;
        wr_qstate_data = '0;

        if (merge_push_ll) begin
            wr_qstate_v             = 1'b1;
            wr_qstate_qid           = wr_qnum_s1_q;
            // Three-way same-q concurrency: push S1 + pop#1 LL-resp S2 +
            // pop#2 S1 to the same qid, where pop#2 (bypassing off
            // pop#1's LL response) is rd_last. pop#2 consumes ll_new_head,
            // emptying the queue, and the push refills with K. Final state
            // is (head=K, tail=K, het=1) — same as the push-only
            // same_q_wr_rd_last case.
            if (same_q_wr_rd_last_s1) begin
                wr_qstate_data.head         = wr_alloc_addr_s1_q;
                wr_qstate_data.tail         = wr_alloc_addr_s1_q;
                wr_qstate_data.head_eq_tail = 1'b1;
            end else begin
                wr_qstate_data.head         = ll_new_head;
                wr_qstate_data.tail         = push_new_tail;
                wr_qstate_data.head_eq_tail = ll_new_het;
            end
        end else if (wr_v_s1_q && !push_pop_net_empty) begin
            wr_qstate_v             = 1'b1;
            wr_qstate_qid           = wr_qnum_s1_q;
            wr_qstate_data.head     = push_new_head;
            wr_qstate_data.tail     = push_new_tail;
            wr_qstate_data.head_eq_tail = push_new_het;
        end else if (ll_rdata_v_q) begin
            wr_qstate_v             = 1'b1;
            wr_qstate_qid           = ll_rdata_qnum_q;
            wr_qstate_data.head     = ll_new_head;
            wr_qstate_data.tail     = ll_tail_snap_q;
            wr_qstate_data.head_eq_tail = ll_new_het;
        end
    end

    // Drive both mirrored SRAMs with the same write each cycle.
    assign qsp_ctrl.wen   = wr_qstate_v;
    assign qsp_ctrl.waddr = wr_qstate_qid;
    assign qsp_ctrl.wdata = wr_qstate_data;
    assign qsp_ctrl.werr  = '0;

    assign qsq_ctrl.wen   = wr_qstate_v;
    assign qsq_ctrl.waddr = wr_qstate_qid;
    assign qsq_ctrl.wdata = wr_qstate_data;
    assign qsq_ctrl.werr  = '0;

    //==========================================================================
    // valid + het flop array next-state
    //==========================================================================
    always_comb begin
        valid_d = valid_q;
        het_d   = het_q;

        // Push S1: queue becomes valid; het = 1 only if initial / drain-fill.
        // Exception: bypass_pop_from_push_v — push allocates K and concurrent
        // pop immediately consumes K. Net result: queue stays empty, no
        // qstate/LL update needed. Skip the valid set (and the qstate write
        // is suppressed below) so a future push sees init=1 again.
        if (wr_v_s1_q && !push_pop_net_empty) begin
            valid_d[wr_qnum_s1_q] = 1'b1;
            het_d  [wr_qnum_s1_q] = (wr_initial_s1_q || same_q_wr_rd_last_s1) ? 1'b1 : 1'b0;
        end

        // Pop S1: rd_last clears valid (unless same-qid push refilled).
        if (pop_rd_last && !same_q_wr_rd_last_s1) begin
            valid_d[rd_qnum_s1_q] = 1'b0;
            het_d  [rd_qnum_s1_q] = 1'b0;
        end

        // LL response (S2): het reflects new_head vs current tail. Skip the
        // het update if a same-cycle same-q push already drove it (merge case
        // already includes it via push branch above) AND for the
        // same_q_wr_rd_last_s1 corner where the push path set het=1.
        if (ll_rdata_v_q) begin
            // If merge with push, push branch above already set het correctly.
            // Otherwise: if a non-merging push touched this qid, the push value
            // is already written; let LL response not stomp on it.
            if (!merge_push_ll && !(wr_v_s1_q && (wr_qnum_s1_q == ll_rdata_qnum_q))) begin
                het_d[ll_rdata_qnum_q] = ll_new_het;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_q <= '0;
            het_q   <= '0;
        end else begin
            valid_q <= valid_d;
            het_q   <= het_d;
        end
    end

    //==========================================================================
    // SystemVerilog Assertions
    //==========================================================================
`ifndef ASSERTS_OFF
    // S0 reads must be to queues that are (or will be after this cycle) valid.
    // valid_d incorporates an in-flight S1 push that validates the qid this
    // cycle, matching the TB-visible o_status_qne (= valid_d).
    a_read_to_valid_queue: assert property (
        @(posedge clk) disable iff (!rst_n)
        i_rd_req_v |-> valid_d[i_rd_req_qnum]
    ) else $error("qstate_sram: read request to invalid queue %0d", i_rd_req_qnum);

    // Push must not be accepted when stalled.
    a_no_accept_when_stalled: assert property (
        @(posedge clk) disable iff (!rst_n)
        o_wr_req_stall |-> !wr_accept_s0
    ) else $error("qstate_sram: push accepted while stalled");

    // qstate writeback may never collide on the SRAM write port (we should
    // have either merged or stalled).
    a_no_qstate_collision: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(wr_v_s1_q && ll_rdata_v_q && (wr_qnum_s1_q != ll_rdata_qnum_q))
    ) else $error("qstate_sram: unmerged push+LL writes to different qids in same cycle");
`endif

endmodule : qstate_sram
