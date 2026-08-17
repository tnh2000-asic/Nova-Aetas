//========================================================================
// CIRCULAR_FIFO_ALMOST_FULL - Circular FIFO with FWFT and early full
//========================================================================
// A low-latency FIFO with parameterized depth and exactly one cycle of
// fall-through latency. Data written on cycle N is available at the
// output on cycle N+1.
//
// Implementation uses a demux to route input data to one of DEPTH
// registers, and a mux to select output data. This provides:
//   - Registered writes (one cycle latency)
//   - Combinational reads (data available immediately once valid)
//   - Constant one-cycle latency regardless of occupancy
//
// The o_full output is asserted when occupancy reaches
// DEPTH - FULL_MARGIN (early backpressure before physical full).
//========================================================================

module circular_fifo_almost_full #(
    parameter int DEPTH       = 4,      // FIFO depth (number of entries), must be >= 1
    parameter int W_DATA      = 32,     // Data width in bits
    parameter int FULL_MARGIN = 1       // o_full asserted when occupancy >= DEPTH - FULL_MARGIN (early backpressure before physical full).
) (
    input  logic              clk,
    input  logic              rst_n,

    // RVHS input ports
    input  logic              i_in_v,   // Input validity
    input  logic [W_DATA-1:0] i_in_d,   // Input data
    output logic              o_in_r,   // Input ready (stall when deasserted)

    // RVHS output ports
    output logic              o_out_v,  // Output validity
    output logic [W_DATA-1:0] o_out_d,  // Output data
    input  logic              i_out_r,  // Output ready (stall when deasserted)

    // Status
    output logic              o_full
);

    // Parameter validation (elaboration-time check)
    initial begin
        assert(DEPTH >= 1) else $fatal(1, "circular_fifo_almost_full: DEPTH must be >= 1, got %0d", DEPTH);
    end

    // Pointer width - handle DEPTH=1 case where $clog2 returns 0
    localparam int W_PTR = (DEPTH > 1) ? $clog2(DEPTH) : 1;

    // Storage registers and valid bits
    logic [W_DATA-1:0] mem   [DEPTH-1:0];
    logic [DEPTH-1:0]  valid;

    // Read and write pointers
    logic [W_PTR-1:0] wr_ptr;
    logic [W_PTR-1:0] rd_ptr;

    // Control signals
    logic empty;
    logic do_write, do_read;
    logic [$clog2(DEPTH+1)-1:0] occupancy;

    //------------------------------------------------------------------
    // Full/Empty/Almost-full detection using valid bits
    //------------------------------------------------------------------
    assign empty       = ~|valid;
    assign occupancy   = $countones(valid);
    assign o_full = (occupancy >= $bits(occupancy)'(DEPTH - FULL_MARGIN));

    //------------------------------------------------------------------
    // Handshake signals
    //------------------------------------------------------------------
    assign o_in_r  = ~(&valid);
    assign o_out_v = ~empty;

    //------------------------------------------------------------------
    // Transfer conditions
    //------------------------------------------------------------------
    assign do_write = i_in_v && o_in_r;
    assign do_read  = o_out_v && i_out_r;

    //------------------------------------------------------------------
    // Write pointer update
    //------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (do_write) begin
            wr_ptr <= (wr_ptr == W_PTR'(DEPTH-1)) ? '0 : wr_ptr + 1'b1;
        end
    end

    //------------------------------------------------------------------
    // Read pointer update
    //------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= '0;
        end else if (do_read) begin
            rd_ptr <= (rd_ptr == W_PTR'(DEPTH-1)) ? '0 : rd_ptr + 1'b1;
        end
    end

    //------------------------------------------------------------------
    // Memory write (demux to registers)
    //------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (do_write) begin
            mem[wr_ptr] <= i_in_d;
        end
    end

    //------------------------------------------------------------------
    // Valid bit management
    //------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= '0;
        end else begin
            if (do_write) begin
                valid[wr_ptr] <= 1'b1;
            end
            if (do_read) begin
                valid[rd_ptr] <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    // Output mux (combinational for FWFT behavior)
    //------------------------------------------------------------------
    assign o_out_d = mem[rd_ptr];

    //------------------------------------------------------------------
    // Assertions
    //------------------------------------------------------------------
    // Pointer consistency: when empty, pointers should be equal
    // (This is a sanity check, not a strict requirement)
    a_empty_ptr_check: assert property (@(posedge clk) disable iff (!rst_n)
        empty |-> (wr_ptr == rd_ptr)
    ) else $error("circular_fifo_almost_full: Pointers inconsistent when empty");

    // No push when every slot is valid (physical depth full; not the same as o_full threshold)
    a_no_push_at_depth_full: assert property (@(posedge clk) disable iff (!rst_n)
        !i_in_v || $isunknown(valid) || !(&valid)
    ) else $error(
        "circular_fifo_almost_full: i_in_v while all valid[] slots full at time %t", $time
    );

endmodule
