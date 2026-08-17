//========================================================================
// CIRCULAR_FIFO Module - Circular FIFO with FWFT Behavior
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
//========================================================================

module circular_fifo #(
    parameter int DEPTH  = 4,           // FIFO depth (number of entries), must be >= 1
    parameter int W_DATA = 32,          // Data width in bits
    localparam int W_LEVEL = $clog2(DEPTH + 1)
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

    // Fill level (0 .. DEPTH)
    output logic [W_LEVEL-1:0] o_level
);

    // Parameter validation (elaboration-time check)
    initial begin
        assert(DEPTH >= 1) else $fatal(1, "circular_fifo: DEPTH must be >= 1, got %0d", DEPTH);
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
    logic full, empty;
    logic do_write, do_read;

    //------------------------------------------------------------------
    // Full/Empty detection using valid bits
    //------------------------------------------------------------------
    assign empty = ~|valid;
    assign full  = &valid;

    //------------------------------------------------------------------
    // Handshake signals
    //------------------------------------------------------------------
    assign o_in_r  = ~full;
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
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem <= '{default: '0};
        end else if (do_write) begin
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
    // Fill level: +1 on push (do_write only), -1 on pop (do_read only)
    //------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            o_level <= '0;
        else begin
            if (do_write && !do_read)
                o_level <= o_level + 1'b1;
            else if (do_read && !do_write)
                o_level <= o_level - 1'b1;
        end
    end

    //------------------------------------------------------------------
    // Assertions
    //------------------------------------------------------------------
    // Pointer consistency: when empty, pointers should be equal
    // (This is a sanity check, not a strict requirement)
    a_empty_ptr_check: assert property (@(posedge clk) disable iff (!rst_n)
        empty |-> (wr_ptr == rd_ptr)
    ) else $error("circular_fifo: Pointers inconsistent when empty");

endmodule
