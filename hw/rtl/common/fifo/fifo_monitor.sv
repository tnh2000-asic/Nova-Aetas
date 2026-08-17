//========================================================================
// mem_1r1w_fifo - Memory-backed FIFO (1R1W)
//========================================================================
// Connects to mem_1r1w_if; RVHS on input/output with ECC. Input: pipes_rv
// (0 or 1 stage per FLOP_INPUT). Output: circular_fifo (READ_LATENCY+1
// entries, 1-cycle fall-through). Read issue gated by token counter.
// When BYPASS=1 and memory empty with no reads in-flight, input can bypass
// into circular_fifo to reduce fall-through latency.
//========================================================================


module fifo_monitor
  #(
    parameter int MEM_DEPTH    = 256,    // Depth of backing 1R1W memory
    parameter int FLOP_INPUT   = 0,      // 0 or 1: add flop stages to input
    parameter int READ_LATENCY = 1,      // Memory read latency (cycles); must be >= 1
    localparam int CAPACITY    = MEM_DEPTH + READ_LATENCY + 1 + FLOP_INPUT,
    localparam int W_LEVEL     = (CAPACITY > 0) ? $clog2(CAPACITY + 1) : 1
) (
    input  logic                 clk,
    input  logic                 rst_n,

    // Fill level Input (0 .. capacity); capacity = MEM_DEPTH + READ_LATENCY + 1 + FLOP_INPUT
    input logic [W_LEVEL-1:0]   i_level,

    // Clear input
    input logic  i_clr,

    // Fill Level Output
    output logic [W_LEVEL-1:0]   o_level,

    // highwater mark Output
    output logic [W_LEVEL-1:0]   o_highwater
);

    assign o_level = i_level ;  // Just echo out

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_highwater    <= '0;
        end else begin
            if ( i_clr ) begin
                o_highwater    <= '0;
            end
            else begin
                if ( i_level > o_highwater ) begin
                    o_highwater    <= i_level ;
                end
            end
        end
    end

endmodule
