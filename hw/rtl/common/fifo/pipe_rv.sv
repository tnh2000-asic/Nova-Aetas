module pipe_rv #(
    parameter int W_DATA    = 32, // Data width in bits
    parameter int FLOP_FWD  = 1,  // 1: Flop forward data/valid path
    parameter int FLOP_BKWD = 1   // 1: Flop backward enable path
) (
    input  logic              clk,
    input  logic              rst_n,

    // RVHS input ports
    input  logic              i_in_v,     // Input validity
    input  logic [W_DATA-1:0] i_in_d,     // Input data
    output logic              o_in_r,     // Input ready (stall when deasserted)

    // RVHS output ports
    output logic              o_out_v,    // Output validity
    output logic [W_DATA-1:0] o_out_d,    // Output data
    input  logic              i_out_r     // Output ready (stall when deasserted)
);

    // Intermediate signals between FLOP_BKWD and FLOP_FWD sections
    logic              mid_v;
    logic [W_DATA-1:0] mid_d;
    logic              mid_r;

    generate
        //***************************************************************
        // FLOP_BKWD: Optionally flop the backward (i.e. enable/ready) path
        //***************************************************************
        if (FLOP_BKWD != 0) begin : gen_flop_bkwd
            logic skid_empty;
            logic [W_DATA-1:0] skid_q;

            // Flop enable and add "skid buffer" to hold data when output stalls
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    skid_empty <= 1'b1;
                    skid_q <= '0;
                end else begin
                    // Capture input data when skid buffer is empty
                    if (skid_empty) begin
                        skid_q <= i_in_d;
                    end
                    // Flop downstream enable
                    if (mid_v) begin
                        skid_empty <= mid_r;
                    end
                end
            end

            always_comb begin
                // Forward path: mux between input and skid buffer
                mid_v = i_in_v | !skid_empty;
                mid_d = skid_empty ? i_in_d : skid_q;
                // Backward path: stall when no space available in skid buffer
                o_in_r = skid_empty;
            end

        // Otherwise, just wire the in ports to the mid signals
        end else begin : gen_no_flop_bkwd
            assign mid_v  = i_in_v;
            assign mid_d  = i_in_d;
            assign o_in_r = mid_r;
        end

        //***************************************************************
        // FLOP_FWD: Optionally flop the forward (i.e. data and valid) path
        //***************************************************************
        if (FLOP_FWD != 0) begin : gen_flop_fwd
            // Combinational logic on enable
            assign mid_r = (i_out_r || !o_out_v);

            // Output registers
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    o_out_v <= 1'b0;
                    o_out_d <= '0;
                end else if (mid_r) begin
                    o_out_v <= mid_v;
                    o_out_d <= mid_d;
                end
            end

        // Otherwise, just wire mid signals straight to out ports
        end else begin : gen_no_flop_fwd
            assign o_out_v = mid_v;
            assign o_out_d = mid_d;
            assign mid_r   = i_out_r;
        end
    endgenerate

endmodule
