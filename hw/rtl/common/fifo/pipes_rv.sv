module pipes_rv #(
    parameter int STAGES = 2,             // Number of pipeline stages
    parameter int W_DATA = 32,            // Data width in bits
    parameter int FLOP_FWD_LAST      = 1, // 1: Flop data/valid path on last stage
    parameter int FLOP_BKWD_FIRST    = 1, // 1: Flop enable path on first stage
    parameter int FLOP_FWD_INTERNAL  = 1, // 1: Flop data/valid path on internal stages
    parameter int FLOP_BKWD_INTERNAL = 1  // 1: Flop enable path on internal stages
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

    // Intermediate signals between pipeline stages
    logic              stage_v [STAGES:0];   // Valid signals: [0]=input, [STAGES]=output
    logic [W_DATA-1:0] stage_d [STAGES:0];   // Data signals: [0]=input, [STAGES]=output
    logic              stage_r [STAGES:0];   // Ready signals: [0]=input, [STAGES]=output

    // Connect input and output ports to stage arrays
    assign stage_v[0] = i_in_v;
    assign stage_d[0] = i_in_d;
    assign o_in_r = stage_r[0];

    assign o_out_v = stage_v[STAGES];
    assign o_out_d = stage_d[STAGES];
    assign stage_r[STAGES] = i_out_r;

    // Instantiate pipeline stages
    generate
        for (genvar i = 0; i < STAGES; i++) begin : gen_stages
            pipe_rv #(
                .W_DATA   (W_DATA),
                .FLOP_FWD (i==STAGES-1 ? FLOP_FWD_LAST   : FLOP_FWD_INTERNAL),
                .FLOP_BKWD(i==0        ? FLOP_BKWD_FIRST : FLOP_BKWD_INTERNAL)
            ) pipe_inst (
                .clk(clk),
                .rst_n(rst_n),

                .i_in_v(stage_v[i]),
                .i_in_d(stage_d[i]),
                .o_in_r(stage_r[i]),

                .o_out_v(stage_v[i+1]),
                .o_out_d(stage_d[i+1]),
                .i_out_r(stage_r[i+1])
            );
        end
    endgenerate

endmodule
