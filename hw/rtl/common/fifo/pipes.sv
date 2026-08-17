module pipes #(
    parameter int STAGES = 2,     // Number of pipeline stages
    parameter int W_DATA = 32     // Data width in bits
) (
    input  logic              clk,
    input  logic              rst_n,
    input  logic [W_DATA-1:0] i_data,     // Input data
    output logic [W_DATA-1:0] o_data      // Output data
);

    // Intermediate signals between pipeline stages
    logic [W_DATA-1:0] stage_data [STAGES:0];    // Data signals: [0]=input, [STAGES]=output

    // Connect input and output ports to stage arrays
    assign stage_data[0] = i_data;
    assign o_data = stage_data[STAGES];

    // Generate pipeline stages
    generate
        for (genvar i = 0; i < STAGES; i++) begin : gen_stages
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    stage_data[i+1] <= '0;
                end else begin
                    stage_data[i+1] <= stage_data[i];
                end
            end
        end
    endgenerate

endmodule
