module free_list #(
    parameter int N_PTRS = 1024,
    parameter int N_PARTITIONS = 32  // Configurable number of partitions
) (
    input  logic                            clk,
    input  logic                            rst_n,
    input  logic                            i_alloc_r,    // request is ready (o_alloc_addr will be taken)
    input  logic                            i_free_v,     // return addr is valid
    input  logic  [($clog2(N_PTRS))-1:0]    i_free_addr,  // return address
    output logic                            o_alloc_v,    // o_alloc_addr is valid
    output logic [($clog2(N_PTRS))-1:0]     o_alloc_addr  // free address

);

    // Local parameters
    localparam int PARTITION_SIZE       = N_PTRS / N_PARTITIONS;
    localparam int W_ADDR               = $clog2(N_PTRS);
    localparam int PARTITION_W_ADDR     = $clog2(PARTITION_SIZE);
    // Minimum width of 1 to avoid zero-width vectors when N_PARTITIONS=1
    localparam int PARTITION_SEL_WIDTH  = (N_PARTITIONS > 1) ? $clog2(N_PARTITIONS) : 1;

    // Parameter validation
    initial begin
        if (N_PTRS % N_PARTITIONS != 0) begin
            $error("N_PTRS (%0d) must be evenly divisible by N_PARTITIONS (%0d)",
                   N_PTRS, N_PARTITIONS);
        end
        if (PARTITION_SIZE != N_PTRS / N_PARTITIONS) begin
            $error("PARTITION_SIZE mismatch");
        end
    end

     // Internal signals
    logic [N_PARTITIONS-1:0] partition_has_zero;
    logic [N_PARTITIONS-1:0][PARTITION_W_ADDR-1:0] partition_first_zero_addr;
    logic [PARTITION_SEL_WIDTH-1:0] first_partition_with_zero;
    logic any_partition_has_zero;
    logic [W_ADDR-1:0] first_zero_addr_comb;
    logic zero_found_comb;
    logic [N_PTRS-1:0] free_list;


    // One-hot vectors for set/clear operations
    logic [N_PTRS-1:0] free_onehot;   // One-hot for freeing (clear bit to 0)
    logic [N_PTRS-1:0] alloc_onehot;  // One-hot for allocating (set bit to 1)
    logic 	       advance_alloc;


    //================================================================================
    // Generate one-hot vectors from addressesx
    //================================================================================
    assign advance_alloc = (i_alloc_r | ~o_alloc_v);

    always_comb begin
        free_onehot  = '0;
        alloc_onehot = '0;
        if (i_free_v) begin
            free_onehot[i_free_addr] = 1'b1;
        end
        if (advance_alloc & zero_found_comb) begin
            alloc_onehot[first_zero_addr_comb] = 1'b1;
        end
    end

    //================================================================================
    // Free list management using one-hot set/clear
    //================================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            free_list <= '0;  // Initialize all bits to 0 (all free)
        end else begin
            // Clear bits for free first, then set bits for allocation (alloc has priority)
            // free_onehot clears bit to 0 (free), alloc_onehot sets bit to 1 (used)
            free_list <= (free_list & ~free_onehot) | alloc_onehot;
        end
    end
    // Output registers - using previous cycle's combinational results
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_alloc_v <= 1'b0;
            o_alloc_addr <= '0;
        end else if (advance_alloc) begin
              o_alloc_v <= zero_found_comb;
              o_alloc_addr <= first_zero_addr_comb;
        end
    end


    //================================================================================
    // Partition search logic
    //================================================================================
    always_comb begin
        partition_has_zero = '0;
        partition_first_zero_addr = '0;
        for (int part_idx = 0; part_idx < N_PARTITIONS; part_idx++) begin
            for (int bit_idx = 0; bit_idx < PARTITION_SIZE; bit_idx++) begin
                if (!free_list[part_idx*PARTITION_SIZE + bit_idx] && !partition_has_zero[part_idx]) begin
                    partition_has_zero[part_idx] = 1'b1;
                    partition_first_zero_addr[part_idx] = bit_idx[PARTITION_W_ADDR-1:0];
                end
            end
        end
    end

    //================================================================================
    // Find first partition that has a zero
    //================================================================================
    always_comb begin
        any_partition_has_zero = 1'b0;
        first_partition_with_zero = '0;
        for (int part_idx = 0; part_idx < N_PARTITIONS; part_idx++) begin
            if (partition_has_zero[part_idx] && !any_partition_has_zero) begin
                any_partition_has_zero = 1'b1;
                first_partition_with_zero = part_idx[PARTITION_SEL_WIDTH-1:0];
            end
        end
    end

    // Combine partition selection with local address to get global address
    always_comb begin
        if (any_partition_has_zero) begin
            first_zero_addr_comb = {first_partition_with_zero, partition_first_zero_addr[first_partition_with_zero]};
            zero_found_comb = 1'b1;
        end else begin
            first_zero_addr_comb = '0;
            zero_found_comb = 1'b0;
        end
    end

    //================================================================================
    // Assertions
    //================================================================================
    // Ensure we never attempt to free and allocate the same address in the same cycle
    // using the combinational selected address.
`ifdef TEMP_REMOVE_FREE_LIST_ASSERT //Cannot enable until all X's are addressed at every instance
`ifndef ASSERTS_OFF
    property p_no_same_addr_update_comb;
        @(posedge clk) disable iff (!rst_n)
            !(i_free_v && i_alloc_r && zero_found_comb && (first_zero_addr_comb == i_free_addr));
    endproperty
    assert property (p_no_same_addr_update_comb)
        else $error("Return and allocation target same address in same cycle (comb).");

    // Ensure we do not free an address that is already free (bit == 0)
    property p_no_freeing_already_free_addr;
        @(posedge clk) disable iff (!rst_n)
            !(i_free_v && (free_list[i_free_addr] == 1'b0));
    endproperty
    assert property (p_no_freeing_already_free_addr)
        else $error("Attempt to free an already-free address: %0d", i_free_addr);
`endif // ASSERTS_OFF
`endif // TEMP_REMOVE

endmodule
