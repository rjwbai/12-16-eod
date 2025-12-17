`timescale 1ns / 1ns

module free_list_fifo #(
    parameter int PHYSREG = 128,
    parameter int AREG    = 32
) (
    input  logic clk_i,
    input  logic rst_i,

    // ------------------------------------------------------------
    // allocate interface (pop)
    // ------------------------------------------------------------
    input  logic alloc_en_i,
    output logic alloc_done_o,
    output logic [$clog2(PHYSREG)-1:0] alloc_index_o,

    // ------------------------------------------------------------
    // free interface (push)
    // ------------------------------------------------------------
    input  logic free_en_i,
    input  logic [$clog2(PHYSREG)-1:0] free_index_i,

    // ------------------------------------------------------------
    // status
    // ------------------------------------------------------------
    output logic empty_o,
    output logic [$clog2(PHYSREG):0] free_count,

    // ------------------------------------------------------------
    // speculation checkpoint (FIFO pointers + count)
    // ------------------------------------------------------------
    output logic [$clog2(PHYSREG)-1:0] chkpt_head_o,
    output logic [$clog2(PHYSREG)-1:0] chkpt_tail_o,
    output logic [$clog2(PHYSREG):0]   chkpt_free_count_o,

    // ------------------------------------------------------------
    // recover (FIFO pointers + count)
    // ------------------------------------------------------------
    input  logic [$clog2(PHYSREG)-1:0] recover_head_i,
    input  logic [$clog2(PHYSREG)-1:0] recover_tail_i,
    input  logic [$clog2(PHYSREG):0]   recover_free_count_i,
    input  logic                       recover_i
);

    localparam int PREG_W     = $clog2(PHYSREG);
    localparam int FIFO_DEPTH = (PHYSREG - AREG);

    // Depth sanity
    initial begin
        if (FIFO_DEPTH <= 0) begin
            $fatal(1, "free_list_fifo: PHYSREG (%0d) must be > AREG (%0d)", PHYSREG, AREG);
        end
    end

    localparam int PTR_W = (FIFO_DEPTH <= 1) ? 1 : $clog2(FIFO_DEPTH);

    // FIFO memory holds physreg IDs that are currently free
    logic [PREG_W-1:0] fifo_mem [0:FIFO_DEPTH-1];

    logic [PTR_W-1:0] head_q, head_d;
    logic [PTR_W-1:0] tail_q, tail_d;
    logic [$clog2(PHYSREG):0] count_q, count_d;

    // ------------------------------------------------------------
    // Handshakes
    // ------------------------------------------------------------
    wire can_alloc = (count_q != 0);
    wire can_free  = (count_q != FIFO_DEPTH); // full check (optional but good)

    wire do_alloc  = alloc_en_i && can_alloc;
    wire do_free   = free_en_i  && can_free;

    assign alloc_done_o = do_alloc;
    assign empty_o      = (count_q == 0);
    assign free_count   = count_q;

    // peek at head (combinational)
    // Note: if empty, alloc_index_o is don't-care
    always_comb begin
        alloc_index_o = fifo_mem[head_q];
    end

    // ------------------------------------------------------------
    // Checkpoint signals
    // ------------------------------------------------------------
    // These are FIFO pointers (not physreg numbers)
    assign chkpt_head_o       = PREG_W'(head_q);
    assign chkpt_tail_o       = PREG_W'(tail_q);
    assign chkpt_free_count_o = count_q;

    // ------------------------------------------------------------
    // Next-state logic
    // ------------------------------------------------------------
    function automatic [PTR_W-1:0] ptr_inc(input [PTR_W-1:0] p);
        if (FIFO_DEPTH == 1) ptr_inc = '0;
        else ptr_inc = (p == PTR_W'(FIFO_DEPTH-1)) ? '0 : (p + 1'b1);
    endfunction

    always_comb begin
        head_d  = head_q;
        tail_d  = tail_q;
        count_d = count_q;

        if (do_alloc) head_d = ptr_inc(head_q);
        if (do_free)  tail_d = ptr_inc(tail_q);

        unique case ({do_free, do_alloc})
            2'b01: count_d = count_q - 1; // alloc only
            2'b10: count_d = count_q + 1; // free only
            2'b11: count_d = count_q;     // both
            default: ;
        endcase
    end

    // ------------------------------------------------------------
    // Sequential state + memory update
    // ------------------------------------------------------------
    integer k;
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            // Initialize free list with physregs AREG..PHYSREG-1
            for (k = 0; k < FIFO_DEPTH; k++) begin
                fifo_mem[k] <= PREG_W'(AREG + k);
            end
            head_q  <= '0;
            tail_q  <= '0;
            count_q <= FIFO_DEPTH;
        end else if (recover_i) begin
            // Restore pointers/count. Memory contents must match your checkpoint scheme.
            head_q  <= recover_head_i[PTR_W-1:0];
            tail_q  <= recover_tail_i[PTR_W-1:0];
            count_q <= recover_free_count_i;
        end else begin
            // Push freed physreg into FIFO
            if (do_free) begin
                fifo_mem[tail_q] <= free_index_i[PREG_W-1:0];
            end

            head_q  <= head_d;
            tail_q  <= tail_d;
            count_q <= count_d;
        end
    end

endmodule