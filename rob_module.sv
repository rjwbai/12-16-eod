`timescale 1ns/1ps
import buffer_pkgs::*;

module rob_module #(
    parameter int DEPTH = 16,
    parameter int PREGS = buffer_pkgs::PREGS
)(
    input  logic clk_i,
    input  logic rst_i,

    // payload from dispatch (rename_t)
    input  rename_t data_i,

    // allocation
    input  logic                 alloc_valid_i,
    output logic                 alloc_ready_o,

    input  logic                 alloc_has_dest_i,
    input  logic [4:0]           alloc_arch_rd_i,
    input  logic [PREG_W-1:0]    alloc_dest_preg_i,
    input  logic [PREG_W-1:0]    alloc_old_preg_i,
    input  logic                 alloc_is_branch_i,
    input  logic [31:0]          alloc_pc_i,
    output logic [ROB_PTR_W-1:0] alloc_rob_index_o,

    // checkpoint state out
    output logic [ROB_PTR_W-1:0] chkpt_tail_o,
    output logic [ROB_PTR_W  :0] chkpt_used_count_o,

    // global recover (restore head/tail/used from controller)
    input  logic                 recover_i,
    input  logic [ROB_PTR_W-1:0] recover_tail_i,
    input  logic [ROB_PTR_W  :0] recover_used_count_i,

    // completion (from WB)
    input  logic                 complete_valid_i,
    input  logic [ROB_PTR_W-1:0] complete_rob_index_i,
    input  logic                 complete_mispredict_i,

    // commit interface
    input  logic                 commit_ready_i,
    output logic                 commit_valid_o,
    output rob_entry_t           commit_entry_o,
    output logic [ROB_PTR_W-1:0] commit_rob_index_o,

    // [CHG] pending producers bitmap (for scoreboard restore)
    output logic [PREGS-1:0]     busy_pregs_o
);

    // ----------------------------
    // ROB storage
    // ----------------------------
    rob_entry_t rob_mem [0:DEPTH-1];

    logic [ROB_PTR_W-1:0] head_q, tail_q;
    logic [ROB_PTR_W  :0] used_q;

    // full/empty
    wire rob_full  = (used_q == DEPTH[ROB_PTR_W:0]);
    wire rob_empty = (used_q == '0);

    assign alloc_ready_o     = !recover_i && !rob_full;
    assign alloc_rob_index_o = tail_q;

    assign chkpt_tail_o        = tail_q;
    assign chkpt_used_count_o  = used_q;

    // ----------------------------
    // commit combinational view
    // commit_valid only when head is valid AND completed
    // ----------------------------
    rob_entry_t head_entry;

    always_comb begin
        head_entry = rob_mem[head_q];

        commit_valid_o     = (!recover_i) && head_entry.valid && head_entry.completed;
        commit_entry_o     = head_entry;
        commit_rob_index_o = head_q;
    end

    wire alloc_fire  = alloc_valid_i && alloc_ready_o;
    wire commit_fire = commit_valid_o && commit_ready_i;

    // ----------------------------
    // COMPLETE: mark ROB entry completed
    // ----------------------------
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            // clear ROB
            for (int i = 0; i < DEPTH; i++) begin
                rob_mem[i] <= '0;
            end
            head_q <= '0;
            tail_q <= '0;
            used_q <= '0;
        end else if (recover_i) begin
            // On recover, we DON'T need to wipe rob_mem for correctness as long as
            // head/tail/used are restored consistently, but it's fine to keep it simple.
            head_q <= recover_tail_i;         // controller provides correct "tail", but you also need head.
            // NOTE: if your controller only supplies tail+used, head is (tail-used) modulo DEPTH.
            tail_q <= recover_tail_i;
            used_q <= recover_used_count_i;

            // Optional: recompute head from tail-used (safer than trusting recover_tail for head)
            head_q <= recover_tail_i - recover_used_count_i[ROB_PTR_W-1:0];
        end else begin
            // 1) mark completion
            if (complete_valid_i) begin
                rob_mem[complete_rob_index_i].completed <= 1'b1;
                // mispredict bit is stored elsewhere in your design; ROB entry type doesn't include it.
                // If you need it here, extend rob_entry_t.
            end

            // 2) allocate new entry
            if (alloc_fire) begin
                rob_mem[tail_q].valid     <= 1'b1;
                rob_mem[tail_q].completed <= 1'b0;
                rob_mem[tail_q].has_dest  <= alloc_has_dest_i;

                rob_mem[tail_q].arch_rd   <= alloc_arch_rd_i;
                rob_mem[tail_q].dest_preg <= alloc_dest_preg_i;
                rob_mem[tail_q].old_preg  <= alloc_old_preg_i;

                rob_mem[tail_q].is_branch <= alloc_is_branch_i;
                rob_mem[tail_q].pc        <= alloc_pc_i;

                tail_q <= tail_q + ROB_PTR_W'(1);
            end

            // 3) commit/pop head **ONLY on commit_fire**
            if (commit_fire) begin
                rob_mem[head_q].valid <= 1'b0;   // free slot
                head_q <= head_q + ROB_PTR_W'(1);
            end

            // 4) used count update (critical!)
            unique case ({alloc_fire, commit_fire})
                2'b10: used_q <= used_q + (ROB_PTR_W+1)'(1);
                2'b01: used_q <= used_q - (ROB_PTR_W+1)'(1);
                default: used_q <= used_q; // 00 or 11 => unchanged
            endcase
        end
    end

    // ----------------------------
    // busy_pregs_o: pending producers (valid && has_dest && !completed)
    // ----------------------------
    always_comb begin
        busy_pregs_o = '0;
        for (int i = 0; i < DEPTH; i++) begin
            if (rob_mem[i].valid && rob_mem[i].has_dest && !rob_mem[i].completed) begin
                if (rob_mem[i].dest_preg != '0)
                    busy_pregs_o[rob_mem[i].dest_preg] = 1'b1;
            end
        end
    end

endmodule
