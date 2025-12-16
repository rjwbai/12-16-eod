`timescale 1ns / 1ns
import buffer_pkgs::*;

module rename_module #(
    parameter type D = decode_t,
    parameter type R = rename_t,
    parameter int  ROB_DEPTH = 16
) (
    input  logic clk_i,
    input  logic rst_i,

    // From decode
    input  D     data_i,
    input  logic valid_prod_i,
    output logic ready_prod_o,

    // To dispatch
    output R     data_o,
    input  logic ready_cons_i,
    output logic valid_cons_o,

    // -------------------------
    // FROM COMMIT (RETIRE)
    // -------------------------
    input  logic              commit_free_valid_i,
    input  logic [PREG_W-1:0] commit_free_preg_i,

    // -------------------------
    // GLOBAL RECOVERY INPUTS
    // -------------------------
    input  logic                     rat_recover_i,
    input  logic [32*PREG_W-1:0]      rat_recover_map_i,

    input  logic                     fl_recover_i,
    input  logic [PREG_W-1:0]         fl_recover_head_i,
    input  logic [PREG_W-1:0]         fl_recover_tail_i,
    input  logic [$clog2(PREGS):0]    fl_recover_free_count_i,

    // -------------------------
    // CHECKPOINT OUTPUTS -> controller
    // -------------------------
    output logic                     ratfl_chkpt_we_o,
    output logic [3:0]               ratfl_chkpt_tag_o,
    output logic [32*PREG_W-1:0]     ratfl_chkpt_rat_map_o,
    output logic [PREG_W-1:0]        ratfl_chkpt_fl_head_o,
    output logic [PREG_W-1:0]        ratfl_chkpt_fl_tail_o,
    output logic [$clog2(PREGS):0]   ratfl_chkpt_fl_free_count_o
);

    localparam int AREG    = 32;
    localparam int PHYSREG = PREGS;
    localparam int COUNT_W = $clog2(PHYSREG) + 1;

    // RAT (speculative)
    logic [PREG_W-1:0] rat [0:AREG-1];
    integer i;

    // Free list
    logic               alloc_en, alloc_done;
    logic [PREG_W-1:0]  alloc_index;
    logic               freelist_empty;
    logic [COUNT_W-1:0] freelist_free_count;

    logic [PREG_W-1:0]  fl_chkpt_head;
    logic [PREG_W-1:0]  fl_chkpt_tail;
    logic [COUNT_W-1:0] fl_chkpt_free_count;

    // Control
    logic instr_fire;
    logic needs_dest;

    logic [PREG_W-1:0] rs1_phys, rs2_phys;
    logic [PREG_W-1:0] rd_old_phys, rd_new_phys;

    assign needs_dest = data_i.rdUsed && (data_i.rd != 5'd0);

    // Accept if consumer ready and (dest not needed OR freelist not empty)
    assign ready_prod_o = ready_cons_i && (!needs_dest || !freelist_empty);
    assign instr_fire   = valid_prod_i && ready_prod_o;

    assign alloc_en     = instr_fire && needs_dest;
    assign valid_cons_o = instr_fire;

    // ------------------------------------------------------------
    // Free list FIFO
    // ------------------------------------------------------------
    free_list_fifo #(
        .PHYSREG(PHYSREG),
        .AREG   (AREG)
    ) freelist (
        .clk_i                (clk_i),
        .rst_i                (rst_i),

        // allocate interface
        .alloc_en_i           (alloc_en),
        .alloc_done_o         (alloc_done),
        .alloc_index_o        (alloc_index),

        // free interface (FROM COMMIT)
        .free_en_i            (commit_free_valid_i),
        .free_index_i         (commit_free_preg_i),

        // status
        .empty_o              (freelist_empty),
        .free_count           (freelist_free_count),

        // checkpoint
        .chkpt_head_o         (fl_chkpt_head),
        .chkpt_tail_o         (fl_chkpt_tail),
        .chkpt_free_count_o   (fl_chkpt_free_count),

        // recover
        .recover_head_i       (fl_recover_head_i),
        .recover_tail_i       (fl_recover_tail_i),
        .recover_free_count_i (fl_recover_free_count_i),
        .recover_i            (fl_recover_i)
    );

    // RAT lookup
    always_comb begin
        rs1_phys    = rat[data_i.rs1];
        rs2_phys    = rat[data_i.rs2];
        rd_old_phys = rat[data_i.rd];
        rd_new_phys = alloc_index;
    end

    // RAT update / restore
    // NOTE: This rat[] is speculative. DO NOT update it from commit.
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (i=0; i<AREG; i++) begin
                rat[i] <= PREG_W'(i);
            end
        end else if (rat_recover_i) begin
            for (i=0; i<AREG; i++) begin
                rat[i] <= rat_recover_map_i[i*PREG_W +: PREG_W];
            end
        end else begin
            if (instr_fire && needs_dest) begin
                rat[data_i.rd] <= rd_new_phys;
            end
        end
    end

    // Build output (rename_t)
    always_comb begin
        data_o = '0;

        if (instr_fire) begin
            data_o.rs1    = rs1_phys;
            data_o.rs2    = rs2_phys;
            data_o.old_rd = rd_old_phys;
            data_o.rd     = needs_dest ? rd_new_phys : '0;

            // Only provide arch_rd when the instruction has a dest
            data_o.arch_rd = needs_dest ? data_i.rd : 5'd0;

            data_o.pc       = data_i.pc;
            data_o.imm      = data_i.imm;
            data_o.imm_used = data_i.immUsed;
            data_o.rd_used  = data_i.rdUsed;
            data_o.aluop    = data_i.aluop;
            data_o.funcU    = data_i.funcU;
            data_o.jump     = data_i.jump;
            data_o.branch   = data_i.branch;

            data_o.load_store_type = {data_i.loadStore, data_i.lsType};
            data_o.speculative     = data_i.spec;
            data_o.opcode          = data_i.opcode;
        end
    end

    // ------------------------------------------------------------
    // CHECKPOINT OUTPUTS (branch)
    // ------------------------------------------------------------
    always_comb begin
        ratfl_chkpt_we_o            = 1'b0;
        ratfl_chkpt_tag_o           = '0;
        ratfl_chkpt_rat_map_o       = '0;

        ratfl_chkpt_fl_head_o       = fl_chkpt_head;
        ratfl_chkpt_fl_tail_o       = fl_chkpt_tail;
        ratfl_chkpt_fl_free_count_o = fl_chkpt_free_count;

        if (instr_fire && data_i.branch) begin
            ratfl_chkpt_we_o  = 1'b1;

            for (int k=0; k<AREG; k++) begin
                ratfl_chkpt_rat_map_o[k*PREG_W +: PREG_W] = rat[k];
            end
        end
    end

endmodule
