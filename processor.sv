//`timescale 1ns / 1ps
//import buffer_pkgs::*;

//module processor #(
//    parameter type F  = fetch_t,
//    parameter type D  = decode_t,
//    parameter type R  = rename_t,
//    parameter type DI = rs_entry_t,
//    parameter type AE = alu_entry_t,
//    parameter type AO = alu_out_t,
//    parameter type BE = branch_entry_t,
//    parameter type BO = branch_out_t,
//    parameter type LE = lsu_entry_t,
//    parameter type LO = lsu_out_t
//) (
//    input  logic clk_i,
//    input  logic reset_i,

//    // ============================================================
//    // Existing debug: DISPATCH -> EXECUTE issue interface (KEEP)
//    // ============================================================
//    output AE    data_alu_dispatch_o,
//    output logic valid_alu_dispatch_o,
//    input  logic ready_alu_dispatch_i,

//    output BE    data_br_dispatch_o,
//    output logic valid_br_dispatch_o,
//    input  logic ready_br_dispatch_i,

//    output LE    data_lsu_dispatch_o,
//    output logic valid_lsu_dispatch_o,
//    input  logic ready_lsu_dispatch_i,

//    // ---- DEBUG: Rename → Dispatch interface ----
//    output logic dbg_rename_valid_o,
//    output logic dbg_rename_ready_o,
//    output R     dbg_rename_data_o,

//    // ---- DEBUG: Dispatch → Issue events ----
//    output logic dbg_alu_issue_fire_o,
//    output logic dbg_lsu_issue_fire_o,
//    output logic dbg_br_issue_fire_o,

//    // ---- DEBUG: Dispatch internal pipe/insert events (not available w/ your dispatch header)
//    output logic dbg_disp_pipe_valid_o,
//    output logic dbg_disp_pipe_ready_o,
//    output logic dbg_disp_pipe_fire_o,

//    output logic dbg_disp_alu_insert_o,
//    output logic dbg_disp_mem_insert_o,
//    output logic dbg_disp_br_insert_o,

//    output logic dbg_disp_dispatch_fire_o,

//    output logic [3:0]  dbg_disp_tag_o,
//    output logic [31:0] dbg_disp_pc_o,
//    output logic [3:0]  dbg_disp_fu_o,

//    // ============================================================
//    // DEBUG writeback outputs (probe from TB)
//    // ============================================================
//    output logic       dbg_wb_valid_o,
//    output logic       dbg_wb_ready_o,
//    output wb_packet_t dbg_wb_packet_o,

//    output logic              dbg_prf_wb_en_o,
//    output logic [PREG_W-1:0] dbg_prf_wb_addr_o,
//    output logic [31:0]       dbg_prf_wb_data_o,

//    output logic              dbg_cdb_valid_o,
//    output logic [PREG_W-1:0] dbg_cdb_tag_o,
//    output logic [31:0]       dbg_cdb_data_o,

//    output logic                  dbg_rob_complete_valid_o,
//    output logic [$clog2(16)-1:0] dbg_rob_complete_idx_o,
//    output logic                  dbg_rob_complete_mispredict_o,

//    output logic        dbg_recover_o,
//    output logic        dbg_redirect_valid_o,
//    output logic [31:0] dbg_redirect_pc_o,

//    // COMMIT debug outputs
//    output logic              dbg_commit_valid_o,
//    output logic [4:0]        dbg_commit_arch_rd_o,
//    output logic [PREG_W-1:0] dbg_commit_dest_preg_o,
//    output logic [31:0]       dbg_commit_pc_o
//);

//    // ============================================================
//    // Fetch <-> Decode
//    // ============================================================
//    logic fetch_decode_valid, fetch_decode_ready;
//    F     fetch_o;

//    // ============================================================
//    // Decode <-> Rename
//    // ============================================================
//    logic decode_rename_valid, decode_rename_ready;
//    D     decode_o;

//    // ============================================================
//    // Rename <-> Dispatch
//    // ============================================================
//    logic rename_dispatch_valid, rename_dispatch_ready;
//    R     rename_o;

//    // ============================================================
//    // Dispatch -> Execute issue channels (internal)
//    // ============================================================
//    logic alu_issue_valid, alu_issue_ready;
//    logic br_issue_valid,  br_issue_ready;
//    logic lsu_issue_valid, lsu_issue_ready;

//    AE alu_issue_data;
//    BE br_issue_data;
//    LE lsu_issue_data;

//    // Mirror internal issue channels onto existing probe ports
//    assign valid_alu_dispatch_o = alu_issue_valid;
//    assign data_alu_dispatch_o  = alu_issue_data;
//    assign valid_br_dispatch_o  = br_issue_valid;
//    assign data_br_dispatch_o   = br_issue_data;
//    assign valid_lsu_dispatch_o = lsu_issue_valid;
//    assign data_lsu_dispatch_o  = lsu_issue_data;

//    // Probe-only ready inputs (ignored by core datapath)
//    /* verilator lint_off UNUSED */
//    wire _unused_ready_alu = ready_alu_dispatch_i;
//    wire _unused_ready_br  = ready_br_dispatch_i;
//    wire _unused_ready_lsu = ready_lsu_dispatch_i;
//    /* verilator lint_on UNUSED */

//    // ============================================================
//    // Execute outputs
//    // ============================================================
//    logic alu_wb_valid; AO alu_wb_data; logic alu_wb_ready;
//    logic br_wb_valid;  BO br_wb_data;  logic br_wb_ready;
//    logic lsu_wb_valid; LO lsu_wb_data; logic lsu_wb_ready;

//    // ============================================================
//    // Writeback arbitration
//    // ============================================================
//    logic       wb_valid;
//    wb_packet_t wb_packet;
//    logic       wb_ready;

//    typedef enum logic [1:0] {SRC_ALU=2'd0, SRC_LSU=2'd1, SRC_BR=2'd2} wb_src_t;
//    wb_src_t wb_src_sel;

//    logic alu_v, lsu_v, br_v;

//    // ============================================================
//    // WB -> Dispatch/PRF/ROB wires (from writeback_module)
//    // ============================================================
//    logic              prf_wb_en_i;
//    logic [PREG_W-1:0] prf_wb_addr_i;
//    logic [31:0]       prf_wb_data_i;

//    logic              cdb_valid_i;
//    logic [PREG_W-1:0] cdb_tag_i;
//    logic [31:0]       cdb_data_i;

//    logic                  rob_complete_valid_i;
//    logic [$clog2(16)-1:0] rob_complete_idx_i;
//    logic                  rob_complete_mispredict_i;

//    logic recover_i;
//    logic redirect_valid_i;
//    logic [31:0] redirect_pc_i;

//    // ============================================================
//    // ROB checkpoint wires (dispatch -> writeback)
//    // ============================================================
//    logic                  rob_chkpt_we;
//    logic [3:0]            rob_chkpt_tag;
//    logic [$clog2(16)-1:0] rob_chkpt_tail;
//    logic [$clog2(16)  :0] rob_chkpt_used;

//    // ============================================================
//    // ROB -> Commit stage interface (from dispatch)
//    // ============================================================
//    logic                         rob_commit_valid;
//    rob_entry_t                    rob_commit_entry;
//    logic [$clog2(16)-1:0]         rob_commit_index;
//    logic                         rob_commit_ready;

//    // ============================================================
//    // Commit outputs
//    // ============================================================
//    logic                         commit_map_valid;
//    logic [4:0]                   commit_arch_rd;
//    logic [PREG_W-1:0]            commit_dest_preg;

//    logic                         commit_free_valid;
//    logic [PREG_W-1:0]            commit_free_preg;

//    // ============================================================
//    // Recovery payloads (from writeback -> dispatch)
//    // ============================================================
//    logic                         rob_recover_o, rat_recover_o, fl_recover_o;
//    logic [$clog2(16)-1:0]        rob_recover_tail_o;
//    logic [$clog2(16)  :0]        rob_recover_used_o;

//    // (Rename recovery not wired in your rename yet; leave tied off there)
//    logic [32*PREG_W-1:0]         rat_recover_map_o;
//    logic [PREG_W-1:0]            fl_recover_head_o, fl_recover_tail_o;
//    logic [$clog2(PREGS):0]       fl_recover_free_count_o;

//    // ============================================================
//    // FETCH
//    // ============================================================
//    fetch #(.DEPTH(64), .F(F)) my_fetch (
//        .clk_i(clk_i),
//        .reset_i(reset_i),

//        .valid_cons_o(fetch_decode_valid),
//        .ready_cons_i(fetch_decode_ready),
//        .data_o(fetch_o),

//        .redirect_i   (redirect_valid_i),
//        .redirect_pc_i(redirect_pc_i)
//    );

//    // ============================================================
//    // DECODE
//    // ============================================================
//    decode_module #(.F(F), .D(D)) my_decode (
//        .clk_i(clk_i),
//        .reset_i(reset_i),

//        .valid_prod_i(fetch_decode_valid),
//        .ready_prod_o(fetch_decode_ready),
//        .data_i(fetch_o),

//        .ready_cons_i(decode_rename_ready),
//        .valid_cons_o(decode_rename_valid),
//        .data_o(decode_o)
//    );

//    // ============================================================
//    // COMMIT
//    // ============================================================
//    commit_stage #(.DEPTH(16)) u_commit (
//        .clk_i(clk_i),
//        .rst_i(reset_i),

//        .rob_commit_valid_i (rob_commit_valid),
//        .rob_commit_entry_i (rob_commit_entry),
//        .rob_commit_index_i (rob_commit_index),
//        .rob_commit_ready_o (rob_commit_ready),

//        .commit_map_valid_o (commit_map_valid),
//        .commit_arch_rd_o   (commit_arch_rd),
//        .commit_dest_preg_o (commit_dest_preg),

//        .commit_free_valid_o(commit_free_valid),
//        .commit_free_preg_o (commit_free_preg)
//    );

//    // ============================================================
//    // RENAME (with commit free inputs)
//    // ============================================================
//    rename_module #(.D(D), .R(R), .ROB_DEPTH(16)) my_rename (
//        .clk_i(clk_i),
//        .rst_i(reset_i),

//        .data_i(decode_o),
//        .valid_prod_i(decode_rename_valid),
//        .ready_prod_o(decode_rename_ready),

//        .data_o(rename_o),
//        .ready_cons_i(rename_dispatch_ready),
//        .valid_cons_o(rename_dispatch_valid),

//        // FROM COMMIT: free old preg
//        .commit_free_valid_i(commit_free_valid),
//        .commit_free_preg_i (commit_free_preg),

//        // recovery interface (tied off for now)
//        .rat_recover_i(),
//        .rat_recover_map_i(),
//        .fl_recover_i(),
//        .fl_recover_head_i(),
//        .fl_recover_tail_i(),
//        .fl_recover_free_count_i(),

//        .ratfl_chkpt_we_o(),
//        .ratfl_chkpt_tag_o(),
//        .ratfl_chkpt_rat_map_o(),
//        .ratfl_chkpt_fl_head_o(),
//        .ratfl_chkpt_fl_tail_o(),
//        .ratfl_chkpt_fl_free_count_o()
//    );

//    // Rename debug
//    assign dbg_rename_valid_o = rename_dispatch_valid;
//    assign dbg_rename_ready_o = rename_dispatch_ready;
//    assign dbg_rename_data_o  = rename_o;

//    // ============================================================
//    // DISPATCH (matches your exact dispatch_module port list)
//    // ============================================================
//    dispatch_module #(.R(R), .DI(DI)) u_dispatch (
//        .clk_i(clk_i),
//        .rst_i(reset_i),

//        .recover_i(recover_i),

//        .data_i(rename_o),
//        .valid_prod_i(rename_dispatch_valid),
//        .ready_prod_o(rename_dispatch_ready),

//        .cdb_valid_i(cdb_valid_i),
//        .cdb_tag_i  (cdb_tag_i),

//        .prf_wb_en_i  (prf_wb_en_i),
//        .prf_wb_addr_i(prf_wb_addr_i),
//        .prf_wb_data_i(prf_wb_data_i),

//        .rob_complete_valid_i     (rob_complete_valid_i),
//        .rob_complete_idx_i       (rob_complete_idx_i),
//        .rob_complete_mispredict_i(rob_complete_mispredict_i),

//        .rob_recover_tail_i(rob_recover_tail_o),
//        .rob_recover_used_i(rob_recover_used_o),

//        .rob_chkpt_we_o  (rob_chkpt_we),
//        .rob_chkpt_tag_o (rob_chkpt_tag),
//        .rob_chkpt_tail_o(rob_chkpt_tail),
//        .rob_chkpt_used_o(rob_chkpt_used),

//        .rob_commit_ready_i(rob_commit_ready),
//        .rob_commit_valid_o(rob_commit_valid),
//        .rob_commit_entry_o(rob_commit_entry),
//        .rob_commit_index_o(rob_commit_index),

//        .alu_issue_valid_o(alu_issue_valid),
//        .alu_issue_ready_i(alu_issue_ready),
//        .alu_issue_o      (alu_issue_data),

//        .lsu_issue_valid_o(lsu_issue_valid),
//        .lsu_issue_ready_i(lsu_issue_ready),
//        .lsu_issue_o      (lsu_issue_data),

//        .br_issue_valid_o (br_issue_valid),
//        .br_issue_ready_i (br_issue_ready),
//        .br_issue_o       (br_issue_data)
//    );

//    // Issue-fire debug
//    assign dbg_alu_issue_fire_o = alu_issue_valid && alu_issue_ready;
//    assign dbg_lsu_issue_fire_o = lsu_issue_valid && lsu_issue_ready;
//    assign dbg_br_issue_fire_o  = br_issue_valid  && br_issue_ready;

//    // Dispatch debug outputs not available from your dispatch header -> drive safe defaults
//    assign dbg_disp_pipe_valid_o    = 1'b0;
//    assign dbg_disp_pipe_ready_o    = 1'b0;
//    assign dbg_disp_pipe_fire_o     = 1'b0;
//    assign dbg_disp_alu_insert_o    = 1'b0;
//    assign dbg_disp_mem_insert_o    = 1'b0;
//    assign dbg_disp_br_insert_o     = 1'b0;
//    assign dbg_disp_dispatch_fire_o = 1'b0;
//    assign dbg_disp_tag_o           = '0;
//    assign dbg_disp_pc_o            = '0;
//    assign dbg_disp_fu_o            = '0;

//    // ============================================================
//    // EXECUTE
//    // ============================================================
//    execute_module #(.AE(AE), .AO(AO), .BE(BE), .BO(BO), .LE(LE), .LO(LO)) my_execute (
//        .clk_i(clk_i),
//        .reset_i(reset_i),

//        .alu_valid_i(alu_issue_valid),
//        .alu_ready_o(alu_issue_ready),
//        .alu_data_i (alu_issue_data),
//        .alu_flush_i(recover_i),

//        .b_valid_i  (br_issue_valid),
//        .b_ready_o  (br_issue_ready),
//        .b_data_i   (br_issue_data),
//        .b_flush_i  (recover_i),

//        .lsu_valid_i(lsu_issue_valid),
//        .lsu_ready_o(lsu_issue_ready),
//        .lsu_data_i (lsu_issue_data),
//        .lsu_flush_i(recover_i),

//        .asb_cons_ready_i(alu_wb_ready),
//        .asb_cons_valid_o(alu_wb_valid),
//        .asb_cons_data_o (alu_wb_data),

//        .bsb_cons_ready_i(br_wb_ready),
//        .bsb_cons_valid_o(br_wb_valid),
//        .bsb_cons_data_o (br_wb_data),

//        .lsb_cons_ready_i(lsu_wb_ready),
//        .lsb_cons_valid_o(lsu_wb_valid),
//        .lsb_cons_data_o (lsu_wb_data)
//    );

//    // ============================================================
//    // Execute -> WB arbitration (X-safe)
//    // ============================================================
//    always_comb begin
//        alu_v = (alu_wb_valid === 1'b1);
//        lsu_v = (lsu_wb_valid === 1'b1);
//        br_v  = (br_wb_valid  === 1'b1);

//        alu_wb_ready = 1'b0;
//        lsu_wb_ready = 1'b0;
//        br_wb_ready  = 1'b0;

//        if (lsu_v)      wb_src_sel = SRC_LSU;
//        else if (alu_v) wb_src_sel = SRC_ALU;
//        else if (br_v)  wb_src_sel = SRC_BR;
//        else            wb_src_sel = SRC_ALU;

//        wb_valid = (lsu_v || alu_v || br_v);

//        if (wb_ready) begin
//            case (wb_src_sel)
//                SRC_LSU: if (lsu_v) lsu_wb_ready = 1'b1;
//                SRC_ALU: if (alu_v) alu_wb_ready = 1'b1;
//                SRC_BR : if (br_v)  br_wb_ready  = 1'b1;
//                default: ;
//            endcase
//        end

//        wb_packet = '0;

//        unique case (wb_src_sel)
//            SRC_ALU: if (alu_v) begin
//                wb_packet.src_fu    = 2'd0;
//                wb_packet.completed = 1'b1;
//                wb_packet.ROB_tag   = alu_wb_data.ROB_tag;
//                wb_packet.rd_addr   = alu_wb_data.rd_addr;
//                wb_packet.rd_val    = alu_wb_data.rd_val;
//            end

//            SRC_LSU: if (lsu_v) begin
//                wb_packet.src_fu    = 2'd1;
//                wb_packet.completed = 1'b1;
//                wb_packet.ROB_tag   = lsu_wb_data.ROB_tag;
//                wb_packet.rd_addr   = lsu_wb_data.rd_addr;
//                wb_packet.rd_val    = lsu_wb_data.rd_val;
//            end

//            SRC_BR: if (br_v) begin
//                wb_packet.src_fu       = 2'd2;
//                wb_packet.completed    = 1'b1;
//                wb_packet.ROB_tag      = br_wb_data.ROB_tag;
//                wb_packet.rd_addr      = br_wb_data.rd_addr;
//                wb_packet.rd_val       = br_wb_data.rd_val;
//                wb_packet.branch_taken = br_wb_data.branch_taken;
//                wb_packet.dest_addr    = br_wb_data.dest_addr;
//                wb_packet.mispredict   = br_wb_data.mispredict;
//            end
//        endcase
//    end

//    // ============================================================
//    // WRITEBACK
//    // ============================================================
//    writeback_module #(.ROB_DEPTH(16), .AREG(32)) my_wb (
//        .clk_i(clk_i),
//        .rst_i(reset_i),

//        .wb_valid_i (wb_valid),
//        .wb_packet_i(wb_packet),
//        .wb_ready_o (wb_ready),

//        // RAT/FL checkpoints not wired yet
//        .ratfl_chkpt_we_i(1'b0),
//        .ratfl_chkpt_tag_i('0),
//        .ratfl_chkpt_rat_map_i('0),
//        .ratfl_chkpt_fl_head_i('0),
//        .ratfl_chkpt_fl_tail_i('0),
//        .ratfl_chkpt_fl_free_count_i('0),

//        // ROB checkpoints from dispatch (IMPORTANT)
//        .rob_chkpt_we_i  (rob_chkpt_we),
//        .rob_chkpt_tag_i (rob_chkpt_tag),
//        .rob_chkpt_tail_i(rob_chkpt_tail),
//        .rob_chkpt_used_i(rob_chkpt_used),

//        .recover_o(recover_i),
//        .redirect_valid_o(redirect_valid_i),
//        .redirect_pc_o(redirect_pc_i),

//        .rob_recover_o(rob_recover_o),
//        .rob_recover_tail_o(rob_recover_tail_o),
//        .rob_recover_used_o(rob_recover_used_o),

//        .rat_recover_o(rat_recover_o),
//        .rat_recover_map_o(rat_recover_map_o),

//        .fl_recover_o(fl_recover_o),
//        .fl_recover_head_o(fl_recover_head_o),
//        .fl_recover_tail_o(fl_recover_tail_o),
//        .fl_recover_free_count_o(fl_recover_free_count_o),

//        .recover_rob_tag_o(),

//        .prf_wb_en_o  (prf_wb_en_i),
//        .prf_wb_addr_o(prf_wb_addr_i),
//        .prf_wb_data_o(prf_wb_data_i),

//        .cdb_valid_o(cdb_valid_i),
//        .cdb_tag_o  (cdb_tag_i),
//        .cdb_data_o (cdb_data_i),

//        .rob_complete_valid_o     (rob_complete_valid_i),
//        .rob_complete_idx_o       (rob_complete_idx_i),
//        .rob_complete_mispredict_o(rob_complete_mispredict_i)
//    );

//    // ============================================================
//    // TOP-LEVEL DEBUG MIRRORS
//    // ============================================================
//    assign dbg_wb_valid_o   = wb_valid;
//    assign dbg_wb_ready_o   = wb_ready;
//    assign dbg_wb_packet_o  = wb_packet;

//    assign dbg_prf_wb_en_o   = prf_wb_en_i;
//    assign dbg_prf_wb_addr_o = prf_wb_addr_i;
//    assign dbg_prf_wb_data_o = prf_wb_data_i;

//    assign dbg_cdb_valid_o = cdb_valid_i;
//    assign dbg_cdb_tag_o   = cdb_tag_i;
//    assign dbg_cdb_data_o  = cdb_data_i;

//    assign dbg_rob_complete_valid_o      = rob_complete_valid_i;
//    assign dbg_rob_complete_idx_o        = rob_complete_idx_i;
//    assign dbg_rob_complete_mispredict_o = rob_complete_mispredict_i;

//    assign dbg_recover_o        = recover_i;
//    assign dbg_redirect_valid_o = redirect_valid_i;
//    assign dbg_redirect_pc_o    = redirect_pc_i;

//    // Commit debug
//    assign dbg_commit_valid_o     = commit_map_valid;
//    assign dbg_commit_arch_rd_o   = commit_arch_rd;
//    assign dbg_commit_dest_preg_o = commit_dest_preg;
//    assign dbg_commit_pc_o        = rob_commit_entry.pc;

//endmodule

`timescale 1ns / 1ps
import buffer_pkgs::*;

module processor #(
    parameter type F  = fetch_t,
    parameter type D  = decode_t,
    parameter type R  = rename_t,
    parameter type DI = rs_entry_t,
    parameter type AE = alu_entry_t,
    parameter type AO = alu_out_t,
    parameter type BE = branch_entry_t,
    parameter type BO = branch_out_t,
    parameter type LE = lsu_entry_t,
    parameter type LO = lsu_out_t
) (
    input  logic clk_i,
    input  logic reset_i,

    // ============================================================
    // Existing debug: DISPATCH -> EXECUTE issue interface (KEEP)
    // ============================================================
    output AE    data_alu_dispatch_o,
    output logic valid_alu_dispatch_o,
    input  logic ready_alu_dispatch_i,

    output BE    data_br_dispatch_o,
    output logic valid_br_dispatch_o,
    input  logic ready_br_dispatch_i,

    output LE    data_lsu_dispatch_o,
    output logic valid_lsu_dispatch_o,
    input  logic ready_lsu_dispatch_i,

    // ---- DEBUG: Rename → Dispatch interface ----
    output logic dbg_rename_valid_o,
    output logic dbg_rename_ready_o,
    output R     dbg_rename_data_o,

    // ---- DEBUG: Dispatch → Issue events ----
    output logic dbg_alu_issue_fire_o,
    output logic dbg_lsu_issue_fire_o,
    output logic dbg_br_issue_fire_o,

    output logic dbg_disp_pipe_valid_o,
    output logic dbg_disp_pipe_ready_o,
    output logic dbg_disp_pipe_fire_o,

    output logic dbg_disp_alu_insert_o,
    output logic dbg_disp_mem_insert_o,
    output logic dbg_disp_br_insert_o,

    output logic dbg_disp_dispatch_fire_o,

    output logic [3:0]  dbg_disp_tag_o,
    output logic [31:0] dbg_disp_pc_o,
    output logic [3:0]  dbg_disp_fu_o,

    // ============================================================
    // DEBUG writeback outputs (probe from TB)
    // ============================================================
    output logic       dbg_wb_valid_o,
    output logic       dbg_wb_ready_o,
    output wb_packet_t dbg_wb_packet_o,

    output logic              dbg_prf_wb_en_o,
    output logic [PREG_W-1:0] dbg_prf_wb_addr_o,
    output logic [31:0]       dbg_prf_wb_data_o,

    output logic              dbg_cdb_valid_o,
    output logic [PREG_W-1:0] dbg_cdb_tag_o,
    output logic [31:0]       dbg_cdb_data_o,

    output logic                  dbg_rob_complete_valid_o,
    output logic [$clog2(16)-1:0] dbg_rob_complete_idx_o,
    output logic                  dbg_rob_complete_mispredict_o,

    output logic        dbg_recover_o,
    output logic        dbg_redirect_valid_o,
    output logic [31:0] dbg_redirect_pc_o
);

    // -----------------------------
    // Fetch <-> Decode
    // -----------------------------
    logic fetch_decode_valid, fetch_decode_ready;
    F     fetch_o;

    // -----------------------------
    // Decode <-> Rename
    // -----------------------------
    logic decode_rename_valid, decode_rename_ready;
    D     decode_o;

    // -----------------------------
    // Rename <-> Dispatch
    // -----------------------------
    logic rename_dispatch_valid, rename_dispatch_ready;
    R     rename_o;

    // ============================================================
    // Dispatch -> Execute issue channels (internal)
    // ============================================================
    logic alu_issue_valid, alu_issue_ready;  AE alu_issue_data;
    logic br_issue_valid,  br_issue_ready;   BE br_issue_data;
    logic lsu_issue_valid, lsu_issue_ready;  LE lsu_issue_data;

    // Mirror internal issue channels onto existing probe ports
    assign valid_alu_dispatch_o = alu_issue_valid;
    assign data_alu_dispatch_o  = alu_issue_data;
    assign valid_br_dispatch_o  = br_issue_valid;
    assign data_br_dispatch_o   = br_issue_data;
    assign valid_lsu_dispatch_o = lsu_issue_valid;
    assign data_lsu_dispatch_o  = lsu_issue_data;

    // Probe-only ready inputs (ignored in core datapath)
    /* verilator lint_off UNUSED */
    wire _unused_ready_alu = ready_alu_dispatch_i;
    wire _unused_ready_br  = ready_br_dispatch_i;
    wire _unused_ready_lsu = ready_lsu_dispatch_i;
    /* verilator lint_on UNUSED */

    // ============================================================
    // Execute outputs
    // ============================================================
    logic alu_wb_valid; AO alu_wb_data; logic alu_wb_ready;
    logic br_wb_valid;  BO br_wb_data;  logic br_wb_ready;
    logic lsu_wb_valid; LO lsu_wb_data; logic lsu_wb_ready;

    // -----------------------------
    // Fetch
    // -----------------------------
    fetch #(.DEPTH(64), .F(F)) my_fetch (
        .clk_i(clk_i),
        .reset_i(reset_i),

        .valid_cons_o(fetch_decode_valid),
        .ready_cons_i(fetch_decode_ready),
        .data_o(fetch_o),

        .redirect_i(1'b0),
        .redirect_pc_i(32'b0)
    );

    // -----------------------------
    // Decode
    // -----------------------------
    decode_module #(.F(F), .D(D)) my_decode (
        .clk_i(clk_i),
        .reset_i(reset_i),

        .valid_prod_i(fetch_decode_valid),
        .ready_prod_o(fetch_decode_ready),
        .data_i(fetch_o),

        .ready_cons_i(decode_rename_ready),
        .valid_cons_o(decode_rename_valid),
        .data_o(decode_o)
    );

    // ============================================================
    // Commit plumbing
    // ============================================================
    logic                         rob_commit_valid;
    rob_entry_t                    rob_commit_entry;
    logic [$clog2(16)-1:0]         rob_commit_index;
    logic                         rob_commit_ready;

    logic                         commit_map_valid;
    logic [4:0]                   commit_arch_rd;
    logic [PREG_W-1:0]            commit_dest_preg;

    logic                         commit_free_valid;
    logic [PREG_W-1:0]            commit_free_preg;

    commit_stage #(.DEPTH(16)) u_commit (
        .clk_i(clk_i),
        .rst_i(reset_i),

        .rob_commit_valid_i (rob_commit_valid),
        .rob_commit_entry_i (rob_commit_entry),
        .rob_commit_index_i (rob_commit_index),
        .rob_commit_ready_o (rob_commit_ready),

        .commit_map_valid_o (commit_map_valid),
        .commit_arch_rd_o   (commit_arch_rd),
        .commit_dest_preg_o (commit_dest_preg),

        .commit_free_valid_o(commit_free_valid),
        .commit_free_preg_o (commit_free_preg)
    );

    // -----------------------------
    // Rename (with commit free inputs)
    // -----------------------------
    rename_module #(.D(D), .R(R), .ROB_DEPTH(16)) my_rename (
        .clk_i(clk_i),
        .rst_i(reset_i),

        .data_i(decode_o),
        .valid_prod_i(decode_rename_valid),
        .ready_prod_o(decode_rename_ready),

        .data_o(rename_o),
        .ready_cons_i(rename_dispatch_ready),
        .valid_cons_o(rename_dispatch_valid),

        // FROM COMMIT: free old preg
        .commit_free_valid_i(commit_free_valid),
        .commit_free_preg_i (commit_free_preg),

        // recovery interface (tied off)
        .rat_recover_i(),
        .rat_recover_map_i(),
        .fl_recover_i(),
        .fl_recover_head_i(),
        .fl_recover_tail_i(),
        .fl_recover_free_count_i(),

        .ratfl_chkpt_we_o(),
        .ratfl_chkpt_tag_o(),
        .ratfl_chkpt_rat_map_o(),
        .ratfl_chkpt_fl_head_o(),
        .ratfl_chkpt_fl_tail_o(),
        .ratfl_chkpt_fl_free_count_o()
    );

    assign dbg_rename_valid_o = rename_dispatch_valid;
    assign dbg_rename_ready_o = rename_dispatch_ready;
    assign dbg_rename_data_o  = rename_o;

    // -----------------------------
    // Dispatch
    // -----------------------------
    logic recover_i;
    logic              cdb_valid_i;
    logic [PREG_W-1:0] cdb_tag_i;

    logic              prf_wb_en_i;
    logic [PREG_W-1:0] prf_wb_addr_i;
    logic [31:0]       prf_wb_data_i;

    logic                  rob_complete_valid_i;
    logic [$clog2(16)-1:0] rob_complete_idx_i;
    logic                  rob_complete_mispredict_i;

    logic [$clog2(16)-1:0] rob_recover_tail_i;
    logic [$clog2(16)  :0] rob_recover_used_i;

    // Tie-offs (for now)
    assign recover_i          = 1'b0;
    assign rob_recover_tail_i = '0;
    assign rob_recover_used_i = '0;

    dispatch_module #(.R(R), .DI(DI)) my_dispatch (
        .clk_i(clk_i),
        .rst_i(reset_i),

        .recover_i(recover_i),

        // DEBUG
        .dbg_pipe_valid_o    (dbg_disp_pipe_valid_o),
        .dbg_pipe_ready_o    (dbg_disp_pipe_ready_o),
        .dbg_pipe_fire_o     (dbg_disp_pipe_fire_o),

        .dbg_alu_insert_o    (dbg_disp_alu_insert_o),
        .dbg_mem_insert_o    (dbg_disp_mem_insert_o),
        .dbg_br_insert_o     (dbg_disp_br_insert_o),

        .dbg_dispatch_fire_o (dbg_disp_dispatch_fire_o),

        .dbg_tag_o           (dbg_disp_tag_o),
        .dbg_pc_o            (dbg_disp_pc_o),
        .dbg_fu_o            (dbg_disp_fu_o),

        .data_i(rename_o),
        .valid_prod_i(rename_dispatch_valid),
        .ready_prod_o(rename_dispatch_ready),

        // wakeups from writeback
        .cdb_valid_i(cdb_valid_i),
        .cdb_tag_i  (cdb_tag_i),

        // update prf from writeback
        .prf_wb_en_i  (prf_wb_en_i),
        .prf_wb_addr_i(prf_wb_addr_i),
        .prf_wb_data_i(prf_wb_data_i),

        // update ROB from writeback
        .rob_complete_valid_i     (rob_complete_valid_i),
        .rob_complete_idx_i       (rob_complete_idx_i),
        .rob_complete_mispredict_i(rob_complete_mispredict_i),

        // ROB recovery payload
        .rob_recover_tail_i(rob_recover_tail_i),
        .rob_recover_used_i(rob_recover_used_i),

        // ROB checkpoint write -> controller
        .rob_chkpt_we_o(),
        .rob_chkpt_tag_o(),
        .rob_chkpt_tail_o(),
        .rob_chkpt_used_o(),

        // ROB -> Commit stage interface
        .rob_commit_ready_i(rob_commit_ready),
        .rob_commit_valid_o(rob_commit_valid),
        .rob_commit_entry_o(rob_commit_entry),
        .rob_commit_index_o(rob_commit_index),

        // Issue outputs
        .alu_issue_valid_o(alu_issue_valid),
        .alu_issue_ready_i(alu_issue_ready),
        .alu_issue_o      (alu_issue_data),

        .lsu_issue_valid_o(lsu_issue_valid),
        .lsu_issue_ready_i(lsu_issue_ready),
        .lsu_issue_o      (lsu_issue_data),

        .br_issue_valid_o (br_issue_valid),
        .br_issue_ready_i (br_issue_ready),
        .br_issue_o       (br_issue_data)
    );

    assign dbg_alu_issue_fire_o = alu_issue_valid && alu_issue_ready;
    assign dbg_lsu_issue_fire_o = lsu_issue_valid && lsu_issue_ready;
    assign dbg_br_issue_fire_o  = br_issue_valid  && br_issue_ready;

    // -----------------------------
    // Execute
    // -----------------------------
    execute_module #(.AE(AE), .AO(AO), .BE(BE), .BO(BO), .LE(LE), .LO(LO)) my_execute (
        .clk_i(clk_i),
        .reset_i(reset_i),

        .alu_valid_i(alu_issue_valid),
        .alu_ready_o(alu_issue_ready),
        .alu_data_i (alu_issue_data),
        .alu_flush_i(1'b0),

        .b_valid_i  (br_issue_valid),
        .b_ready_o  (br_issue_ready),
        .b_data_i   (br_issue_data),
        .b_flush_i  (1'b0),

        .lsu_valid_i(lsu_issue_valid),
        .lsu_ready_o(lsu_issue_ready),
        .lsu_data_i (lsu_issue_data),
        .lsu_flush_i(1'b0),

        .asb_cons_ready_i(alu_wb_ready),
        .asb_cons_valid_o(alu_wb_valid),
        .asb_cons_data_o (alu_wb_data),

        .bsb_cons_ready_i(br_wb_ready),
        .bsb_cons_valid_o(br_wb_valid),
        .bsb_cons_data_o (br_wb_data),

        .lsb_cons_ready_i(lsu_wb_ready),
        .lsb_cons_valid_o(lsu_wb_valid),
        .lsb_cons_data_o (lsu_wb_data)
    );

    // -----------------------------
    // Execute -> WB arbitration
    // -----------------------------
    logic       wb_valid;
    wb_packet_t wb_packet;
    logic       wb_ready;

    typedef enum logic [1:0] {SRC_ALU=2'd0, SRC_LSU=2'd1, SRC_BR=2'd2} wb_src_t;
    wb_src_t wb_src_sel;

    always_comb begin
        if (lsu_wb_valid)      wb_src_sel = SRC_LSU;
        else if (alu_wb_valid) wb_src_sel = SRC_ALU;
        else if (br_wb_valid)  wb_src_sel = SRC_BR;
        else                   wb_src_sel = SRC_ALU;

        wb_valid  = (lsu_wb_valid || alu_wb_valid || br_wb_valid);

        alu_wb_ready = wb_ready && alu_wb_valid && (wb_src_sel == SRC_ALU);
        lsu_wb_ready = wb_ready && lsu_wb_valid && (wb_src_sel == SRC_LSU);
        br_wb_ready  = wb_ready && br_wb_valid  && (wb_src_sel == SRC_BR);

        wb_packet = '0;

        unique case (wb_src_sel)
            SRC_ALU: begin
                wb_packet.src_fu    = 2'd0;
                wb_packet.completed = 1'b1;
                wb_packet.ROB_tag   = alu_wb_data.ROB_tag;
                wb_packet.rd_addr   = alu_wb_data.rd_addr;
                wb_packet.rd_val    = alu_wb_data.rd_val;
            end

            SRC_LSU: begin
                wb_packet.src_fu    = 2'd1;
                wb_packet.completed = 1'b1;
                wb_packet.ROB_tag   = lsu_wb_data.ROB_tag;
                wb_packet.rd_addr   = lsu_wb_data.rd_addr;
                wb_packet.rd_val    = lsu_wb_data.rd_val;
            end

            SRC_BR: begin
                wb_packet.src_fu       = 2'd2;
                wb_packet.completed    = 1'b1;
                wb_packet.ROB_tag      = br_wb_data.ROB_tag;
                wb_packet.rd_addr      = br_wb_data.rd_addr;
                wb_packet.rd_val       = br_wb_data.rd_val;
                wb_packet.branch_taken = br_wb_data.branch_taken;
                wb_packet.dest_addr    = br_wb_data.dest_addr;
                wb_packet.mispredict   = br_wb_data.mispredict;
            end
        endcase
    end

    // -----------------------------
    // Writeback
    // -----------------------------
    logic              prf_wb_en_o;
    logic [PREG_W-1:0] prf_wb_addr_o;
    logic [31:0]       prf_wb_data_o;

    logic              cdb_valid_o;
    logic [PREG_W-1:0] cdb_tag_o;
    logic [31:0]       cdb_data_o;

    logic                  rob_complete_valid_o;
    logic [$clog2(16)-1:0] rob_complete_idx_o;
    logic                  rob_complete_mispredict_o;

    logic recover_o, redirect_valid_o;
    logic [31:0] redirect_pc_o;
    logic rob_recover_o, rat_recover_o, fl_recover_o;
    logic [$clog2(16)-1:0] rob_recover_tail_o;
    logic [$clog2(16)  :0] rob_recover_used_o;
    logic [32*PREG_W-1:0]  rat_recover_map_o;
    logic [PREG_W-1:0]     fl_recover_head_o, fl_recover_tail_o;
    logic [$clog2(PREGS):0] fl_recover_free_count_o;

    writeback_module #(.ROB_DEPTH(16), .AREG(32)) my_wb (
        .clk_i(clk_i),
        .rst_i(reset_i),

        .wb_valid_i (wb_valid),
        .wb_packet_i(wb_packet),
        .wb_ready_o (wb_ready),

        .ratfl_chkpt_we_i(1'b0),
        .ratfl_chkpt_tag_i('0),
        .ratfl_chkpt_rat_map_i('0),
        .ratfl_chkpt_fl_head_i('0),
        .ratfl_chkpt_fl_tail_i('0),
        .ratfl_chkpt_fl_free_count_i('0),

        .rob_chkpt_we_i(1'b0),
        .rob_chkpt_tag_i('0),
        .rob_chkpt_tail_i('0),
        .rob_chkpt_used_i('0),

        .recover_o(recover_o),
        .redirect_valid_o(redirect_valid_o),
        .redirect_pc_o(redirect_pc_o),

        .rob_recover_o(rob_recover_o),
        .rob_recover_tail_o(rob_recover_tail_o),
        .rob_recover_used_o(rob_recover_used_o),

        .rat_recover_o(rat_recover_o),
        .rat_recover_map_o(rat_recover_map_o),

        .fl_recover_o(fl_recover_o),
        .fl_recover_head_o(fl_recover_head_o),
        .fl_recover_tail_o(fl_recover_tail_o),
        .fl_recover_free_count_o(fl_recover_free_count_o),

        .recover_rob_tag_o(),

        .prf_wb_en_o  (prf_wb_en_o),
        .prf_wb_addr_o(prf_wb_addr_o),
        .prf_wb_data_o(prf_wb_data_o),

        .cdb_valid_o(cdb_valid_o),
        .cdb_tag_o  (cdb_tag_o),
        .cdb_data_o (cdb_data_o),

        .rob_complete_valid_o     (rob_complete_valid_o),
        .rob_complete_idx_o       (rob_complete_idx_o),
        .rob_complete_mispredict_o(rob_complete_mispredict_o)
    );

    // Hook WB -> Dispatch
    assign cdb_valid_i   = cdb_valid_o;
    assign cdb_tag_i     = cdb_tag_o;

    assign prf_wb_en_i   = prf_wb_en_o;
    assign prf_wb_addr_i = prf_wb_addr_o;
    assign prf_wb_data_i = prf_wb_data_o;

    assign rob_complete_valid_i      = rob_complete_valid_o;
    assign rob_complete_idx_i        = rob_complete_idx_o;
    assign rob_complete_mispredict_i = rob_complete_mispredict_o;

    // Mirror internal WB signals to top-level debug outputs
    assign dbg_wb_valid_o   = wb_valid;
    assign dbg_wb_ready_o   = wb_ready;
    assign dbg_wb_packet_o  = wb_packet;

    assign dbg_prf_wb_en_o   = prf_wb_en_o;
    assign dbg_prf_wb_addr_o = prf_wb_addr_o;
    assign dbg_prf_wb_data_o = prf_wb_data_o;

    assign dbg_cdb_valid_o = cdb_valid_o;
    assign dbg_cdb_tag_o   = cdb_tag_o;
    assign dbg_cdb_data_o  = cdb_data_o;

    assign dbg_rob_complete_valid_o      = rob_complete_valid_o;
    assign dbg_rob_complete_idx_o        = rob_complete_idx_o;
    assign dbg_rob_complete_mispredict_o = rob_complete_mispredict_o;

    assign dbg_recover_o        = recover_o;
    assign dbg_redirect_valid_o = redirect_valid_o;
    assign dbg_redirect_pc_o    = redirect_pc_o;

endmodule

