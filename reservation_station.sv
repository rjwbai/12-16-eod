`timescale 1ns / 1ns
import buffer_pkgs::*;

module reservation_station #(
    parameter type RS_ENTRY_T = rs_entry_t,
    parameter int  RS_DEPTH   = 8
)(
    input  logic      clk_i,
    input  logic      rst_i,
    input  logic      recover_i,

    input  logic      disp_valid_i,
    output logic      disp_ready_o,
    input  RS_ENTRY_T disp_entry_i,

    output logic      issue_valid_o,
    input  logic      issue_ready_i,
    output RS_ENTRY_T issue_entry_o,

    input  logic              cdb_valid_i,
    input  logic [PREG_W-1:0] cdb_tag_i,
    input  logic [31:0]       cdb_data_i
);

    RS_ENTRY_T entries [RS_DEPTH];

    logic [RS_DEPTH-1:0] valid_vec;
    logic [RS_DEPTH-1:0] ready_vec;

    always_comb begin
        for (int i = 0; i < RS_DEPTH; i++) begin
            valid_vec[i] = entries[i].valid;
            ready_vec[i] = entries[i].rs1_rdy & entries[i].rs2_rdy;
        end
    end

    // Find free slot
    logic [RS_DEPTH-1:0] free_mask;
    assign free_mask = ~valid_vec;

    logic [$clog2(RS_DEPTH)-1:0] free_idx;
    logic have_free;

    prioritydecoder #(.WIDTH(RS_DEPTH)) u_dec_free (
        .in   (free_mask),
        .out  (free_idx),
        .valid(have_free)
    );

    assign disp_ready_o = have_free && !recover_i;
    wire do_insert = disp_valid_i && disp_ready_o;

    // Find issue candidate (combinational)
    logic [RS_DEPTH-1:0] issue_mask;
    assign issue_mask = valid_vec & ready_vec;

    logic [$clog2(RS_DEPTH)-1:0] issue_idx;
    logic have_issue;

    prioritydecoder #(.WIDTH(RS_DEPTH)) u_dec_issue (
        .in   (issue_mask),
        .out  (issue_idx),
        .valid(have_issue)
    );

    // ============================================================
    // HOLD LOGIC: hold only an INDEX so wakeups update the issued entry
    // ============================================================
    logic                       hold_valid;
    logic [$clog2(RS_DEPTH)-1:0] hold_idx;

    wire cand_valid = have_issue && !recover_i;

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i || recover_i) begin
            hold_valid <= 1'b0;
            hold_idx   <= '0;
        end else begin
            if (!hold_valid && cand_valid) begin
                hold_valid <= 1'b1;
                hold_idx   <= issue_idx;
            end

            // when accepted, drop hold
            if (hold_valid && issue_ready_i) begin
                hold_valid <= 1'b0;
            end
        end
    end

    assign issue_valid_o = hold_valid && !recover_i;
    assign issue_entry_o = (hold_valid) ? entries[hold_idx] : '0;

    wire do_issue = issue_valid_o && issue_ready_i;

    // ============================================================
    // MAIN STATE UPDATES
    // ============================================================
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i || recover_i) begin
            for (int i = 0; i < RS_DEPTH; i++) begin
                entries[i] <= '0;
                entries[i].valid   <= 1'b0;
                entries[i].rs1_rdy <= 1'b0;
                entries[i].rs2_rdy <= 1'b0;
            end
        end else begin
            // ----------------------------
            // Wakeup existing entries
            // ----------------------------
            if (cdb_valid_i && (cdb_tag_i != '0)) begin
                for (int i = 0; i < RS_DEPTH; i++) begin
                    if (entries[i].valid) begin
                        if (!entries[i].rs1_rdy && (entries[i].rs1_tag == cdb_tag_i)) begin
                            entries[i].rs1_rdy <= 1'b1;
                            entries[i].rs1_val <= cdb_data_i;
                            entries[i].rs1_tag <= '0;
                        end
                        if (!entries[i].rs2_rdy && (entries[i].rs2_tag == cdb_tag_i)) begin
                            entries[i].rs2_rdy <= 1'b1;
                            entries[i].rs2_val <= cdb_data_i;
                            entries[i].rs2_tag <= '0;
                        end
                    end
                end
            end

            // ----------------------------
            // Insert new entry
            // (trust dispatch to set rs*_rdy/val/tag correctly)
            // also handle same-cycle CDB match (belt+suspenders)
            // ----------------------------
            if (do_insert) begin
                entries[free_idx] <= disp_entry_i;
                entries[free_idx].valid <= 1'b1;

                // same-cycle wakeup for inserted entry
                if (cdb_valid_i && (cdb_tag_i != '0)) begin
                    if (!disp_entry_i.rs1_rdy && (disp_entry_i.rs1_tag == cdb_tag_i)) begin
                        entries[free_idx].rs1_rdy <= 1'b1;
                        entries[free_idx].rs1_val <= cdb_data_i;
                        entries[free_idx].rs1_tag <= '0;
                    end
                    if (!disp_entry_i.rs2_rdy && (disp_entry_i.rs2_tag == cdb_tag_i)) begin
                        entries[free_idx].rs2_rdy <= 1'b1;
                        entries[free_idx].rs2_val <= cdb_data_i;
                        entries[free_idx].rs2_tag <= '0;
                    end
                end
            end

            // ----------------------------
            // Remove on issue (only when accepted)
            // avoid clobber if we inserted into same slot (rare but safe)
            // ----------------------------
            if (do_issue && !(do_insert && (hold_idx == free_idx))) begin
                entries[hold_idx] <= '0;
                entries[hold_idx].valid   <= 1'b0;
                entries[hold_idx].rs1_rdy <= 1'b0;
                entries[hold_idx].rs2_rdy <= 1'b0;
            end
        end
    end

endmodule
