//`timescale 1ns / 1ns
//import buffer_pkgs::*;

//module reservation_station #(
//    parameter type RS_ENTRY_T = rs_entry_t,
//    parameter int  RS_DEPTH   = 8
//)(
//    input  logic      clk_i,
//    input  logic      rst_i,
//    input  logic      recover_i,

//    input  logic      disp_valid_i,
//    output logic      disp_ready_o,
//    input  RS_ENTRY_T disp_entry_i,

//    output logic      issue_valid_o,
//    input  logic      issue_ready_i,
//    output RS_ENTRY_T issue_entry_o,

//    // CDB tag (optional if you also have PRF WB)
//    input  logic              cdb_valid_i,
//    input  logic [PREG_W-1:0] cdb_tag_i,

//    // *** NEW: PRF writeback port so RS can latch operand VALUES on wakeup ***
//    input  logic              prf_wb_en_i,
//    input  logic [PREG_W-1:0] prf_wb_addr_i,
//    input  logic [31:0]       prf_wb_data_i
//);

//    RS_ENTRY_T entries [RS_DEPTH];

//    logic [RS_DEPTH-1:0] valid_vec;
//    logic [RS_DEPTH-1:0] ready_vec;

//    // Treat tag==0 as "already ready" (x0 / unused convention)
//    function automatic logic src_ready(input logic rdy, input logic [PREG_W-1:0] tag);
//        return rdy || (tag == '0);
//    endfunction

//    always_comb begin
//        for (int i = 0; i < RS_DEPTH; i++) begin
//            valid_vec[i] = entries[i].valid;
//            ready_vec[i] = src_ready(entries[i].rs1_rdy, entries[i].rs1_tag) &
//                           src_ready(entries[i].rs2_rdy, entries[i].rs2_tag);
//        end
//    end

//    logic [RS_DEPTH-1:0] free_mask;
//    assign free_mask = ~valid_vec;

//    logic [$clog2(RS_DEPTH)-1:0] free_idx;
//    logic have_free;
//    prioritydecoder #(.WIDTH(RS_DEPTH)) u_dec_free (
//        .in   (free_mask),
//        .out  (free_idx),
//        .valid(have_free)
//    );

//    assign disp_ready_o = have_free && !recover_i;
//    wire do_insert = disp_valid_i && disp_ready_o;

//    logic [RS_DEPTH-1:0] issue_mask;
//    assign issue_mask = valid_vec & ready_vec;

//    logic [$clog2(RS_DEPTH)-1:0] issue_idx;
//    logic have_issue;
//    prioritydecoder #(.WIDTH(RS_DEPTH)) u_dec_issue (
//        .in   (issue_mask),
//        .out  (issue_idx),
//        .valid(have_issue)
//    );

//    // Hold register for stable issue under backpressure
//    logic                       hold_valid;
//    RS_ENTRY_T                  hold_entry;
//    logic [$clog2(RS_DEPTH)-1:0] hold_idx;

//    wire cand_valid = have_issue && !recover_i;
//    wire do_issue   = issue_valid_o && issue_ready_i;

//    always_ff @(posedge clk_i or posedge rst_i) begin
//        if (rst_i || recover_i) begin
//            hold_valid <= 1'b0;
//            hold_entry <= '0;
//            hold_idx   <= '0;
//        end else begin
//            if (!hold_valid && cand_valid) begin
//                hold_valid <= 1'b1;
//                hold_entry <= entries[issue_idx];
//                hold_idx   <= issue_idx;
//            end
//            if (do_issue) begin
//                hold_valid <= 1'b0;
//            end
//        end
//    end

//    assign issue_valid_o = hold_valid && !recover_i;
//    assign issue_entry_o = hold_valid ? hold_entry : '0;

//    always_ff @(posedge clk_i or posedge rst_i) begin
//        if (rst_i || recover_i) begin
//            for (int i = 0; i < RS_DEPTH; i++) begin
//                entries[i]         <= '0;
//                entries[i].valid   <= 1'b0;
//                entries[i].rs1_rdy <= 1'b0;
//                entries[i].rs2_rdy <= 1'b0;
//            end
//        end else begin

//            // =====================================================
//            // WAKEUP + VALUE CAPTURE (THIS IS THE BIG FIX)
//            // =====================================================
//            if (prf_wb_en_i) begin
//                for (int i = 0; i < RS_DEPTH; i++) begin
//                    if (entries[i].valid) begin
//                        if (!entries[i].rs1_rdy && (entries[i].rs1_tag == prf_wb_addr_i)) begin
//                            entries[i].rs1_rdy <= 1'b1;
//                            entries[i].rs1_val <= prf_wb_data_i;
//                        end
//                        if (!entries[i].rs2_rdy && (entries[i].rs2_tag == prf_wb_addr_i)) begin
//                            entries[i].rs2_rdy <= 1'b1;
//                            entries[i].rs2_val <= prf_wb_data_i;
//                        end
//                    end
//                end
//            end

//            // (Optional) CDB can also mark ready if you want, but without data it's weaker.
//            if (cdb_valid_i) begin
//                for (int i = 0; i < RS_DEPTH; i++) begin
//                    if (entries[i].valid) begin
//                        if (!entries[i].rs1_rdy && (entries[i].rs1_tag == cdb_tag_i))
//                            entries[i].rs1_rdy <= 1'b1;
//                        if (!entries[i].rs2_rdy && (entries[i].rs2_tag == cdb_tag_i))
//                            entries[i].rs2_rdy <= 1'b1;
//                    end
//                end
//            end

//            // Insert
//            if (do_insert) begin
//                entries[free_idx]       <= disp_entry_i;
//                entries[free_idx].valid <= 1'b1;

//                // Ensure unused/x0 sources don't block issue
//                if (disp_entry_i.rs1_tag == '0) entries[free_idx].rs1_rdy <= 1'b1;
//                if (disp_entry_i.rs2_tag == '0) entries[free_idx].rs2_rdy <= 1'b1;
//            end

//            // Remove on issue using held index
//            if (do_issue && !(do_insert && (hold_idx == free_idx))) begin
//                entries[hold_idx].valid   <= 1'b0;
//                entries[hold_idx].rs1_rdy <= 1'b0;
//                entries[hold_idx].rs2_rdy <= 1'b0;
//            end
//        end
//    end

//endmodule

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

    input  logic       cdb_valid_i,
    input  logic [6:0] cdb_tag_i
);

    RS_ENTRY_T entries [RS_DEPTH];

    logic [RS_DEPTH-1:0] valid_vec;
    logic [RS_DEPTH-1:0] ready_vec;

    always_comb begin
        for (int i = 0; i < RS_DEPTH; i++) begin
            valid_vec[i] = entries[i].valid;
            // ----------------------------
            // FIX: ready is derived, not stored
            // ----------------------------
            ready_vec[i] = entries[i].rs1_rdy & entries[i].rs2_rdy; // FIX
        end
    end

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
    // [CHG] Hold register to make issue_entry_o stable under stall
    // ============================================================
    logic                          hold_valid;     // [CHG]
    RS_ENTRY_T                      hold_entry;     // [CHG]
    logic [$clog2(RS_DEPTH)-1:0]    hold_idx;       // [CHG]

    // [CHG] "raw" candidate exists (combinational arbitration)
    wire cand_valid = have_issue && !recover_i;     // [CHG]
    wire do_issue   = issue_valid_o && issue_ready_i; // [CHG] (moved below)

    // [CHG] Latch a candidate when not currently holding one.
    //       Once held, keep it stable until accepted.
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i || recover_i) begin
            hold_valid <= 1'b0;                    // [CHG]
            hold_entry <= '0;                      // [CHG]
            hold_idx   <= '0;                      // [CHG]
        end else begin
            // Capture a new candidate only if we are not already holding one
            if (!hold_valid && cand_valid) begin   // [CHG]
                hold_valid <= 1'b1;                // [CHG]
                hold_entry <= entries[issue_idx];  // [CHG]
                hold_idx   <= issue_idx;           // [CHG]
            end

            // Drop hold once accepted
            if (hold_valid && issue_ready_i) begin // [CHG]
                hold_valid <= 1'b0;                // [CHG]
            end
        end
    end

    // [CHG] Drive issue interface from hold regs (stable under backpressure)
    assign issue_valid_o = hold_valid && !recover_i;      // [CHG]
    assign issue_entry_o = hold_valid ? hold_entry : '0;  // [CHG] (also fixes X-prop)

    // [CHG] Handshake uses held valid
    // NOTE: do_issue is already declared above.
    // wire do_issue = issue_valid_o && issue_ready_i;

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i || recover_i) begin
            for (int i = 0; i < RS_DEPTH; i++) begin
                entries[i]        <= '0;
                entries[i].valid  <= 1'b0;
                entries[i].rs1_rdy<= 1'b0;
                entries[i].rs2_rdy<= 1'b0;
            end
        end else begin
            // Wakeup
            if (cdb_valid_i) begin
                for (int i=0; i<RS_DEPTH; i++) begin
                    if (entries[i].valid) begin
                        if (!entries[i].rs1_rdy && (entries[i].rs1_tag == cdb_tag_i))
                            entries[i].rs1_rdy <= 1'b1;
                        if (!entries[i].rs2_rdy && (entries[i].rs2_tag == cdb_tag_i))
                            entries[i].rs2_rdy <= 1'b1;
                    end
                end
            end

            // Insert
            if (do_insert) begin
                entries[free_idx]       <= disp_entry_i;
                entries[free_idx].valid <= 1'b1;
            end

            // ============================================================
            // [CHG] Remove on issue uses held index, not comb issue_idx
            //       (prevents removing the wrong entry under backpressure)
            // ============================================================
            if (do_issue && !(do_insert && (hold_idx == free_idx))) begin // [CHG]
                entries[hold_idx].valid   <= 1'b0;                        // [CHG]
                entries[hold_idx].rs1_rdy <= 1'b0;                        // [CHG]
                entries[hold_idx].rs2_rdy <= 1'b0;                        // [CHG]
            end
        end
    end

endmodule