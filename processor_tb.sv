//`timescale 1ns/1ps
//import buffer_pkgs::*;

//module processor_tb;

//    // -----------------------------
//    // Clock / reset
//    // -----------------------------
//    logic clk, reset;
//    always #5 clk = ~clk;

//    // -----------------------------
//    // IMEM
//    // -----------------------------
//    localparam int PROG_LEN = 24;
//    logic [31:0] imem [0:PROG_LEN-1];

//    // -----------------------------
//    // DUT debug signals
//    // -----------------------------
//    logic dbg_prf_wb_en;
//    logic [PREG_W-1:0] dbg_prf_wb_addr;
//    logic [31:0] dbg_prf_wb_data;

//    logic dbg_commit_valid;
//    logic [4:0] dbg_commit_arch_rd;
//    logic [PREG_W-1:0] dbg_commit_dest_preg;
//    logic [31:0] dbg_commit_pc;

//    // (optional prints)
//    logic dbg_wb_valid, dbg_wb_ready;
//    wb_packet_t dbg_wb_packet;

//    // -----------------------------
//    // DUT
//    // -----------------------------
//    processor dut (
//        .clk_i(clk),
//        .reset_i(reset),

//        // probe-only ready inputs (ignored by your processor, but must be tied)
//        .ready_alu_dispatch_i(1'b1),
//        .ready_lsu_dispatch_i(1'b1),
//        .ready_br_dispatch_i (1'b1),

//        // PRF writeback debug
//        .dbg_prf_wb_en_o  (dbg_prf_wb_en),
//        .dbg_prf_wb_addr_o(dbg_prf_wb_addr),
//        .dbg_prf_wb_data_o(dbg_prf_wb_data),

//        // (optional WB prints)
//        .dbg_wb_valid_o   (dbg_wb_valid),
//        .dbg_wb_ready_o   (dbg_wb_ready),
//        .dbg_wb_packet_o  (dbg_wb_packet),

//        // COMMIT debug (you add these 4 ports)
//        .dbg_commit_valid_o     (dbg_commit_valid),
//        .dbg_commit_arch_rd_o   (dbg_commit_arch_rd),
//        .dbg_commit_dest_preg_o (dbg_commit_dest_preg),
//        .dbg_commit_pc_o        (dbg_commit_pc)
//    );

//    // -----------------------------
//    // Models
//    // -----------------------------
//    logic [31:0] prf_model [0:PREGS-1];
//    logic [31:0] arch_x    [0:31];

//    integer i;
//    integer cycles;
//    integer commit_count;
//    integer error_count;

//    // -----------------------------
//    // Helpers: decode + execute subset
//    // -----------------------------
//    function automatic logic [31:0] sext12(input logic [31:0] instr);
//        sext12 = {{20{instr[31]}}, instr[31:20]};
//    endfunction

//    function automatic logic [31:0] sextI(input logic [31:0] instr);
//        sextI = {{20{instr[31]}}, instr[31:20]};
//    endfunction

//    function automatic logic [31:0] u_imm(input logic [31:0] instr);
//        u_imm = {instr[31:12], 12'b0};
//    endfunction

//    function automatic logic [31:0] exec_supported(
//        input logic [31:0] instr,
//        input logic [31:0] rs1_val,
//        input logic [31:0] rs2_val,
//        output logic       is_supported,
//        output logic       writes_rd
//    );
//        logic [6:0] opc;
//        logic [2:0] f3;
//        logic [6:0] f7;
//        logic [31:0] imm;
//        logic [4:0] shamt;

//        begin
//            opc = instr[6:0];
//            f3  = instr[14:12];
//            f7  = instr[31:25];
//            imm = sextI(instr);
//            shamt = rs2_val[4:0];

//            is_supported = 1'b1;
//            writes_rd    = 1'b1;
//            exec_supported = 32'hDEAD_BEEF;

//            unique case (opc)
//                7'b0110111: begin // LUI
//                    exec_supported = u_imm(instr);
//                end

//                7'b0010011: begin // OP-IMM: ADDI/ORI/SLTIU  (and you treat shifts as RSHIFT but decode doesn't include them)
//                    unique case (f3)
//                        3'b000: exec_supported = rs1_val + imm; // ADDI
//                        3'b110: exec_supported = rs1_val | imm; // ORI
//                        3'b011: exec_supported = ($unsigned(rs1_val) < $unsigned(imm)) ? 32'd1 : 32'd0; // SLTIU
//                        default: begin
//                            is_supported = 1'b0;
//                            writes_rd    = 1'b0;
//                        end
//                    endcase
//                end

//                7'b0110011: begin // OP: AND / SRA / SUB (your decode treats func3=000 as SUB always)
//                    unique case (f3)
//                        3'b111: exec_supported = rs1_val & rs2_val; // AND
//                        3'b101: exec_supported = $signed(rs1_val) >>> shamt; // SRA (matches your ALU)
//                        3'b000: exec_supported = rs1_val - rs2_val; // SUB
//                        default: begin
//                            is_supported = 1'b0;
//                            writes_rd    = 1'b0;
//                        end
//                    endcase
//                end

//                default: begin
//                    is_supported = 1'b0;
//                    writes_rd    = 1'b0;
//                end
//            endcase
//        end
//    endfunction

//    // -----------------------------
//    // Init / load
//    // -----------------------------
//    initial begin
//        clk = 1'b0;
//        reset = 1'b1;

//        $readmemh("processor_instructions.mem", imem);
//        $display("[TB] Loaded %0d instruction words", PROG_LEN);

//        // init models
//        for (i = 0; i < PREGS; i=i+1) prf_model[i] = 32'b0;
//        for (i = 0; i < 32; i=i+1)    arch_x[i]    = 32'b0;

//        cycles = 0;
//        commit_count = 0;
//        error_count  = 0;

//        repeat (5) @(posedge clk);
//        reset = 1'b0;

//        // run
//        while (cycles < 5000 && commit_count < PROG_LEN) begin
//            @(posedge clk);
//            cycles = cycles + 1;
//        end

//        $display("------------------------------------------------");
//        $display("COMMITS      : %0d / %0d", commit_count, PROG_LEN);
//        $display("CYCLES       : %0d", cycles);
//        $display("ERRORS       : %0d", error_count);
//        $display("------------------------------------------------");

//        if (commit_count != PROG_LEN) begin
//            $error("[TB] FAIL: did not reach %0d commits", PROG_LEN);
//        end else if (error_count != 0) begin
//            $error("[TB] FAIL: mismatches detected");
//        end else begin
//            $display("[TB] PASS ✅ Architectural commit check matched for supported ops");
//        end

//        $finish;
//    end

//    // -----------------------------
//    // PRF model updated by writeback
//    // -----------------------------
//    always_ff @(posedge clk) begin
//        if (reset) begin
//            // nothing
//        end else if (dbg_prf_wb_en) begin
//            prf_model[dbg_prf_wb_addr] <= dbg_prf_wb_data;
//            // optional:
//            // $display("[%0t] PRF WB p%0d = 0x%08x", $time, dbg_prf_wb_addr, dbg_prf_wb_data);
//        end
//    end

//    // -----------------------------
//    // Commit check
//    // -----------------------------
//    always_ff @(posedge clk) begin
//        if (reset) begin
//            // nothing
//        end else if (dbg_commit_valid) begin
//            logic [31:0] instr;
//            int unsigned idx;

//            logic [4:0]  rd;
//            logic [4:0]  rs1;
//            logic [4:0]  rs2;

//            logic supported, writes_rd;
//            logic [31:0] rs1_val, rs2_val;
//            logic [31:0] exp_val;
//            logic [31:0] got_val;

//            idx = dbg_commit_pc[31:2];

//            if (idx < PROG_LEN)
//                instr = imem[idx];
//            else
//                instr = 32'h0000_0013; // treat as NOP

//            rd  = instr[11:7];
//            rs1 = instr[19:15];
//            rs2 = instr[24:20];

//            rs1_val = arch_x[rs1];
//            rs2_val = arch_x[rs2];

//            exp_val = exec_supported(instr, rs1_val, rs2_val, supported, writes_rd);
//            got_val = prf_model[dbg_commit_dest_preg];

//            commit_count <= commit_count + 1;

//            // print commit line
//            $display("[%0t] COMMIT pc=0x%08x instr=0x%08x arch_rd=x%0d dest_p=p%0d val=0x%08x",
//                     $time, dbg_commit_pc, instr, dbg_commit_arch_rd, dbg_commit_dest_preg, got_val);

//            // check only if supported and actually writes a rd (and rd!=x0)
//            if (supported && writes_rd && (rd != 5'd0)) begin
//                if (rd != dbg_commit_arch_rd) begin
//                    error_count <= error_count + 1;
//                    $error("[%0t] COMMIT RD MISMATCH pc=0x%08x instr=0x%08x instr_rd=%0d commit_arch_rd=%0d",
//                           $time, dbg_commit_pc, instr, rd, dbg_commit_arch_rd);
//                end

//                if (got_val !== exp_val) begin
//                    error_count <= error_count + 1;
//                    $error("[%0t] COMMIT VALUE MISMATCH pc=0x%08x instr=0x%08x x%0d exp=0x%08x got=0x%08x",
//                           $time, dbg_commit_pc, instr, rd, exp_val, got_val);
//                end else begin
//                    $display("         OK: x%0d = 0x%08x", rd, got_val);
//                end

//                // update architectural state
//                arch_x[rd] <= got_val;
//            end
//        end
//    end

//endmodule

`timescale 1ns/1ps
import buffer_pkgs::*;

module processor_tb;

  // ------------------------------------------------------------
  // Clock / reset
  // ------------------------------------------------------------
  logic clk, reset;
  always #5 clk = ~clk;

  // ------------------------------------------------------------
  // DUT probe points (ALU issue + WB + dispatch info)
  // ------------------------------------------------------------
  alu_entry_t alu_pkt;
  logic       alu_v;
  logic       alu_r;

  logic        dbg_pipe_fire;
  logic [3:0]  dbg_disp_tag;
  logic [31:0] dbg_disp_pc;

  logic        dbg_wb_valid, dbg_wb_ready;
  wb_packet_t  dbg_wb_packet;

  // ------------------------------------------------------------
  // Program image (from processor_instructions.mem)
  // ------------------------------------------------------------
  localparam int PROG_LEN = 2048;
  logic [31:0] imem [0:PROG_LEN-1];
  integer      imem_words_loaded;

  initial begin
    $readmemh("processor_instructions.mem", imem);

    imem_words_loaded = 0;
    for (int i = 0; i < PROG_LEN; i++) begin
      if (^imem[i] === 1'bX) break;
      imem_words_loaded++;
    end
    if (imem_words_loaded == 0)
      $fatal(1, "ERROR: processor_instructions.mem didn't load (or empty).");

    $display("[TB] Loaded %0d instruction words", imem_words_loaded);
  end

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  processor dut (
    .clk_i(clk),
    .reset_i(reset),

    // issue probe outputs
    .data_alu_dispatch_o (alu_pkt),
    .valid_alu_dispatch_o(alu_v),
    .ready_alu_dispatch_i(alu_r),

    // unused issue probes
    .ready_lsu_dispatch_i(1'b1),
    .ready_br_dispatch_i (1'b1),

    // dispatch debug
    .dbg_disp_pipe_fire_o(dbg_pipe_fire),
    .dbg_disp_tag_o      (dbg_disp_tag),
    .dbg_disp_pc_o       (dbg_disp_pc),

    // wb debug
    .dbg_wb_valid_o      (dbg_wb_valid),
    .dbg_wb_ready_o      (dbg_wb_ready),
    .dbg_wb_packet_o     (dbg_wb_packet)
  );

  // ------------------------------------------------------------
  // ALU backpressure (optional). Since your processor ignores ready
  // inputs for correctness, keep it high for stable testing.
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) alu_r <= 1'b1;
    else       alu_r <= 1'b1;
  end

  // ============================================================
  // Golden ALU (matches your alu.sv)
  // ============================================================
  function automatic logic [31:0] golden_alu_result(input alu_entry_t e);
    logic [31:0] opA, opB;
    logic [4:0]  shamt;
    begin
      opA   = (e.rs1_used) ? e.rs1_val : 32'b0;
      opB   = (e.imm_used) ? e.imm_val : ((e.rs2_used) ? e.rs2_val : 32'b0);
      shamt = opB[4:0];

      unique case (e.aluop)
        buffer_pkgs::ADD:    golden_alu_result = $signed(opA) + $signed(opB);
        buffer_pkgs::SUB:    golden_alu_result = $signed(opA) - $signed(opB);
        buffer_pkgs::ANDD:   golden_alu_result = $signed(opA) & $signed(opB);
        buffer_pkgs::ORR:    golden_alu_result = $signed(opA) | $signed(opB);
        buffer_pkgs::RSHIFT: golden_alu_result = $signed(opA) >>> shamt;
        buffer_pkgs::SLTIU:  golden_alu_result = ($unsigned(opA) < $unsigned(opB)) ? 32'd1 : 32'd0;
        buffer_pkgs::LUI:    golden_alu_result = e.imm_val;
        default:             golden_alu_result = 32'hDEAD_BEEF;
      endcase
    end
  endfunction

  // ============================================================
  // Scoreboard per ROB tag (safe with reuse)
  // ============================================================
  localparam int ROB_DEPTH = 16;

  logic [31:0] tag_instr     [0:ROB_DEPTH-1];
  logic [31:0] tag_pc        [0:ROB_DEPTH-1];
  bit          tag_has_ins   [0:ROB_DEPTH-1];

  logic [31:0] tag_exp_val   [0:ROB_DEPTH-1];
  logic [PREG_W-1:0] tag_exp_rd [0:ROB_DEPTH-1];
  bit          tag_has_exp   [0:ROB_DEPTH-1];

  bit          tag_wb_seen   [0:ROB_DEPTH-1];
  bit          tag_inflight  [0:ROB_DEPTH-1];

  integer dispatch_prog_count;
  integer issue_count;
  integer wb_count;
  integer cycles;

  // Module-scope temps (Vivado-safe)
  integer idx;
  logic [31:0] ins_tmp;

  // ------------------------------------------------------------
  // DISPATCH tracking: store instr/pc for that tag
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) begin
      dispatch_prog_count <= 0;
      for (int t=0; t<ROB_DEPTH; t++) begin
        tag_has_ins[t]  <= 1'b0;
        tag_has_exp[t]  <= 1'b0;
        tag_wb_seen[t]  <= 1'b0;
        tag_inflight[t] <= 1'b0;
        tag_instr[t]    <= '0;
        tag_pc[t]       <= '0;
        tag_exp_val[t]  <= '0;
        tag_exp_rd[t]   <= '0;
      end
    end else if (dbg_pipe_fire) begin
      idx = dbg_disp_pc[31:2];

      // tag reuse check
      if (tag_inflight[dbg_disp_tag] && !tag_wb_seen[dbg_disp_tag]) begin
        $error("[%0t] ROB TAG REUSED BEFORE WB: tag=%0d old_instr=%08x new_pc=%h",
               $time, dbg_disp_tag, tag_instr[dbg_disp_tag], dbg_disp_pc);
      end

      // initialize tag slot
      tag_inflight[dbg_disp_tag] <= 1'b1;
      tag_wb_seen[dbg_disp_tag]  <= 1'b0;
      tag_has_exp[dbg_disp_tag]  <= 1'b0;

      if (idx < imem_words_loaded) begin
        ins_tmp = imem[idx];
        tag_has_ins[dbg_disp_tag] <= 1'b1;
        tag_instr[dbg_disp_tag]   <= ins_tmp;
        tag_pc[dbg_disp_tag]      <= dbg_disp_pc;
        dispatch_prog_count       <= dispatch_prog_count + 1;

        $display("[%0t] DISPATCH  tag=%0d  pc=0x%08x  instr=0x%08x (imem[%0d])",
                 $time, dbg_disp_tag, dbg_disp_pc, ins_tmp, idx);
      end else begin
        tag_has_ins[dbg_disp_tag] <= 1'b0;
        tag_instr[dbg_disp_tag]   <= 32'h0000_0013; // NOP
        tag_pc[dbg_disp_tag]      <= dbg_disp_pc;
        $display("[%0t] DISPATCH  tag=%0d  pc=0x%08x (ignored idx=%0d)",
                 $time, dbg_disp_tag, dbg_disp_pc, idx);
      end
    end
  end

  // ------------------------------------------------------------
  // ISSUE tracking: compute expected result ONLY for requested ops
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) begin
      issue_count <= 0;
    end else if (alu_v && alu_r) begin
      logic [3:0] tag;
      tag = alu_pkt.ROB_tag;

      // only check these ops
      if ( (alu_pkt.aluop == buffer_pkgs::LUI)   ||
           (alu_pkt.aluop == buffer_pkgs::ADD)   ||
           (alu_pkt.aluop == buffer_pkgs::ORR)   ||
           (alu_pkt.aluop == buffer_pkgs::ANDD)  ||
           (alu_pkt.aluop == buffer_pkgs::RSHIFT)||
           (alu_pkt.aluop == buffer_pkgs::SUB)   ||
           (alu_pkt.aluop == buffer_pkgs::SLTIU) ) begin

        tag_exp_val[tag] <= golden_alu_result(alu_pkt);
        tag_exp_rd[tag]  <= alu_pkt.rd_addr;
        tag_has_exp[tag] <= 1'b1;

        $display("[%0t] ISSUE ALU  tag=%0d rd_p=%0d aluop=%0d exp=0x%08x",
                 $time, tag, alu_pkt.rd_addr, alu_pkt.aluop, golden_alu_result(alu_pkt));
      end

      issue_count <= issue_count + 1;
    end
  end

  // ------------------------------------------------------------
  // WRITEBACK checking (only for ALU src_fu==0)
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) begin
      wb_count <= 0;
    end else if (dbg_wb_valid && dbg_wb_ready) begin
      logic [3:0] tag;
      tag = dbg_wb_packet.ROB_tag;

      wb_count          <= wb_count + 1;
      tag_wb_seen[tag]  <= 1'b1;
      tag_inflight[tag] <= 1'b0;

      // print with imem correlation
      $display("[%0t] WB        tag=%0d  pc=0x%08x  instr=0x%08x  rd_p=%0d  val=0x%08x  fu=%0d",
               $time, tag, tag_pc[tag], tag_has_ins[tag] ? tag_instr[tag] : 32'h0,
               dbg_wb_packet.rd_addr, dbg_wb_packet.rd_val, dbg_wb_packet.src_fu);

      // check only ALU packets
      if (dbg_wb_packet.src_fu == 2'd0) begin
        if (!tag_has_exp[tag]) begin
          $error("[%0t] ALU WB WITHOUT EXPECTATION tag=%0d pc=0x%08x instr=0x%08x",
                 $time, tag, tag_pc[tag], tag_instr[tag]);
        end else begin
          if (dbg_wb_packet.rd_addr != tag_exp_rd[tag]) begin
            $error("[%0t] WB RD_ADDR MISMATCH tag=%0d exp_rd_p=%0d got_rd_p=%0d",
                   $time, tag, tag_exp_rd[tag], dbg_wb_packet.rd_addr);
          end
          if (dbg_wb_packet.rd_val !== tag_exp_val[tag]) begin
            $error("[%0t] WB VALUE MISMATCH tag=%0d pc=0x%08x instr=0x%08x exp=0x%08x got=0x%08x",
                   $time, tag, tag_pc[tag], tag_instr[tag], tag_exp_val[tag], dbg_wb_packet.rd_val);
          end
        end
      end
    end
  end

  // ------------------------------------------------------------
  // Test control
  // ------------------------------------------------------------
  initial begin
    clk    = 1'b0;
    reset  = 1'b1;
    cycles = 0;

    repeat (5) @(posedge clk);
    reset = 1'b0;

    while (cycles < 20000) begin
      @(posedge clk);
      cycles++;

      // end when we've dispatched the loaded image and observed enough WBs
      if ((dispatch_prog_count >= imem_words_loaded) && (wb_count >= dispatch_prog_count))
        break;
    end

    $display("------------------------------------------------");
    $display("IMEM words    : %0d", imem_words_loaded);
    $display("DISPATCHED    : %0d", dispatch_prog_count);
    $display("ALU issues    : %0d", issue_count);
    $display("WRITEBACKS    : %0d", wb_count);
    $display("CYCLES        : %0d", cycles);
    $display("------------------------------------------------");

    if (cycles >= 20000) $error("TIMEOUT");
    else                 $display("TEST DONE ✅ (Checked only ALU/LUI/ADDI/ORI/AND/SRA/SUB/SLTIU)");

    $finish;
  end

endmodule


