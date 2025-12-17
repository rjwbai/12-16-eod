`timescale 1ns/1ps
import buffer_pkgs::*;

module processor_tb;

  // ------------------------------------------------------------
  // Clock / reset
  // ------------------------------------------------------------
  logic clk, reset;
  always #5 clk = ~clk;

  // ------------------------------------------------------------
  // Program image (from processor_instructions.mem)
  // ------------------------------------------------------------
  localparam int PROG_MAX = 2048;
  logic [31:0] imem [0:PROG_MAX-1];
  int          imem_words_loaded;

  initial begin
    $readmemh("processor_instructions.mem", imem);

    imem_words_loaded = 0;
    for (int i = 0; i < PROG_MAX; i++) begin
      if (^imem[i] === 1'bX) break;
      imem_words_loaded++;
    end
    if (imem_words_loaded == 0)
      $fatal(1, "ERROR: processor_instructions.mem didn't load (or empty).");

    $display("[TB] Loaded %0d instruction words", imem_words_loaded);
  end

  // ------------------------------------------------------------
  // DUT debug wires (you already have these)
  // ------------------------------------------------------------
  // Issue probe outputs (not required for commit-based TB, but keep wired)
  alu_entry_t alu_pkt;
  logic       alu_v;
  logic       alu_r;

  logic       dbg_pipe_fire;
  logic [3:0] dbg_disp_tag;
  logic [31:0] dbg_disp_pc;

  logic       dbg_wb_valid, dbg_wb_ready;
  wb_packet_t dbg_wb_packet;

  // ------------------------------------------------------------
  // DUT instance
  // ------------------------------------------------------------
  processor dut (
    .clk_i(clk),
    .reset_i(reset),

    // keep ALU issue probe hooked
    .data_alu_dispatch_o (alu_pkt),
    .valid_alu_dispatch_o(alu_v),
    .ready_alu_dispatch_i(alu_r),

    // unused issue probes
    .data_br_dispatch_o  (),
    .valid_br_dispatch_o (),
    .ready_br_dispatch_i (1'b1),

    .data_lsu_dispatch_o (),
    .valid_lsu_dispatch_o(),
    .ready_lsu_dispatch_i(1'b1),

    // rename debug (unused here)
    .dbg_rename_valid_o(),
    .dbg_rename_ready_o(),
    .dbg_rename_data_o (),

    // issue fires (unused)
    .dbg_alu_issue_fire_o(),
    .dbg_lsu_issue_fire_o(),
    .dbg_br_issue_fire_o(),

    // dispatch debug (WE USE THESE)
    .dbg_disp_pipe_valid_o(),
    .dbg_disp_pipe_ready_o(),
    .dbg_disp_pipe_fire_o(dbg_pipe_fire),

    .dbg_disp_alu_insert_o(),
    .dbg_disp_mem_insert_o(),
    .dbg_disp_br_insert_o(),
    .dbg_disp_dispatch_fire_o(),

    .dbg_disp_tag_o(dbg_disp_tag),
    .dbg_disp_pc_o (dbg_disp_pc),
    .dbg_disp_fu_o (),

    // writeback debug (WE USE THESE)
    .dbg_wb_valid_o (dbg_wb_valid),
    .dbg_wb_ready_o (dbg_wb_ready),
    .dbg_wb_packet_o(dbg_wb_packet),

    // optional extra debug outputs (unused)
    .dbg_prf_wb_en_o(),
    .dbg_prf_wb_addr_o(),
    .dbg_prf_wb_data_o(),
    .dbg_cdb_valid_o(),
    .dbg_cdb_tag_o(),
    .dbg_cdb_data_o(),
    .dbg_rob_complete_valid_o(),
    .dbg_rob_complete_idx_o(),
    .dbg_rob_complete_mispredict_o(),
    .dbg_recover_o(),
    .dbg_redirect_valid_o(),
    .dbg_redirect_pc_o()
  );

  // Always-ready for ALU issue probe
  always_ff @(posedge clk) begin
    if (reset) alu_r <= 1'b1;
    else       alu_r <= 1'b1;
  end

  // ============================================================
  // Golden execute for supported subset (matches your decode/ALU)
  // ============================================================
  function automatic logic [31:0] sext12(input logic [31:0] instr);
    sext12 = {{20{instr[31]}}, instr[31:20]};
  endfunction

  function automatic logic [31:0] u_imm(input logic [31:0] instr);
    u_imm = {instr[31:12], 12'b0};
  endfunction

  function automatic logic is_supported_and_writes(
      input  logic [31:0] instr,
      output logic        supported,
      output logic        writes_rd
  );
    logic [6:0] opc;
    logic [2:0] f3;
    begin
      opc = instr[6:0];
      f3  = instr[14:12];

      supported = 1'b0;
      writes_rd = 1'b0;

      unique case (opc)
        7'b0110111: begin // LUI
          supported = 1'b1; writes_rd = 1'b1;
        end

        7'b0010011: begin // OP-IMM: ADDI/ORI/SLTIU
          unique case (f3)
            3'b000, // ADDI
            3'b110, // ORI
            3'b011: begin // SLTIU
              supported = 1'b1; writes_rd = 1'b1;
            end
            default: begin
              supported = 1'b0; writes_rd = 1'b0;
            end
          endcase
        end

        7'b0110011: begin // OP: AND/SUB/SRA (per your decode)
          unique case (f3)
            3'b111, // AND
            3'b000, // SUB (you treat func3=000 as SUB)
            3'b101: begin // SRA
              supported = 1'b1; writes_rd = 1'b1;
            end
            default: begin
              supported = 1'b0; writes_rd = 1'b0;
            end
          endcase
        end

        default: begin
          supported = 1'b0; writes_rd = 1'b0;
        end
      endcase

      return supported;
    end
  endfunction

  function automatic logic [31:0] golden_exec(
      input  logic [31:0] instr,
      input  logic [31:0] rs1_val,
      input  logic [31:0] rs2_val,
      output logic        supported,
      output logic        writes_rd
  );
    logic [6:0] opc;
    logic [2:0] f3;
    logic [31:0] imm;
    logic [4:0] shamt;
    begin
      opc  = instr[6:0];
      f3   = instr[14:12];
      imm  = sext12(instr);
      shamt = rs2_val[4:0];

      is_supported_and_writes(instr, supported, writes_rd);

      golden_exec = 32'hDEAD_BEEF;

      if (!supported) begin
        golden_exec = 32'hDEAD_BEEF;
      end else begin
        unique case (opc)
          7'b0110111: golden_exec = u_imm(instr); // LUI

          7'b0010011: begin
            unique case (f3)
              3'b000: golden_exec = $signed(rs1_val) + $signed(imm);                       // ADDI
              3'b110: golden_exec = rs1_val | imm;                                         // ORI
              3'b011: golden_exec = ($unsigned(rs1_val) < $unsigned(imm)) ? 32'd1 : 32'd0; // SLTIU
              default: golden_exec = 32'hDEAD_BEEF;
            endcase
          end

          7'b0110011: begin
            unique case (f3)
              3'b111: golden_exec = rs1_val & rs2_val;                    // AND
              3'b000: golden_exec = $signed(rs1_val) - $signed(rs2_val);  // SUB
              3'b101: golden_exec = $signed(rs1_val) >>> shamt;           // SRA
              default: golden_exec = 32'hDEAD_BEEF;
            endcase
          end

          default: golden_exec = 32'hDEAD_BEEF;
        endcase
      end
    end
  endfunction

  // ============================================================
  // TB ROB model (commit in order)
  // ============================================================
  localparam int ROB_DEPTH = 16;

  typedef struct packed {
    logic        valid;
    logic        completed;
    logic [31:0] pc;
    logic [31:0] instr;
    logic [PREG_W-1:0] rd_p;   // phys dest from WB
    logic [31:0] rd_val;       // value from WB
  } tb_rob_entry_t;

  tb_rob_entry_t rob [0:ROB_DEPTH-1];

  int unsigned rob_head;
  int unsigned commit_count;
  int unsigned dispatch_count;
  int unsigned wb_count;
  int unsigned cycles;
  int unsigned error_count;

  // Architectural model state (x0..x31)
  logic [31:0] arch_x [0:31];

  // ------------------------------------------------------------
  // Reset/init
  // ------------------------------------------------------------
  task automatic rob_clear();
    for (int i=0; i<ROB_DEPTH; i++) begin
      rob[i].valid     = 1'b0;
      rob[i].completed = 1'b0;
      rob[i].pc        = '0;
      rob[i].instr     = '0;
      rob[i].rd_p      = '0;
      rob[i].rd_val    = '0;
    end
  endtask

  // ------------------------------------------------------------
  // DISPATCH: fill TB ROB slot by tag (tag == ROB index)
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) begin
      dispatch_count <= 0;
      rob_head       <= 0;
      commit_count   <= 0;
      wb_count       <= 0;
      error_count    <= 0;
      rob_clear();
      for (int r=0; r<32; r++) arch_x[r] <= 32'b0;
    end else if (dbg_pipe_fire) begin
      int unsigned idx;
      idx = dbg_disp_pc[31:2];

      if (rob[dbg_disp_tag].valid && !rob[dbg_disp_tag].completed) begin
        $error("[%0t] TB ROB SLOT REUSED BEFORE COMPLETE: tag=%0d old_pc=%h new_pc=%h",
               $time, dbg_disp_tag, rob[dbg_disp_tag].pc, dbg_disp_pc);
        error_count <= error_count + 1;
      end

      rob[dbg_disp_tag].valid     <= 1'b1;
      rob[dbg_disp_tag].completed <= 1'b0;
      rob[dbg_disp_tag].pc        <= dbg_disp_pc;

      if (idx < imem_words_loaded) begin
        rob[dbg_disp_tag].instr <= imem[idx];
        $display("[%0t] DISPATCH  tag=%0d  pc=0x%08x  instr=0x%08x (imem[%0d])",
                 $time, dbg_disp_tag, dbg_disp_pc, imem[idx], idx);
      end else begin
        rob[dbg_disp_tag].instr <= 32'h0000_0013; // NOP
        $display("[%0t] DISPATCH  tag=%0d  pc=0x%08x  instr=NOP (idx=%0d OOB)",
                 $time, dbg_disp_tag, dbg_disp_pc, idx);
      end

      dispatch_count <= dispatch_count + 1;
    end
  end

  // ------------------------------------------------------------
  // WRITEBACK: mark TB ROB entry completed + capture value
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!reset && dbg_wb_valid && dbg_wb_ready && dbg_wb_packet.completed) begin
      logic [3:0] tag;
      tag = dbg_wb_packet.ROB_tag[3:0];

      wb_count <= wb_count + 1;

      // record completion
      rob[tag].completed <= 1'b1;
      rob[tag].rd_p      <= dbg_wb_packet.rd_addr;
      rob[tag].rd_val    <= dbg_wb_packet.rd_val;

      $display("[%0t] WB        tag=%0d  pc=0x%08x  instr=0x%08x  rd_p=%0d  val=0x%08x  fu=%0d",
               $time, tag, rob[tag].pc, rob[tag].instr,
               dbg_wb_packet.rd_addr, dbg_wb_packet.rd_val, dbg_wb_packet.src_fu);
    end
  end

  // ------------------------------------------------------------
  // COMMIT (in-order): retire from TB ROB head when completed
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!reset) begin
      // try to commit at most 1 per cycle (like your current design)
      if (rob[rob_head].valid && rob[rob_head].completed) begin
        logic [31:0] instr;
        logic [4:0]  rd, rs1, rs2;
        logic [31:0] rs1_val, rs2_val;
        logic        supported, writes_rd;
        logic [31:0] exp_val;

        instr = rob[rob_head].instr;
        rd  = instr[11:7];
        rs1 = instr[19:15];
        rs2 = instr[24:20];

        rs1_val = arch_x[rs1];
        rs2_val = arch_x[rs2];

        exp_val = golden_exec(instr, rs1_val, rs2_val, supported, writes_rd);

        $display("[%0t] COMMIT    head=%0d pc=0x%08x instr=0x%08x rd=x%0d rd_p=%0d val=0x%08x",
                 $time, rob_head, rob[rob_head].pc, instr, rd, rob[rob_head].rd_p, rob[rob_head].rd_val);

        // Only check/update for supported ops that write rd and rd!=x0
        if (supported && writes_rd && (rd != 5'd0)) begin
          if (rob[rob_head].rd_val !== exp_val) begin
            $error("[%0t] COMMIT VALUE MISMATCH pc=0x%08x instr=0x%08x x%0d exp=0x%08x got=0x%08x",
                   $time, rob[rob_head].pc, instr, rd, exp_val, rob[rob_head].rd_val);
            error_count <= error_count + 1;
          end else begin
            $display("         OK: x%0d = 0x%08x", rd, rob[rob_head].rd_val);
          end

          arch_x[rd] <= rob[rob_head].rd_val;
        end

        // pop head
        rob[rob_head].valid     <= 1'b0;
        rob[rob_head].completed <= 1'b0;

        rob_head     <= (rob_head + 1) % ROB_DEPTH;
        commit_count <= commit_count + 1;
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

    // Run until we've committed all loaded instructions (or timeout)
    while (cycles < 20000) begin
      @(posedge clk);
      cycles++;

      if (commit_count >= imem_words_loaded) break;
    end

    $display("------------------------------------------------");
    $display("IMEM words    : %0d", imem_words_loaded);
    $display("DISPATCHED    : %0d", dispatch_count);
    $display("WRITEBACKS    : %0d", wb_count);
    $display("COMMITS       : %0d", commit_count);
    $display("CYCLES        : %0d", cycles);
    $display("ERRORS        : %0d", error_count);
    $display("------------------------------------------------");

    if (cycles >= 20000) $error("TIMEOUT");
    else if (error_count != 0) $error("TEST FAIL ❌ (mismatches detected)");
    else $display("TEST PASS ✅ (Commit-order check matched for supported ops)");

    $finish;
  end

endmodule
