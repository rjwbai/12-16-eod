`timescale 1ns / 1ps
import buffer_pkgs::*;

module alu #(
    parameter type AE = alu_entry_t,
    parameter type AO = alu_out_t
)(
    input  logic clk_i,
    input  logic reset_i,
    input  logic flush_i,

    // ===========================
    // Input ready-valid handshake
    // ===========================
    input  logic req_valid_i,
    output logic req_ready_o,

    // ============================
    // Output ready-valid handshake
    // ============================
    output logic resp_valid_o,
    input  logic resp_ready_i,

    input  AE data_i,
    output AO data_o
);

    // -------------------------
    // Core ALU control signals
    // -------------------------
    logic [3:0] aluop;
    assign aluop = data_i.aluop;

    logic [31:0] result_next;
    AO           alu_out_next;

    // -------------------------
    // Handshake / pipeline regs
    // -------------------------
    logic fire_in;   // accepted input
    logic fire_out;  // output consumed

    AO    data_q;
    logic resp_valid_q;

    assign data_o       = data_q;
    assign resp_valid_o = resp_valid_q;

    // Input can be accepted if not flushing and output reg is empty or will be freed
    assign req_ready_o = !flush_i && (!resp_valid_q || resp_ready_i);

    assign fire_in  = req_valid_i && req_ready_o;
    assign fire_out = resp_valid_q && resp_ready_i;

    // ==================================
    // Combinational: ALU operation
    // ==================================
    always_comb begin
        // CRITICAL: default the whole struct so no fields remain X
        alu_out_next = '0;

        // Default result to a known value (avoids X-prop if aluop is unexpected)
        result_next  = 32'h0000_0000;

        // Only build a "completed" output when we actually have a valid request.
        // (We still only latch on fire_in, but this prevents X math on invalid cycles.)
        if (req_valid_i) begin
            unique case (aluop)
                ADD: begin
                    // ADD / ADDI selection via rs2_used & imm_used
                    if (data_i.rs2_used)        result_next = $signed(data_i.rs1_val) + $signed(data_i.rs2_val);
                    else if (data_i.imm_used)   result_next = $signed(data_i.rs1_val) + $signed(data_i.imm_val);
                    else                        result_next = $signed(data_i.rs1_val); // benign
                end

                SUB:    result_next = $signed(data_i.rs1_val) - $signed(data_i.rs2_val);
                ANDD:   result_next = data_i.rs1_val & data_i.rs2_val;
                ORR:    result_next = data_i.rs1_val | data_i.imm_val;
                RSHIFT: result_next = $signed(data_i.rs1_val) >>> data_i.rs2_val[4:0];
                SLTIU:  result_next = ($unsigned(data_i.rs1_val) < $unsigned(data_i.imm_val)) ? 32'd1 : 32'd0;
                LUI:    result_next = data_i.imm_val;

                default: result_next = 32'h0000_0000;
            endcase

            // Populate output payload (all other fields already zeroed)
            alu_out_next.rd_addr   = data_i.rd_addr;
            alu_out_next.rd_val    = result_next;
            alu_out_next.ROB_tag   = data_i.ROB_tag;
            alu_out_next.completed = 1'b1;
        end

        // If flushing, mark not-completed (even if req_valid_i is high)
        if (flush_i) begin
            alu_out_next.completed = 1'b0;
        end
    end

    // ==================================
    // 1-cycle latency + output handshake
    // ==================================
    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            data_q       <= '0;
            resp_valid_q <= 1'b0;
        end else if (flush_i) begin
            // On flush, drop any buffered output
            data_q       <= '0;
            resp_valid_q <= 1'b0;
        end else begin
            // If consumer accepts the current result, clear valid
            if (fire_out) begin
                resp_valid_q <= 1'b0;
            end

            // If we accept a new request, overwrite output regs
            if (fire_in) begin
                if (fire_in) begin
                  if ($isunknown(data_i.rs1_val) || $isunknown(data_i.imm_val) || $isunknown(data_i.aluop)) begin
                    $display("[%0t][ALU FIRE_IN] X DETECTED: aluop=%h rs1_val=%h rs2_val=%h imm_val=%h rs2_used=%b imm_used=%b rd=%0d ROB=%0d",
                             $time, data_i.aluop, data_i.rs1_val, data_i.rs2_val, data_i.imm_val,
                             data_i.rs2_used, data_i.imm_used, data_i.rd_addr, data_i.ROB_tag);
                  end
                end
                data_q       <= alu_out_next;
                resp_valid_q <= 1'b1;
            end
        end
    end

endmodule
