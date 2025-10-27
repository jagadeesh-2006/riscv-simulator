// /**
//  * @file rvss_vm_pipelined.cpp
//  * @brief Simple 5-stage pipelined RVSS VM (no hazards, no forwarding)
//  */

// #include "vm/rvss/rvss_vm_pipelined.h"

// #include "common/instructions.h"
// #include "globals.h"
// #include "utils.h"

// #include <iostream>
// #include <tuple>

// using instruction_set::get_instr_encoding;
// using instruction_set::Instruction;

// RVSSVMPipelined::RVSSVMPipelined() : VmBase()
// {
//   // initialize PC, registers, etc. reuse VmBase defaults
// }

// RVSSVMPipelined::~RVSSVMPipelined() = default;

// void RVSSVMPipelined::Reset()
// {
//   VmBase::Reset();
//   if_id_ = IF_ID();
//   if_id_next_ = IF_ID();
//   id_ex_ = ID_EX();
//   id_ex_next_ = ID_EX();
//   ex_mem_ = EX_MEM();
//   ex_mem_next_ = EX_MEM();
//   mem_wb_ = MEM_WB();
//   mem_wb_next_ = MEM_WB();
//   pc_update_pending_ = false;
//   pc_update_value_ = 0;
//   stall_ = false;
// }

// // void RVSSVMPipelined::IF_stage()
// // {
// //   // Fetch instruction at program_counter_
// //   if (stall_)
// //   {
// //     // When stalled, freeze IF/ID (no new fetch, PC does not advance)
// //     if_id_next_ = if_id_;
// //     return;
// //   }

// //   if (!pc_update_pending_)
// //   {
// //     if (program_counter_ < program_size_)
// //     {
// //       if_id_next_.pc = program_counter_;
// //       if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
// //       if_id_next_.valid = true;
// //       // advance PC for next fetch
// //       program_counter_ += 4;
// //     }
// //     else
// //     {
// //       // No more instructions to fetch; mark IF/ID as invalid
// //       if_id_next_.valid = false;
// //     }
// //   }
// //   else
// //   {
// //     // If some other stage set PC (branch from EX), honor it next cycle
// //     program_counter_ = pc_update_value_;
// //     pc_update_pending_ = false;
// //     pc_update_value_ = 0;
// //     // fetch at new PC if within program bounds
// //     if (program_counter_ < program_size_)
// //     {
// //       if_id_next_.pc = program_counter_;
// //       if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
// //       if_id_next_.valid = true;
// //       program_counter_ += 4;
// //     }
// //     else
// //     {
// //       if_id_next_.valid = false;
// //     }
// //   }
// // }

// // void RVSSVMPipelined::ID_stage()
// // {
// //   if (!if_id_.valid)
// //   {
// //     id_ex_next_.valid = false;
// //     return;
// //   }

// //   uint32_t instr = if_id_.instruction;
// //   // detect load-use hazard: check EX stage (id_ex_) against current ID-stage sources
// //   uint8_t curr_rs1 = (instr >> 15) & 0b11111;
// //   uint8_t curr_rs2 = (instr >> 20) & 0b11111;
// //   if (hazard_unit_.DetectLoadUseHazard(id_ex_.rd, id_ex_.mem_read, curr_rs1, curr_rs2))
// //   {
// //     // Insert a stall: freeze IF/ID (handled in IF_stage) and insert bubble into ID/EX
// //     stall_ = true;
// //     // account for the stall
// //     stall_cycles_++;
// //     id_ex_next_ = ID_EX(); // bubble
// //     return;
// //   }

// //   id_ex_next_.valid = true;
// //   id_ex_next_.pc = if_id_.pc;
// //   id_ex_next_.instruction = instr;

// //   uint8_t opcode = instr & 0b1111111;
// //   id_ex_next_.rs1 = (instr >> 15) & 0b11111;
// //   id_ex_next_.rs2 = (instr >> 20) & 0b11111;
// //   id_ex_next_.rd = (instr >> 7) & 0b11111;
// //   id_ex_next_.funct3 = (instr >> 12) & 0b111;
// //   id_ex_next_.funct7 = (instr >> 25) & 0b1111111;
// //   id_ex_next_.imm = ImmGenerator(instr);

// //   // read registers
// //   id_ex_next_.reg1_value = registers_.ReadGpr(id_ex_next_.rs1);
// //   id_ex_next_.reg2_value = registers_.ReadGpr(id_ex_next_.rs2);

// //   // set control signals via control unit
// //   control_unit_.SetControlSignals(instr);
// //   id_ex_next_.reg_write = control_unit_.GetRegWrite();
// //   id_ex_next_.mem_read = control_unit_.GetMemRead();
// //   id_ex_next_.mem_write = control_unit_.GetMemWrite();
// //   id_ex_next_.mem_to_reg = control_unit_.GetMemToReg();
// //   id_ex_next_.alu_src = control_unit_.GetAluSrc();
// //   id_ex_next_.branch = control_unit_.GetBranch();
// //   id_ex_next_.is_float = instruction_set::isFInstruction(instr) || instruction_set::isDInstruction(instr);
// // }

// void RVSSVMPipelined::IF_stage()
// {
//     // When stalled, freeze IF/ID (no new fetch, PC does not advance)
//     if (stall_)
//     {
//         if_id_next_ = if_id_; // freeze IF/ID for one cycle
//         return;
//     }

//     // Normal instruction fetch logic
//     if (!pc_update_pending_)
//     {
//         if (program_counter_ < program_size_)
//         {
//             if_id_next_.pc = program_counter_;
//             if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
//             if_id_next_.valid = true;
//             program_counter_ += 4;
//         }
//         else
//         {
//             if_id_next_.valid = false;
//         }
//     }
//     else
//     {
//         // Branch taken: update program_counter and fetch new instruction
//         program_counter_ = pc_update_value_;
//         pc_update_pending_ = false;
//         pc_update_value_ = 0;
//         if (program_counter_ < program_size_)
//         {
//             if_id_next_.pc = program_counter_;
//             if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
//             if_id_next_.valid = true;
//             program_counter_ += 4;
//         }
//         else
//         {
//             if_id_next_.valid = false;
//         }
//     }
// }

// // void RVSSVMPipelined::ID_stage()
// // {
// //     // If IF/ID is invalid (bubble/drain), pass bubble into ID/EX
// //     if (!if_id_.valid)
// //     {
// //         id_ex_next_ = ID_EX(); // bubble downstream; set .valid to false
// //         return;
// //     }

// //     uint32_t instr = if_id_.instruction;
// //     uint8_t curr_rs1 = (instr >> 15) & 0b11111;
// //     uint8_t curr_rs2 = (instr >> 20) & 0b11111;

// //     // Load-use hazard detection (stall, bubble insertion for next cycle)
// //     if (hazard_unit_.DetectLoadUseHazard(id_ex_.rd, id_ex_.mem_read, curr_rs1, curr_rs2))
// //     {
// //         stall_ = true;          // Freeze IF/ID in IF_stage next cycle
// //         stall_cycles_++;        // Increment stall counter
// //         id_ex_next_ = ID_EX();  // Insert bubble into ID/EX for this cycle
// //         return;
// //     }

// //     // Normal operation: propagate instruction
// //     id_ex_next_.valid = true;
// //     id_ex_next_.pc = if_id_.pc;
// //     id_ex_next_.instruction = instr;

// //     // Extract fields
// //     id_ex_next_.rs1 = (instr >> 15) & 0b11111;
// //     id_ex_next_.rs2 = (instr >> 20) & 0b11111;
// //     id_ex_next_.rd = (instr >> 7) & 0b11111;
// //     id_ex_next_.funct3 = (instr >> 12) & 0b111;
// //     id_ex_next_.funct7 = (instr >> 25) & 0b1111111;
// //     id_ex_next_.imm = ImmGenerator(instr);

// //     // Register reads
// //     id_ex_next_.reg1_value = registers_.ReadGpr(id_ex_next_.rs1);
// //     id_ex_next_.reg2_value = registers_.ReadGpr(id_ex_next_.rs2);

// //     // Control signals
// //     control_unit_.SetControlSignals(instr);
// //     id_ex_next_.reg_write   = control_unit_.GetRegWrite();
// //     id_ex_next_.mem_read    = control_unit_.GetMemRead();
// //     id_ex_next_.mem_write   = control_unit_.GetMemWrite();
// //     id_ex_next_.mem_to_reg  = control_unit_.GetMemToReg();
// //     id_ex_next_.alu_src     = control_unit_.GetAluSrc();
// //     id_ex_next_.branch      = control_unit_.GetBranch();
// //     id_ex_next_.is_float    = instruction_set::isFInstruction(instr) || instruction_set::isDInstruction(instr);
// // }
// void RVSSVMPipelined::ID_stage()
// {
//     if (!if_id_.valid) {
//         id_ex_next_ = ID_EX();
//         return;
//     }
//     uint32_t instr = if_id_.instruction;
//     uint8_t curr_rs1 = (instr >> 15) & 0b11111;
//     uint8_t curr_rs2 = (instr >> 20) & 0b11111;

//     // Enable/disable hazard usage based on mode
//     if (hazard_detection_enabled_ &&
//         hazard_unit_.DetectLoadUseHazard(id_ex_.rd, id_ex_.mem_read, curr_rs1, curr_rs2)) {
//         stall_ = true;
//         stall_cycles_++;
//         id_ex_next_ = ID_EX(); // bubble
//         return;
//     }
//     // Normal decode & propagate
//     id_ex_next_.valid = true;
//     id_ex_next_.pc = if_id_.pc;
//     id_ex_next_.instruction = instr;
//     id_ex_next_.rs1 = (instr >> 15) & 0b11111;
//     id_ex_next_.rs2 = (instr >> 20) & 0b11111;
//     id_ex_next_.rd = (instr >> 7) & 0b11111;
//     id_ex_next_.funct3 = (instr >> 12) & 0b111;
//     id_ex_next_.funct7 = (instr >> 25) & 0b1111111;
//     id_ex_next_.imm = ImmGenerator(instr);
//     id_ex_next_.reg1_value = registers_.ReadGpr(id_ex_next_.rs1);
//     id_ex_next_.reg2_value = registers_.ReadGpr(id_ex_next_.rs2);
//     control_unit_.SetControlSignals(instr);
//     id_ex_next_.reg_write = control_unit_.GetRegWrite();
//     id_ex_next_.mem_read = control_unit_.GetMemRead();
//     id_ex_next_.mem_write = control_unit_.GetMemWrite();
//     id_ex_next_.mem_to_reg = control_unit_.GetMemToReg();
//     id_ex_next_.alu_src = control_unit_.GetAluSrc();
//     id_ex_next_.branch = control_unit_.GetBranch();
//     id_ex_next_.is_float = instruction_set::isFInstruction(instr) || instruction_set::isDInstruction(instr);
// }

// void RVSSVMPipelined::EX_stage()
// {
//   if (!id_ex_.valid)
//   {
//     ex_mem_next_.valid = false;
//     return;
//   }

//   ex_mem_next_.valid = true;
//   ex_mem_next_.pc = id_ex_.pc;
//   ex_mem_next_.instruction = id_ex_.instruction;
//   ex_mem_next_.rd = id_ex_.rd;
//   ex_mem_next_.reg_write = id_ex_.reg_write;
//   ex_mem_next_.mem_read = id_ex_.mem_read;
//   ex_mem_next_.mem_write = id_ex_.mem_write;
//   ex_mem_next_.mem_to_reg = id_ex_.mem_to_reg;
//   ex_mem_next_.reg2_value = id_ex_.reg2_value;

//   // select alu input
//   uint64_t op1 = id_ex_.reg1_value;
//   uint64_t op2 = id_ex_.alu_src ? static_cast<uint64_t>(static_cast<int64_t>(id_ex_.imm)) : id_ex_.reg2_value;

//   // for floating point and CSR some instructions are handled elsewhere in single-cycle; here we'll call the ALU generically
//   alu::AluOp aluOperation = control_unit_.GetAluSignal(id_ex_.instruction, control_unit_.GetAluOp());
//   bool overflow = false;
//   std::tie(ex_mem_next_.alu_result, overflow) = alu_.execute(aluOperation, op1, op2);

//   // branch handling: compute branch target and set PC update pending immediately
//   if (id_ex_.branch)
//   {
//     int32_t imm = id_ex_.imm;
//     // simple branch decision similar to single-cycle: rely on alu result
//     uint8_t funct3 = id_ex_.funct3;
//     bool take = false;
//     switch (funct3)
//     {
//     case 0b000:
//       take = (ex_mem_next_.alu_result == 0);
//       break; // BEQ
//     case 0b001:
//       take = (ex_mem_next_.alu_result != 0);
//       break; // BNE
//     case 0b100:
//       take = (ex_mem_next_.alu_result == 1);
//       break; // BLT (alu gave slt)
//     case 0b101:
//       take = (ex_mem_next_.alu_result == 0);
//       break; // BGE
//     case 0b110:
//       take = (ex_mem_next_.alu_result == 1);
//       break; // BLTU
//     case 0b111:
//       take = (ex_mem_next_.alu_result == 0);
//       break; // BGEU
//     default:
//       take = false;
//       break;
//     }

//     if (take)
//     {
//       // branch target computed relative to id_ex_.pc (which is inst addr)
//       ex_mem_next_.branch_taken = true;
//       ex_mem_next_.branch_target = static_cast<uint64_t>(static_cast<int64_t>(id_ex_.pc) + id_ex_.imm);
//       // set PC to target for next IF
//       pc_update_pending_ = true;
//       pc_update_value_ = ex_mem_next_.branch_target;
//     }
//     else
//     {
//       ex_mem_next_.branch_taken = false;
//     }
//   }
//   else
//   {
//     ex_mem_next_.branch_taken = false;
//   }
// }

// void RVSSVMPipelined::MEM_stage()
// {
//   if (!ex_mem_.valid)
//   {
//     mem_wb_next_.valid = false;
//     return;
//   }

//   mem_wb_next_.valid = true;
//   mem_wb_next_.rd = ex_mem_.rd;
//   mem_wb_next_.reg_write = ex_mem_.reg_write;
//   mem_wb_next_.mem_to_reg = ex_mem_.mem_to_reg;
//   mem_wb_next_.alu_result = ex_mem_.alu_result;

//   // memory operations
//   if (ex_mem_.mem_read)
//   {
//     uint8_t funct3 = (ex_mem_.instruction >> 12) & 0b111;
//     switch (funct3)
//     {
//     case 0b000:
//       mem_wb_next_.mem_data = static_cast<int8_t>(memory_controller_.ReadByte(ex_mem_.alu_result));
//       break; // LB
//     case 0b001:
//       mem_wb_next_.mem_data = static_cast<int16_t>(memory_controller_.ReadHalfWord(ex_mem_.alu_result));
//       break; // LH
//     case 0b010:
//       mem_wb_next_.mem_data = static_cast<int32_t>(memory_controller_.ReadWord(ex_mem_.alu_result));
//       break; // LW
//     case 0b011:
//       mem_wb_next_.mem_data = memory_controller_.ReadDoubleWord(ex_mem_.alu_result);
//       break; // LD
//     case 0b100:
//       mem_wb_next_.mem_data = memory_controller_.ReadByte(ex_mem_.alu_result);
//       break; // LBU
//     case 0b101:
//       mem_wb_next_.mem_data = memory_controller_.ReadHalfWord(ex_mem_.alu_result);
//       break; // LHU
//     case 0b110:
//       mem_wb_next_.mem_data = memory_controller_.ReadWord(ex_mem_.alu_result);
//       break; // LWU
//     default:
//       mem_wb_next_.mem_data = 0;
//       break;
//     }
//   }

//   if (ex_mem_.mem_write)
//   {
//     uint8_t funct3 = (ex_mem_.instruction >> 12) & 0b111;
//     switch (funct3)
//     {
//     case 0b000:
//       memory_controller_.WriteByte(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFF);
//       break; // SB
//     case 0b001:
//       memory_controller_.WriteHalfWord(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFFFF);
//       break; // SH
//     case 0b010:
//       memory_controller_.WriteWord(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFFFFFFFF);
//       break; // SW
//     case 0b011:
//       memory_controller_.WriteDoubleWord(ex_mem_.alu_result, ex_mem_.reg2_value);
//       break; // SD
//     default:
//       break;
//     }
//   }
// }

// void RVSSVMPipelined::WB_stage()
// {
//   // if (!mem_wb_.valid)
//   //   return;

//   // if (mem_wb_.reg_write)
//   // {
//   //   uint64_t write_val = mem_wb_.mem_to_reg ? mem_wb_.mem_data : mem_wb_.alu_result;
//   //   // zero register x0 should remain zero
//   //   if (mem_wb_.rd != 0)
//   //   {
//   //     uint64_t old = registers_.ReadGpr(mem_wb_.rd);
//   //     registers_.WriteGpr(mem_wb_.rd, write_val);
//   //     if (old != write_val)
//   //     {
//   //       // no undo/redo tracking in pipelined stub; could be added later
//   //     }
//   //   }
//   // }
//   if (!mem_wb_.valid)
//     return;

// if (mem_wb_.reg_write) {
//     uint64_t write_val = mem_wb_.mem_to_reg ? mem_wb_.mem_data : mem_wb_.alu_result;
//     if (mem_wb_.rd != 0) {
//         uint64_t old = registers_.ReadGpr(mem_wb_.rd);
//         registers_.WriteGpr(mem_wb_.rd, write_val);
//         // Optional: old != write_val change tracking
//     }
// }

// }

// // void RVSSVMPipelined::advance_pipeline_registers()
// // {
// //   // advance pipeline: WB consumes mem_wb_, MEM consumes ex_mem_, etc.
// //   mem_wb_ = mem_wb_next_;
// //   ex_mem_ = ex_mem_next_;
// //   id_ex_ = id_ex_next_;
// //   if_id_ = if_id_next_;

// //   // clear next stage holders
// //   mem_wb_next_ = MEM_WB();
// //   ex_mem_next_ = EX_MEM();
// //   id_ex_next_ = ID_EX();
// //   if_id_next_ = IF_ID();
// //   // clear stall after registers have advanced (one-cycle stall)
// //   stall_ = false;
// // }
// void RVSSVMPipelined::advance_pipeline_registers()
// {
//     // Advance pipeline registers: next becomes current for each stage
//     mem_wb_ = mem_wb_next_;
//     ex_mem_ = ex_mem_next_;
//     id_ex_  = id_ex_next_;
//     if_id_  = if_id_next_;

//     // Clear "next" stage holders so old data isn't reused next cycle
//     mem_wb_next_ = MEM_WB();
//     ex_mem_next_ = EX_MEM();
//     id_ex_next_  = ID_EX();
//     if_id_next_  = IF_ID();

//     // Clear stall after the pipeline has advanced (one-cycle stall only)
//     stall_ = false;
// }


// // void RVSSVMPipelined::Run()
// // {
// //   stop_requested_ = false;
// //   uint64_t instruction_executed = 0;

// //   // pipeline warmup: ensure registers are cleared
// //   Reset();

// //   // Run until stop requested AND the pipeline has drained after last fetch.
// //   while (!stop_requested_ && (program_counter_ < program_size_ || if_id_.valid || id_ex_.valid || ex_mem_.valid || mem_wb_.valid))
// //   {
// //     if (instruction_executed > vm_config::config.getInstructionExecutionLimit())
// //       break;

// //     // Progress pipeline in reverse order to model same-cycle behavior
// //     WB_stage();
// //     MEM_stage();
// //     EX_stage();
// //     ID_stage();
// //     IF_stage();

// //     // Count any instruction that reached WB as retired (mem_wb_.valid)
// //     // WB_stage() above consumed the current mem_wb_, so capture that
// //     // retired status before we advance pipeline registers.
// //     instructions_retired_ += (mem_wb_.valid) ? 1 : 0;

// //     // Advance pipeline registers (this moves mem_wb_next_ into mem_wb_ etc.)
// //     advance_pipeline_registers();

// //     instruction_executed++;
// //     // One cycle elapsed for this pipeline progress. Stall cycles are
// //     // already recorded separately (stall_cycles_) and correspond to
// //     // iterations where ID_stage inserted a bubble; do not double-count
// //     // them here — cycle_s_ should be incremented once per loop.
// //     cycle_s_++;
// //   }

// //   // Final performance metrics update
// //   if (instructions_retired_ > 0)
// //   {
// //     // cycle_s_ already counts all cycles (including those where a stall
// //     // was inserted). Use those values to compute CPI and IPC.
// //     cpi_ = static_cast<float>(cycle_s_) / instructions_retired_;
// //     ipc_ = static_cast<float>(instructions_retired_) / static_cast<float>(cycle_s_);
// //   }

// //   if (program_counter_ >= program_size_)
// //   {
// //     std::cout << "VM_PROGRAM_END" << std::endl;
// //     output_status_ = "VM_PROGRAM_END";
// //   }

// //   // Dump final state to files
// //   DumpRegisters(globals::registers_dump_file_path, registers_);
// //   DumpState(globals::vm_state_dump_file_path);
// // }
// void RVSSVMPipelined::Run()
// {
//     stop_requested_ = false;
//     uint64_t instruction_executed = 0;

//     // Pipeline warmup: ensure registers and pipeline state are cleared
//     Reset();

//     // Run until stop requested OR pipeline is completely drained
//     while (!stop_requested_ &&
//            (program_counter_ < program_size_ ||
//             if_id_.valid || id_ex_.valid || ex_mem_.valid || mem_wb_.valid))
//     {
//         // Optional hard execution limit (prevent runaway)
//         if (instruction_executed > vm_config::config.getInstructionExecutionLimit())
//             break;

//         // Execute all pipeline stages in order (WB first!)
//         WB_stage();
//         MEM_stage();
//         EX_stage();
//         ID_stage();
//         IF_stage();

//         // Retire any instruction that reaches WB stage this cycle
//         instructions_retired_ += (mem_wb_.valid) ? 1 : 0;

//         // Advance pipeline registers, for next cycle's exec
//         advance_pipeline_registers();

//         instruction_executed++;
//         // Count one cycle per pipeline advancement, not per stall
//         cycle_s_++;
//         // Stall cycles already counted separately in ID_stage (stall_cycles_)
//     }

//     // Final CPI and IPC calculation
//     if (instructions_retired_ > 0)
//     {
//         cpi_ = static_cast<float>(cycle_s_) / static_cast<float>(instructions_retired_);
//         ipc_ = static_cast<float>(instructions_retired_) / static_cast<float>(cycle_s_);
//     }

//     // Completion message
//     if (program_counter_ >= program_size_)
//     {
//         std::cout << "VM_PROGRAM_END" << std::endl;
//         output_status_ = "VM_PROGRAM_END";
//     }

//     // Dump the final register and pipeline state to file for checking
//     DumpRegisters(globals::registers_dump_file_path, registers_);
//     DumpState(globals::vm_state_dump_file_path);
// }

// void RVSSVMPipelined::DebugRun()
// {
//   // For now, DebugRun will behave same as Run but step with some printing
//   Run();
// }

// void RVSSVMPipelined::Step()
// {
//   // Single cycle step: run one cycle of pipeline
//   WB_stage();
//   MEM_stage();
//   EX_stage();
//   ID_stage();
//   IF_stage();
//   advance_pipeline_registers();
//   // Advance cycle count for this step
//   cycle_s_++;
// }

// void RVSSVMPipelined::Undo()
// {
//   std::cerr << "Undo not supported in pipelined VM stub" << std::endl;
// }

// void RVSSVMPipelined::Redo()
// {
//   std::cerr << "Redo not supported in pipelined VM stub" << std::endl;
// }


/**
 * @file rvss_vm_pipelined.cpp
 * @brief Simple 5-stage pipelined RVSS VM (no hazards, no forwarding, hazard configurable)
 */

#include "vm/rvss/rvss_vm_pipelined.h"
#include "common/instructions.h"
#include "globals.h"
#include "utils.h"

#include <iostream>
#include <tuple>

using instruction_set::get_instr_encoding;
using instruction_set::Instruction;

RVSSVMPipelined::RVSSVMPipelined() : VmBase() {
    // Do not redeclare hazard_detection_enabled_ here, class member only!
}

RVSSVMPipelined::~RVSSVMPipelined() = default;

void RVSSVMPipelined::Reset() {
    VmBase::Reset();
    if_id_ = IF_ID();
    if_id_next_ = IF_ID();
    id_ex_ = ID_EX();
    id_ex_next_ = ID_EX();
    ex_mem_ = EX_MEM();
    ex_mem_next_ = EX_MEM();
    mem_wb_ = MEM_WB();
    mem_wb_next_ = MEM_WB();
    pc_update_pending_ = false;
    pc_update_value_ = 0;
    stall_ = false;
    // Do not redeclare hazard_detection_enabled_ here!
}

void RVSSVMPipelined::IF_stage() {
    if (stall_) {
        if_id_next_ = if_id_; // freeze IF/ID for one cycle
        return;
    }
    if (!pc_update_pending_) {
        if (program_counter_ < program_size_) {
            if_id_next_.pc = program_counter_;
            if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
            if_id_next_.valid = true;
            program_counter_ += 4;
        } else {
            if_id_next_.valid = false;
        }
    } else {
        program_counter_ = pc_update_value_;
        pc_update_pending_ = false;
        pc_update_value_ = 0;
        if (program_counter_ < program_size_) {
            if_id_next_.pc = program_counter_;
            if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
            if_id_next_.valid = true;
            program_counter_ += 4;
        } else {
            if_id_next_.valid = false;
        }
    }
}

void RVSSVMPipelined::ID_stage() {
   std::cout << "haz flag address " << (void*)&hazard_detection_enabled_ << ", value: " << hazard_detection_enabled_ << std::endl;
    if (!if_id_.valid) {
        id_ex_next_ = ID_EX();
        return;
    }
    uint32_t instr = if_id_.instruction;
    uint8_t curr_rs1 = (instr >> 15) & 0b11111;
    uint8_t curr_rs2 = (instr >> 20) & 0b11111;
    if (hazard_detection_enabled_ &&
        hazard_unit_.DetectLoadUseHazard(id_ex_.rd, id_ex_.mem_read, curr_rs1, curr_rs2)) {
          std::cout << "DATA HAZARD detected, stalling\n";
        stall_ = true;
        stall_cycles_++;
        id_ex_next_ = ID_EX();
        return;
    }
    id_ex_next_.valid = true;
    id_ex_next_.pc = if_id_.pc;
    id_ex_next_.instruction = instr;
    id_ex_next_.rs1 = curr_rs1;
    id_ex_next_.rs2 = curr_rs2;
    id_ex_next_.rd = (instr >> 7) & 0b11111;
    id_ex_next_.funct3 = (instr >> 12) & 0b111;
    id_ex_next_.funct7 = (instr >> 25) & 0b1111111;
    id_ex_next_.imm = ImmGenerator(instr);
    id_ex_next_.reg1_value = registers_.ReadGpr(id_ex_next_.rs1);
    id_ex_next_.reg2_value = registers_.ReadGpr(id_ex_next_.rs2);
    control_unit_.SetControlSignals(instr);
    id_ex_next_.reg_write   = control_unit_.GetRegWrite();
    id_ex_next_.mem_read    = control_unit_.GetMemRead();
    id_ex_next_.mem_write   = control_unit_.GetMemWrite();
    id_ex_next_.mem_to_reg  = control_unit_.GetMemToReg();
    id_ex_next_.alu_src     = control_unit_.GetAluSrc();
    id_ex_next_.branch      = control_unit_.GetBranch();
    id_ex_next_.is_float    = instruction_set::isFInstruction(instr) ||
                              instruction_set::isDInstruction(instr);
}

void RVSSVMPipelined::EX_stage() {
    if (!id_ex_.valid) {
        ex_mem_next_.valid = false;
        return;
    }
    ex_mem_next_.valid = true;
    ex_mem_next_.pc = id_ex_.pc;
    ex_mem_next_.instruction = id_ex_.instruction;
    ex_mem_next_.rd = id_ex_.rd;
    ex_mem_next_.reg_write = id_ex_.reg_write;
    ex_mem_next_.mem_read = id_ex_.mem_read;
    ex_mem_next_.mem_write = id_ex_.mem_write;
    ex_mem_next_.mem_to_reg = id_ex_.mem_to_reg;
    ex_mem_next_.reg2_value = id_ex_.reg2_value;
    uint64_t op1 = id_ex_.reg1_value;
    uint64_t op2 = id_ex_.alu_src ? static_cast<uint64_t>(static_cast<int64_t>(id_ex_.imm)) : id_ex_.reg2_value;
    alu::AluOp aluOperation = control_unit_.GetAluSignal(id_ex_.instruction, control_unit_.GetAluOp());
    bool overflow = false;
    std::tie(ex_mem_next_.alu_result, overflow) = alu_.execute(aluOperation, op1, op2);
    if (id_ex_.branch) {
        int32_t imm = id_ex_.imm;
        uint8_t funct3 = id_ex_.funct3;
        bool take = false;
        switch (funct3) {
            case 0b000: take = (ex_mem_next_.alu_result == 0); break;
            case 0b001: take = (ex_mem_next_.alu_result != 0); break;
            case 0b100: take = (ex_mem_next_.alu_result == 1); break;
            case 0b101: take = (ex_mem_next_.alu_result == 0); break;
            case 0b110: take = (ex_mem_next_.alu_result == 1); break;
            case 0b111: take = (ex_mem_next_.alu_result == 0); break;
            default: take = false; break;
        }
        if (take) {
            ex_mem_next_.branch_taken = true;
            ex_mem_next_.branch_target = static_cast<uint64_t>(static_cast<int64_t>(id_ex_.pc) + id_ex_.imm);
            pc_update_pending_ = true;
            pc_update_value_ = ex_mem_next_.branch_target;
        } else {
            ex_mem_next_.branch_taken = false;
        }
    } else {
        ex_mem_next_.branch_taken = false;
    }
}

void RVSSVMPipelined::MEM_stage() {
    if (!ex_mem_.valid) {
        mem_wb_next_.valid = false;
        return;
    }
    mem_wb_next_.valid = true;
    mem_wb_next_.rd = ex_mem_.rd;
    mem_wb_next_.reg_write = ex_mem_.reg_write;
    mem_wb_next_.mem_to_reg = ex_mem_.mem_to_reg;
    mem_wb_next_.alu_result = ex_mem_.alu_result;
    if (ex_mem_.mem_read) {
        uint8_t funct3 = (ex_mem_.instruction >> 12) & 0b111;
        switch (funct3) {
            case 0b000: mem_wb_next_.mem_data = static_cast<int8_t>(memory_controller_.ReadByte(ex_mem_.alu_result)); break;
            case 0b001: mem_wb_next_.mem_data = static_cast<int16_t>(memory_controller_.ReadHalfWord(ex_mem_.alu_result)); break;
            case 0b010: mem_wb_next_.mem_data = static_cast<int32_t>(memory_controller_.ReadWord(ex_mem_.alu_result)); break;
            case 0b011: mem_wb_next_.mem_data = memory_controller_.ReadDoubleWord(ex_mem_.alu_result); break;
            case 0b100: mem_wb_next_.mem_data = memory_controller_.ReadByte(ex_mem_.alu_result); break;
            case 0b101: mem_wb_next_.mem_data = memory_controller_.ReadHalfWord(ex_mem_.alu_result); break;
            case 0b110: mem_wb_next_.mem_data = memory_controller_.ReadWord(ex_mem_.alu_result); break;
            default: mem_wb_next_.mem_data = 0; break;
        }
    }
    if (ex_mem_.mem_write) {
        uint8_t funct3 = (ex_mem_.instruction >> 12) & 0b111;
        switch (funct3) {
            case 0b000: memory_controller_.WriteByte(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFF); break;
            case 0b001: memory_controller_.WriteHalfWord(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFFFF); break;
            case 0b010: memory_controller_.WriteWord(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFFFFFFFF); break;
            case 0b011: memory_controller_.WriteDoubleWord(ex_mem_.alu_result, ex_mem_.reg2_value); break;
            default: break;
        }
    }
}

void RVSSVMPipelined::WB_stage() {
    if (!mem_wb_.valid) return;
    if (mem_wb_.reg_write && mem_wb_.rd != 0) {
        uint64_t write_val = mem_wb_.mem_to_reg ? mem_wb_.mem_data : mem_wb_.alu_result;
        registers_.WriteGpr(mem_wb_.rd, write_val);
    }
}

void RVSSVMPipelined::advance_pipeline_registers() {
    mem_wb_ = mem_wb_next_;
    ex_mem_ = ex_mem_next_;
    id_ex_  = id_ex_next_;
    if_id_  = if_id_next_;
    mem_wb_next_ = MEM_WB();
    ex_mem_next_ = EX_MEM();
    id_ex_next_  = ID_EX();
    if_id_next_  = IF_ID();
    stall_ = false;
}

void RVSSVMPipelined::Run() {
    stop_requested_ = false;
    uint64_t instruction_executed = 0;
    Reset();

    while (!stop_requested_ &&
           (program_counter_ < program_size_ ||
            if_id_.valid || id_ex_.valid || ex_mem_.valid || mem_wb_.valid)) {
        if (instruction_executed > vm_config::config.getInstructionExecutionLimit())
            break;
        WB_stage();
        MEM_stage();
        EX_stage();
        ID_stage();
        IF_stage();
        instructions_retired_ += (mem_wb_.valid) ? 1 : 0;
        advance_pipeline_registers();
        instruction_executed++;
        cycle_s_++;
    }
    if (instructions_retired_ > 0) {
        cpi_ = static_cast<float>(cycle_s_) / static_cast<float>(instructions_retired_);
        ipc_ = static_cast<float>(instructions_retired_) / static_cast<float>(cycle_s_);
    }
    if (program_counter_ >= program_size_) {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSVMPipelined::DebugRun() {
    Run();
}

void RVSSVMPipelined::Step() {
    WB_stage();
    MEM_stage();
    EX_stage();
    ID_stage();
    IF_stage();
    advance_pipeline_registers();
    cycle_s_++;
}
void RVSSVMPipelined::Undo() { std::cerr << "Undo not supported in pipelined VM stub" << std::endl; }
void RVSSVMPipelined::Redo() { std::cerr << "Redo not supported in pipelined VM stub" << std::endl; }
