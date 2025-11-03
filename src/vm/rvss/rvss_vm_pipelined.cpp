
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

RVSSVMPipelined::RVSSVMPipelined() : VmBase()
{
    // Do not redeclare hazard_detection_enabled_ here, class member only!
}

RVSSVMPipelined::~RVSSVMPipelined() = default;
// small helper to print a quick register snapshot (first 16 regs)
static void PrintGprSnapshot(const RegisterFile &regs, uint64_t cycle)
{
    std::cout << "[REGS] cycle=" << cycle << " snapshot x0..x15: ";
    auto gp = regs.GetGprValues();
    for (size_t i = 0; i <= 15 && i < gp.size(); ++i)
    {
        std::cout << "x" << i << "=0x" << std::hex << gp[i] << " ";
    }
    std::cout << std::dec << "\n";
}

// Print the contents of pipeline registers and a small snapshot of GPRs
void RVSSVMPipelined::PrintPipelineState()
{
    std::cout << "[PIPE] cycle=" << cycle_s_ << " IF_ID.valid=" << if_id_.valid
              << " ID_EX.valid=" << id_ex_.valid
              << " EX_MEM.valid=" << ex_mem_.valid
              << " MEM_WB.valid=" << mem_wb_.valid << "\n";

    if (if_id_.valid)
    {
        std::cout << "  IF/ID: pc=0x" << std::hex << if_id_.pc << " instr=0x" << if_id_.instruction << std::dec << "\n";
    }
    if (id_ex_.valid)
    {
        std::cout << "  ID/EX: pc=0x" << std::hex << id_ex_.pc << std::dec
                  << " rs1=x" << int(id_ex_.rs1) << "=0x" << std::hex << id_ex_.reg1_value
                  << " rs2=x" << int(id_ex_.rs2) << "=0x" << id_ex_.reg2_value << std::dec << "\n";
    }
    if (ex_mem_.valid)
    {
        std::cout << "  EX/MEM: rd=x" << int(ex_mem_.rd) << " alu_res=0x" << std::hex << ex_mem_.alu_result << std::dec << "\n";
    }
    if (mem_wb_.valid)
    {
        std::cout << "  MEM/WB: rd=x" << int(mem_wb_.rd) << " alu_res=0x" << std::hex << mem_wb_.alu_result << std::dec << "\n";
    }

    // small GPR snapshot
    PrintGprSnapshot(registers_, cycle_s_);
}

void RVSSVMPipelined::Reset()
{
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

void RVSSVMPipelined::IF_stage()
{
    if (stall_)
    {
        if_id_next_ = if_id_; // freeze IF/ID for one cycle
        return;
    }
    if (!pc_update_pending_)
    {
        if (program_counter_ < program_size_)
        {
            if_id_next_.pc = program_counter_;
            if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
            if_id_next_.valid = true;
            program_counter_ += 4;
        }
        else
        {
            if_id_next_.valid = false;
        }
    }
    else
    {
        program_counter_ = pc_update_value_;
        pc_update_pending_ = false;
        pc_update_value_ = 0;
        if (program_counter_ < program_size_)
        {
            if_id_next_.pc = program_counter_;
            if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
            if_id_next_.valid = true;
            program_counter_ += 4;
        }
        else
        {
            if_id_next_.valid = false;
        }
    }
}

void RVSSVMPipelined::ID_stage()
{
    if (!if_id_.valid)
    {
        id_ex_next_ = ID_EX();
        return;
    }
    uint32_t instr = if_id_.instruction;
    uint8_t curr_rs1 = (instr >> 15) & 0b11111;
    uint8_t curr_rs2 = (instr >> 20) & 0b11111;

    // Check for system call (ecall)
    uint32_t opcode = instr & 0x7f;
    uint8_t funct3 = (instr >> 12) & 0x7;
    auto ecall_encoding = get_instr_encoding(Instruction::kecall);
    id_ex_next_.is_syscall = (opcode == static_cast<uint32_t>(ecall_encoding.opcode) &&
                              funct3 == static_cast<uint8_t>(ecall_encoding.funct3));

    if (hazard_detection_enabled_)
    {
        // Without forwarding, must stall for all hazards
        bool load_use = hazard_unit_.DetectLoadUseHazard(id_ex_.rd, id_ex_.mem_read, curr_rs1, curr_rs2);
        bool ex_hazard = hazard_unit_.DetectEXHazard(id_ex_.rd, id_ex_.reg_write, curr_rs1, curr_rs2);
        bool mem_hazard = hazard_unit_.DetectMEMHazard(ex_mem_.rd, ex_mem_.reg_write, curr_rs1, curr_rs2);

        if (load_use || ex_hazard || mem_hazard)
        {
            std::cout << "[HAZARD] Detected hazard in ID stage: "
                      << (load_use ? "LOAD-USE " : "")
                      << (ex_hazard ? "EX " : "")
                      << (mem_hazard ? "MEM " : "")
                      << "for rs1=x" << int(curr_rs1)
                      << " rs2=x" << int(curr_rs2)
                      << " with EX.rd=x" << int(id_ex_.rd)
                      << " MEM.rd=x" << int(ex_mem_.rd)
                      << "\n";

            // Stall the pipeline by inserting a bubble into ID/EX
            stall_ = true;
            stall_cycles_++;
            id_ex_next_ = ID_EX(); // insert bubble
            return;
        }
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
    std::cout << "[ID] pc=0x" << std::hex << if_id_.pc << std::dec
              << " instr=0x" << std::hex << instr << std::dec
              << " rs1=x" << int(id_ex_next_.rs1) << "=0x" << std::hex << id_ex_next_.reg1_value
              << " rs2=x" << int(id_ex_next_.rs2) << "=0x" << std::hex << id_ex_next_.reg2_value
              << " rd=x" << int(id_ex_next_.rd)
              << " reg_write=" << int(control_unit_.GetRegWrite())
              << "\n";
    control_unit_.SetControlSignals(instr);
    id_ex_next_.reg_write = control_unit_.GetRegWrite();
    id_ex_next_.mem_read = control_unit_.GetMemRead();
    id_ex_next_.mem_write = control_unit_.GetMemWrite();
    id_ex_next_.mem_to_reg = control_unit_.GetMemToReg();
    id_ex_next_.alu_src = control_unit_.GetAluSrc();
    id_ex_next_.branch = control_unit_.GetBranch();
    id_ex_next_.is_float = instruction_set::isFInstruction(instr) ||
                           instruction_set::isDInstruction(instr);
}

void RVSSVMPipelined::EX_stage()
{
    if (!id_ex_.valid)
    {
        ex_mem_next_.valid = false;
        return;
    }
    ex_mem_next_.valid = true;
    ex_mem_next_.pc = id_ex_.pc;
    ex_mem_next_.instruction = id_ex_.instruction;
    ex_mem_next_.rd = id_ex_.rd;
    ex_mem_next_.reg_write = id_ex_.reg_write;
    ex_mem_next_.mem_read = id_ex_.mem_read;

    // Use values directly from ID stage (no forwarding)
    uint64_t op1 = id_ex_.reg1_value;
    uint64_t op2 = id_ex_.alu_src ? static_cast<uint64_t>(static_cast<int64_t>(id_ex_.imm)) : id_ex_.reg2_value;

    // Handle system calls
    if (id_ex_.is_syscall)
    {
        uint64_t syscall_number = registers_.ReadGpr(17); // a7 contains syscall number

        switch (syscall_number)
        {
        case SYSCALL_WRITE:
        {
            uint64_t fd = registers_.ReadGpr(10);
            uint64_t buf = registers_.ReadGpr(11);
            uint64_t len = registers_.ReadGpr(12);
            uint64_t bytes_written = 0;

            if (fd == 1)
            { // stdout
                std::cout << "VM_STDOUT_START";
                output_status_ = "VM_STDOUT_START";
                for (uint64_t i = 0; i < len; ++i)
                {
                    char c = memory_controller_.ReadByte(buf + i);
                    std::cout << c;
                    bytes_written++;
                }
                std::cout << std::flush;
                output_status_ = "VM_STDOUT_END";
                std::cout << "VM_STDOUT_END" << std::endl;

                // Set up result to be written back through pipeline
                ex_mem_next_.alu_result = bytes_written;
                ex_mem_next_.reg_write = true;
            }
            break;
        }
        case SYSCALL_EXIT:
        {
            std::cout << "VM_EXIT" << std::endl;
            output_status_ = "VM_EXIT";
            stop_requested_ = true;
            ex_mem_next_.reg_write = false;
            break;
        }
        // Other syscalls would go here
        default:
            std::cerr << "Unsupported syscall in pipelined mode: " << syscall_number << std::endl;
            ex_mem_next_.reg_write = false;
            break;
        }
        return;
    }
    ex_mem_next_.mem_write = id_ex_.mem_write;
    ex_mem_next_.mem_to_reg = id_ex_.mem_to_reg;
    ex_mem_next_.reg2_value = id_ex_.reg2_value;
    // Use forwarded op1 and alu_op2 values
    alu::AluOp aluOperation = control_unit_.GetAluSignal(id_ex_.instruction, control_unit_.GetAluOp());
    bool overflow = false;
    std::tie(ex_mem_next_.alu_result, overflow) = alu_.execute(aluOperation, op1, op2);
    // Debug: ALU / EX output
    std::cout << "[EX] pc=0x" << std::hex << id_ex_.pc << std::dec
              << " instr=0x" << std::hex << id_ex_.instruction << std::dec
              << " rd=x" << int(id_ex_.rd)
              << " alu_result=0x" << std::hex << ex_mem_next_.alu_result
              << " reg_write=" << int(id_ex_.reg_write)
              << " mem_read=" << int(id_ex_.mem_read)
              << " mem_write=" << int(id_ex_.mem_write)
              << " mem_to_reg=" << int(id_ex_.mem_to_reg)
              << "\n";

    if (id_ex_.branch)
    {
        // int32_t imm = id_ex_.imm;
        uint8_t funct3 = id_ex_.funct3;
        bool take = false;
        switch (funct3)
        {
        case 0b000:
            take = (ex_mem_next_.alu_result == 0);
            break;
        case 0b001:
            take = (ex_mem_next_.alu_result != 0);
            break;
        case 0b100:
            take = (ex_mem_next_.alu_result == 1);
            break;
        case 0b101:
            take = (ex_mem_next_.alu_result == 0);
            break;
        case 0b110:
            take = (ex_mem_next_.alu_result == 1);
            break;
        case 0b111:
            take = (ex_mem_next_.alu_result == 0);
            break;
        default:
            take = false;
            break;
        }
        if (take)
        {
            ex_mem_next_.branch_taken = true;
            ex_mem_next_.branch_target = static_cast<uint64_t>(static_cast<int64_t>(id_ex_.pc) + id_ex_.imm);
            pc_update_pending_ = true;
            pc_update_value_ = ex_mem_next_.branch_target;
        }
        else
        {
            ex_mem_next_.branch_taken = false;
        }
    }
    else
    {
        ex_mem_next_.branch_taken = false;
    }
}

void RVSSVMPipelined::MEM_stage()
{
    if (!ex_mem_.valid)
    {
        mem_wb_next_.valid = false;
        return;
    }
    std::cout << "[MEM] ex_mem_.valid=" << ex_mem_.valid
              << " rd=x" << int(ex_mem_.rd)
              << " alu_result=0x" << std::hex << ex_mem_.alu_result
              << " reg_write=" << int(ex_mem_.reg_write)
              << " mem_read=" << int(ex_mem_.mem_read)
              << " mem_write=" << int(ex_mem_.mem_write) << std::dec << "\n";
    mem_wb_next_.valid = true;
    mem_wb_next_.rd = ex_mem_.rd;
    mem_wb_next_.reg_write = ex_mem_.reg_write;
    mem_wb_next_.mem_to_reg = ex_mem_.mem_to_reg;
    mem_wb_next_.alu_result = ex_mem_.alu_result;
    if (ex_mem_.mem_read)
    {
        uint8_t funct3 = (ex_mem_.instruction >> 12) & 0b111;
        switch (funct3)
        {
        case 0b000:
            mem_wb_next_.mem_data = static_cast<int8_t>(memory_controller_.ReadByte(ex_mem_.alu_result));
            break;
        case 0b001:
            mem_wb_next_.mem_data = static_cast<int16_t>(memory_controller_.ReadHalfWord(ex_mem_.alu_result));
            break;
        case 0b010:
            mem_wb_next_.mem_data = static_cast<int32_t>(memory_controller_.ReadWord(ex_mem_.alu_result));
            break;
        case 0b011:
            mem_wb_next_.mem_data = memory_controller_.ReadDoubleWord(ex_mem_.alu_result);
            break;
        case 0b100:
            mem_wb_next_.mem_data = memory_controller_.ReadByte(ex_mem_.alu_result);
            break;
        case 0b101:
            mem_wb_next_.mem_data = memory_controller_.ReadHalfWord(ex_mem_.alu_result);
            break;
        case 0b110:
            mem_wb_next_.mem_data = memory_controller_.ReadWord(ex_mem_.alu_result);
            break;
        default:
            mem_wb_next_.mem_data = 0;
            break;
        }
    }
    std::cout << "[MEM] Preparing mem_wb_next_: rd=x" << int(mem_wb_next_.rd)
              << " mem_to_reg=" << int(mem_wb_next_.mem_to_reg)
              << " alu_result=0x" << std::hex << mem_wb_next_.alu_result
              << " mem_data=0x" << mem_wb_next_.mem_data << std::dec << "\n";
    if (ex_mem_.mem_write)
    {
        uint8_t funct3 = (ex_mem_.instruction >> 12) & 0b111;
        switch (funct3)
        {
        case 0b000:
            memory_controller_.WriteByte(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFF);
            break;
        case 0b001:
            memory_controller_.WriteHalfWord(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFFFF);
            break;
        case 0b010:
            memory_controller_.WriteWord(ex_mem_.alu_result, ex_mem_.reg2_value & 0xFFFFFFFF);
            break;
        case 0b011:
            memory_controller_.WriteDoubleWord(ex_mem_.alu_result, ex_mem_.reg2_value);
            break;
        default:
            break;
        }
    }
}

void RVSSVMPipelined::WB_stage()
{
    if (!mem_wb_.valid)
        return;
    if (mem_wb_.reg_write && mem_wb_.rd != 0)
    {
        uint64_t write_val = mem_wb_.mem_to_reg ? mem_wb_.mem_data : mem_wb_.alu_result;
        registers_.WriteGpr(mem_wb_.rd, write_val);
        std::cout << "[WB] cycle=" << cycle_s_
                  << " writing x" << mem_wb_.rd
                  << " = 0x" << std::hex << write_val << std::dec
                  << " (mem_to_reg=" << mem_wb_.mem_to_reg << ", valid=" << mem_wb_.valid << ")\n";
    }
}

void RVSSVMPipelined::advance_pipeline_registers()
{
    mem_wb_ = mem_wb_next_;
    ex_mem_ = ex_mem_next_;
    id_ex_ = id_ex_next_;
    if_id_ = if_id_next_;
    mem_wb_next_ = MEM_WB();
    ex_mem_next_ = EX_MEM();
    id_ex_next_ = ID_EX();
    if_id_next_ = IF_ID();
    stall_ = false;
    // Debug: print GPR snapshot after advancing pipeline registers — shows the register file
    PrintGprSnapshot(registers_, cycle_s_);
}
void RVSSVMPipelined::Run()
{
    // memory_controller_.LoadProgramToMemory(assembled_program_);
    std::cout << "[DEBUG] Loaded program into memory for pipelined VM\n";

    stop_requested_ = false;
    uint64_t instruction_executed = 0;
    // Reset();
    std::cout << "[DEBUG] Program load check: first 8 bytes of memory:\n";
    for (int i = 0; i < 8; ++i)
    {
        std::cout << "mem[" << i * 4 << "] = 0x"
                  << std::hex << memory_controller_.ReadWord(i * 4) << std::dec << "\n";
    }

    while (!stop_requested_ &&
           (program_counter_ < program_size_ ||
            if_id_.valid || id_ex_.valid || ex_mem_.valid || mem_wb_.valid))
    {
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

    //  Drain all remaining pipeline stages before dump
    while (if_id_.valid || id_ex_.valid || ex_mem_.valid || mem_wb_.valid)
    {
        WB_stage();
        MEM_stage();
        EX_stage();
        ID_stage();
        IF_stage();

        instructions_retired_ += (mem_wb_.valid) ? 1 : 0;
        advance_pipeline_registers();
        cycle_s_++;
    }

    try
    {
        std::cout << "[VM] About to call DumpRegisters; mem_wb_.valid=" << mem_wb_.valid
                  << " ex_mem_.valid=" << ex_mem_.valid << " cycle=" << cycle_s_ << "\n";
        DumpRegisters(globals::registers_dump_file_path, registers_);
        std::cout << "[VM] DumpRegisters returned successfully.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "[VM][ERROR] DumpRegisters exception: " << e.what() << "\n";
    }

    if (instructions_retired_ > 0)
    {
        cpi_ = static_cast<float>(cycle_s_) / static_cast<float>(instructions_retired_);
        ipc_ = static_cast<float>(instructions_retired_) / static_cast<float>(cycle_s_);
    }

    DumpState(globals::vm_state_dump_file_path);

    if (program_counter_ >= program_size_)
    {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
}

// void RVSSVMPipelined::Run()
// {
//     stop_requested_ = false;
//     uint64_t instruction_executed = 0;
//     Reset();

//     while (!stop_requested_ &&
//            (program_counter_ < program_size_ ||
//             if_id_.valid || id_ex_.valid || ex_mem_.valid || mem_wb_.valid))
//     {
//         if (instruction_executed > vm_config::config.getInstructionExecutionLimit())
//             break;
//         WB_stage();
//         MEM_stage();
//         EX_stage();
//         ID_stage();
//         IF_stage();
//         instructions_retired_ += (mem_wb_.valid) ? 1 : 0;
//         advance_pipeline_registers();
//         instruction_executed++;
//         cycle_s_++;
//     }
//     // Ensure all pipeline stages complete their writes
//     while (ex_mem_.valid || mem_wb_.valid||(id_ex_.valid || if_id_.valid))
//     {
//         WB_stage();
//         MEM_stage();
//         EX_stage();
//         instructions_retired_ += (mem_wb_.valid) ? 1 : 0;
//         advance_pipeline_registers();
//         cycle_s_++;
//     }
//     // before dumping
// std::cout << "[VM] About to call DumpRegisters; mem_wb_.valid=" << mem_wb_.valid
//           << " ex_mem_.valid=" << ex_mem_.valid << " cycle=" << cycle_s_ << "\n";
// try {
//     std::cout << "[VM] About to call DumpRegisters; mem_wb_.valid=" << mem_wb_.valid
//               << " ex_mem_.valid=" << ex_mem_.valid << " cycle=" << cycle_s_ << "\n";
//     DumpRegisters(globals::registers_dump_file_path, registers_);
//     std::cout << "[VM] DumpRegisters returned successfully.\n";
// } catch (const std::exception &e) {
//     std::cerr << "[VM][ERROR] DumpRegisters exception: " << e.what() << "\n";
// }

//     if (instructions_retired_ > 0)
//     {
//         cpi_ = static_cast<float>(cycle_s_) / static_cast<float>(instructions_retired_);
//         ipc_ = static_cast<float>(instructions_retired_) / static_cast<float>(cycle_s_);
//     }
//     DumpState(globals::vm_state_dump_file_path);
//     if (program_counter_ >= program_size_)
//     {
//         std::cout << "VM_PROGRAM_END" << std::endl;
//         output_status_ = "VM_PROGRAM_END";
//     }

// }

void RVSSVMPipelined::DebugRun()
{
    Run();
}

void RVSSVMPipelined::Step()
{
    // Print current pipeline state before executing a single step
    PrintPipelineState();

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
