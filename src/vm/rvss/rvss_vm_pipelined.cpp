/**
 * @file rvss_vm_pipelined.cpp
 * @brief Pipelined VM implementation
 */

#include "vm/rvss/rvss_vm_pipelined.h"
#include <iostream>
#include <iomanip>

#define GET_OPCODE(inst) ((inst) & 0x7F)
#define GET_RD(inst) (((inst) >> 7) & 0x1F)
#define GET_FUNCT3(inst) (((inst) >> 12) & 0x07)
#define GET_RS1(inst) (((inst) >> 15) & 0x1F)
#define GET_RS2(inst) (((inst) >> 20) & 0x1F)
#define GET_FUNCT7(inst) (((inst) >> 25) & 0x7F)

RVSSVMPipelined::RVSSVMPipelined() {
    Reset();
}

RVSSVMPipelined::~RVSSVMPipelined() = default;

// main executon methods

void RVSSVMPipelined::Reset() {
    program_counter_ = 0;
    cycle_s_ = 0;
    instructions_retired_ = 0;
    stall_cycles_ = 0;
    branch_mispredictions_ = 0;
    pipeline_stalls_ = 0;
    pipeline_flushes_ = 0;
    
    if_id_.Reset();
    id_ex_.Reset();
    ex_mem_.Reset();
    mem_wb_.Reset();
    
    if_id_next_.Reset();
    id_ex_next_.Reset();
    ex_mem_next_.Reset();
    mem_wb_next_.Reset();
    
    stall_if_ = stall_id_ = false;
    flush_if_ = flush_id_ = flush_ex_ = false;
    forward_a_ = forward_b_ = ForwardSource::NONE;
    
    registers_.Reset();
    memory_controller_.Reset();
    stop_requested_ = false;
}

// Continue in rvss_vm_pipelined.cpp

void RVSSVMPipelined::Run() {
    ClearStop();
    
    while (!stop_requested_) {
        if (program_counter_ >= program_size_ && IsPipelineEmpty()) {
            break;
        }
        
        ExecuteCycle();
        
        if (CheckBreakpoint(program_counter_)) {
            std::cout << "Breakpoint at PC: 0x" << std::hex 
                      << program_counter_ << std::dec << std::endl;
            break;
        }
    }
    
    PrintPipelineStats();
}

void RVSSVMPipelined::DebugRun() {
    ClearStop();
    
    while (!stop_requested_) {
        if (program_counter_ >= program_size_ && IsPipelineEmpty()) {
            break;
        }
        
        std::cout << "\n=== Cycle " << cycle_s_ << " ===\n";
        ExecuteCycle();
        PrintPipelineState();
        
        if (CheckBreakpoint(program_counter_)) {
            break;
        }
        
        std::cout << "Press Enter...";
        std::cin.get();
    }
    
    PrintPipelineStats();
}

void RVSSVMPipelined::Step() {
    if (program_counter_ >= program_size_ && IsPipelineEmpty()) {
        std::cout << "Program completed.\n";
        return;
    }
    
    ExecuteCycle();
    PrintPipelineState();
}

void RVSSVMPipelined::ExecuteCycle() {
    DetectHazards();
    
    // Execute in reverse order
    StageWriteBack();
    StageMemory();
    StageExecute();
    StageDecode();
    StageFetch();
    
    UpdatePipelineRegisters();
    cycle_s_++;
}

void RVSSVMPipelined::UpdatePipelineRegisters() {
    if_id_ = if_id_next_;
    id_ex_ = id_ex_next_;
    ex_mem_ = ex_mem_next_;
    mem_wb_ = mem_wb_next_;
}


//pipeline stage implementations


void RVSSVMPipelined::StageFetch() {
    if (stall_if_ || flush_if_) {
        if (flush_if_) {
            if_id_next_.Reset();
            flush_if_ = false;
        }
        return;
    }
    
    if (program_counter_ >= program_size_) {
        if_id_next_.valid = false;
        return;
    }
    
    if_id_next_.instruction = memory_controller_.ReadWord(program_counter_);
    if_id_next_.pc = program_counter_;
    if_id_next_.valid = true;
    
    program_counter_ += 4;
}

void RVSSVMPipelined::StageDecode() {
    if (stall_id_ || flush_id_) {
        if (flush_id_) {
            id_ex_next_.Reset();
            flush_id_ = false;
        } else {
            id_ex_next_ = id_ex_;
        }
        return;
    }
    
    if (!if_id_.valid) {
        id_ex_next_.Reset();
        return;
    }
    
    uint32_t inst = if_id_.instruction;
    uint8_t opcode = GET_OPCODE(inst);
    uint8_t rd = GET_RD(inst);
    uint8_t funct3 = GET_FUNCT3(inst);
    uint8_t rs1 = GET_RS1(inst);
    uint8_t rs2 = GET_RS2(inst);
    uint8_t funct7 = GET_FUNCT7(inst);
    
    control_unit_.SetControlSignals(inst);
    
    uint64_t read_data1 = registers_.ReadGpr(rs1);
    uint64_t read_data2 = registers_.ReadGpr(rs2);
    
    bool is_float = (opcode == 0x07 || opcode == 0x27 || 
                     opcode == 0x53);
    
    if (is_float) {
        read_data1 = registers_.ReadFpr(rs1);
        read_data2 = registers_.ReadFpr(rs2);
    }
    
    int32_t imm = ImmGenerator(inst);
    alu::AluOp alu_op = control_unit_.GetAluSignal(inst, 
                                      control_unit_.GetAluOp());
    
    id_ex_next_.reg_write = control_unit_.GetRegWrite();
    id_ex_next_.mem_to_reg = control_unit_.GetMemToReg();
    id_ex_next_.mem_read = control_unit_.GetMemRead();
    id_ex_next_.mem_write = control_unit_.GetMemWrite();
    id_ex_next_.branch = control_unit_.GetBranch();
    id_ex_next_.alu_src = control_unit_.GetAluSrc();
    id_ex_next_.alu_op = alu_op;
    
    id_ex_next_.pc = if_id_.pc;
    id_ex_next_.read_data1 = read_data1;
    id_ex_next_.read_data2 = read_data2;
    id_ex_next_.immediate = imm;
    
    id_ex_next_.rs1 = rs1;
    id_ex_next_.rs2 = rs2;
    id_ex_next_.rd = rd;
    
    id_ex_next_.funct3 = funct3;
    id_ex_next_.funct7 = funct7;
    id_ex_next_.is_float = is_float;
    id_ex_next_.is_double = false;
    
    id_ex_next_.valid = true;
}

void RVSSVMPipelined::StageExecute() {
    if (flush_ex_) {
        ex_mem_next_.Reset();
        flush_ex_ = false;
        return;
    }
    
    if (!id_ex_.valid) {
        ex_mem_next_.Reset();
        return;
    }
    
    // Get forwarded operands
    uint64_t alu_input1 = GetForwardedValue(forward_a_, id_ex_.read_data1);
    uint64_t alu_input2 = id_ex_.alu_src ? 
                          static_cast<uint64_t>(id_ex_.immediate) : 
                          GetForwardedValue(forward_b_, id_ex_.read_data2);
    
    // Execute ALU operation
    uint64_t alu_result;
    bool overflow = false;
    
    auto [result, ovf] = alu_.execute(id_ex_.alu_op, alu_input1, alu_input2);
    alu_result = result;
    overflow = ovf;
    
    // Calculate branch target
    uint64_t branch_target = id_ex_.pc + id_ex_.immediate;
    bool branch_taken = false;
    
    if (id_ex_.branch) {
        switch (id_ex_.funct3) {
            case 0x0: // BEQ
                branch_taken = (alu_input1 == alu_input2);
                break;
            case 0x1: // BNE
                branch_taken = (alu_input1 != alu_input2);
                break;
            case 0x4: // BLT
                branch_taken = (static_cast<int64_t>(alu_input1) < 
                               static_cast<int64_t>(alu_input2));
                break;
            case 0x5: // BGE
                branch_taken = (static_cast<int64_t>(alu_input1) >= 
                               static_cast<int64_t>(alu_input2));
                break;
            case 0x6: // BLTU
                branch_taken = (alu_input1 < alu_input2);
                break;
            case 0x7: // BGEU
                branch_taken = (alu_input1 >= alu_input2);
                break;
        }
    }
    
    // Fill EX/MEM register
    ex_mem_next_.reg_write = id_ex_.reg_write;
    ex_mem_next_.mem_to_reg = id_ex_.mem_to_reg;
    ex_mem_next_.mem_read = id_ex_.mem_read;
    ex_mem_next_.mem_write = id_ex_.mem_write;
    
    ex_mem_next_.alu_result = alu_result;
    ex_mem_next_.write_data = GetForwardedValue(forward_b_, id_ex_.read_data2);
    ex_mem_next_.branch_target = branch_target;
    ex_mem_next_.branch_taken = branch_taken;
    
    ex_mem_next_.rd = id_ex_.rd;
    ex_mem_next_.mem_size = id_ex_.funct3 & 0x3;
    ex_mem_next_.is_float = id_ex_.is_float;
    
    ex_mem_next_.valid = true;
}

void RVSSVMPipelined::StageMemory() {
    if (!ex_mem_.valid) {
        mem_wb_next_.Reset();
        return;
    }
    
    uint64_t mem_data = 0;
    
    // Memory read
    if (ex_mem_.mem_read) {
        switch (ex_mem_.mem_size) {
            case 0: // Byte
                mem_data = memory_controller_.ReadByte(ex_mem_.alu_result);
                break;
            case 1: // Half word
                mem_data = memory_controller_.ReadHalfWord(ex_mem_.alu_result);
                break;
            case 2: // Word
                mem_data = memory_controller_.ReadWord(ex_mem_.alu_result);
                break;
            case 3: // Double word
                mem_data = memory_controller_.ReadDoubleWord(ex_mem_.alu_result);
                break;
        }
    }
    
    // Memory write
    if (ex_mem_.mem_write) {
        switch (ex_mem_.mem_size) {
            case 0:
                memory_controller_.WriteByte(ex_mem_.alu_result, 
                                            static_cast<uint8_t>(ex_mem_.write_data));
                break;
            case 1:
                memory_controller_.WriteHalfWord(ex_mem_.alu_result, 
                                                static_cast<uint16_t>(ex_mem_.write_data));
                break;
            case 2:
                memory_controller_.WriteWord(ex_mem_.alu_result, 
                                            static_cast<uint32_t>(ex_mem_.write_data));
                break;
            case 3:
                memory_controller_.WriteDoubleWord(ex_mem_.alu_result, 
                                                   ex_mem_.write_data);
                break;
        }
    }
    
    // Fill MEM/WB register
    mem_wb_next_.reg_write = ex_mem_.reg_write;
    mem_wb_next_.mem_to_reg = ex_mem_.mem_to_reg;
    mem_wb_next_.alu_result = ex_mem_.alu_result;
    mem_wb_next_.mem_data = mem_data;
    mem_wb_next_.rd = ex_mem_.rd;
    mem_wb_next_.is_float = ex_mem_.is_float;
    mem_wb_next_.valid = true;
}

void RVSSVMPipelined::StageWriteBack() {
    if (!mem_wb_.valid) {
        return;
    }
    
    if (mem_wb_.reg_write && mem_wb_.rd != 0) {
        uint64_t write_data = mem_wb_.mem_to_reg ? 
                              mem_wb_.mem_data : 
                              mem_wb_.alu_result;
        
        if (mem_wb_.is_float) {
            registers_.WriteFpr(mem_wb_.rd, write_data);
        } else {
            registers_.WriteGpr(mem_wb_.rd, write_data);
        }
        
        instructions_retired_++;
    }
}

//hazard detection implementation


void RVSSVMPipelined::DetectHazards() {
    // Reset control signals
    stall_if_ = false;
    stall_id_ = false;
    flush_if_ = false;
    flush_id_ = false;
    flush_ex_ = false;
    
    // Extract rs1 and rs2 from current IF/ID instruction
    uint8_t rs1 = GET_RS1(if_id_.instruction);
    uint8_t rs2 = GET_RS2(if_id_.instruction);
    
    // 1. Load-use hazard detection
    if (hazard_unit_.DetectLoadUseHazard(id_ex_, rs1, rs2)) {
        stall_if_ = true;
        stall_id_ = true;
        flush_ex_ = true;
        pipeline_stalls_++;
        stall_cycles_++;
    }
    
    // 2. Branch hazard detection
    if (hazard_unit_.DetectBranchHazard(ex_mem_)) {
        flush_if_ = true;
        flush_id_ = true;
        flush_ex_ = true;
        
        program_counter_ = ex_mem_.branch_target;
        
        pipeline_flushes_ += 3;
        branch_mispredictions_++;
    }
    
    // 3. Data forwarding
    HandleForwarding();
}

void RVSSVMPipelined::HandleForwarding() {
    forward_a_ = hazard_unit_.GetForwardA(ex_mem_, mem_wb_, id_ex_.rs1);
    forward_b_ = hazard_unit_.GetForwardB(ex_mem_, mem_wb_, id_ex_.rs2);
}

uint64_t RVSSVMPipelined::GetForwardedValue(ForwardSource source, 
                                            uint64_t original_value) {
    switch (source) {
        case ForwardSource::EX_MEM:
            return ex_mem_.alu_result;
        case ForwardSource::MEM_WB:
            return mem_wb_.mem_to_reg ? mem_wb_.mem_data : mem_wb_.alu_result;
        case ForwardSource::NONE:
        default:
            return original_value;
    }
}

void RVSSVMPipelined::FlushPipeline() {
    if_id_.Reset();
    id_ex_.Reset();
    ex_mem_.Reset();
}

void RVSSVMPipelined::InsertBubble() {
    id_ex_next_.Reset();
}

//helper methods

void RVSSVMPipelined::Undo() {
    std::cout << "Undo not implemented for pipelined execution.\n";
}

void RVSSVMPipelined::Redo() {
    std::cout << "Redo not implemented for pipelined execution.\n";
}

bool RVSSVMPipelined::IsPipelineEmpty() const {
    return !if_id_.valid && !id_ex_.valid && 
           !ex_mem_.valid && !mem_wb_.valid;
}

void RVSSVMPipelined::PrintPipelineState() const {
    std::cout << "\n----- Pipeline State (Cycle " << cycle_s_ << ") -----\n";
    
    std::cout << "IF/ID:  ";
    if (if_id_.valid) {
        std::cout << "PC=0x" << std::hex << if_id_.pc << std::dec 
                  << " Inst=0x" << std::hex << if_id_.instruction << std::dec;
    } else {
        std::cout << "[BUBBLE]";
    }
    std::cout << "\n";
    
    std::cout << "ID/EX:  ";
    if (id_ex_.valid) {
        std::cout << "PC=0x" << std::hex << id_ex_.pc << std::dec
                  << " RD=x" << (int)id_ex_.rd 
                  << " RS1=x" << (int)id_ex_.rs1
                  << " RS2=x" << (int)id_ex_.rs2;
    } else {
        std::cout << "[BUBBLE]";
    }
    std::cout << "\n";
    
    std::cout << "EX/MEM: ";
    if (ex_mem_.valid) {
        std::cout << "ALU=0x" << std::hex << ex_mem_.alu_result << std::dec
                  << " RD=x" << (int)ex_mem_.rd;
        if (ex_mem_.branch_taken) {
            std::cout << " [BRANCH TAKEN]";
        }
    } else {
        std::cout << "[BUBBLE]";
    }
    std::cout << "\n";
    
    std::cout << "MEM/WB: ";
    if (mem_wb_.valid) {
        std::cout << "Data=0x" << std::hex << mem_wb_.mem_data << std::dec
                  << " ALU=0x" << std::hex << mem_wb_.alu_result << std::dec
                  << " RD=x" << (int)mem_wb_.rd;
    } else {
        std::cout << "[BUBBLE]";
    }
    std::cout << "\n";
    
    std::cout << "PC: 0x" << std::hex << program_counter_ << std::dec << "\n";
    
    // Show forwarding status
    if (forward_a_ != ForwardSource::NONE || forward_b_ != ForwardSource::NONE) {
        std::cout << "Forwarding: ";
        if (forward_a_ != ForwardSource::NONE) {
            std::cout << "A=" << (forward_a_ == ForwardSource::EX_MEM ? "EX/MEM" : "MEM/WB") << " ";
        }
        if (forward_b_ != ForwardSource::NONE) {
            std::cout << "B=" << (forward_b_ == ForwardSource::EX_MEM ? "EX/MEM" : "MEM/WB");
        }
        std::cout << "\n";
    }
    
    // Show stall/flush status
    if (stall_if_ || stall_id_ || flush_if_ || flush_id_ || flush_ex_) {
        std::cout << "Control: ";
        if (stall_if_) std::cout << "[STALL_IF] ";
        if (stall_id_) std::cout << "[STALL_ID] ";
        if (flush_if_) std::cout << "[FLUSH_IF] ";
        if (flush_id_) std::cout << "[FLUSH_ID] ";
        if (flush_ex_) std::cout << "[FLUSH_EX] ";
        std::cout << "\n";
    }
    
    std::cout << "-----------------------------------\n";
}

void RVSSVMPipelined::PrintPipelineStats() const {
    std::cout << "\n========== Pipeline Statistics ==========\n";
    std::cout << "Total Cycles:              " << cycle_s_ << "\n";
    std::cout << "Instructions Retired:      " << instructions_retired_ << "\n";
    std::cout << "Pipeline Stalls:           " << pipeline_stalls_ << "\n";
    std::cout << "Pipeline Flushes:          " << pipeline_flushes_ << "\n";
    std::cout << "Branch Mispredictions:     " << branch_mispredictions_ << "\n";
    
    if (instructions_retired_ > 0) {
        float cpi = static_cast<float>(cycle_s_) / instructions_retired_;
        float ipc = static_cast<float>(instructions_retired_) / cycle_s_;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "CPI (Cycles Per Inst):     " << cpi << "\n";
        std::cout << "IPC (Inst Per Cycle):      " << ipc << "\n";
        
        float stall_percent = (static_cast<float>(pipeline_stalls_) / cycle_s_) * 100;
        float flush_percent = (static_cast<float>(pipeline_flushes_) / cycle_s_) * 100;
        std::cout << "Stall Percentage:          " << stall_percent << "%\n";
        std::cout << "Flush Percentage:          " << flush_percent << "%\n";
    }
    
    std::cout << "=========================================\n";
}

void RVSSVMPipelined::HandleSyscall() {
    uint64_t syscall_num = registers_.ReadGpr(17); // a7
    
    switch (syscall_num) {
        case SYSCALL_EXIT:
            stop_requested_ = true;
            break;
        case SYSCALL_PRINT_INT:
            std::cout << registers_.ReadGpr(10); // a0
            break;
        case SYSCALL_PRINT_STRING: {
            uint64_t str_addr = registers_.ReadGpr(10);
            PrintString(str_addr);
            break;
        }
        default:
            std::cerr << "Unknown syscall: " << syscall_num << "\n";
            break;
    }
}