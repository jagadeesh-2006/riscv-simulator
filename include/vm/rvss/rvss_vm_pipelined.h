/**
 * @file rvss_vm_pipelined.h
 * @brief Pipelined RISC-V VM
 */
#ifndef RVSS_VM_PIPELINED_H
#define RVSS_VM_PIPELINED_H

#include "vm/vm_base.h"
#include "vm/rvss/rvss_control_unit.h"
#include "pipeline_registers.h"
#include "hazard_unit.h"

#include <stack>
#include <atomic>

class RVSSVMPipelined : public VmBase {
public:
    RVSSControlUnit control_unit_;
    std::atomic<bool> stop_requested_ = false;

    // Pipeline registers
    IFIDRegister if_id_;
    IDEXRegister id_ex_;
    EXMEMRegister ex_mem_;
    MEMWBRegister mem_wb_;
    
    // Next state
    IFIDRegister if_id_next_;
    IDEXRegister id_ex_next_;
    EXMEMRegister ex_mem_next_;
    MEMWBRegister mem_wb_next_;
    
    HazardUnit hazard_unit_;
    
    // Control
    bool stall_if_ = false;
    bool stall_id_ = false;
    bool flush_if_ = false;
    bool flush_id_ = false;
    bool flush_ex_ = false;
    
    ForwardSource forward_a_ = ForwardSource::NONE;
    ForwardSource forward_b_ = ForwardSource::NONE;
    
    // Statistics
    unsigned int pipeline_stalls_ = 0;
    unsigned int pipeline_flushes_ = 0;
    
    // Undo/Redo
    std::stack<StepDelta> undo_stack_;
    std::stack<StepDelta> redo_stack_;
    StepDelta current_delta_;

    RVSSVMPipelined();
    ~RVSSVMPipelined();

    // Pipeline stages
    void StageFetch();
    void StageDecode();
    void StageExecute();
    void StageMemory();
    void StageWriteBack();
    
    // Hazards
    void DetectHazards();
    void HandleForwarding();
    void FlushPipeline();
    void InsertBubble();
    
    // Execution
    void ExecuteCycle();
    void UpdatePipelineRegisters();
    
    // Override VmBase
    void Run() override;
    void DebugRun() override;
    void Step() override;
    void Undo() override;
    void Redo() override;
    void Reset() override;
    
    void RequestStop() { stop_requested_ = true; }
    bool IsStopRequested() const { return stop_requested_; }
    void ClearStop() { stop_requested_ = false; }
    
    // Helpers
    bool IsPipelineEmpty() const;
    uint64_t GetForwardedValue(ForwardSource source, uint64_t original);
    void PrintPipelineState() const;
    void PrintPipelineStats() const;
    void HandleSyscall();
};

#endif // RVSS_VM_PIPELINED_H