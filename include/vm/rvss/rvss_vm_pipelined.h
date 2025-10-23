/**
 * @file rvss_vm_pipelined.h
 * @brief Simple 5-stage pipelined RVSS VM (no hazards, no forwarding)
 */
#ifndef RVSS_VM_PIPELINED_H
#define RVSS_VM_PIPELINED_H

#include "vm/vm_base.h"
#include "vm/rvss/rvss_control_unit.h"
#include "vm/rvss/hazard_unit.h"

#include <cstdint>
#include <optional>

class RVSSVMPipelined : public VmBase {
 public:
  RVSSVMPipelined();
  ~RVSSVMPipelined();

  // Main control
  void Run() override;
  void DebugRun() override;
  void Step() override;
  void Undo() override;
  void Redo() override;
  void Reset() override;

 private:
  // Simple control unit reused from single-cycle design
  RVSSControlUnit control_unit_;

  // Pipeline register structs
  struct IF_ID {
    bool valid = false;
    uint64_t pc = 0;
    uint32_t instruction = 0;
  } if_id_, if_id_next_;

  struct ID_EX {
    bool valid = false;
    uint64_t pc = 0;
    uint32_t instruction = 0;

    // decoded fields
    uint8_t rs1 = 0;
    uint8_t rs2 = 0;
    uint8_t rd = 0;
    uint8_t funct3 = 0;
    uint8_t funct7 = 0;
    int32_t imm = 0;

    // register read values
    uint64_t reg1_value = 0;
    uint64_t reg2_value = 0;

    // control signals
    bool reg_write = false;
    bool mem_read = false;
    bool mem_write = false;
    bool mem_to_reg = false;
    bool alu_src = false;
    bool branch = false;
    bool is_float = false;
  } id_ex_, id_ex_next_;

  struct EX_MEM {
    bool valid = false;
    uint64_t pc = 0;
    uint32_t instruction = 0;

    uint8_t rd = 0;
    bool reg_write = false;
    bool mem_read = false;
    bool mem_write = false;
    bool mem_to_reg = false;

    uint64_t alu_result = 0;
    uint64_t reg2_value = 0; // for stores

    // branch
    bool branch_taken = false;
    uint64_t branch_target = 0;
  } ex_mem_, ex_mem_next_;

  struct MEM_WB {
    bool valid = false;
    uint8_t rd = 0;
    bool reg_write = false;
    bool mem_to_reg = false;
    uint64_t alu_result = 0;
    uint64_t mem_data = 0;
  } mem_wb_, mem_wb_next_;

  // pipeline control helpers
  bool pc_update_pending_ = false;
  uint64_t pc_update_value_ = 0;
    // hazard detection
    HazardDetectionUnit hazard_unit_;
    bool stall_ = false; // when true, IF/ID is frozen and ID/EX gets a bubble

  // stage functions
  void IF_stage();
  void ID_stage();
  void EX_stage();
  void MEM_stage();
  void WB_stage();

  // utilities
  void advance_pipeline_registers();
};

#endif // RVSS_VM_PIPELINED_H
