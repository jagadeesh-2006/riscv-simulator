/**
 * @file hazard_unit.cpp
 * @brief Simple hazard detection unit implementation
 */

#include "vm/rvss/hazard_unit.h"

bool HazardDetectionUnit::DetectLoadUseHazard(uint8_t ex_rd, bool ex_mem_read, uint8_t id_rs1, uint8_t id_rs2) const {
  if (!ex_mem_read) return false; // EX stage is not a load
  if (ex_rd == 0) return false; // x0 is not a hazard
  return (ex_rd == id_rs1) || (ex_rd == id_rs2);
}
