/**
 * @file hazard_unit.h
 * @brief Simple hazard detection unit for load-use hazards
 */
#ifndef HAZARD_UNIT_H
#define HAZARD_UNIT_H

#include <cstdint>

class HazardDetectionUnit {
 public:
  HazardDetectionUnit() = default;
  ~HazardDetectionUnit() = default;

  // Detect a load-use hazard: returns true when EX-stage instruction is a load
  // and ID-stage instruction reads the loaded register (rs1 or rs2).
  bool DetectLoadUseHazard(uint8_t ex_rd, bool ex_mem_read, uint8_t id_rs1, uint8_t id_rs2) const;
};

#endif // HAZARD_UNIT_H
