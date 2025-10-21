/**
 * File Name: test_config.cpp
 * Author: Vishank Singh
 * Github: https://github.com/VishankSingh
 */

#include <gtest/gtest.h>

#include "config.h"

// TEST(ConfigTest, GetKeyValueTest) {
//   std::string processor_type = vm_config::ini::Get("Execution", "processor_type");
//   std::string memory_size = vm_config::ini::Get("Memory", "memory_size");
//   ASSERT_EQ(processor_type, "single_stage");
//   ASSERT_EQ(memory_size, "0xffffffffffffffff");
// }

// TEST(ConfigTest, SetKeyValueTest) {
//   vm_config::ini::Set("Execution", "hazard_detection", "false");
//   std::string hazard_detection_status = vm_config::ini::Get("Execution", "hazard_detection");
//   ASSERT_EQ(hazard_detection_status, "false");

//   vm_config::ini::Set("Cache", "cache_enabled", "false");
//   std::string cache_enabled = vm_config::ini::Get("Cache", "cache_enabled");
//   ASSERT_EQ(cache_enabled, "false");
// }

// TEST(ConfigTest, InvalidKeyValueTest) {
//   ASSERT_THROW(vm_config::ini::Get("NonExistentSection", "non_existent_key"), std::invalid_argument);
//   ASSERT_THROW(vm_config::ini::Set("NonExistentSection", "non_existent_key", "value"), std::invalid_argument);
// }
