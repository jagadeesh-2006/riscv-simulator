// #include "main.h"
// #include "assembler/assembler.h"
// #include "utils.h"
// #include "globals.h"
// #include "vm/rvss/rvss_vm.h"
// #include "vm/rvss/rvss_vm_pipelined.h" // pipelined variants
// #include "vm_runner.h"
// #include "command_handler.h"
// #include "config.h"

// #include <iostream>
// #include <thread>
// #include <memory>
// #include <string>
// #include <stdexcept>

// int main(int argc, char *argv[]) {
//   // Basic arg validation
//   if (argc <= 1) {
//     std::cerr << "No arguments provided. Use --help for usage information.\n";
//     return 1;
//   }

//   // Default VM mode
//   std::string vm_mode = "single"; // options: single, pipelined_no_hazard, pipelined_with_hazard
//   bool start_vm = false;

//   // Parse CLI
//   for (int i = 1; i < argc; ++i) {
//     std::string arg = argv[i];

//     if (arg == "--help" || arg == "-h") {
//       std::cout << "Usage: " << argv[0] << " [options]\n"
//                 << "Options:\n"
//                 << "  --help, -h                 Show this help message\n"
//                 << "  --assemble <file>          Assemble the specified file\n"
//                 << "  --run <file>               Run the specified file (one-shot)\n"
//                 << "  --verbose-errors           Enable verbose error printing\n"
//                 << "  --start-vm                 Start the interactive VM shell\n"
//                 << "  --vm-mode <mode>           Select VM mode: single | pipelined_no_hazard | pipelined_with_hazard\n"
//                 << "  --vm-as-backend            Start VM with default program in backend mode\n";
//       return 0;

//     } else if (arg == "--assemble") {
//       if (++i >= argc) {
//         std::cerr << "Error: No file specified for assembly.\n";
//         return 1;
//       }
//       try {
//         AssembledProgram program = assemble(argv[i]);
//         std::cout << "Assembled program: " << program.filename << '\n';
//         return 0;
//       } catch (const std::exception& e) {
//         std::cerr << "Assemble error: " << e.what() << '\n';
//         return 1;
//       }

//     } else if (arg == "--run") {
//       if (++i >= argc) {
//         std::cerr << "Error: No file specified to run.\n";
//         return 1;
//       }
//       try {
//         AssembledProgram program = assemble(argv[i]);
//         RVSSVM vm;
//         vm.LoadProgram(program);
//         vm.Run();
//         std::cout << "Program finished: " << program.filename << '\n';
//         return 0;
//       } catch (const std::exception& e) {
//         std::cerr << "Run error: " << e.what() << '\n';
//         return 1;
//       }

//     } else if (arg == "--verbose-errors") {
//       globals::verbose_errors_print = true;

//     } else if (arg == "--vm-as-backend") {
//       globals::vm_as_backend = true;

//     } else if (arg == "--vm-mode") {
//       if (++i >= argc) {
//         std::cerr << "Error: No vm mode specified.\n";
//         return 1;
//       }
//       vm_mode = argv[i];

//     } else if (arg == "--start-vm") {
//       start_vm = true;

//     } else {
//       std::cerr << "Unknown option: " << arg << '\n';
//       return 1;
//     }
//   }

//   if (!start_vm) {
//     std::cerr << "No operation requested; use --start-vm or --run/--assemble.\n";
//     return 1;
//   }

//   // Prepare VM state directory etc.
//   try {
//     setupVmStateDirectory();
//   } catch (const std::exception &e) {
//     std::cerr << "Failed to set up VM state directory: " << e.what() << '\n';
//     return 1;
//   }

//   // Use pointer to VmBase so we can instantiate different implementations at runtime
//   std::unique_ptr<VmBase> vm;

//   if (vm_mode == "single") {
//     vm = std::make_unique<RVSSVM>();
//   }else if (vm_mode == "pipelined_no_hazard") {
//     vm = std::make_unique<RVSSVMPipelined>();
//     vm->hazard_detection_enabled_ = false;
// } else if (vm_mode == "pipelined_with_hazard") {
//     vm = std::make_unique<RVSSVMPipelined>();
//     vm->hazard_detection_enabled_ = true;
// }
//  else {
//     std::cerr << "Unknown vm_mode: " << vm_mode << "; falling back to single-cycle\n";
//     vm = std::make_unique<RVSSVM>();
//   }

//   std::cout << "VM_STARTED mode=" << vm_mode << "\n";

//   std::thread vm_thread;
//   bool vm_running = false;

//   auto launch_vm_thread = [&](auto fn) {
//     // stop and join existing thread if any
//     if (vm_thread.joinable()) {
//       vm->RequestStop();
//       vm_thread.join();
//     }
//     vm_running = true;
//     vm_thread = std::thread([&]() {
//       fn();
//       vm_running = false;
//     });
//   };

//   // Interactive loop: read commands from stdin
//   std::string line;
//   while (std::getline(std::cin, line)) {
//     auto cmd = command_handler::ParseCommand(line);

//     switch (cmd.type) {
//       case command_handler::CommandType::LOAD:
//         if (cmd.args.empty()) {
//           std::cout << "VM_PARSE_ERROR\n";
//           break;
//         }
//         try {
//           AssembledProgram p = assemble(cmd.args[0]);
//           vm->LoadProgram(p);
//           std::cout << "VM_PARSE_SUCCESS\n";
//         } catch (...) {
//           std::cout << "VM_PARSE_ERROR\n";
//         }
//         break;

//       case command_handler::CommandType::RUN:
//         launch_vm_thread([&]() { vm->Run(); });
//         break;

//       case command_handler::CommandType::DEBUG_RUN:
//         launch_vm_thread([&]() { vm->DebugRun(); });
//         break;

//       case command_handler::CommandType::STEP:
//         if (!vm_running) launch_vm_thread([&]() { vm->Step(); });
//         break;

//       case command_handler::CommandType::STOP:
//         vm->RequestStop();
//         break;

//       case command_handler::CommandType::EXIT:
//         vm->RequestStop();
//         if (vm_thread.joinable()) vm_thread.join();
//         return 0;

//       case command_handler::CommandType::MODIFY_CONFIG:
//         if (cmd.args.size() != 3) {
//           std::cout << "VM_MODIFY_CONFIG_ERROR\n";
//         } else {
//           try {
//             vm_config::config.modifyConfig(cmd.args[0], cmd.args[1], cmd.args[2]);
//             std::cout << "VM_MODIFY_CONFIG_SUCCESS\n";
//           } catch (const std::exception &e) {
//             std::cout << "VM_MODIFY_CONFIG_ERROR\n";
//             std::cerr << e.what() << '\n';
//           }
//         }
//         break;

//       case command_handler::CommandType::MODIFY_REGISTER:
//         if (cmd.args.size() != 2) {
//           std::cout << "VM_MODIFY_REGISTER_ERROR\n";
//         } else {
//           try {
//             std::string reg_name = cmd.args[0];
//             uint64_t value = std::stoull(cmd.args[1], nullptr, 16);
//             vm->ModifyRegister(reg_name, value);
//             DumpRegisters(globals::registers_dump_file_path, vm->registers_);
//             std::cout << "VM_MODIFY_REGISTER_SUCCESS\n";
//           } catch (...) {
//             std::cout << "VM_MODIFY_REGISTER_ERROR\n";
//           }
//         }
//         break;

//       case command_handler::CommandType::GET_REGISTER: {
//         if (cmd.args.empty()) {
//           std::cout << "VM_GET_REGISTER_ERROR\n";
//         } else {
//           std::string reg_str = cmd.args[0];
//           if (!reg_str.empty() && reg_str[0] == 'x') {
//             try {
//               uint64_t val = vm->registers_.ReadGpr(std::stoi(reg_str.substr(1)));
//               std::cout << "VM_REGISTER_VAL_START";
//               std::cout << "0x" << std::hex << val << std::dec;
//               std::cout << "VM_REGISTER_VAL_END" << std::endl;
//             } catch (...) {
//               std::cout << "VM_GET_REGISTER_ERROR\n";
//             }
//           } else {
//             std::cout << "VM_GET_REGISTER_ERROR\n";
//           }
//         }
//         break;
//       }

//       case command_handler::CommandType::MODIFY_MEMORY: {
//         if (cmd.args.size() != 3) {
//           std::cout << "VM_MODIFY_MEMORY_ERROR\n";
//           break;
//         }
//         try {
//           uint64_t address = std::stoull(cmd.args[0], nullptr, 16);
//           std::string type = cmd.args[1];
//           uint64_t value = std::stoull(cmd.args[2], nullptr, 16);

//           if (type == "byte") {
//             vm->memory_controller_.WriteByte(address, static_cast<uint8_t>(value));
//           } else if (type == "half") {
//             vm->memory_controller_.WriteHalfWord(address, static_cast<uint16_t>(value));
//           } else if (type == "word") {
//             vm->memory_controller_.WriteWord(address, static_cast<uint32_t>(value));
//           } else if (type == "double") {
//             vm->memory_controller_.WriteDoubleWord(address, value);
//           } else {
//             std::cout << "VM_MODIFY_MEMORY_ERROR\n";
//             break;
//           }
//           std::cout << "VM_MODIFY_MEMORY_SUCCESS\n";
//         } catch (...) {
//           std::cout << "VM_MODIFY_MEMORY_ERROR\n";
//         }
//         break;
//       }

//       case command_handler::CommandType::DUMP_MEMORY: {
//         try {
//           vm->memory_controller_.DumpMemory(cmd.args);
//         } catch (...) {
//           std::cout << "VM_MEMORY_DUMP_ERROR\n";
//         }
//         break;
//       }

//       case command_handler::CommandType::PRINT_MEMORY: {
//         for (size_t i = 0; i + 1 < cmd.args.size(); i += 2) {
//           try {
//             uint64_t address = std::stoull(cmd.args[i], nullptr, 16);
//             uint64_t rows = std::stoull(cmd.args[i + 1]);
//             vm->memory_controller_.PrintMemory(address, static_cast<unsigned int>(rows));
//           } catch (...) {
//             std::cout << "VM_PRINT_MEMORY_ERROR\n";
//           }
//         }
//         std::cout << std::endl;
//         break;
//       }

//       case command_handler::CommandType::GET_MEMORY_POINT: {
//         if (cmd.args.size() != 1) {
//           std::cout << "VM_GET_MEMORY_POINT_ERROR\n";
//         } else {
//           try {
//             vm->memory_controller_.GetMemoryPoint(cmd.args[0]);
//           } catch (...) {
//             std::cout << "VM_GET_MEMORY_POINT_ERROR\n";
//           }
//         }
//         break;
//       }

//       case command_handler::CommandType::VM_STDIN: {
//         if (!cmd.args.empty()) vm->PushInput(cmd.args[0]);
//         break;
//       }

//       case command_handler::CommandType::DUMP_CACHE: {
//         // Placeholder: vm has cache functionality in future
//         std::cout << "VM_CACHE_DUMPED" << std::endl;
//         break;
//       }

//       case command_handler::CommandType::ADD_BREAKPOINT: {
//         if (!cmd.args.empty()) vm->AddBreakpoint(std::stoul(cmd.args[0], nullptr, 10));
//         break;
//       }

//       case command_handler::CommandType::REMOVE_BREAKPOINT: {
//         if (!cmd.args.empty()) vm->RemoveBreakpoint(std::stoul(cmd.args[0], nullptr, 10));
//         break;
//       }

//       case command_handler::CommandType::UNDO: {
//         if (!vm_running) vm->Undo();
//         break;
//       }

//       case command_handler::CommandType::REDO: {
//         if (!vm_running) vm->Redo();
//         break;
//       }

//       case command_handler::CommandType::RESET: {
//         vm->Reset();
//         break;
//       }

//       default:
//         std::cout << "Unknown or unhandled command\n";
//         break;
//     } // switch
//   } // while getline

//   // Clean up before exit
//   vm->RequestStop();
//   if (vm_thread.joinable()) vm_thread.join();

//   return 0;
// }


#include "main.h"
#include "assembler/assembler.h"
#include "utils.h"
#include "globals.h"
#include "vm/rvss/rvss_vm.h"
#include "vm/rvss/rvss_vm_pipelined.h" // pipelined variants
#include "vm_runner.h"
#include "command_handler.h"
#include "config.h"

#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <stdexcept>

int main(int argc, char *argv[]) {
    // Basic arg validation
    if (argc <= 1) {
        std::cerr << "No arguments provided. Use --help for usage information.\n";
        return 1;
    }

    // Default VM mode
    std::string vm_mode = "single"; // options: single, pipelined_no_hazard, pipelined_with_hazard
    bool start_vm = false;

    // Parse CLI
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                    << "Options:\n"
                    << "  --help, -h                 Show this help message\n"
                    << "  --assemble <file>          Assemble the specified file\n"
                    << "  --run <file>               Run the specified file (one-shot)\n"
                    << "  --verbose-errors           Enable verbose error printing\n"
                    << "  --start-vm                 Start the interactive VM shell\n"
                    << "  --vm-mode <mode>           Select VM mode: single | pipelined_no_hazard | pipelined_with_hazard\n"
                    << "  --vm-as-backend            Start VM with default program in backend mode\n";
            return 0;

        } else if (arg == "--assemble") {
            if (++i >= argc) {
                std::cerr << "Error: No file specified for assembly.\n";
                return 1;
            }
            try {
                AssembledProgram program = assemble(argv[i]);
                std::cout << "Assembled program: " << program.filename << '\n';
                return 0;
            } catch (const std::exception& e) {
                std::cerr << "Assemble error: " << e.what() << '\n';
                return 1;
            }

        } else if (arg == "--run") {
            if (++i >= argc) {
                std::cerr << "Error: No file specified to run.\n";
                return 1;
            }
            try {
                AssembledProgram program = assemble(argv[i]);
                RVSSVM vm;
                vm.LoadProgram(program);
                vm.Run();
                std::cout << "Program finished: " << program.filename << '\n';
                return 0;
            } catch (const std::exception& e) {
                std::cerr << "Run error: " << e.what() << '\n';
                return 1;
            }

        } else if (arg == "--verbose-errors") {
            globals::verbose_errors_print = true;

        } else if (arg == "--vm-as-backend") {
            globals::vm_as_backend = true;

        } else if (arg == "--vm-mode") {
            if (++i >= argc) {
                std::cerr << "Error: No vm mode specified.\n";
                return 1;
            }
            vm_mode = argv[i];

        } else if (arg == "--start-vm") {
            start_vm = true;

        } else {
            std::cerr << "Unknown option: " << arg << '\n';
            return 1;
        }
    }

    if (!start_vm) {
        std::cerr << "No operation requested; use --start-vm or --run/--assemble.\n";
        return 1;
    }

    // Prepare VM state directory etc.
    try {
        setupVmStateDirectory();
    } catch (const std::exception &e) {
        std::cerr << "Failed to set up VM state directory: " << e.what() << '\n';
        return 1;
    }

    // Use pointer to VmBase so we can instantiate different implementations at runtime
    std::unique_ptr<VmBase> vm;

    if (vm_mode == "single") {
        vm = std::make_unique<RVSSVM>();
    } else if (vm_mode == "pipelined_no_hazard") {
        auto* pipelined_vm = new RVSSVMPipelined();
        pipelined_vm->hazard_detection_enabled_ = false;
        vm.reset(pipelined_vm);
    } else if (vm_mode == "pipelined_with_hazard") {
        auto* pipelined_vm = new RVSSVMPipelined();
        pipelined_vm->hazard_detection_enabled_ = true;
        vm.reset(pipelined_vm);
    } else {
        std::cerr << "Unknown vm_mode: " << vm_mode << "; falling back to single-cycle\n";
        vm = std::make_unique<RVSSVM>();
    }

    std::cout << "VM_STARTED mode=" << vm_mode << "\n";

    std::thread vm_thread;
    bool vm_running = false;

    auto launch_vm_thread = [&](auto fn) {
        if (vm_thread.joinable()) {
            vm->RequestStop();
            vm_thread.join();
        }
        vm_running = true;
        vm_thread = std::thread([&]() {
            fn();
            vm_running = false;
        });
    };

    // Interactive loop: read commands from stdin
    std::string line;
    while (std::getline(std::cin, line)) {
        auto cmd = command_handler::ParseCommand(line);

        switch (cmd.type) {
        case command_handler::CommandType::LOAD:
            if (cmd.args.empty()) {
                std::cout << "VM_PARSE_ERROR\n";
                break;
            }
            try {
                AssembledProgram p = assemble(cmd.args[0]);
                vm->LoadProgram(p);
                std::cout << "VM_PARSE_SUCCESS\n";
            } catch (...) {
                std::cout << "VM_PARSE_ERROR\n";
            }
            break;

        case command_handler::CommandType::RUN:
            launch_vm_thread([&]() { vm->Run(); });
            break;

        case command_handler::CommandType::DEBUG_RUN:
            launch_vm_thread([&]() { vm->DebugRun(); });
            break;

        case command_handler::CommandType::STEP:
            if (!vm_running) launch_vm_thread([&]() { vm->Step(); });
            break;

        case command_handler::CommandType::STOP:
            vm->RequestStop();
            break;

        case command_handler::CommandType::EXIT:
            vm->RequestStop();
            if (vm_thread.joinable()) vm_thread.join();
            return 0;

        // ... rest of your command handling unchanged ...

        case command_handler::CommandType::MODIFY_CONFIG:
        if (cmd.args.size() != 3) {
          std::cout << "VM_MODIFY_CONFIG_ERROR\n";
        } else {
          try {
            vm_config::config.modifyConfig(cmd.args[0], cmd.args[1], cmd.args[2]);
            std::cout << "VM_MODIFY_CONFIG_SUCCESS\n";
          } catch (const std::exception &e) {
            std::cout << "VM_MODIFY_CONFIG_ERROR\n";
            std::cerr << e.what() << '\n';
          }
        }
        break;

      case command_handler::CommandType::MODIFY_REGISTER:
        if (cmd.args.size() != 2) {
          std::cout << "VM_MODIFY_REGISTER_ERROR\n";
        } else {
          try {
            std::string reg_name = cmd.args[0];
            uint64_t value = std::stoull(cmd.args[1], nullptr, 16);
            vm->ModifyRegister(reg_name, value);
            DumpRegisters(globals::registers_dump_file_path, vm->registers_);
            std::cout << "VM_MODIFY_REGISTER_SUCCESS\n";
          } catch (...) {
            std::cout << "VM_MODIFY_REGISTER_ERROR\n";
          }
        }
        break;

      case command_handler::CommandType::GET_REGISTER: {
        if (cmd.args.empty()) {
          std::cout << "VM_GET_REGISTER_ERROR\n";
        } else {
          std::string reg_str = cmd.args[0];
          if (!reg_str.empty() && reg_str[0] == 'x') {
            try {
              uint64_t val = vm->registers_.ReadGpr(std::stoi(reg_str.substr(1)));
              std::cout << "VM_REGISTER_VAL_START";
              std::cout << "0x" << std::hex << val << std::dec;
              std::cout << "VM_REGISTER_VAL_END" << std::endl;
            } catch (...) {
              std::cout << "VM_GET_REGISTER_ERROR\n";
            }
          } else {
            std::cout << "VM_GET_REGISTER_ERROR\n";
          }
        }
        break;
      }

      case command_handler::CommandType::MODIFY_MEMORY: {
        if (cmd.args.size() != 3) {
          std::cout << "VM_MODIFY_MEMORY_ERROR\n";
          break;
        }
        try {
          uint64_t address = std::stoull(cmd.args[0], nullptr, 16);
          std::string type = cmd.args[1];
          uint64_t value = std::stoull(cmd.args[2], nullptr, 16);

          if (type == "byte") {
            vm->memory_controller_.WriteByte(address, static_cast<uint8_t>(value));
          } else if (type == "half") {
            vm->memory_controller_.WriteHalfWord(address, static_cast<uint16_t>(value));
          } else if (type == "word") {
            vm->memory_controller_.WriteWord(address, static_cast<uint32_t>(value));
          } else if (type == "double") {
            vm->memory_controller_.WriteDoubleWord(address, value);
          } else {
            std::cout << "VM_MODIFY_MEMORY_ERROR\n";
            break;
          }
          std::cout << "VM_MODIFY_MEMORY_SUCCESS\n";
        } catch (...) {
          std::cout << "VM_MODIFY_MEMORY_ERROR\n";
        }
        break;
      }

      case command_handler::CommandType::DUMP_MEMORY: {
        try {
          vm->memory_controller_.DumpMemory(cmd.args);
        } catch (...) {
          std::cout << "VM_MEMORY_DUMP_ERROR\n";
        }
        break;
      }

      case command_handler::CommandType::PRINT_MEMORY: {
        for (size_t i = 0; i + 1 < cmd.args.size(); i += 2) {
          try {
            uint64_t address = std::stoull(cmd.args[i], nullptr, 16);
            uint64_t rows = std::stoull(cmd.args[i + 1]);
            vm->memory_controller_.PrintMemory(address, static_cast<unsigned int>(rows));
          } catch (...) {
            std::cout << "VM_PRINT_MEMORY_ERROR\n";
          }
        }
        std::cout << std::endl;
        break;
      }

      case command_handler::CommandType::GET_MEMORY_POINT: {
        if (cmd.args.size() != 1) {
          std::cout << "VM_GET_MEMORY_POINT_ERROR\n";
        } else {
          try {
            vm->memory_controller_.GetMemoryPoint(cmd.args[0]);
          } catch (...) {
            std::cout << "VM_GET_MEMORY_POINT_ERROR\n";
          }
        }
        break;
      }

      case command_handler::CommandType::VM_STDIN: {
        if (!cmd.args.empty()) vm->PushInput(cmd.args[0]);
        break;
      }

      case command_handler::CommandType::DUMP_CACHE: {
        // Placeholder: vm has cache functionality in future
        std::cout << "VM_CACHE_DUMPED" << std::endl;
        break;
      }

      case command_handler::CommandType::ADD_BREAKPOINT: {
        if (!cmd.args.empty()) vm->AddBreakpoint(std::stoul(cmd.args[0], nullptr, 10));
        break;
      }

      case command_handler::CommandType::REMOVE_BREAKPOINT: {
        if (!cmd.args.empty()) vm->RemoveBreakpoint(std::stoul(cmd.args[0], nullptr, 10));
        break;
      }

      case command_handler::CommandType::UNDO: {
        if (!vm_running) vm->Undo();
        break;
      }

      case command_handler::CommandType::REDO: {
        if (!vm_running) vm->Redo();
        break;
      }

      case command_handler::CommandType::RESET: {
        vm->Reset();
        break;
      }

      default:
        std::cout << "Unknown or unhandled command\n";
        break;
       
        }
    }

    // Clean up before exit
    vm->RequestStop();
    if (vm_thread.joinable()) vm_thread.join();

    return 0;
}
