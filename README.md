# RISC-V simulator

## Building the Project

The code base is written in C++17, to build the project use cmake. (You might want to use 
ninja for faster builds.)

## Usage

1. Build

Windows (MinGW) example from project root:

```powershell
Set-Location -Path 'D:\sem3\carch\riscv-simulator\build'
cmake -G "MinGW Makefiles" ..
mingw32-make -j 4
```

2. Run interactive VM shell

Default (single-cycle VM):

```powershell
.\vm.exe --start-vm
```

Select VM mode at startup with `--vm-mode`:

```powershell
.\vm.exe --vm-mode single --start-vm
.\vm.exe --vm-mode pipelined_no_hazard --start-vm
.\vm.exe --vm-mode pipelined_with_hazard --start-vm
```

3. One-shot run (assemble and run):

```powershell
.\vm.exe --run examples\load_store_test_1.s
```
vm_stdin "hello\n"

modify_memory 0x10000 byte 0xAA         #to change the memory

print_mem 0x10000 2                 #print the memory and data from that posiiton 

get_register x10

l ..\examples\gcd_1.s
run
reset    #to load the other program

dump_mem 0x10000 2

exit

Once in the interactive shell, use commands from `COMMANDS.md` (LOAD, RUN, STEP, STOP, RESET, etc.).


remove_breakpoint <line_number>     # Remove breakpoint at a specific line number


## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.


## References
- [RISC-V Specifications](https://riscv.org/specifications/)
- [Five EmbedDev ISA manual](https://five-embeddev.com/riscv-isa-manual/)