# Commands

- `load` or `l`: `Absolute FilePath`  
  - Loads the specified file into the virtual machine.
  - The file must be a valid riscv64 imfd file. If some error occurs, it is dumped in `vm_state/errors_dump.json`.

- `run`
  - Executes the loaded file, without considering breakpoints and no delay in steps.

- `run_debug` or `rd`
  - Executes the loaded file, considering breakpoints and with a delay in steps (run_step_delay).

- `step` or `s`
  - Executes the next step in the loaded file.

- `undo` or `u`
  - Reverts the last executed step in the loaded file.

- `add_breakpoint`: `LineNumber` (unsigned int)
  - Adds a breakpoint at the specified line number in the loaded file.

- `remove_breakpoint`: `LineNumber` (unsigned int)
  - Removes the breakpoint at the specified line number in the loaded file.

- `vm_stdin` or `vmsin`: `Input` (string)
  - Sends input to the virtual machine's standard input.
  - Note: use double quotes for strings with spaces.

- `print_mem` or `pm`: `StartAddress1` (Hex) `NumOfRows1` (unsigned int) [`StartAddress2` `NumOfRows2` ...]
  - Prints the memory contents for each specified address and row count pair.
  - You can provide multiple pairs of start addresses and number of rows to print multiple memory regions in one command.

- `dump_mem` or `dm`: `StartAddress1` (Hex) `NumOfRows1` (unsigned int) [`StartAddress2` `NumOfRows2` ...]
  - Dumps the memory contents for each specified address and row count pair in the file `vm_state/memory_dump.json`.
  - You can provide multiple pairs of start addresses and number of rows to dump multiple memory regions in one command.

- `modify_config` or `mconfig`: `Section`, `Key`, `Value`
  - Modifies the internal configuration by setting the specified key in the given section to the provided value.
  - `Execution`
    - `processor_type` (string) : `single_stage` | `multi_stage`  
    - `run_step_delay` (unsigned int) : milliseconds
    - `instruction_execution_limit` (unsigned int) : Specifies the number of instruction to run on one use of `run` button. Set to `0` for no limit.
  - `Memory`
    - `memory_size` (unsigned int) : bytes
    - `memory_block_size` (unsigned int) : bytes  
