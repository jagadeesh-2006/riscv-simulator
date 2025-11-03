# addi x1,x0,5

   .data
result1: .word 0
result2: .word 0

     .text

    # Setup constants in registers (independent)
    li  x1, 10        # x1 = 10
    li  x2, 20        # x2 = 20

    li  x5, 3         # x5 = 3
    li  x6, 7         # x6 = 7

    # Two independent additions (no RAW between them)
    add x3, x1, x2    # x3 = x1 + x2   (10 + 20 = 30)
    add x4, x5, x6    # x4 = x5 + x6   (3 + 7 = 10)

    # Store results to memory (still independent)
    la  x10, result1  # x10 = address of result1
    sw  x3, 0(x10)    # store x3 -> result1

    la  x11, result2  # x11 = address of result2
    sw  x4, 0(x11)    # store x4 -> result2

    # Exit (for environments expecting an ecall; optional)
    li  x17, 93       # ecall number for exit on many Linux riscv ports
    li  x10, 0        # exit code 0
    ecall


# .text

# _start:
#     addi x1, x0, 5      # x1 = 5
#     addi x2, x0, 10     # x2 = 10

#     add  x3, x1, x2     # x3 = x1 + x2 = 15
#     add  x4, x3, x1     # ❗ hazard: uses x3 immediately after it's written
#     add  x5, x4, x2     # continues using results
