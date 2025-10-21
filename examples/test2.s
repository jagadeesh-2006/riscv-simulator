.section .data
dnum1: .double 1.5
dnum2: .double 3.7
newline: .string "\n"
big_num: .dword 0x100000000

buffer: .zero 255  # Buffer for reading input

.text

li a7, 63               # syscall number for read
li a0, 0                # file descriptor: stdin
la a1, buffer           # Load address of buffer 
addi a2, x0,15          # read 15 bytes
ecall

addi a7, x0, 64         # syscall number for write
addi a0, x0, 1          # file descriptor: stdout
la a1, buffer           # Load address of buffer
addi a2, x0, 15         # write 15 bytes
ecall

jal ra, print_newline

test:
la x3, dnum1            # Load address of dnum1 into x3
fld f0, 0(x3)           # Load the first double from memory into f
la x3, dnum2            # Load address of dnum2 into x3
fld f1, 0(x3)           # Load the second double from memory into f

fadd.d f2, f0, f1       # Add the two doubles and store the result in f2
fmv.x.d a0, f2          # Move the result from f2 to a0 (integer register)
li a7, 3                # custom syscall number for print double floating point
ecall

jal ra, print_newline

li a7, 1                # syscall number for print integer
li a0, 42
ecall


jal x0, exit


print_newline:
    li a7, 64           # syscall number for write
    li a0, 1            # file descriptor: stdout
    la a1, newline
    li a0, 1            # write 1 byte
    ecall
    ret

exit:
    nop
