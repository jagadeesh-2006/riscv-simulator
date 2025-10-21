.section .data
fnum1: .float 1.5
fnum2: .float -3.7
fnum3: .float 2.5
fnum4: .float 4.2

newline: .string "\n"

.section .text
la x3, fnum1            
fld f0, 0(x3)           # Load the first float from memory into f0
la x3, fnum2            
fld f1, 0(x3)           # Load the second float from memory into f
fadd.s f10, f0, f1       # Add the two floats and store the result in f10

jal ra, print_float_f10



jal ra, print_newline

print_newline:
    li a7, 64           # syscall number for write
    li a0, 1            # file descriptor: stdout
    la a1, newline
    li a2, 1            # write 1 byte
    ecall
    ret

print_float_f10:
    li a7, 2            # custom syscall number for print float
    fmv.x.w a0, f10      # Move the result from f10 to a0 (integer register)
    ecall
    jal ra, print_newline

