label:
lui x3, 0x10000
# addi x10, x10, 23
# addi x10, x10, -27
# srli x10, x10, 2
# addi x12, x0, -1
# addi x13, x0, -90
# mul x14, x12, x13
# addi x12, x0, -100
# addi x13, x0, 23
# rem x14, x12, x13
# divu x15, x12, x13

# fadd.s f10, f2, f3

mul x10, x10, x10


addi x16, x0, -1
sd x16, 0(x3)
lw x17, 0(x3)

ld x16, 0(x0)

jal x1, label
