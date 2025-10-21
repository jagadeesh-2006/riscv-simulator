addi x1, x0, 1
addi x10, x0, 1

fdsff:
lol:
addi x10, x10, 1
jal x1, 12

mul x1, x0, x10

#
#lb x0, 0(x1)
#lh x0, 0(x1)
#lw x0, 0(x1)
#ld x0, 0(x1)
#lbu x0, 0(x1)
#lhu x0, 0(x1)
#lwu x0, 0(x1)
#
sb x0, 0(x1)
#sh x0, 0(x1)
#sw x0, 0(x1)
#sd x0, 0(x1)
#
beq x0, x0, lol