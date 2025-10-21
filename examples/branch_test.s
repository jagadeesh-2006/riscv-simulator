label:

lui x3, 0x10000

ld x10, 0(x0)
sd x10, 0(x3)

addi x10, x10, 23
addi x10, x10, 27
blt x0 , x10, label