.section .data
.float -2.3, 3.2
.double -2.3, 3.2
# .word 0
# .byte 1
# .dword 1

# test: .dword 0x123456789abcdef0

.section .text

addi x3, x1, 0


lui x3, 0x10000
# ld x4, 0(x0)
# sd x4, 0(x3)
# addi sp, x3, 0x100


# fld f7, 8(x3)

# flw f10, 0(x3)
# flw f11, 4(x3)
# fadd.s f12, f10, f11

fld f13, 8(x3)
fld f14, 16(x3)
fadd.d f15, f13, f14

fsd f15, 24(x3)


#################################


fcvt.w.s x17, f10
fcvt.wu.s x17, f10
fcvt.s.w f17, x10



lw x10, 0(x0)

feq.s x10, f1, f2 
fadd.s f10, f11, f12, rne
fmv.x.w x10, f10
add x10, x10, x10

fmadd.s f0, f3, f6, f27

flw f10, 0(x10)
fsw f10, 0(x10)
