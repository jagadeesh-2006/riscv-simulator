.data
dd: .dword 12

.text
    li a7, 64
    li a1, 0
    la a0, dd
    ecall
