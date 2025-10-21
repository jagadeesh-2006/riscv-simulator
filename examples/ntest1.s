.data
dd1: .float 2.3
dd2: .float 2.7

.text
la x3, dd1
flw f10, 0(x3)

la x3, dd2
flw f11, 0(x3)

fle.s x10, f10, f11