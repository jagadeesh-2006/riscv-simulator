.data

fnum1: .float 2.44

fnum2: .float 3.14

fnum3: .float -4.56

fnum4: .float -8.732



.text


la x3, fnum1
flw f10, 0(x3)

la x3, fnum2
flw f11, 0(x3)

la x3, fnum3
flw f12, 0(x3)

la x3, fnum4
flw f13, 0(x3)


# fmax.s f14, f10, f11
# fmax.s f15, f12, f13

fclass.s x10, f10




nop













