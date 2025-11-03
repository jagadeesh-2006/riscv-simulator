# Test data hazards
# Initialize values
addi x1, x0, 10      # x1 = 10
addi x3, x0, 5       # x3 = 5
addi x5, x0, 3       # x5 = 3

# Test sequence with data hazards
add x2, x1, x3       # x2 = x1 + x3 = 15
add x4, x2, x5       # x4 = x2 + x5 = 18 (depends on previous x2)
add x6, x4, x1       # x6 = x4 + x1 = 28 (depends on previous x4)

# # Store results to verify
# sd x2, 0(x0)         # Store x2 (should be 15)
# sd x4, 8(x0)         # Store x4 (should be 18)
# sd x6, 16(x0)        # Store x6 (should be 28)