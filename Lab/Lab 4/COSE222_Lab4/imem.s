start:
lw x4, 0(x0)
lw x5, 8(x0)
add x7, x5, x4
sw x7, 24(x0)
sub x8, x5, x4
lw x6, 8(x8)
sw x8, 32(x0)
and x9, x5, x4
sw x9, 40(x0)
or x10, x5, x4
sw x10, 48(x0)
add x11, x4, x6
add x11, x11, x11
add x12, x11, x11
add x11, x4, x6
beq x6, x8, T2
T1:
add x4, x4, x4
sub x5, x4, x4
or x13, x11, x0
beq x0, x5, T3
T2:
lw x9, 8(x8)
lw x10, 16(x0)
beq x9, x10, T1
T3: