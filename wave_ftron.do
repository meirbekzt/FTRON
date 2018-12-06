vlib work
vlog -timescale 1ns/1ns test.v
vsim -L altera_mf_ver test
log {/*}
add wave -r {/*}

force {CLOCK_50} 0 0, 1 1 -r 2
force {resetn} 0 0, 1 5
force {outCode} 2#00101001 10

run 45ns


