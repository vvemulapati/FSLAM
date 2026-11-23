# Timing constraints for hw_top (Ultra96, Zynq UltraScale+ ZU3EG).
# Run standalone synthesis/implementation with -mode out_of_context since the
# AXI4-Stream ports connect inside the Zynq PL fabric, not package I/O pins.

# 150 MHz PL clock from Zynq PS
create_clock -period 6.667 -name aclk -waveform {0.000 3.333} [get_ports aclk]
set_clock_uncertainty -setup 0.200 [get_clocks aclk]
set_property HD.CLK_SRC BUFG_PS_X0Y48 [get_ports aclk]

# Active-low reset, excluded from timing
set_false_path -from [get_ports aresetn]
