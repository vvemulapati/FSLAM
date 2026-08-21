# Timing constraints for hw_top (Ultra96, Zynq UltraScale+ ZU3EG).

# 150 MHz
create_clock -period 6.667 -name aclk -waveform {0.000 3.333} [get_ports aclk]
set_clock_uncertainty 0.200 [get_clocks aclk]

# Active-low reset, excluded from timing
set_false_path -from [get_ports aresetn]

# I/O delays: 2.0 ns max (about 30% of the period), 0.5 ns min
set_input_delay -clock [get_clocks aclk] -max 2.000 [get_ports { \
    s_axis_tdata[*] \
    s_axis_tvalid \
    s_axis_tlast \
    m_axis_tready \
}]

set_input_delay -clock [get_clocks aclk] -min 0.500 [get_ports { \
    s_axis_tdata[*] \
    s_axis_tvalid \
    s_axis_tlast \
    m_axis_tready \
}]

set_output_delay -clock [get_clocks aclk] -max 2.000 [get_ports { \
    s_axis_tready \
    m_axis_tdata[*] \
    m_axis_tvalid \
    m_axis_tlast \
    interrupt \
}]

set_output_delay -clock [get_clocks aclk] -min 0.500 [get_ports { \
    s_axis_tready \
    m_axis_tdata[*] \
    m_axis_tvalid \
    m_axis_tlast \
    interrupt \
}]
