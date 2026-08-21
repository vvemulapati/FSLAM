# ==============================================================================
# File: fslam_clock.xdc
# Description: Clock and timing constraints for FSLAM ORB Accelerator (hw_top)
# Target Architecture: Xilinx Zynq-7000 / Zynq UltraScale+ MPSoC
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. Primary Clock Definition (150 MHz = 6.667 ns period)
# ------------------------------------------------------------------------------
# For 150 MHz (Standard operating frequency):
create_clock -period 6.667 -name aclk -waveform {0.000 3.333} [get_ports aclk]

# Alternative Frequencies:
# For 200 MHz (Fast Mode / 5.000 ns period):
# create_clock -period 5.000 -name aclk -waveform {0.000 2.500} [get_ports aclk]
# For 100 MHz (Conservative / 10.000 ns period):
# create_clock -period 10.000 -name aclk -waveform {0.000 5.000} [get_ports aclk]

# ------------------------------------------------------------------------------
# 2. Clock Uncertainty / Jitter
# ------------------------------------------------------------------------------
set_clock_uncertainty 0.200 [get_clocks aclk]

# ------------------------------------------------------------------------------
# 3. Asynchronous Reset Constraints
# ------------------------------------------------------------------------------
# Treat active-low reset as a false path for setup/hold timing checks
set_false_path -from [get_ports aresetn]

# ------------------------------------------------------------------------------
# 4. Input / Output Interface Timing Constraints
# ------------------------------------------------------------------------------
# Assuming 30% of clock period (~2.0 ns at 150 MHz) for setup budget on SoC bus

# AXI-Stream & AXI-Lite Inputs
set_input_delay -clock [get_clocks aclk] -max 2.000 [get_ports { \
    s_axis_tdata[*] \
    s_axis_tvalid \
    s_axis_tlast \
    m_axis_tready \
    s_axi_awaddr[*] \
    s_axi_awvalid \
    s_axi_wdata[*] \
    s_axi_wstrb[*] \
    s_axi_wvalid \
    s_axi_bready \
    s_axi_araddr[*] \
    s_axi_arvalid \
    s_axi_rready \
}]

set_input_delay -clock [get_clocks aclk] -min 0.500 [get_ports { \
    s_axis_tdata[*] \
    s_axis_tvalid \
    s_axis_tlast \
    m_axis_tready \
    s_axi_awaddr[*] \
    s_axi_awvalid \
    s_axi_wdata[*] \
    s_axi_wstrb[*] \
    s_axi_wvalid \
    s_axi_bready \
    s_axi_araddr[*] \
    s_axi_arvalid \
    s_axi_rready \
}]

# AXI-Stream & AXI-Lite Outputs
set_output_delay -clock [get_clocks aclk] -max 2.000 [get_ports { \
    s_axis_tready \
    m_axis_tdata[*] \
    m_axis_tvalid \
    m_axis_tlast \
    s_axi_awready \
    s_axi_wready \
    s_axi_bresp[*] \
    s_axi_bvalid \
    s_axi_arready \
    s_axi_rdata[*] \
    s_axi_rresp[*] \
    s_axi_rvalid \
    interrupt \
}]

set_output_delay -clock [get_clocks aclk] -min 0.500 [get_ports { \
    s_axis_tready \
    m_axis_tdata[*] \
    m_axis_tvalid \
    m_axis_tlast \
    s_axi_awready \
    s_axi_wready \
    s_axi_bresp[*] \
    s_axi_bvalid \
    s_axi_arready \
    s_axi_rdata[*] \
    s_axi_rresp[*] \
    s_axi_rvalid \
    interrupt \
}]

# ------------------------------------------------------------------------------
# 5. Multicycle / Optimization Guidance (Optional Vivado Synthesis)
# ------------------------------------------------------------------------------
# Optimize RAM/SRL mapping for deep line buffers
# set_property RAM_STYLE block [get_cells -hier -filter {NAME =~ *u_brief_win_buf*mem*}]
