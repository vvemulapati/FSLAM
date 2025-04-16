#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: scaling.sv / test_scaling.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/scaling.sv
VERILOG_SOURCES += $(PWD)/../../bilinear_interpolator/hdl/bilinear_interpolator.sv
VERILOG_SOURCES += $(PWD)/../../linebuffer/hdl/linebuffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/window_buffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/shift_register.sv

# Name of the top‐level module in scaling.sv
TOPLEVEL         := scaling

# Python module name (without .py) for your Cocotb tests
MODULE           := test_scaling

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump