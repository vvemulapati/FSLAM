#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: nms.sv / test_nms.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/nms.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/window_buffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/shift_register.sv

# Name of the top‐level module in nms.sv
TOPLEVEL         := nms

# Python module name (without .py) for your Cocotb tests
MODULE           := test_nms

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump