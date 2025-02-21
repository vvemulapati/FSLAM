#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: gaussian_blur.sv / test_gaussian_blur.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/gaussian_blur.sv
VERILOG_SOURCES += $(PWD)/../../linebuffer/hdl/linebuffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/window_buffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/shift_register.sv

# Name of the top‐level module in gaussian_blur.sv
TOPLEVEL         := gaussian_blur

# Python module name (without .py) for your Cocotb tests
MODULE           := test_gaussian_blur

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump