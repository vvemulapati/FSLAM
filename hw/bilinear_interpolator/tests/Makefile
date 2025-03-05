#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: bilinear_interpolator.sv / test_bilinear_interpolator.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/bilinear_interpolator.sv

# Name of the top‐level module in bilinear_interpolator.sv
TOPLEVEL         := bilinear_interpolator

# Python module name (without .py) for your Cocotb tests
MODULE           := test_bilinear_interpolator

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump