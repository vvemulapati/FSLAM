#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: brief.sv / test_brief.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/brief.sv

# Name of the top‐level module in brief.sv
TOPLEVEL         := brief

# Python module name (without .py) for your Cocotb tests
MODULE           := test_brief

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump