
#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: linebuffer.sv / test_linebuffer.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/pipelined_adder_tree.sv

# Name of the top‐level module in linebuffer.sv
TOPLEVEL         := pipelined_adder_tree

# Python module name (without .py) for your Cocotb tests
MODULE           := test_adder

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb’s Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump
