#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: feature_matcher.sv / test_feature_matcher.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/feature_matcher.sv

# Name of the top‐level module in feature_matcher.sv
TOPLEVEL         := feature_matcher

# Python module name (without .py) for your Cocotb tests
MODULE           := test_feature_matcher

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump