#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: image_pyramid.sv / test_image_pyramid.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/image_pyramid.sv
VERILOG_SOURCES += $(PWD)/../../scaling/hdl/scaling.sv
VERILOG_SOURCES += $(PWD)/../../bilinear_interpolator/hdl/bilinear_interpolator.sv
VERILOG_SOURCES += $(PWD)/../../linebuffer/hdl/linebuffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/window_buffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/shift_register.sv

# Name of the top‐level module in image_pyramid.sv
TOPLEVEL         := image_pyramid

# Python module name (without .py) for your Cocotb tests
MODULE           := test_image_pyramid

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump