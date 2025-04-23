#------------------------------------------------------------------------------#
# Makefile for Cocotb testbench: hw_top.sv / test_hw_top.py
#------------------------------------------------------------------------------#

# Top‐level language & sources
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES  = $(PWD)/../hdl/hw_top.sv
VERILOG_SOURCES += $(PWD)/../../fast/hdl/fast.sv
VERILOG_SOURCES += $(PWD)/../../brief/hdl/brief.sv
VERILOG_SOURCES += $(PWD)/../../orientation/hdl/orientation.sv
VERILOG_SOURCES += $(PWD)/../../nms/hdl/nms.sv
VERILOG_SOURCES += $(PWD)/../../orb/hdl/orb.sv
VERILOG_SOURCES += $(PWD)/../../image_pyramid/hdl/image_pyramid.sv
VERILOG_SOURCES += $(PWD)/../../bilinear_interpolator/hdl/bilinear_interpolator.sv
VERILOG_SOURCES += $(PWD)/../../linebuffer/hdl/linebuffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/window_buffer.sv
VERILOG_SOURCES += $(PWD)/../../window_buffer/hdl/shift_register.sv
VERILOG_SOURCES += $(PWD)/../../pipelined_adder/hdl/pipelined_adder_tree.sv
VERILOG_SOURCES += $(PWD)/../../rotator/hdl/rotator.sv
VERILOG_SOURCES += $(PWD)/../../gaussian_blur/hdl/gaussian_blur.sv
VERILOG_SOURCES += $(PWD)/../../scaling/hdl/scaling.sv
VERILOG_SOURCES += $(PWD)/../../feature_matcher/hdl/feature_matcher.sv

# Name of the top‐level module in hw_top.sv
TOPLEVEL         := hw_top

# Python module name (without .py) for your Cocotb tests
MODULE           := test_hw_top

# Select simulator (uncomment or override on cmd line)
# SIM = icarus
# SIM = verilator
# SIM = modelsim
# SIM = xcelium

# If your DUT or tests import local Python packages, uncomment:
# export PYTHONPATH := $(PWD):$(PYTHONPATH)

# Include Cocotb's Makefile (auto‐finds cocotb‐config)
include ../../cocotb_common/Makefile.dump