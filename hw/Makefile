# Simulation targets for hw_top (VCS, Verilator, co-sim). See 'make help'.

SHELL := /bin/bash

VCS             ?= vcs
VERDI           ?= verdi
VERILATOR_BIN   ?= /tools/veripool/verilator/4-020/bin/verilator
VERILATOR_SHARE ?= /tools/veripool/verilator/4-020/share/verilator

RTL_FILELIST    := filelist.f
TB_SV           := tb_orb_cosim.sv
TB_CPP          := tb_verilator.cpp
SIMV_VCS        := simv_cosim
VERILATOR_DIR   := obj_dir_verilator
VERILATOR_EXE   := $(VERILATOR_DIR)/Vhw_top
HEX_STIMULUS    := input_pixels.hex

VCS_FLAGS       := -full64 -sverilog +v2k -timescale=1ns/1ps -f $(RTL_FILELIST) $(TB_SV) -o $(SIMV_VCS)
VCS_DEBUG_FLAGS := $(VCS_FLAGS) -debug_access+all -kdb -lca
VCS_FSDB_FLAGS  := $(VCS_DEBUG_FLAGS) +define+DUMP_FSDB
VCS_VCD_FLAGS   := $(VCS_FLAGS) +define+DUMP_VCD

VLT_FLAGS       := --cc -Wall -Wno-fatal \
                   -Wno-DECLFILENAME -Wno-UNOPTFLAT -Wno-WIDTH -Wno-PINMISSING \
                   -Wno-UNUSED -Wno-CASEINCOMPLETE -Wno-SYMRSVDWORD -Wno-PINCONNECTEMPTY \
                   -f $(RTL_FILELIST) \
                   --top-module hw_top \
                   --exe $(TB_CPP) \
                   --Mdir $(VERILATOR_DIR)

.PHONY: all vcs vcs_debug vcs_fsdb vcs_vcd sim_vcs sim_fsdb sim_vcd verdi verilator sim_verilator cosim clean help

all: vcs verilator

help:
	@echo "Targets:"
	@echo "  make vcs            : Compile RTL with Synopsys VCS"
	@echo "  make sim_vcs        : Run VCS simulation"
	@echo "  make vcs_fsdb       : Compile RTL with FSDB wave dumping"
	@echo "  make sim_fsdb       : Run simulation and write dump.fsdb"
	@echo "  make vcs_vcd        : Compile RTL with VCD wave dumping"
	@echo "  make sim_vcd        : Run simulation and write dump.vcd"
	@echo "  make verdi          : Open dump.fsdb in Verdi"
	@echo "  make verilator      : Build Verilator simulation binary"
	@echo "  make sim_verilator  : Run Verilator simulation"
	@echo "  make cosim          : Run Python/RTL co-simulation"
	@echo "  make clean          : Remove simulation outputs and waveforms"

$(HEX_STIMULUS):
	python3 ../sw/run_cosim.py .

vcs: $(SIMV_VCS)

RTL_SRCS := $(wildcard */hdl/*.sv */hdl/*.svh)

$(SIMV_VCS): $(RTL_FILELIST) $(RTL_SRCS) $(TB_SV)
	$(VCS) $(VCS_FLAGS)

sim_vcs: $(SIMV_VCS) $(HEX_STIMULUS)
	./$(SIMV_VCS)

vcs_fsdb: $(RTL_FILELIST) $(TB_SV)
	$(VCS) $(VCS_FSDB_FLAGS)

sim_fsdb: vcs_fsdb $(HEX_STIMULUS)
	./$(SIMV_VCS)

vcs_vcd: $(RTL_FILELIST) $(TB_SV)
	$(VCS) $(VCS_VCD_FLAGS)

sim_vcd: vcs_vcd $(HEX_STIMULUS)
	./$(SIMV_VCS)

verdi:
	$(VERDI) -ssf dump.fsdb -dbdir $(SIMV_VCS).daidir &

verilator: $(VERILATOR_EXE)

$(VERILATOR_EXE): $(RTL_FILELIST) $(RTL_SRCS) $(TB_CPP)
	$(VERILATOR_BIN) $(VLT_FLAGS)
	make -C $(VERILATOR_DIR) -f Vhw_top.mk VERILATOR_ROOT=$(VERILATOR_SHARE) Vhw_top -j$$(nproc)

sim_verilator: $(VERILATOR_EXE) $(HEX_STIMULUS)
	./$(VERILATOR_EXE) $(HEX_STIMULUS)

cosim:
	python3 ../sw/run_cosim.py .

clean:
	rm -rf $(SIMV_VCS) $(SIMV_VCS).daidir $(SIMV_VCS)_debug $(SIMV_VCS)_debug.daidir
	rm -rf $(VERILATOR_DIR) csrc ucli.key vc_hdrs.h DVEfiles verdiLog novas.*
	rm -rf rtl_features.txt rtl_features_verilator.txt ref_features.txt input_pixels.hex
	rm -rf dump.fsdb dump.vcd dump.vpd inter.fsdb novas.rc novas.conf
