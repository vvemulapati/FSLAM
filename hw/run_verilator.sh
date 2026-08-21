#!/usr/bin/env bash
#-------------------------------------------------------------------------------
# File: run_verilator.sh
# Description: Compiles and runs the FSLAM ORB RTL accelerator using Verilator
#-------------------------------------------------------------------------------

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

VERILATOR_BIN="/tools/veripool/verilator/4-020/bin/verilator"
VERILATOR_SHARE="/tools/veripool/verilator/4-020/share/verilator"

echo "================================================================"
echo " FSLAM: Building & Running with Verilator"
echo "================================================================"

# Check if input pixels file exists, otherwise generate it
if [ ! -f "input_pixels.hex" ]; then
    echo "[INFO] Generating test frame vectors with Python reference model..."
    python3 ../sw/run_cosim.py .
fi

# Step 1: Translate RTL to C++ with Verilator
echo "[1] Translating SystemVerilog to C++..."
${VERILATOR_BIN} --cc -Wall -Wno-fatal \
    -Wno-DECLFILENAME -Wno-UNOPTFLAT -Wno-WIDTH -Wno-PINMISSING \
    -Wno-UNUSED -Wno-CASEINCOMPLETE -Wno-SYMRSVDWORD -Wno-PINCONNECTEMPTY \
    -f filelist.f \
    --top-module hw_top \
    --exe tb_verilator.cpp \
    --Mdir obj_dir_verilator

# Step 2: Compile C++ Model with g++
echo "[2] Compiling C++ simulation model..."
make -C obj_dir_verilator -f Vhw_top.mk VERILATOR_ROOT="${VERILATOR_SHARE}" Vhw_top -j$(nproc)

# Step 3: Execute C++ Simulation
echo "[3] Running simulation..."
./obj_dir_verilator/Vhw_top input_pixels.hex

echo "================================================================"
echo " Results saved to rtl_features_verilator.txt"
echo " Total features: $(wc -l < rtl_features_verilator.txt)"
echo "================================================================"
