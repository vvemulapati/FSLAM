#!/usr/bin/env bash
# Build and run the Verilator simulation of hw_top.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

VERILATOR_BIN="/tools/veripool/verilator/4-020/bin/verilator"
VERILATOR_SHARE="/tools/veripool/verilator/4-020/share/verilator"

# Generate the stimulus if it's missing
if [ ! -f "input_pixels.hex" ]; then
    echo "Generating input_pixels.hex..."
    python3 ../sw/run_cosim.py .
fi

echo "Verilating..."
${VERILATOR_BIN} --cc -Wall -Wno-fatal \
    -Wno-DECLFILENAME -Wno-UNOPTFLAT -Wno-WIDTH -Wno-PINMISSING \
    -Wno-UNUSED -Wno-CASEINCOMPLETE -Wno-SYMRSVDWORD -Wno-PINCONNECTEMPTY \
    -f filelist.f \
    --top-module hw_top \
    --exe tb_verilator.cpp \
    --Mdir obj_dir_verilator

echo "Building..."
make -C obj_dir_verilator -f Vhw_top.mk VERILATOR_ROOT="${VERILATOR_SHARE}" Vhw_top -j$(nproc)

echo "Running..."
./obj_dir_verilator/Vhw_top input_pixels.hex

echo "$(wc -l < rtl_features_verilator.txt) features written to rtl_features_verilator.txt"
