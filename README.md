# FSLAM
If you use FSLAM in an academic work, please cite the following publication:

```
@inproceedings{vemulapati2022fslam,
  title={FSLAM: an Efficient and Accurate SLAM Accelerator on SoC FPGAs},
  author={Vemulapati, Vibhakar and Chen, Deming},
  booktitle={2022 International Conference on Field-Programmable Technology (ICFPT)},
  pages={1--9},
  year={2022},
  organization={IEEE}
}
```

## Contents of project

### HW
SystemVerilog RTL and simulation testbenches for the ORB feature extraction accelerator (`hw/`):

- `hw_top`: Top-level AXI4-Stream wrapper. Unpacks four 6-bit pixels per 32-bit word, buffers output features in `descriptor_fifo`, and streams each feature out as a 10-word AXI4-Stream packet.
- `orb` and `image_pyramid`: 4-level streaming image pyramid with 5/6 bilinear downscaling (`scaling`, `bilinear_interpolator`), feeding one `feature_extractor` per level.
- `feature_extractor`: Single-scale pipeline with FAST-9 corner detection (`fast`), 3x3 non-maximum suppression (`nms`), `keypoint_fifo`, 7x7 binomial filter (`gaussian_blur`), shared `window_buffer` / `linebuffer`, 64-sector intensity-centroid `orientation`, and 256-bit steered rBRIEF (`brief`, `rotator`).
- `feature_matcher`: Pipelined Hamming-distance descriptor matcher.
- `tb_orb_cosim.sv` and `tb_verilator.cpp`: VCS and Verilator testbenches for `hw_top`.

### SW
ORB-SLAM2 C++ source code (forked from ORB-SLAM2) along with Python reference models and co-simulation scripts (`sw/`):

- `sw/orb_hardware_reference_model.py`: Cycle/bit-accurate Python model of the hardware pipeline, used as the reference for RTL co-simulation.
- `sw/orb_golden_model.py`: Python port of the OpenCV / ORB-SLAM2 extractor (`sw/src/ORBextractor.cc`) to compare the hardware pipeline against OpenCV across pyramid scaling, FAST-9/NMS, orientation, and rBRIEF descriptors.
- `sw/run_cosim.py`: Generates stimulus from the test images in `sw/test_images/`, runs RTL simulation in VCS or Verilator, and checks the RTL output against `orb_hardware_reference_model.py`.

## Running simulation

Run the Python reference and OpenCV comparison models:
```bash
python3 sw/orb_hardware_reference_model.py
python3 sw/orb_golden_model.py
```

Run RTL co-simulation with VCS or Verilator (use `--image <name_or_pgm_path>` for a single image):
```bash
python3 sw/run_cosim.py hw --sim vcs --image all
python3 sw/run_cosim.py hw --sim verilator --image all
```

Or build and run directly from `hw/Makefile`:
```bash
cd hw
make vcs
make verilator
make cosim
```

## Setup instructions

### Required tools/items
- Vivado Design Suite 2022.1
- Ultra96 Development board
- Libraries: OpenCV, LibEigen3, Pangolin (Prerequisites for ORB-SLAM2)
- TUM dataset located at https://cvg.cit.tum.de/data/datasets/rgbd-dataset

### Build ORBSLAM2

Build ORBSLAM2 code located in `sw/` without ROS using the instructions here: https://github.com/raulmur/ORB_SLAM2

### Compile the HW IP using Vivado

Compile the `hw/` folder IP with the top module as `hw_top`.

Wrap the IP, and export the bitstream for use in SDSoC.

### Build ORBSLAM2 in Xilinx SDSoC

Build the ORBSLAM2 source code in SDSoC by including/linking the appropriate libraries mentioned above.

## Results

### Resource Utilization

| Resource | Utilization |
| -------- | ----------- |
| LUTs     | 76424       |
| FFs      | 101694      |
| DSP      | 80          |
| BRAM     | 120         |
