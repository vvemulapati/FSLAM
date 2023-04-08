# FSLAM
If you use FSLAM in an academic work, please cite the following publication

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
Contains RTL source files for the hardware accelerator

### SW
ORBSLAM-2 test environment forked from the ORBSLAM2 source code

## Setup instructions

### Required tools/items
- Vivado Design Suite 2018.3
- Ultra96 Development board
- Libraries: OpenCV, LibEigen3, Pangolin (Prerequisites for ORB-SLAM2)
- TUM dataset located at https://cvg.cit.tum.de/data/datasets/rgbd-dataset

### Build ORBSLAM2

Build ORBSLAM2 code located in SW without ROS using the instructions here: https://github.com/raulmur/ORB_SLAM2

### Compile the HW IP using Vivado

Compile the HW folder IP with the top module as hw_top. 

Wrap the IP, and export the bitstream for use in SDSoC

### Build ORBSLAM2 in Xilinx SDSoC

Build the ORBSLAM2 source code in SDSoC by Including/Linking the appropriate libraries mentioned above.



## Results

### Resource Utilization

|             | Description |
| ----------- | ----------- |
| LUTs        | 76424       |
| FFs         | 101694        |
| DSP         | 80        |
| BRAM        | 120        |



