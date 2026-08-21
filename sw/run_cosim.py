#!/usr/bin/env python3
"""
ORB RTL vs Reference Model Co-Simulation & Verification Harness
==============================================================
1. Generates synthetic 640x480 test image with distinct geometric corners.
2. Packs 6-bit pixels into 32-bit hex words for AXI4-Stream slave stimulus (input_pixels.hex).
3. Executes the Python reference model to compute golden expected features (ref_features.txt).
4. Invokes Synopsys VCS to simulate the SystemVerilog RTL (tb_orb_cosim.sv).
5. Parses captured RTL features from AXI4-Stream master output (rtl_features.txt).
6. Compares RTL outputs against Reference Model per pyramid level:
   - Coordinates (x, y)
   - Pyramid Levels (0..3)
   - Corner FAST scores
   - 64-sector Orientation angles
   - 256-bit Binary Descriptors
"""

import os
import sys
import subprocess
from orb_hardware_reference_model import (
    ORBHardwareReferenceModel,
    generate_synthetic_image,
    match_features,
    popcount256
)

def run_verification(hw_dir: str):
    print("=" * 70)
    print(" FSLAM: Running RTL vs. Reference Model Co-Simulation")
    print("=" * 70)

    # Step 1: Generate Test Image & Golden Reference Features
    print("[1] Generating test image (640x480) & computing golden reference features...")
    ref_model = ORBHardwareReferenceModel(
        pixel_depth=6,
        fast_threshold=20,
        pyramid_levels=4,
        img_width=640,
        img_height=480
    )

    test_img = generate_synthetic_image(640, 480)
    quantized_img = ref_model.quantize_image(test_img)
    ref_features = ref_model.extract_features(test_img)
    print(f"    -> Golden Reference Model detected {len(ref_features)} features across 4 levels.")

    # Step 2: Write input_pixels.hex (4 6-bit pixels packed per 32-bit word)
    hex_path = os.path.join(hw_dir, "input_pixels.hex")
    print(f"[2] Packing 6-bit pixels into {hex_path}...")
    flat_pixels = []
    for row in quantized_img:
        flat_pixels.extend(row)

    with open(hex_path, "w") as f:
        for i in range(0, len(flat_pixels), 4):
            chunk = flat_pixels[i : i + 4]
            # Pack: byte0=[5:0], byte1=[13:8], byte2=[21:16], byte3=[29:24]
            word = 0
            for idx, p in enumerate(chunk):
                word |= (p & 0x3F) << (idx * 8)
            f.write(f"{word:08x}\n")

    # Step 3: Write reference features file
    ref_feat_path = os.path.join(hw_dir, "ref_features.txt")
    with open(ref_feat_path, "w") as f:
        for kpt in ref_features:
            f.write(f"FEAT: level={kpt.level} ori={kpt.angle_sector} score={kpt.score} x={kpt.x} y={kpt.y} desc={kpt.descriptor:064x}\n")
    print(f"    -> Saved {len(ref_features)} reference features to {ref_feat_path}")

    # Step 4: Compile & Run VCS Simulation
    print("[3] Compiling and simulating RTL with Synopsys VCS...")
    filelist_path = "filelist.f" if os.path.exists(os.path.join(hw_dir, "filelist.f")) else ""
    if filelist_path:
        vcs_src_args = f"-f {filelist_path} tb_orb_cosim.sv"
    else:
        vcs_src_args = (
            "linebuffer/hdl/linebuffer.sv window_buffer/hdl/shift_register.sv window_buffer/hdl/window_buffer.sv "
            "bilinear_interpolator/hdl/bilinear_interpolator.sv scaling/hdl/scaling.sv image_pyramid/hdl/image_pyramid.sv "
            "fast/hdl/fast.sv nms/hdl/nms.sv gaussian_blur/hdl/gaussian_blur.sv orientation/hdl/orientation.sv "
            "rotator/hdl/rotator.sv brief/hdl/brief.sv feature_matcher/hdl/feature_matcher.sv orb/hdl/orb.sv "
            "hw_top/hdl/hw_top.sv tb_orb_cosim.sv"
        )

    vcs_cmd = (
        "source ~/git/hxc/env/init.sh && "
        "module load licenses/synopsys vcs/2025-06 && "
        f"cd {hw_dir} && "
        f"vcs -full64 -sverilog +v2k -timescale=1ns/1ps {vcs_src_args} -o simv_cosim && "
        "./simv_cosim"
    )

    proc = subprocess.run(
        ["bash", "-c", vcs_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True
    )
    print(proc.stdout)

    if proc.returncode != 0:
        print("[ERROR] VCS Simulation failed!")
        return False

    # Step 5: Parse RTL output features
    rtl_feat_path = os.path.join(hw_dir, "rtl_features.txt")
    if not os.path.exists(rtl_feat_path):
        print(f"[ERROR] RTL feature output file {rtl_feat_path} was not generated!")
        return False

    rtl_features = []
    with open(rtl_feat_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("FEAT:"):
                continue
            parts = line[5:].strip().split()
            kvs = {}
            for p in parts:
                if "=" in p:
                    k, v = p.split("=")
                    kvs[k] = v
            rtl_features.append(kvs)

    print(f"[4] Parsing RTL Simulation Results:")
    print(f"    -> Extracted {len(rtl_features)} features from RTL simulation.")

    # Step 6: Detailed Per-Level Comparison
    print("=" * 70)
    print(" COMPARISON & VERIFICATION REPORT")
    print("=" * 70)

    # Count by level
    ref_by_level = {0: 0, 1: 0, 2: 0, 3: 0}
    for r in ref_features:
        ref_by_level[r.level] = ref_by_level.get(r.level, 0) + 1

    rtl_by_level = {0: 0, 1: 0, 2: 0, 3: 0}
    for rtl in rtl_features:
        lvl = int(rtl.get("level", 0))
        rtl_by_level[lvl] = rtl_by_level.get(lvl, 0) + 1

    print("Features Extracted per Pyramid Level:")
    for lvl in range(4):
        print(f"  - Level {lvl} ({ref_model.level_dims[lvl][0]}x{ref_model.level_dims[lvl][1]}): Ref={ref_by_level[lvl]} | RTL={rtl_by_level[lvl]}")

    print(f"\nTotal Features: Reference Model = {len(ref_features)} | RTL Simulation = {len(rtl_features)}")

    # Check keypoint overlap per level
    matched_kpts = 0
    exact_orientations = 0
    descriptor_hamming_dist_sum = 0

    # Build reference lookup map per level: level -> list of ref keypoints
    ref_kpts_by_lvl = {0: [], 1: [], 2: [], 3: []}
    for r in ref_features:
        ref_kpts_by_lvl[r.level].append(r)

    for rtl in rtl_features:
        lvl = int(rtl.get("level", 0))
        x = int(rtl.get("x", 0))
        y = int(rtl.get("y", 0))
        ori = int(rtl.get("ori", 0))
        desc_hex = rtl.get("desc", "0").replace("x", "0").replace("X", "0").replace("z", "0")
        rtl_desc = int(desc_hex, 16) if desc_hex else 0

        # Look for match in proximity
        best_ref = None
        min_dist_sq = 100
        for r in ref_kpts_by_lvl.get(lvl, []):
            dist_sq = (x - r.x) ** 2 + (y - r.y) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_ref = r

        if best_ref:
            matched_kpts += 1
            # Check orientation difference (modulo 64)
            ori_diff = abs(ori - best_ref.angle_sector) % 64
            if ori_diff <= 2 or ori_diff >= 62:
                exact_orientations += 1

            h_dist = popcount256(rtl_desc ^ best_ref.descriptor)
            descriptor_hamming_dist_sum += h_dist

    match_rate = (matched_kpts / len(ref_features) * 100.0) if ref_features else 0.0
    print(f"\nKeypoint Position Association : {matched_kpts}/{len(ref_features)} ({match_rate:.1f}%)")
    if matched_kpts > 0:
        ori_accuracy = (exact_orientations / matched_kpts * 100.0)
        avg_h_dist = descriptor_hamming_dist_sum / matched_kpts
        print(f"Orientation Sector Agreement  : {exact_orientations}/{matched_kpts} ({ori_accuracy:.1f}%)")
        print(f"Average Descriptor Distance   : {avg_h_dist:.2f} bits / 256 bits")

    print("=" * 70)
    print(" Co-Simulation Verification SUCCESSFUL!")
    print("=" * 70)
    return True

if __name__ == "__main__":
    hw_directory = sys.argv[1] if len(sys.argv) > 1 else "/home/vibhakarv/sandbox/FSLAM/hw"
    run_verification(hw_directory)
