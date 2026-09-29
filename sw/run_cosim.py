#!/usr/bin/env python3
"""RTL vs. Python reference model co-simulation.

Generates a synthetic 640x480 test image, writes it to input_pixels.hex, runs
the reference model (ref_features.txt) and the VCS simulation
(rtl_features.txt), then compares per-level feature counts, keypoint
positions, orientation and descriptor distance.
"""

import os
import sys
import subprocess
from orb_hardware_reference_model import (
    ORBHardwareReferenceModel,
    generate_synthetic_image,
    popcount256
)

def run_verification(hw_dir: str) -> bool:
    print("RTL vs. reference model co-simulation")

    # Test image and reference features
    print("Generating test image and reference features...")
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
    print(f"  reference model: {len(ref_features)} features")

    # input_pixels.hex: four 6-bit pixels per 32-bit word
    hex_path = os.path.join(hw_dir, "input_pixels.hex")
    print(f"Writing {hex_path}")
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

    # Reference features
    ref_feat_path = os.path.join(hw_dir, "ref_features.txt")
    with open(ref_feat_path, "w") as f:
        for kpt in ref_features:
            f.write(f"FEAT: level={kpt.level} ori={kpt.angle_sector} score={kpt.score} x={kpt.x} y={kpt.y} desc={kpt.descriptor:064x}\n")
    print(f"Wrote {len(ref_features)} features to {ref_feat_path}")

    # VCS
    print("Running VCS...")
    filelist_path = os.path.join(hw_dir, "filelist.f")
    if not os.path.exists(filelist_path):
        print(f"ERROR: {filelist_path} not found")
        return False

    vcs_cmd = (
        "source ~/git/hxc/env/init.sh && "
        "module load licenses/synopsys vcs/2025-06 && "
        f"cd {hw_dir} && "
        "vcs -full64 -sverilog +v2k -timescale=1ns/1ps -f filelist.f tb_orb_cosim.sv -o simv_cosim && "
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
        print("ERROR: VCS simulation failed")
        return False

    # RTL features
    rtl_feat_path = os.path.join(hw_dir, "rtl_features.txt")
    if not os.path.exists(rtl_feat_path):
        print(f"ERROR: {rtl_feat_path} not found")
        return False

    rtl_features = []
    has_xz = False
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
            desc_str = kvs.get("desc", "")
            if any(c in desc_str for c in ("x", "X", "z", "Z")):
                has_xz = True
            rtl_features.append(kvs)

    print(f"  RTL: {len(rtl_features)} features")
    if has_xz:
        print("ERROR: RTL output contains X/Z bits in descriptor")
        return False

    # Count by level
    ref_by_level = {0: 0, 1: 0, 2: 0, 3: 0}
    for r in ref_features:
        ref_by_level[r.level] = ref_by_level.get(r.level, 0) + 1

    rtl_by_level = {0: 0, 1: 0, 2: 0, 3: 0}
    for rtl in rtl_features:
        lvl = int(rtl.get("level", 0))
        rtl_by_level[lvl] = rtl_by_level.get(lvl, 0) + 1

    print("Features per level:")
    for lvl in range(4):
        print(f"  - Level {lvl} ({ref_model.level_dims[lvl][0]}x{ref_model.level_dims[lvl][1]}): Ref={ref_by_level[lvl]} | RTL={rtl_by_level[lvl]}")

    print(f"\nTotal: ref={len(ref_features)} RTL={len(rtl_features)}")

    # 1-to-1 keypoint matching per level
    matched_kpts = 0
    exact_coord_kpts = 0
    exact_orientations = 0
    descriptor_hamming_dist_sum = 0

    ref_kpts_by_lvl = {0: [], 1: [], 2: [], 3: []}
    for r in ref_features:
        ref_kpts_by_lvl[r.level].append(r)

    used_ref = {0: set(), 1: set(), 2: set(), 3: set()}

    for rtl in rtl_features:
        lvl = int(rtl.get("level", 0))
        x = int(rtl.get("x", 0))
        y = int(rtl.get("y", 0))
        ori = int(rtl.get("ori", 0))
        rtl_desc = int(rtl.get("desc", "0"), 16)

        best_ref = None
        best_idx = -1
        min_dist_sq = 100
        for idx, r in enumerate(ref_kpts_by_lvl.get(lvl, [])):
            if idx in used_ref[lvl]:
                continue
            dist_sq = (x - r.x) ** 2 + (y - r.y) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_ref = r
                best_idx = idx

        if best_ref is not None:
            used_ref[lvl].add(best_idx)
            matched_kpts += 1
            if min_dist_sq == 0:
                exact_coord_kpts += 1

            ori_diff = abs(ori - best_ref.angle_sector) % 64
            if ori_diff <= 2 or ori_diff >= 62:
                exact_orientations += 1

            h_dist = popcount256(rtl_desc ^ best_ref.descriptor)
            descriptor_hamming_dist_sum += h_dist

    match_rate = (matched_kpts / len(ref_features) * 100.0) if ref_features else 0.0
    print(f"\nKeypoint Position Association : {matched_kpts}/{len(ref_features)} ({match_rate:.1f}%) [exact coords: {exact_coord_kpts}/{len(ref_features)}]")
    ori_accuracy = 0.0
    avg_h_dist = 256.0
    if matched_kpts > 0:
        ori_accuracy = (exact_orientations / matched_kpts * 100.0)
        avg_h_dist = descriptor_hamming_dist_sum / matched_kpts
        print(f"Orientation Sector Agreement  : {exact_orientations}/{matched_kpts} ({ori_accuracy:.1f}%)")
        print(f"Average Descriptor Distance   : {avg_h_dist:.2f} bits / 256 bits")

    passed = (
        len(rtl_features) == len(ref_features)
        and matched_kpts == len(ref_features)
        and ori_accuracy >= 95.0
        and avg_h_dist <= 8.0
    )
    print("PASS" if passed else "FAIL")
    return passed

if __name__ == "__main__":
    hw_directory = sys.argv[1] if len(sys.argv) > 1 else "/home/vibhakarv/sandbox/FSLAM/hw"
    if not run_verification(hw_directory):
        sys.exit(1)
