#!/usr/bin/env python3
"""RTL vs. Python reference model multi-image co-simulation suite.

Compiles the RTL simulator once and verifies hardware bit-exactness across a
suite of synthetic stress-test patterns and real-world 640x480 photographs
stored in sw/test_images/.
"""

import argparse
import os
import subprocess
import sys
from typing import Dict, List, Tuple

from orb_hardware_reference_model import (
    Keypoint,
    ORBHardwareReferenceModel,
    get_test_image_suite,
    load_pgm_image,
    popcount256,
)


def write_hex_stimulus(quantized_img: List[List[int]], hex_path: str) -> None:
    """Packs four 6-bit pixels per 32-bit word into input_pixels.hex."""
    flat_pixels = []
    for row in quantized_img:
        flat_pixels.extend(row)

    with open(hex_path, "w") as f:
        for i in range(0, len(flat_pixels), 4):
            chunk = flat_pixels[i : i + 4]
            word = 0
            for idx, p in enumerate(chunk):
                word |= (p & 0x3F) << (idx * 8)
            f.write(f"{word:08x}\n")


def write_ref_features(ref_features: List[Keypoint], ref_feat_path: str) -> None:
    """Writes reference model features to ref_features.txt."""
    with open(ref_feat_path, "w") as f:
        for kpt in ref_features:
            f.write(
                f"FEAT: level={kpt.level} ori={kpt.angle_sector} "
                f"score={kpt.score} x={kpt.x} y={kpt.y} desc={kpt.descriptor:064x}\n"
            )


def parse_rtl_features(rtl_feat_path: str) -> Tuple[List[Dict[str, str]], bool]:
    """Parses rtl_features.txt and checks for any X/Z bits."""
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
    return rtl_features, has_xz


def compare_features(
    ref_model: ORBHardwareReferenceModel,
    ref_features: List[Keypoint],
    rtl_features: List[Dict[str, str]],
    has_xz: bool,
) -> Tuple[bool, Dict[str, float]]:
    """Compares RTL features against reference model features per pyramid level."""
    if has_xz:
        print("  ERROR: RTL output contains X/Z bits in descriptor")
        return False, {}

    ref_by_level = {0: 0, 1: 0, 2: 0, 3: 0}
    for r in ref_features:
        ref_by_level[r.level] = ref_by_level.get(r.level, 0) + 1

    rtl_by_level = {0: 0, 1: 0, 2: 0, 3: 0}
    for rtl in rtl_features:
        lvl = int(rtl.get("level", 0))
        rtl_by_level[lvl] = rtl_by_level.get(lvl, 0) + 1

    for lvl in range(4):
        w, h = ref_model.level_dims[lvl]
        print(f"    Level {lvl} ({w}x{h}): Ref={ref_by_level[lvl]:4d} | RTL={rtl_by_level[lvl]:4d}")

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
            if ori_diff == 0:
                exact_orientations += 1

            h_dist = popcount256(rtl_desc ^ best_ref.descriptor)
            descriptor_hamming_dist_sum += h_dist

    total_ref = len(ref_features)
    total_rtl = len(rtl_features)
    match_rate = (matched_kpts / total_ref * 100.0) if total_ref else 100.0
    ori_accuracy = (exact_orientations / matched_kpts * 100.0) if matched_kpts else 100.0
    avg_h_dist = (descriptor_hamming_dist_sum / matched_kpts) if matched_kpts else 0.0

    passed = (
        total_rtl == total_ref
        and exact_coord_kpts == total_ref
        and exact_orientations == total_ref
        and descriptor_hamming_dist_sum == 0
    )

    stats = {
        "ref_count": total_ref,
        "rtl_count": total_rtl,
        "exact_coords": exact_coord_kpts,
        "exact_ori": exact_orientations,
        "match_rate": match_rate,
        "ori_accuracy": ori_accuracy,
        "avg_h_dist": avg_h_dist,
    }
    return passed, stats


def compile_vcs(hw_dir: str) -> bool:
    """Compiles the RTL design with Synopsys VCS once before running test images."""
    filelist_path = os.path.join(hw_dir, "filelist.f")
    if not os.path.exists(filelist_path):
        print(f"ERROR: {filelist_path} not found")
        return False

    print("Compiling RTL with Synopsys VCS...")
    vcs_compile_cmd = (
        "source ~/git/hxc/env/init.sh && "
        "module load licenses/synopsys vcs/2025-06 && "
        f"cd {hw_dir} && "
        "vcs -full64 -sverilog +v2k -timescale=1ns/1ps -f filelist.f tb_orb_cosim.sv -o simv_cosim"
    )
    proc = subprocess.run(
        ["bash", "-c", vcs_compile_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )
    if proc.returncode != 0:
        print(proc.stdout)
        print("ERROR: VCS compilation failed")
        return False
    return True


def run_vcs_sim(hw_dir: str) -> bool:
    """Runs the compiled VCS binary simv_cosim on input_pixels.hex."""
    vcs_run_cmd = (
        "source ~/git/hxc/env/init.sh && "
        "module load licenses/synopsys vcs/2025-06 && "
        f"cd {hw_dir} && "
        "./simv_cosim"
    )
    proc = subprocess.run(
        ["bash", "-c", vcs_run_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )
    if proc.returncode != 0:
        print(proc.stdout)
        return False
    return True


def compile_verilator(hw_dir: str) -> bool:
    """Compiles the RTL design with Verilator once before running test images."""
    print("Compiling RTL with Verilator...")
    vlt_cmd = f"cd {hw_dir} && make verilator"
    proc = subprocess.run(
        ["bash", "-c", vlt_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )
    if proc.returncode != 0:
        print(proc.stdout)
        print("ERROR: Verilator compilation failed")
        return False
    return True


def run_verilator_sim(hw_dir: str) -> bool:
    """Runs the compiled Verilator binary Vhw_top on input_pixels.hex."""
    vlt_run_cmd = f"cd {hw_dir} && ./obj_dir_verilator/Vhw_top input_pixels.hex"
    proc = subprocess.run(
        ["bash", "-c", vlt_run_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )
    if proc.returncode != 0:
        print(proc.stdout)
        return False
    return True


def run_verification(hw_dir: str, image_filter: str = "all", sim: str = "vcs") -> bool:
    print("==========================================================================")
    print(f"  FSLAM ORB Accelerator: Multi-Image RTL ({sim.upper()}) vs. Reference Co-Sim")
    print("==========================================================================")

    ref_model = ORBHardwareReferenceModel(
        pixel_depth=6,
        fast_threshold=5,
        pyramid_levels=4,
        img_width=640,
        img_height=480,
    )

    sw_dir = os.path.dirname(os.path.abspath(__file__))
    suite = get_test_image_suite(640, 480, os.path.join(sw_dir, "test_images"))

    if image_filter and image_filter != "all":
        if os.path.exists(image_filter):
            suite = [(os.path.basename(image_filter), "Custom PGM image", load_pgm_image(image_filter, 640, 480))]
        else:
            suite = [item for item in suite if image_filter in item[0]]
            if not suite:
                print(f"ERROR: No test image matching '{image_filter}'")
                return False

    use_verilator = sim.lower() == "verilator"
    if use_verilator:
        if not compile_verilator(hw_dir):
            return False
    else:
        if not compile_vcs(hw_dir):
            return False

    hex_path = os.path.join(hw_dir, "input_pixels.hex")
    ref_feat_path = os.path.join(hw_dir, "ref_features.txt")
    rtl_feat_path = os.path.join(
        hw_dir, "rtl_features_verilator.txt" if use_verilator else "rtl_features.txt"
    )

    summary_rows = []
    all_passed = True

    for idx, (name, desc, test_img) in enumerate(suite, 1):
        print(f"\n[{idx}/{len(suite)}] Test Image: {name} ({desc})")
        quantized_img = ref_model.quantize_image(test_img)
        ref_features = ref_model.extract_features(test_img)

        write_hex_stimulus(quantized_img, hex_path)
        write_ref_features(ref_features, ref_feat_path)

        ok = run_verilator_sim(hw_dir) if use_verilator else run_vcs_sim(hw_dir)
        if not ok:
            print(f"  ERROR: {sim.upper()} simulation failed on {name}")
            all_passed = False
            continue

        rtl_features, has_xz = parse_rtl_features(rtl_feat_path)
        passed, stats = compare_features(ref_model, ref_features, rtl_features, has_xz)
        status_str = "PASS" if passed else "FAIL"
        print(
            f"  -> Result: {status_str} | Features: {stats.get('rtl_count', 0)}/{stats.get('ref_count', 0)} "
            f"| Exact Coords: {stats.get('exact_coords', 0)}/{stats.get('ref_count', 0)} "
            f"| Exact Ori: {stats.get('exact_ori', 0)}/{stats.get('ref_count', 0)} "
            f"| Avg Hamming Dist: {stats.get('avg_h_dist', 0.0):.2f} bits"
        )
        summary_rows.append((name, desc, passed, stats))
        if not passed:
            all_passed = False

    print("\n==========================================================================================")
    print(f"{'Test Image':<26} | {'Ref':>5} | {'RTL':>5} | {'Exact (x,y)':>12} | {'Exact Ori':>12} | {'Hamming':>9} | {'Status':<6}")
    print("------------------------------------------------------------------------------------------")
    total_ref_all = 0
    total_rtl_all = 0
    total_coord_all = 0
    total_ori_all = 0
    for name, _, passed, st in summary_rows:
        r_cnt = int(st.get("ref_count", 0))
        rtl_cnt = int(st.get("rtl_count", 0))
        c_cnt = int(st.get("exact_coords", 0))
        o_cnt = int(st.get("exact_ori", 0))
        h_avg = st.get("avg_h_dist", 0.0)
        total_ref_all += r_cnt
        total_rtl_all += rtl_cnt
        total_coord_all += c_cnt
        total_ori_all += o_cnt
        print(
            f"{name:<26} | {r_cnt:5d} | {rtl_cnt:5d} | {f'{c_cnt}/{r_cnt}':>12} | "
            f"{f'{o_cnt}/{r_cnt}':>12} | {h_avg:6.2f} b  | {'PASS' if passed else 'FAIL':<6}"
        )
    print("------------------------------------------------------------------------------------------")
    print(
        f"{'TOTAL (' + str(len(summary_rows)) + ' images)':<26} | {total_ref_all:5d} | {total_rtl_all:5d} | "
        f"{f'{total_coord_all}/{total_ref_all}':>12} | {f'{total_ori_all}/{total_ref_all}':>12} |   0.00 b  | "
        f"{'PASS' if all_passed else 'FAIL':<6}"
    )
    print("==========================================================================================")
    print("PASS" if all_passed else "FAIL")
    return all_passed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run ORB RTL vs. Reference Model Co-Simulation")
    parser.add_argument("hw_dir", nargs="?", default="/home/vibhakarv/sandbox/FSLAM/hw", help="Path to hw directory")
    parser.add_argument("--image", default="all", help="Test image name (or 'all', or path to a .pgm file)")
    parser.add_argument("--sim", default="vcs", choices=["vcs", "verilator"], help="Simulator to run (vcs or verilator)")
    args = parser.parse_args()

    if not run_verification(args.hw_dir, args.image, args.sim):
        sys.exit(1)
