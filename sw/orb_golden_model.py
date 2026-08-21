#!/usr/bin/env python3
"""Python model of the OpenCV / ORB-SLAM2 ORB extractor (sw/src/ORBextractor.cc)
and comparison against the hardware ORB model across pyramid scaling, FAST-9,
orientation and 256-bit rBRIEF descriptors.
"""

import argparse
import math
from typing import List, Tuple, Dict, Optional

from orb_hardware_reference_model import (
    BINOMIAL_KERNEL_7X7,
    KERNEL_SUM,
    BRESENHAM_CIRCLE_OFFSETS,
    TAN_THRESH_15,
    OPENCV_ORB_PATTERN_256,
    ORBHardwareReferenceModel,
    Keypoint,
    popcount256,
    match_features,
    generate_synthetic_image,
)

# Lookup tables (64 sectors, Q1.7 signed fixed-point)

GOLDEN_COS_LUT_64 = [
    int(round(127.0 * math.cos(2.0 * math.pi * i / 64.0))) for i in range(64)
]

GOLDEN_SIN_LUT_64 = [
    int(round(127.0 * math.sin(2.0 * math.pi * i / 64.0))) for i in range(64)
]

# OpenCV ORB constants from sw/src/ORBextractor.cc
PATCH_SIZE = 31
HALF_PATCH_SIZE = 15
EDGE_THRESHOLD = 19


def cv_round(val: float) -> int:
    """Matches OpenCV cvRound() (round half to even)."""
    return int(round(val))


def build_opencv_umax(half_patch_size: int = HALF_PATCH_SIZE) -> List[int]:
    """Matches umax circular patch precomputation in ORBextractor.cc:454-471."""
    umax = [0] * (half_patch_size + 1)
    vmax = math.floor(half_patch_size * math.sqrt(2.0) / 2.0 + 1)
    vmin = math.ceil(half_patch_size * math.sqrt(2.0) / 2.0)
    hp2 = float(half_patch_size * half_patch_size)
    for v in range(vmax + 1):
        umax[v] = cv_round(math.sqrt(hp2 - v * v))

    v0 = 0
    for v in range(half_patch_size, vmin - 1, -1):
        while umax[v0] == umax[v0 + 1]:
            v0 += 1
        umax[v] = v0
        v0 += 1
    return umax


OPENCV_UMAX_15 = build_opencv_umax(HALF_PATCH_SIZE)


def build_opencv_gaussian_kernel_7x7(sigma: float = 2.0) -> List[List[float]]:
    """Matches cv::GaussianBlur(Size(7, 7), sigmaX=2, sigmaY=2)."""
    k1d = [math.exp(-(x * x) / (2.0 * sigma * sigma)) for x in range(-3, 4)]
    s = sum(k1d)
    k1d = [v / s for v in k1d]
    return [[k1d[r] * k1d[c] for c in range(7)] for r in range(7)]


OPENCV_GAUSSIAN_7X7 = build_opencv_gaussian_kernel_7x7(2.0)


def reflect_101(idx: int, length: int) -> int:
    """Matches OpenCV BORDER_REFLECT_101 (gfedcb|abcdefgh|gfedcba)."""
    if length <= 1:
        return 0
    while idx < 0 or idx >= length:
        if idx < 0:
            idx = -idx
        if idx >= length:
            idx = 2 * length - 2 - idx
    return idx


# Quadtree node and DistributeOctTree (ORBextractor.cc)

class ExtractorNode:
    def __init__(
        self,
        ul_x: int,
        ul_y: int,
        ur_x: int,
        ur_y: int,
        bl_x: int,
        bl_y: int,
        br_x: int,
        br_y: int,
        keys: List[Keypoint],
        no_more: bool = False,
    ):
        self.ul_x = ul_x
        self.ul_y = ul_y
        self.ur_x = ur_x
        self.ur_y = ur_y
        self.bl_x = bl_x
        self.bl_y = bl_y
        self.br_x = br_x
        self.br_y = br_y
        self.keys = keys
        self.no_more = no_more

    def divide_node(self) -> Tuple["ExtractorNode", "ExtractorNode", "ExtractorNode", "ExtractorNode"]:
        """Matches ExtractorNode::DivideNode in ORBextractor.cc:486-541."""
        half_x = math.ceil(float(self.ur_x - self.ul_x) / 2.0)
        half_y = math.ceil(float(self.br_y - self.ul_y) / 2.0)

        n1 = ExtractorNode(
            self.ul_x, self.ul_y,
            self.ul_x + half_x, self.ul_y,
            self.ul_x, self.ul_y + half_y,
            self.ul_x + half_x, self.ul_y + half_y,
            []
        )
        n2 = ExtractorNode(
            n1.ur_x, self.ur_y,
            self.ur_x, self.ur_y,
            n1.br_x, n1.br_y,
            self.ur_x, self.ul_y + half_y,
            []
        )
        n3 = ExtractorNode(
            n1.bl_x, n1.bl_y,
            n1.br_x, n1.br_y,
            self.bl_x, self.bl_y,
            n1.br_x, self.bl_y,
            []
        )
        n4 = ExtractorNode(
            n3.ur_x, n3.ur_y,
            n2.br_x, n2.br_y,
            n3.br_x, n3.br_y,
            self.br_x, self.br_y,
            []
        )

        for kp in self.keys:
            if kp.x < n1.ur_x:
                if kp.y < n1.br_y:
                    n1.keys.append(kp)
                else:
                    n3.keys.append(kp)
            elif kp.y < n1.br_y:
                n2.keys.append(kp)
            else:
                n4.keys.append(kp)

        if len(n1.keys) == 1:
            n1.no_more = True
        if len(n2.keys) == 1:
            n2.no_more = True
        if len(n3.keys) == 1:
            n3.no_more = True
        if len(n4.keys) == 1:
            n4.no_more = True

        return n1, n2, n3, n4


def distribute_oct_tree(
    to_distribute_keys: List[Keypoint],
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
    n_features: int,
) -> List[Keypoint]:
    """Exact port of ORBextractor::DistributeOctTree (ORBextractor.cc:543-767)."""
    if not to_distribute_keys or n_features <= 0:
        return []

    n_ini = max(1, int(round(float(max_x - min_x) / float(max_y - min_y))))
    h_x = float(max_x - min_x) / float(n_ini)

    nodes: List[ExtractorNode] = []
    for i in range(n_ini):
        ul_x = int(h_x * float(i))
        ur_x = int(h_x * float(i + 1))
        nodes.append(
            ExtractorNode(
                ul_x, 0,
                ur_x, 0,
                ul_x, max_y - min_y,
                ur_x, max_y - min_y,
                []
            )
        )

    for kp in to_distribute_keys:
        idx = min(max(int(kp.x / h_x), 0), n_ini - 1)
        nodes[idx].keys.append(kp)

    # Filter empty nodes and mark single-keypoint nodes
    active_nodes: List[ExtractorNode] = []
    for node in nodes:
        if len(node.keys) == 1:
            node.no_more = True
            active_nodes.append(node)
        elif len(node.keys) > 1:
            active_nodes.append(node)
    nodes = active_nodes

    b_finish = False
    size_and_node: List[Tuple[int, ExtractorNode]] = []

    while not b_finish:
        prev_size = len(nodes)
        n_to_expand = 0
        size_and_node.clear()
        new_nodes: List[ExtractorNode] = []

        for node in nodes:
            if node.no_more:
                new_nodes.append(node)
            else:
                n1, n2, n3, n4 = node.divide_node()
                for child in (n1, n2, n3, n4):
                    if len(child.keys) > 0:
                        # ORBextractor.cc uses lNodes.push_front(child)
                        new_nodes.insert(0, child)
                        if len(child.keys) > 1:
                            n_to_expand += 1
                            size_and_node.append((len(child.keys), child))

        nodes = new_nodes

        if len(nodes) >= n_features or len(nodes) == prev_size:
            b_finish = True
        elif (len(nodes) + n_to_expand * 3) > n_features:
            while not b_finish:
                prev_size = len(nodes)
                prev_size_and_node = list(size_and_node)
                size_and_node.clear()

                # Sort ascending by number of keys, then iterate backwards
                prev_size_and_node.sort(key=lambda item: item[0])
                for j in range(len(prev_size_and_node) - 1, -1, -1):
                    _, target_node = prev_size_and_node[j]
                    if target_node not in nodes:
                        continue
                    n1, n2, n3, n4 = target_node.divide_node()
                    for child in (n1, n2, n3, n4):
                        if len(child.keys) > 0:
                            nodes.insert(0, child)
                            if len(child.keys) > 1:
                                size_and_node.append((len(child.keys), child))
                    nodes.remove(target_node)
                    if len(nodes) >= n_features:
                        break

                if len(nodes) >= n_features or len(nodes) == prev_size:
                    b_finish = True

    # Retain the single highest-response keypoint in each leaf node
    result_keys: List[Keypoint] = []
    for node in nodes:
        best_kp = node.keys[0]
        for kp in node.keys[1:]:
            if kp.score > best_kp.score:
                best_kp = kp
        result_keys.append(best_kp)

    return result_keys


# OpenCV ORB reference model (sw/src/ORBextractor.cc)

class OpenCVORBExtractorModel:
    """Pure-Python model of ORB_SLAM2::ORBextractor (sw/src/ORBextractor.cc)."""

    def __init__(
        self,
        nfeatures: int = 1000,
        scale_factor: float = 1.2,
        nlevels: int = 4,
        ini_th_fast: int = 20,
        min_th_fast: int = 7,
        use_octtree: bool = False,
        discretize_angle_128: bool = True,
        fast_score_mode: str = "opencv",  # "opencv" (threshold margin) or "sad16" (hardware sum)
    ):
        self.nfeatures = nfeatures
        self.scale_factor = scale_factor
        self.nlevels = nlevels
        self.ini_th_fast = ini_th_fast
        self.min_th_fast = min_th_fast
        self.use_octtree = use_octtree
        self.discretize_angle_128 = discretize_angle_128
        self.fast_score_mode = fast_score_mode

        # Scale factors and feature distribution per level (ORBextractor.cc:415-446)
        self.scale_factors = [1.0] * nlevels
        for i in range(1, nlevels):
            self.scale_factors[i] = self.scale_factors[i - 1] * scale_factor
        self.inv_scale_factors = [1.0 / s for s in self.scale_factors]

        factor = 1.0 / scale_factor
        desired = nfeatures * (1.0 - factor) / (1.0 - (factor ** nlevels))
        self.features_per_level = [0] * nlevels
        sum_features = 0
        for lvl in range(nlevels - 1):
            self.features_per_level[lvl] = cv_round(desired)
            sum_features += self.features_per_level[lvl]
            desired *= factor
        self.features_per_level[nlevels - 1] = max(nfeatures - sum_features, 0)

    def resize_bilinear_opencv(
        self, src: List[List[int]], dst_w: int, dst_h: int
    ) -> List[List[int]]:
        """Matches cv::resize(..., INTER_LINEAR) pixel center mapping."""
        src_h = len(src)
        src_w = len(src[0])
        scale_x = float(src_w) / float(dst_w)
        scale_y = float(src_h) / float(dst_h)
        dst = [[0] * dst_w for _ in range(dst_h)]

        for dy in range(dst_h):
            fy = (dy + 0.5) * scale_y - 0.5
            sy = math.floor(fy)
            vy = fy - sy
            if sy < 0:
                sy, vy = 0, 0.0
            if sy >= src_h - 1:
                sy, vy = src_h - 2, 1.0

            row0 = src[sy]
            row1 = src[sy + 1]
            for dx in range(dst_w):
                fx = (dx + 0.5) * scale_x - 0.5
                sx = math.floor(fx)
                vx = fx - sx
                if sx < 0:
                    sx, vx = 0, 0.0
                if sx >= src_w - 1:
                    sx, vx = src_w - 2, 1.0

                val = (
                    (1.0 - vx) * (1.0 - vy) * row0[sx]
                    + vx * (1.0 - vy) * row0[sx + 1]
                    + (1.0 - vx) * vy * row1[sx]
                    + vx * vy * row1[sx + 1]
                )
                dst[dy][dx] = cv_round(val)
        return dst

    def compute_pyramid(self, image_8bit: List[List[int]]) -> List[List[List[int]]]:
        """Matches ORBextractor::ComputePyramid (ORBextractor.cc:1111-1136)."""
        h0 = len(image_8bit)
        w0 = len(image_8bit[0])
        pyramid = [image_8bit]
        for lvl in range(1, self.nlevels):
            scale = self.inv_scale_factors[lvl]
            sz_w = cv_round(w0 * scale)
            sz_h = cv_round(h0 * scale)
            pyramid.append(self.resize_bilinear_opencv(pyramid[lvl - 1], sz_w, sz_h))
        return pyramid

    def fast9_corner_score(
        self, img: List[List[int]], x: int, y: int, threshold: int
    ) -> int:
        """Computes FAST-9 corner score at (x, y), or 0 if not a corner."""
        center = img[y][x]
        ring = [img[y + dy][x + dx] for dy, dx in BRESENHAM_CIRCLE_OFFSETS]
        ring_ext = ring + ring[:8]

        is_corner = False
        for start in range(16):
            arc = ring_ext[start : start + 9]
            if all(p > center + threshold for p in arc) or all(
                p < center - threshold for p in arc
            ):
                is_corner = True
                break

        if not is_corner:
            return 0

        if self.fast_score_mode == "sad16":
            return sum(abs(p - center) for p in ring)

        # OpenCV cornerScore<16>: max threshold for which 9 contiguous pixels pass
        best_pos = 0
        best_neg = 0
        for start in range(16):
            arc = ring_ext[start : start + 9]
            min_pos = min(p - center for p in arc)
            min_neg = min(center - p for p in arc)
            if min_pos > best_pos:
                best_pos = min_pos
            if min_neg > best_neg:
                best_neg = min_neg
        return max(best_pos, best_neg) - 1

    def detect_keypoints_raw(
        self, img: List[List[int]], level: int, threshold: Optional[int] = None
    ) -> List[Keypoint]:
        """Runs FAST-9 + 3x3 NMS over the valid interior [EDGE_THRESHOLD..W-EDGE_THRESHOLD)."""
        th = self.ini_th_fast if threshold is None else threshold
        h = len(img)
        w = len(img[0])
        scores = [[0] * w for _ in range(h)]

        min_b = EDGE_THRESHOLD - 3
        max_bx = w - EDGE_THRESHOLD + 3
        max_by = h - EDGE_THRESHOLD + 3

        for y in range(max(3, min_b), min(h - 3, max_by)):
            for x in range(max(3, min_b), min(w - 3, max_bx)):
                scores[y][x] = self.fast9_corner_score(img, x, y, th)

        kpts: List[Keypoint] = []
        for y in range(EDGE_THRESHOLD, h - EDGE_THRESHOLD):
            for x in range(EDGE_THRESHOLD, w - EDGE_THRESHOLD):
                c = scores[y][x]
                if c == 0:
                    continue
                # 3x3 non-maximum suppression
                n00, n01, n02 = scores[y - 1][x - 1], scores[y - 1][x], scores[y - 1][x + 1]
                n10,      n12 = scores[y][x - 1],                       scores[y][x + 1]
                n20, n21, n22 = scores[y + 1][x - 1], scores[y + 1][x], scores[y + 1][x + 1]
                if (
                    c > n00 and c > n01 and c > n02 and
                    c > n10 and             c >= n12 and
                    c >= n20 and c >= n21 and c >= n22
                ):
                    kpts.append(Keypoint(x=x, y=y, score=c, level=level))
        return kpts

    def compute_keypoints_octtree(
        self, img: List[List[int]], level: int
    ) -> List[Keypoint]:
        """Matches ORBextractor::ComputeKeyPointsOctTree (ORBextractor.cc:769-858)."""
        h = len(img)
        w = len(img[0])
        min_bx = EDGE_THRESHOLD - 3
        min_by = min_bx
        max_bx = w - EDGE_THRESHOLD + 3
        max_by = h - EDGE_THRESHOLD + 3

        width = float(max_bx - min_bx)
        height = float(max_by - min_by)
        if width <= 0 or height <= 0:
            return []

        cell_w_target = 30.0
        n_cols = max(1, int(width / cell_w_target))
        n_rows = max(1, int(height / cell_w_target))
        w_cell = int(math.ceil(width / n_cols))
        h_cell = int(math.ceil(height / n_rows))

        to_distribute: List[Keypoint] = []

        for i in range(n_rows):
            ini_y = min_by + i * h_cell
            max_y = min(ini_y + h_cell + 6, max_by)
            if ini_y >= max_by - 3:
                continue

            for j in range(n_cols):
                ini_x = min_bx + j * w_cell
                max_x = min(ini_x + w_cell + 6, max_bx)
                if ini_x >= max_bx - 6:
                    continue

                cell_keys = self._fast_in_cell(img, ini_x, max_x, ini_y, max_y, self.ini_th_fast, level)
                if not cell_keys:
                    cell_keys = self._fast_in_cell(img, ini_x, max_x, ini_y, max_y, self.min_th_fast, level)

                for kp in cell_keys:
                    # Relative to (min_bx, min_by) as in ORBextractor.cc:826-828
                    kp.x = (kp.x - ini_x) + j * w_cell
                    kp.y = (kp.y - ini_y) + i * h_cell
                    to_distribute.append(kp)

        distributed = distribute_oct_tree(
            to_distribute,
            min_bx,
            max_bx,
            min_by,
            max_by,
            self.features_per_level[level],
        )

        for kp in distributed:
            kp.x += min_bx
            kp.y += min_by
            kp.level = level

        return distributed

    def _fast_in_cell(
        self,
        img: List[List[int]],
        ini_x: int,
        max_x: int,
        ini_y: int,
        max_y: int,
        th: int,
        level: int,
    ) -> List[Keypoint]:
        """Runs FAST-9 + 3x3 NMS inside a sub-cell [ini_x..max_x) x [ini_y..max_y)."""
        scores: Dict[Tuple[int, int], int] = {}
        for y in range(ini_y + 3, max_y - 3):
            for x in range(ini_x + 3, max_x - 3):
                s = self.fast9_corner_score(img, x, y, th)
                if s > 0:
                    scores[(x, y)] = s

        cell_kpts: List[Keypoint] = []
        for (x, y), c in scores.items():
            if (
                c > scores.get((x - 1, y - 1), 0)
                and c > scores.get((x, y - 1), 0)
                and c > scores.get((x + 1, y - 1), 0)
                and c > scores.get((x - 1, y), 0)
                and c >= scores.get((x + 1, y), 0)
                and c >= scores.get((x - 1, y + 1), 0)
                and c >= scores.get((x, y + 1), 0)
                and c >= scores.get((x + 1, y + 1), 0)
            ):
                cell_kpts.append(Keypoint(x=x, y=y, score=c, level=level))
        return cell_kpts

    def compute_ic_angle(self, img: List[List[int]], x: int, y: int) -> float:
        """Matches IC_Angle & computeOrientation (ORBextractor.cc:77-104, 473-484)."""
        m_10 = 0
        m_01 = 0

        for u in range(-HALF_PATCH_SIZE, HALF_PATCH_SIZE + 1):
            m_10 += u * img[y][x + u]

        for v in range(1, HALF_PATCH_SIZE + 1):
            v_sum = 0
            d = OPENCV_UMAX_15[v]
            row_plus = img[y + v]
            row_minus = img[y - v]
            for u in range(-d, d + 1):
                val_plus = row_plus[x + u]
                val_minus = row_minus[x + u]
                v_sum += val_plus - val_minus
                m_10 += u * (val_plus + val_minus)
            m_01 += v * v_sum

        angle = math.degrees(math.atan2(float(m_01), float(m_10)))
        if angle < 0.0:
            angle += 360.0

        if self.discretize_angle_128:
            sector = 360.0 / 128.0
            angle = round(angle / sector) * sector
            if angle >= 360.0:
                angle -= 360.0
        return angle

    def gaussian_blur_opencv(self, img: List[List[int]]) -> List[List[int]]:
        """Matches cv::GaussianBlur(workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101)."""
        h = len(img)
        w = len(img[0])
        blurred = [[0] * w for _ in range(h)]
        for y in range(h):
            for x in range(w):
                acc = 0.0
                for ky in range(-3, 4):
                    ry = reflect_101(y + ky, h)
                    row = img[ry]
                    krow = OPENCV_GAUSSIAN_7X7[ky + 3]
                    for kx in range(-3, 4):
                        rx = reflect_101(x + kx, w)
                        acc += row[rx] * krow[kx + 3]
                blurred[y][x] = cv_round(acc)
        return blurred

    def compute_orb_descriptor(
        self, blurred_img: List[List[int]], x: int, y: int, angle_deg: float
    ) -> int:
        """Matches computeOrbDescriptor (ORBextractor.cc:108-145)."""
        rad = angle_deg * (math.pi / 180.0)
        a = math.cos(rad)
        b = math.sin(rad)

        desc = 0
        for bit_idx, (x1, y1, x2, y2) in enumerate(OPENCV_ORB_PATTERN_256):
            r1 = y + cv_round(x1 * b + y1 * a)
            c1 = x + cv_round(x1 * a - y1 * b)
            r2 = y + cv_round(x2 * b + y2 * a)
            c2 = x + cv_round(x2 * a - y2 * b)

            t0 = blurred_img[r1][c1]
            t1 = blurred_img[r2][c2]
            if t0 < t1:
                desc |= (1 << bit_idx)
        return desc

    def extract_features(self, image_8bit: List[List[int]]) -> List[Keypoint]:
        """Runs the full OpenCV C++ ORBextractor pipeline."""
        pyramid = self.compute_pyramid(image_8bit)
        all_kpts: List[Keypoint] = []

        for lvl in range(self.nlevels):
            img = pyramid[lvl]
            if self.use_octtree:
                kpts = self.compute_keypoints_octtree(img, lvl)
            else:
                kpts = self.detect_keypoints_raw(img, lvl)

            blurred = self.gaussian_blur_opencv(img)
            for kp in kpts:
                angle_deg = self.compute_ic_angle(img, kp.x, kp.y)
                kp.angle_sector = int(round(angle_deg / 5.625)) % 64
                kp.descriptor = self.compute_orb_descriptor(blurred, kp.x, kp.y, angle_deg)
                all_kpts.append(kp)

        return all_kpts


# Hardware ORB model

class ORBGoldenHardwareModel:
    """Bit-accurate hardware ORB model for the FSLAM accelerator."""

    def __init__(
        self,
        fast_threshold: int = 20,
        pyramid_levels: int = 4,
        img_width: int = 640,
        img_height: int = 480,
        fast_on_8bit: bool = True,          # Thesis Sec 5.2: 8-bit FAST, 6-bit post-FAST
        circular_orientation: bool = False, # False = 37x37 square (RTL), True = 31x31 circle
        brief_less_than: bool = True,       # True = (val1 < val2) matching OpenCV C++
        round_rotation: bool = False,       # False = arithmetic >> 7 (RTL), True = rounded >> 7
    ):
        self.fast_threshold = fast_threshold
        self.pyramid_levels = pyramid_levels
        self.img_width = img_width
        self.img_height = img_height
        self.fast_on_8bit = fast_on_8bit
        self.circular_orientation = circular_orientation
        self.brief_less_than = brief_less_than
        self.round_rotation = round_rotation

        self.level_dims = [(img_width, img_height)]
        for _ in range(1, pyramid_levels):
            prev_w, prev_h = self.level_dims[-1]
            self.level_dims.append(((prev_w * 5) // 6, (prev_h * 5) // 6))

    @staticmethod
    def quantize_to_6bit(img_8bit: List[List[int]]) -> List[List[int]]:
        """Truncates 8-bit [0..255] pixels to 6-bit [0..63] (>> 2)."""
        return [[min(max(p >> 2, 0), 63) for p in row] for row in img_8bit]

    def resize_5_6_fixed_point(
        self, src_img: List[List[int]], src_w: int, src_h: int
    ) -> Tuple[List[List[int]], int, int]:
        """5/6 scale-down using 3-bit fixed-point bilinear weights (>> 6)."""
        dst_w = (src_w * 5) // 6
        dst_h = (src_h * 5) // 6
        dst_img = [[0] * dst_w for _ in range(dst_h)]
        w_lut = [0, 2, 3, 5, 6, 8]

        for dy in range(dst_h):
            sy_num = dy * 6
            sy = sy_num // 5
            rem_y = sy_num % 5
            wy1 = w_lut[rem_y]
            wy0 = 8 - wy1
            sy1 = min(sy + 1, src_h - 1)

            for dx in range(dst_w):
                sx_num = dx * 6
                sx = sx_num // 5
                rem_x = sx_num % 5
                wx1 = w_lut[rem_x]
                wx0 = 8 - wx1
                sx1 = min(sx + 1, src_w - 1)

                p00 = src_img[sy][sx]
                p01 = src_img[sy][sx1]
                p10 = src_img[sy1][sx]
                p11 = src_img[sy1][sx1]

                interp = (
                    (wx0 * wy0 * p00)
                    + (wx1 * wy0 * p01)
                    + (wx0 * wy1 * p10)
                    + (wx1 * wy1 * p11)
                ) >> 6
                dst_img[dy][dx] = interp

        return dst_img, dst_w, dst_h

    def build_pyramid(self, img: List[List[int]]) -> List[List[List[int]]]:
        """Builds 4-level 5/6 pyramid."""
        pyr = [img]
        cur = img
        for lvl in range(1, self.pyramid_levels):
            w, h = self.level_dims[lvl - 1]
            cur, _, _ = self.resize_5_6_fixed_point(cur, w, h)
            pyr.append(cur)
        return pyr

    def gaussian_blur_7x7_binomial(
        self, img: List[List[int]], width: int, height: int
    ) -> List[List[int]]:
        """7x7 binomial filter [1, 6, 15, 20, 15, 6, 1] (sum = 4096, >> 12)."""
        blurred = [[0] * width for _ in range(height)]
        for y in range(3, height - 3):
            for x in range(3, width - 3):
                acc = 0
                for ky in range(7):
                    row = img[y + ky - 3]
                    krow = BINOMIAL_KERNEL_7X7[ky]
                    for kx in range(7):
                        acc += row[x + kx - 3] * krow[kx]
                blurred[y][x] = acc >> 12
        return blurred

    def detect_fast_and_nms(
        self, img_for_fast: List[List[int]], width: int, height: int, level: int
    ) -> List[Keypoint]:
        """Streaming FAST-9 + 3x3 NMS with safe 37x37 orientation/BRIEF border."""
        # Threshold scaled automatically if running on 6-bit pixels
        th = self.fast_threshold if self.fast_on_8bit else max(1, self.fast_threshold >> 2)
        scores = [[0] * width for _ in range(height)]

        # Safe border: blurred image is valid on [3, W-4] x [3, H-4], and the
        # 37x37 patch (radius 18) requires [21, W-22] x [21, H-22].
        margin = 21
        for y in range(margin - 1, height - margin + 1):
            for x in range(margin - 1, width - margin + 1):
                center = img_for_fast[y][x]
                ring = [img_for_fast[y + dy][x + dx] for dy, dx in BRESENHAM_CIRCLE_OFFSETS]
                ring_ext = ring + ring[:8]

                is_corner = False
                for start in range(16):
                    arc = ring_ext[start : start + 9]
                    if all(p > center + th for p in arc) or all(p < center - th for p in arc):
                        is_corner = True
                        break

                if is_corner:
                    scores[y][x] = sum(abs(p - center) for p in ring)

        keypoints: List[Keypoint] = []
        for y in range(margin, height - margin):
            for x in range(margin, width - margin):
                c = scores[y][x]
                if c == 0:
                    continue
                n00, n01, n02 = scores[y - 1][x - 1], scores[y - 1][x], scores[y - 1][x + 1]
                n10,      n12 = scores[y][x - 1],                       scores[y][x + 1]
                n20, n21, n22 = scores[y + 1][x - 1], scores[y + 1][x], scores[y + 1][x + 1]
                if (
                    c > n00 and c > n01 and c > n02 and
                    c > n10 and             c >= n12 and
                    c >= n20 and c >= n21 and c >= n22
                ):
                    keypoints.append(Keypoint(x=x, y=y, score=c, level=level))

        return keypoints

    def compute_orientation_golden(
        self, blurred_img: List[List[int]], kpt: Keypoint
    ) -> int:
        """Computes 64-sector intensity-centroid orientation."""
        m10 = 0
        m01 = 0

        if self.circular_orientation:
            for dy in range(-HALF_PATCH_SIZE, HALF_PATCH_SIZE + 1):
                d = OPENCV_UMAX_15[abs(dy)]
                row = blurred_img[kpt.y + dy]
                for dx in range(-d, d + 1):
                    pix = row[kpt.x + dx]
                    m10 += dx * pix
                    m01 += dy * pix
        else:
            for dy in range(-18, 19):
                row = blurred_img[kpt.y + dy]
                for dx in range(-18, 19):
                    pix = row[kpt.x + dx]
                    m10 += dx * pix
                    m01 += dy * pix

        abs_m10 = abs(m10)
        abs_m01 = abs(m01)

        # Base sector in [0..15] measuring angle from horizontal (+/- X) axis
        sector_in_quad = 15
        for s in range(15):
            if (abs_m10 * TAN_THRESH_15[s]) >= (abs_m01 << 8):
                sector_in_quad = s
                break

        if m10 >= 0 and m01 >= 0:
            # Quadrant 0: [0, 90) deg -> sectors 0..15
            return sector_in_quad
        elif m10 < 0 and m01 >= 0:
            # Quadrant 1: [90, 180) deg -> sectors 16..31
            return 16 + (15 - sector_in_quad)
        elif m10 < 0 and m01 < 0:
            # Quadrant 2: [180, 270) deg -> sectors 32..47
            return 32 + sector_in_quad
        else:
            # Quadrant 3: [270, 360) deg -> sectors 48..63
            return 48 + (15 - sector_in_quad)

    def generate_brief_descriptor_golden(
        self, blurred_img: List[List[int]], kpt: Keypoint
    ) -> int:
        """Computes 256-bit steered rBRIEF using Q1.7 sin/cos tables."""
        sector = kpt.angle_sector & 0x3F
        cos_val = GOLDEN_COS_LUT_64[sector]
        sin_val = GOLDEN_SIN_LUT_64[sector]

        descriptor = 0
        for bit_idx, (x1, y1, x2, y2) in enumerate(OPENCV_ORB_PATTERN_256):
            if self.round_rotation:
                rot_x1 = (x1 * cos_val - y1 * sin_val + 64) >> 7
                rot_y1 = (x1 * sin_val + y1 * cos_val + 64) >> 7
                rot_x2 = (x2 * cos_val - y2 * sin_val + 64) >> 7
                rot_y2 = (x2 * sin_val + y2 * cos_val + 64) >> 7
            else:
                rot_x1 = (x1 * cos_val - y1 * sin_val) >> 7
                rot_y1 = (x1 * sin_val + y1 * cos_val) >> 7
                rot_x2 = (x2 * cos_val - y2 * sin_val) >> 7
                rot_y2 = (x2 * sin_val + y2 * cos_val) >> 7

            val1 = blurred_img[kpt.y + rot_y1][kpt.x + rot_x1]
            val2 = blurred_img[kpt.y + rot_y2][kpt.x + rot_x2]

            bit = (val1 < val2) if self.brief_less_than else (val1 > val2)
            if bit:
                descriptor |= (1 << bit_idx)

        return descriptor

    def extract_features(self, raw_image_8bit: List[List[int]]) -> List[Keypoint]:
        """Runs the hardware ORB pipeline."""
        pyr_8bit = self.build_pyramid(raw_image_8bit)
        pyr_6bit = [self.quantize_to_6bit(lvl_img) for lvl_img in pyr_8bit]

        all_keypoints: List[Keypoint] = []
        for lvl in range(self.pyramid_levels):
            w, h = self.level_dims[lvl]
            fast_input = pyr_8bit[lvl] if self.fast_on_8bit else pyr_6bit[lvl]
            kpts = self.detect_fast_and_nms(fast_input, w, h, lvl)
            blurred_6bit = self.gaussian_blur_7x7_binomial(pyr_6bit[lvl], w, h)

            for kp in kpts:
                kp.angle_sector = self.compute_orientation_golden(blurred_6bit, kp)
                kp.descriptor = self.generate_brief_descriptor_golden(blurred_6bit, kp)
                all_keypoints.append(kp)

        return all_keypoints


# Test image and OpenCV comparison

def generate_rich_test_image(width: int = 320, height: int = 240) -> List[List[int]]:
    """Generates a multi-contrast test image with features across all 4 quadrants."""
    img = [[40] * width for _ in range(height)]

    # Varying-intensity blocks (including medium-contrast 8-bit edges in [30..75] diff)
    intensities = [220, 180, 130, 95, 75]
    idx = 0
    for sq_y in range(30, height - 50, 45):
        for sq_x in range(30, width - 50, 45):
            val = intensities[idx % len(intensities)]
            idx += 1
            for y in range(sq_y, min(sq_y + 22, height - 25)):
                for x in range(sq_x, min(sq_x + 22, width - 25)):
                    img[y][x] = val

    # Add asymmetric L-shapes and triangles so orientations span Q0, Q1, Q2, Q3
    for cy in range(45, height - 45, 55):
        for cx in range(45, width - 45, 65):
            for dy in range(-8, 9):
                for dx in range(-8, 9):
                    if dx + dy > 2:
                        img[cy + dy][cx + dx] = min(255, img[cy + dy][cx + dx] + 60)

    return img


def angular_diff_deg(a_deg: float, b_deg: float) -> float:
    """Smallest absolute difference between two angles in [0, 360)."""
    d = abs(a_deg - b_deg) % 360.0
    return 360.0 - d if d > 180.0 else d


def run_correlation_report(width: int = 320, height: int = 240) -> None:
    print(f"Hardware model vs. OpenCV ORB ({width}x{height})")

    img = generate_rich_test_image(width, height)

    opencv_model = OpenCVORBExtractorModel(
        nfeatures=500, scale_factor=1.2, nlevels=4, ini_th_fast=20, min_th_fast=7, use_octtree=False
    )
    opencv_octtree_model = OpenCVORBExtractorModel(
        nfeatures=200, scale_factor=1.2, nlevels=4, ini_th_fast=20, min_th_fast=7, use_octtree=True
    )
    hw_6bit_model = ORBHardwareReferenceModel(
        pixel_depth=6, fast_threshold=5, pyramid_levels=4, img_width=width, img_height=height
    )
    golden_hw_model = ORBGoldenHardwareModel(
        fast_threshold=20, pyramid_levels=4, img_width=width, img_height=height,
        fast_on_8bit=True, circular_orientation=False, brief_less_than=True
    )

    # Scale pyramid
    cv_pyr = opencv_model.compute_pyramid(img)
    hw_pyr_8bit = golden_hw_model.build_pyramid(img)

    print("\nScale pyramid (OpenCV INTER_LINEAR vs. HW 5/6 fixed-point):")
    for lvl in range(4):
        cv_lvl = cv_pyr[lvl]
        hw_lvl = hw_pyr_8bit[lvl]
        h_min = min(len(cv_lvl), len(hw_lvl))
        w_min = min(len(cv_lvl[0]), len(hw_lvl[0]))
        diffs = [
            abs(cv_lvl[y][x] - hw_lvl[y][x])
            for y in range(h_min)
            for x in range(w_min)
        ]
        mae = sum(diffs) / len(diffs)
        max_err = max(diffs)
        print(
            f"  Level {lvl}: OpenCV={len(cv_lvl[0])}x{len(cv_lvl)}, "
            f"HW={len(hw_lvl[0])}x{len(hw_lvl)} | Mean Abs Diff={mae:.2f} LSB, Max Diff={max_err} LSB"
        )

    # Level-0 FAST-9 + NMS keypoints
    # Compare pre-NMS FAST-9 corner masks and post-NMS keypoints
    cv_pre_nms = {
        (x, y)
        for y in range(21, height - 21)
        for x in range(21, width - 21)
        if opencv_model.fast9_corner_score(img, x, y, 20) > 0
    }
    hw6_q = hw_6bit_model.quantize_image(img)
    hw6_scores = hw_6bit_model.detect_fast_corners(hw6_q, width, height, 0)
    hw6_pre_nms = {
        (x, y)
        for y in range(21, height - 21)
        for x in range(21, width - 21)
        if hw6_scores[y][x] > 0
    }
    golden_pre_nms = {
        (x, y)
        for y in range(21, height - 21)
        for x in range(21, width - 21)
        if opencv_model.fast9_corner_score(img, x, y, 20) > 0
    }

    cv_kpts_l0 = [
        kp for kp in opencv_model.detect_keypoints_raw(img, 0)
        if 21 <= kp.x < width - 21 and 21 <= kp.y < height - 21
    ]
    cv_oct_kpts_l0 = [
        kp for kp in opencv_octtree_model.compute_keypoints_octtree(img, 0)
        if 21 <= kp.x < width - 21 and 21 <= kp.y < height - 21
    ]
    hw6_kpts_l0 = [
        kp for kp in hw_6bit_model.non_maximum_suppression(hw6_scores, width, height, 0)
        if 21 <= kp.x < width - 21 and 21 <= kp.y < height - 21
    ]
    golden_kpts_l0 = golden_hw_model.detect_fast_and_nms(img, width, height, 0)

    cv_coords = {(kp.x, kp.y) for kp in cv_kpts_l0}
    oct_coords = {(kp.x, kp.y) for kp in cv_oct_kpts_l0}
    hw6_coords = {(kp.x, kp.y) for kp in hw6_kpts_l0}
    golden_coords = {(kp.x, kp.y) for kp in golden_kpts_l0}

    def within_radius(pt: Tuple[int, int], ref_set: set, r: int = 2) -> bool:
        px, py = pt
        return any((px + dx, py + dy) in ref_set for dy in range(-r, r + 1) for dx in range(-r, r + 1))

    golden_near_cv = sum(1 for pt in cv_coords if within_radius(pt, golden_coords, r=2))
    hw6_near_cv = sum(1 for pt in cv_coords if within_radius(pt, hw6_coords, r=2))

    print("\nLevel-0 keypoint detection:")
    print(
        f"  Pre-NMS FAST-9 pixels (OpenCV): {len(cv_pre_nms)} | "
        f"6-bit HW: {len(hw6_pre_nms & cv_pre_nms)}/{len(cv_pre_nms)} "
        f"({100.0 * len(hw6_pre_nms & cv_pre_nms) / max(1, len(cv_pre_nms)):.1f}%) | "
        f"8-bit HW: {len(golden_pre_nms & cv_pre_nms)}/{len(cv_pre_nms)} "
        f"({100.0 * len(golden_pre_nms & cv_pre_nms) / max(1, len(cv_pre_nms)):.1f}%)"
    )
    print(f"  Post-NMS OpenCV raw           : {len(cv_coords)} keypoints")
    print(f"  Post-NMS OpenCV DistributeOctTree: {len(oct_coords)} keypoints")
    print(
        f"  Post-NMS 6-bit HW (SAD-16)    : {len(hw6_coords)} keypoints | "
        f"Exact (x,y): {len(hw6_coords & cv_coords)}/{len(cv_coords)}, "
        f"<=2px: {hw6_near_cv}/{len(cv_coords)} ({100.0 * hw6_near_cv / max(1, len(cv_coords)):.1f}%)"
    )
    print(
        f"  Post-NMS 8-bit HW (SAD-16)    : {len(golden_coords)} keypoints | "
        f"Exact (x,y): {len(golden_coords & cv_coords)}/{len(cv_coords)}, "
        f"<=2px: {golden_near_cv}/{len(cv_coords)} ({100.0 * golden_near_cv / max(1, len(cv_coords)):.1f}%)"
    )

    # Orientation across 4 quadrants
    print("\nOrientation (64-sector LUT vs. atan2):")
    quad_errors_golden = {0: [], 1: [], 2: [], 3: []}

    for deg in range(0, 360, 2):
        rad = math.radians(deg)
        m10 = int(round(10000 * math.cos(rad)))
        m01 = int(round(10000 * math.sin(rad)))
        q = 0 if (m10 >= 0 and m01 >= 0) else (1 if (m10 < 0 and m01 >= 0) else (2 if (m10 < 0 and m01 < 0) else 3))

        abs_m10, abs_m01 = abs(m10), abs(m01)
        sec_in_q = 15
        for i, th in enumerate(TAN_THRESH_15):
            if (abs_m01 << 8) < abs_m10 * th:
                sec_in_q = i
                break

        gld_sec = (q << 4) | ((15 - sec_in_q) if q in (1, 3) else sec_in_q)
        quad_errors_golden[q].append(angular_diff_deg(deg, gld_sec * 5.625))

    for q in range(4):
        gld_max = max(quad_errors_golden[q])
        gld_mean = sum(quad_errors_golden[q]) / len(quad_errors_golden[q])
        print(
            f"  Quadrant {q} ([{q*90:3d}, {(q+1)*90:3d}) deg): "
            f"Mean Error = {gld_mean:4.2f} deg, Max Error = {gld_max:4.2f} deg"
        )

    # 256-bit steered rBRIEF descriptors
    print("\n256-bit rBRIEF descriptors vs. OpenCV:")
    cv_blurred_l0 = opencv_model.gaussian_blur_opencv(img)
    hw_pyr6_l0 = golden_hw_model.quantize_to_6bit(img)
    hw_blurred_l0 = golden_hw_model.gaussian_blur_7x7_binomial(hw_pyr6_l0, width, height)

    shared_pts = sorted(cv_coords & golden_coords)
    hamming_golden_same_angle = []
    hamming_golden_end_to_end = []

    for x, y in shared_pts:
        cv_angle = opencv_model.compute_ic_angle(img, x, y)
        cv_desc = opencv_model.compute_orb_descriptor(cv_blurred_l0, x, y, cv_angle)
        cv_sector = int(round(cv_angle / 5.625)) % 64

        kp_gld_same = Keypoint(x=x, y=y, score=1, level=0, angle_sector=cv_sector)
        gld_desc_same = golden_hw_model.generate_brief_descriptor_golden(hw_blurred_l0, kp_gld_same)
        hamming_golden_same_angle.append(popcount256(cv_desc ^ gld_desc_same))

        kp_gld_e2e = Keypoint(x=x, y=y, score=1, level=0)
        kp_gld_e2e.angle_sector = golden_hw_model.compute_orientation_golden(hw_blurred_l0, kp_gld_e2e)
        gld_desc_e2e = golden_hw_model.generate_brief_descriptor_golden(hw_blurred_l0, kp_gld_e2e)
        hamming_golden_end_to_end.append(popcount256(cv_desc ^ gld_desc_e2e))

    if shared_pts:
        mean_same = sum(hamming_golden_same_angle) / len(hamming_golden_same_angle)
        mean_e2e = sum(hamming_golden_end_to_end) / len(hamming_golden_end_to_end)
        match_rate_same = 100.0 * sum(1 for d in hamming_golden_same_angle if d <= 50) / len(shared_pts)
        match_rate_e2e = 100.0 * sum(1 for d in hamming_golden_end_to_end if d <= 50) / len(shared_pts)

        print(f"  Evaluated across {len(shared_pts)} shared Level-0 keypoints:")
        print(
            f"  - Hardware model (same sector) vs. OpenCV : "
            f"Mean Hamming = {mean_same:6.1f} / 256 bits | Match rate (<=50) = {match_rate_same:5.1f}%"
        )
        print(
            f"  - Hardware model (end-to-end)  vs. OpenCV : "
            f"Mean Hamming = {mean_e2e:6.1f} / 256 bits | Match rate (<=50) = {match_rate_e2e:5.1f}%"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FSLAM Golden ORB Reference & OpenCV C++ Correlation")
    parser.add_argument("--width", type=int, default=320, help="Test image width")
    parser.add_argument("--height", type=int, default=240, help="Test image height")
    args = parser.parse_args()
    run_correlation_report(width=args.width, height=args.height)
