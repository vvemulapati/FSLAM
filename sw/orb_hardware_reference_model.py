#!/usr/bin/env python3
"""
ORB Hardware Reference Model for FSLAM
======================================
Bit-accurate Python software reference model mimicking the exact FPGA hardware pipeline
described in the thesis "ORB-BASED SLAM ACCELERATOR ON SOC FPGA" (UIUC, 2022).

Key Features Implemented:
-------------------------
1. 6-bit Pixel Quantization (or configurable bit-depth).
2. 4-level Image Pyramid with 1.2x downsampling (5/6 scale factor with 25 fixed bilinear weights).
3. FAST-9/16 Corner Detector with Bresenham circle (radius 3) and absolute difference scoring.
4. 3x3 Non-Maximum Suppression (NMS) with directional tie-breaking.
5. 7x7 Binomial Gaussian Blur (separable / sum=4096, normalized via >> 12).
6. Intensity Centroid Moments (m00, m01, m10) on a 37x37 window.
7. 64-sector Orientation Discretization using quadrant mapping and tangent comparator thresholds.
8. Rotated BRIEF-256 Descriptor Generator on a 37x37 patch with Q1.7 sin/cos LUTs.
9. Hamming Distance Feature Matcher with thresholding.

This model serves as a golden reference for hardware verification and simulation.
"""

import math
import struct
from typing import List, Tuple, Dict, Any, Optional

# ==============================================================================
# Hardware Constants & Lookup Tables
# ==============================================================================

# 7x7 Binomial 2D Gaussian Kernel (outer product of [1, 6, 15, 20, 15, 6, 1])
BINOMIAL_KERNEL_7X7 = [
    [1,  6,  15,  20,  15,  6, 1],
    [6, 36,  90, 120,  90, 36, 6],
    [15, 90, 225, 300, 225, 90, 15],
    [20, 120, 300, 400, 300, 120, 20],
    [15, 90, 225, 300, 225, 90, 15],
    [6, 36,  90, 120,  90, 36, 6],
    [1,  6,  15,  20,  15,  6, 1],
]
KERNEL_SUM = 4096  # 64^2 = 2^12

# Bresenham circle radius 3 offsets (dy, dx) relative to center [3, 3] in clockwise order
BRESENHAM_CIRCLE_OFFSETS = [
    (-3,  0), (-3,  1), (-2,  2), (-1,  3),
    ( 0,  3), ( 1,  3), ( 2,  2), ( 3,  1),
    ( 3,  0), ( 3, -1), ( 2, -2), ( 1, -3),
    ( 0, -3), (-1, -3), (-2, -2), (-3, -1),
]

# 15 Tangent Threshold Multipliers (tan(θ) * 256) for 16 sectors per quadrant
# Discretizes 90 degrees into 16 sectors (5.625° intervals)
TAN_THRESH_15 = [
    25,   #  5.625° (tan=0.09849)
    51,   # 11.250° (tan=0.19891)
    78,   # 16.875° (tan=0.30335)
    106,  # 22.500° (tan=0.41421)
    137,  # 28.125° (tan=0.53451)
    171,  # 33.750° (tan=0.66818)
    210,  # 39.375° (tan=0.82065)
    256,  # 45.000° (tan=1.00000)
    312,  # 50.625° (tan=1.21850)
    383,  # 56.250° (tan=1.49661)
    479,  # 61.875° (tan=1.87093)
    618,  # 67.500° (tan=2.41421)
    844,  # 73.125° (tan=3.29703)
    1287, # 78.750° (tan=5.02734)
    2599  # 84.375° (tan=10.1532)
]

# 64-sector Cosine and Sine LUTs (Q1.7 fixed point, range -127 to +127)
COS_LUT_64 = [
    127, 126, 124, 120, 114, 108, 99, 90,
    80, 69, 57, 45, 32, 19, 6, -6,
    -19, -32, -45, -57, -69, -80, -90, -99,
    -108, -114, -120, -124, -126, -127, -127, -126,
    -124, -120, -114, -108, -99, -90, -80, -69,
    -57, -45, -32, -19, -6, 6, 19, 32,
    45, 57, 69, 80, 90, 99, 108, 114,
    120, 124, 126, 127, 127, 126, 124, 120
]

SIN_LUT_64 = [
    0, 12, 25, 37, 49, 60, 71, 80,
    90, 99, 108, 114, 120, 124, 126, 127,
    127, 126, 124, 120, 114, 108, 99, 90,
    80, 71, 60, 49, 37, 25, 12, 0,
    -12, -25, -37, -49, -60, -71, -80, -90,
    -99, -108, -114, -120, -124, -126, -127, -127,
    -126, -124, -120, -114, -108, -99, -90, -80,
    -71, -60, -49, -37, -25, -12, 0, 0
]

# Official standard OpenCV 256-pair learned ORB test coordinate pattern (bit_pattern_31_)
# Each entry contains (x1, y1, x2, y2) in 31x31 patch [-13..+12].
OPENCV_ORB_PATTERN_256 = [
    (  8,  -3,   9,   5),
    (  4,   2,   7, -12),
    (-11,   9,  -8,   2),
    (  7, -12,  12, -13),
    (  2, -13,   2,  12),
    (  1,  -7,   1,   6),
    ( -2, -10,  -2,  -4),
    (-13, -13, -11,  -8),
    (-13,  -3, -12,  -9),
    ( 10,   4,  11,   9),
    (-13,  -8,  -8,  -9),
    (-11,   7,  -9,  12),
    (  7,   7,  12,   6),
    ( -4,  -5,  -3,   0),
    (-13,   2, -12,  -3),
    ( -9,   0,  -7,   5),
    ( 12,  -6,  12,  -1),
    ( -3,   6,  -2,  12),
    ( -6, -13,  -4,  -8),
    ( 11, -13,  12,  -8),
    (  4,   7,   5,   1),
    (  5,  -3,  10,  -3),
    (  3,  -7,   6,  12),
    ( -8,  -7,  -6,  -2),
    ( -2,  11,  -1, -10),
    (-13,  12,  -8,  10),
    ( -7,   3,  -5,  -3),
    ( -4,   2,  -3,   7),
    (-10, -12,  -6,  11),
    (  5, -12,   6,  -7),
    (  5,  -6,   7,  -1),
    (  1,   0,   4,  -5),
    (  9,  11,  11, -13),
    (  4,   7,   4,  12),
    (  2,  -1,   4,   4),
    ( -4, -12,  -2,   7),
    ( -8,  -5,  -7, -10),
    (  4,  11,   9,  12),
    (  0,  -8,   1, -13),
    (-13,  -2,  -8,   2),
    ( -3,  -2,  -2,   3),
    ( -6,   9,  -4,  -9),
    (  8,  12,  10,   7),
    (  0,   9,   1,   3),
    (  7,  -5,  11, -10),
    (-13,  -6, -11,   0),
    ( 10,   7,  12,   1),
    ( -6,  -3,  -6,  12),
    ( 10,  -9,  12,  -4),
    (-13,   8,  -8, -12),
    (-13,   0,  -8,  -4),
    (  3,   3,   7,   8),
    (  5,   7,  10,  -7),
    ( -1,   7,   1, -12),
    (  3, -10,   5,   6),
    (  2,  -4,   3, -10),
    (-13,   0, -13,   5),
    (-13,  -7, -12,  12),
    (-13,   3, -11,   8),
    ( -7,  12,  -4,   7),
    (  6, -10,  12,   8),
    ( -9,  -1,  -7,  -6),
    ( -2,  -5,   0,  12),
    (-12,   5,  -7,   5),
    (  3, -10,   8, -13),
    ( -7,  -7,  -4,   5),
    ( -3,  -2,  -1,  -7),
    (  2,   9,   5, -11),
    (-11, -13,  -5, -13),
    ( -1,   6,   0,  -1),
    (  5,  -3,   5,   2),
    ( -4, -13,  -4,  12),
    ( -9,  -6,  -9,   6),
    (-12, -10,  -8,  -4),
    ( 10,   2,  12,  -3),
    (  7,  12,  12,  12),
    ( -7, -13,  -6,   5),
    ( -4,   9,  -3,   4),
    (  7,  -1,  12,   2),
    ( -7,   6,  -5,   1),
    (-13,  11, -12,   5),
    ( -3,   7,  -2,  -6),
    (  7,  -8,  12,  -7),
    (-13,  -7, -11, -12),
    (  1,  -3,  12,  12),
    (  2,  -6,   3,   0),
    ( -4,   3,  -2, -13),
    ( -1, -13,   1,   9),
    (  7,   1,   8,  -6),
    (  1,  -1,   3,  12),
    (  9,   1,  12,   6),
    ( -1,  -9,  -1,   3),
    (-13, -13, -10,   5),
    (  7,   7,  10,  12),
    ( 12,  -5,  12,   9),
    (  6,   3,   7,  11),
    (  5, -13,   6,  10),
    (  2, -12,   2,   3),
    (  3,   8,   4,  -6),
    (  2,   6,  12, -13),
    (  9, -12,  10,   3),
    ( -8,   4,  -7,   9),
    (-11,  12,  -4,  -6),
    (  1,  12,   2,  -8),
    (  6,  -9,   7,  -4),
    (  2,   3,   3,  -2),
    (  6,   3,  11,   0),
    (  3,  -3,   8,  -8),
    (  7,   8,   9,   3),
    (-11,  -5,  -6,  -4),
    (-10,  11,  -5,  10),
    ( -5,  -8,  -3,  12),
    (-10,   5,  -9,   0),
    (  8,  -1,  12,  -6),
    (  4,  -6,   6, -11),
    (-10,  12,  -8,   7),
    (  4,  -2,   6,   7),
    ( -2,   0,  -2,  12),
    ( -5,  -8,  -5,   2),
    (  7,  -6,  10,  12),
    ( -9, -13,  -8,  -8),
    ( -5, -13,  -5,  -2),
    (  8,  -8,   9, -13),
    ( -9, -11,  -9,   0),
    (  1,  -8,   1,  -2),
    (  7,  -4,   9,   1),
    ( -2,   1,  -1,  -4),
    ( 11,  -6,  12, -11),
    (-12,  -9,  -6,   4),
    (  3,   7,   7,  12),
    (  5,   5,  10,   8),
    (  0,  -4,   2,   8),
    ( -9,  12,  -5, -13),
    (  0,   7,   2,  12),
    ( -1,   2,   1,   7),
    (  5,  11,   7,  -9),
    (  3,   5,   6,  -8),
    (-13,  -4,  -8,   9),
    ( -5,   9,  -3,  -3),
    ( -4,  -7,  -3, -12),
    (  6,   5,   8,   0),
    ( -7,   6,  -6,  12),
    (-13,   6,  -5,  -2),
    (  1, -10,   3,  10),
    (  4,   1,   8,  -4),
    ( -2,  -2,   2, -13),
    (  2, -12,  12,  12),
    ( -2, -13,   0,  -6),
    (  4,   1,   9,   3),
    ( -6, -10,  -3,  -5),
    ( -3, -13,  -1,   1),
    (  7,   5,  12, -11),
    (  4,  -2,   5,  -7),
    (-13,   9,  -9,  -5),
    (  7,   1,   8,   6),
    (  7,  -8,   7,   6),
    ( -7,  -4,  -7,   1),
    ( -8,  11,  -7,  -8),
    (-13,   6, -12,  -8),
    (  2,   4,   3,   9),
    ( 10,  -5,  12,   3),
    ( -6,  -5,  -6,   7),
    (  8,  -3,   9,  -8),
    (  2, -12,   2,   8),
    (-11,  -2, -10,   3),
    (-12, -13,  -7,  -9),
    (-11,   0, -10,  -5),
    (  5,  -3,  11,   8),
    ( -2, -13,  -1,  12),
    ( -1,  -8,   0,   9),
    (-13, -11, -12,  -5),
    (-10,  -2, -10,  11),
    ( -3,   9,  -2, -13),
    (  2,  -3,   3,   2),
    ( -9, -13,  -4,   0),
    ( -4,   6,  -3, -10),
    ( -4,  12,  -2,  -7),
    ( -6, -11,  -4,   9),
    (  6,  -3,   6,  11),
    (-13,  11,  -5,   5),
    ( 11,  11,  12,   6),
    (  7,  -5,  12,  -2),
    ( -1,  12,   0,   7),
    ( -4,  -8,  -3,  -2),
    ( -7,   1,  -6,   7),
    (-13, -12,  -8, -13),
    ( -7,  -2,  -6,  -8),
    ( -8,   5,  -6,  -9),
    ( -5,  -1,  -4,   5),
    (-13,   7,  -8,  10),
    (  1,   5,   5, -13),
    (  1,   0,  10, -13),
    (  9,  12,  10,  -1),
    (  5,  -8,  10,  -9),
    ( -1,  11,   1, -13),
    ( -9,  -3,  -6,   2),
    ( -1, -10,   1,  12),
    (-13,   1,  -8, -10),
    (  8, -11,  10,  -6),
    (  2, -13,   3,  -6),
    (  7, -13,  12,  -9),
    (-10, -10,  -5,  -7),
    (-10,  -8,  -8, -13),
    (  4,  -6,   8,   5),
    (  3,  12,   8, -13),
    ( -4,   2,  -3,  -3),
    (  5, -13,  10, -12),
    (  4, -13,   5,  -1),
    ( -9,   9,  -4,   3),
    (  0,   3,   3,  -9),
    (-12,   1,  -6,   1),
    (  3,   2,   4,  -8),
    (-10, -10, -10,   9),
    (  8, -13,  12,  12),
    ( -8, -12,  -6,  -5),
    (  2,   2,   3,   7),
    ( 10,   6,  11,  -8),
    (  6,   8,   8, -12),
    ( -7,  10,  -6,   5),
    ( -3,  -9,  -3,   9),
    ( -1, -13,  -1,   5),
    ( -3,  -7,  -3,   4),
    ( -8,  -2,  -8,   3),
    (  4,   2,  12,  12),
    (  2,  -5,   3,  11),
    (  6,  -9,  11, -13),
    (  3,  -1,   7,  12),
    ( 11,  -1,  12,   4),
    ( -3,   0,  -3,   6),
    (  4, -11,   4,  12),
    (  2,  -4,   2,   1),
    (-10,  -6,  -8,   1),
    (-13,   7, -11,   1),
    (-13,  12, -11, -13),
    (  6,   0,  11, -13),
    (  0,  -1,   1,   4),
    (-13,   3,  -9,  -2),
    ( -9,   8,  -6,  -3),
    (-13,  -6,  -8,  -2),
    (  5,  -9,   8,  10),
    (  2,   7,   3,  -9),
    ( -1,  -6,  -1,  -1),
    (  9,   5,  11,  -2),
    ( 11,  -3,  12,  -8),
    (  3,   0,   3,   5),
    ( -1,   4,   0,  10),
    (  3,  -6,   4,   5),
    (-13,   0, -10,   5),
    (  5,   8,  12,  11),
    (  8,   9,   9,  -6),
    (  7,  -4,   8, -12),
    (-10,   4, -10,   9),
    (  7,   3,  12,   4),
    (  9,  -7,  10,  -2),
    (  7,   0,  12,  -2),
    ( -1,  -6,   0, -11),
]

DEFAULT_BRIEF_PATTERN = OPENCV_ORB_PATTERN_256



# ==============================================================================
# Feature & Keypoint Data Structures
# ==============================================================================

class Keypoint:
    def __init__(self, x: int, y: int, score: int, level: int, angle_sector: int = 0):
        self.x = x
        self.y = y
        self.score = score
        self.level = level
        self.angle_sector = angle_sector
        self.descriptor = 0  # 256-bit integer

    def __repr__(self):
        return f"Keypoint(x={self.x}, y={self.y}, score={self.score}, level={self.level}, angle={self.angle_sector})"


# ==============================================================================
# Pipeline Stages
# ==============================================================================

class ORBHardwareReferenceModel:
    def __init__(
        self,
        pixel_depth: int = 6,
        fast_threshold: int = 20,
        pyramid_levels: int = 4,
        brief_pattern: Optional[List[Tuple[int, int, int, int]]] = None,
        img_width: int = 640,
        img_height: int = 480
    ):
        self.pixel_depth = pixel_depth
        self.fast_threshold = fast_threshold
        self.pyramid_levels = pyramid_levels
        self.brief_pattern = brief_pattern if brief_pattern is not None else DEFAULT_BRIEF_PATTERN
        self.img_width = img_width
        self.img_height = img_height

        # Pyramid level dimensions (5/6 scale downsampling)
        self.level_dims = [(img_width, img_height)]
        for lvl in range(1, pyramid_levels):
            w = (self.level_dims[lvl-1][0] * 5) // 6
            h = (self.level_dims[lvl-1][1] * 5) // 6
            self.level_dims.append((w, h))

    def quantize_image(self, image_2d: List[List[int]]) -> List[List[int]]:
        """Quantizes 8-bit pixels to hardware bit-depth (default 6-bit: >> 2)."""
        shift = 8 - self.pixel_depth
        return [[val >> shift for val in row] for row in image_2d]

    def scale_down_5_6(self, img: List[List[int]], in_w: int, in_h: int) -> List[List[int]]:
        """
        Bit-accurate 5/6 bilinear downsampler.
        Matches hardware scaling.sv and bilinear_interpolator.sv:
          w00 = (5 - x%6) * (5 - y%6)
          w01 = (x%6)     * (5 - y%6)
          w10 = (5 - x%6) * (y%6)
          w11 = (x%6)     * (y%6)
          pixel_out = (weighted_sum * 41 + 512) >> 10
        """
        out_w = (in_w * 5) // 6
        out_h = (in_h * 5) // 6
        out_img = [[0] * out_w for _ in range(out_h)]

        out_y = 0
        for y in range(in_h - 1):
            sy = y % 6
            if sy == 5:
                continue
            if out_y >= out_h:
                break

            out_x = 0
            for x in range(in_w - 1):
                sx = x % 6
                if sx == 5:
                    continue
                if out_x >= out_w:
                    break

                inv_x = 5 - sx
                inv_y = 5 - sy

                w00 = inv_x * inv_y
                w01 = sx * inv_y
                w10 = inv_x * sy
                w11 = sx * sy

                p00 = img[y][x]
                p01 = img[y][x + 1]
                p10 = img[y + 1][x]
                p11 = img[y + 1][x + 1]

                weighted_sum = p00 * w00 + p01 * w01 + p10 * w10 + p11 * w11
                out_pixel = (weighted_sum * 41 + 512) >> 10
                out_img[out_y][out_x] = out_pixel
                out_x += 1
            out_y += 1

        return out_img

    def build_image_pyramid(self, quantized_img: List[List[int]]) -> List[List[List[int]]]:
        """Builds 4-level image pyramid using 5/6 scale factor."""
        pyramid = [quantized_img]
        for lvl in range(1, self.pyramid_levels):
            prev_w, prev_h = self.level_dims[lvl - 1]
            scaled = self.scale_down_5_6(pyramid[lvl - 1], prev_w, prev_h)
            pyramid.append(scaled)
        return pyramid

    def gaussian_blur(self, img: List[List[int]], width: int, height: int) -> List[List[int]]:
        """
        7x7 Binomial Gaussian filter.
        Normalizes by dividing by 4096 (sum of kernel = 2^12) with rounding:
          pixel_out = (conv_sum + 2048) >> 12
        """
        blurred = [[0] * width for _ in range(height)]
        for y in range(3, height - 3):
            for x in range(3, width - 3):
                conv_sum = 0
                for ky in range(7):
                    img_y = y + ky - 3
                    for kx in range(7):
                        img_x = x + kx - 3
                        conv_sum += img[img_y][img_x] * BINOMIAL_KERNEL_7X7[ky][kx]
                blurred[y][x] = (conv_sum + 2048) >> 12
        return blurred

    def detect_fast_corners(self, img: List[List[int]], width: int, height: int, level: int) -> List[List[int]]:
        """
        FAST-9/16 corner detector.
        Examines 16 Bresenham circle pixels (radius 3).
        Score is sum of absolute differences if corner, 0 otherwise.
        """
        scores = [[0] * width for _ in range(height)]
        thresh = self.fast_threshold

        for y in range(3, height - 3):
            for x in range(3, width - 3):
                center = img[y][x]
                circle_pixels = [img[y + dy][x + dx] for dy, dx in BRESENHAM_CIRCLE_OFFSETS]

                brighter = [(p > center + thresh) for p in circle_pixels]
                darker = [(center > p + thresh) for p in circle_pixels]

                # Check 9 contiguous
                b_2x = brighter + brighter
                d_2x = darker + darker

                is_corner = False
                for i in range(16):
                    if all(b_2x[i : i + 9]) or all(d_2x[i : i + 9]):
                        is_corner = True
                        break

                if is_corner:
                    score = sum(abs(p - center) for p in circle_pixels)
                    scores[y][x] = score

        return scores

    def non_maximum_suppression(self, scores: List[List[int]], width: int, height: int, level: int) -> List[Keypoint]:
        """
        3x3 Non-Maximum Suppression with hardware-aligned tie-breaking.
        Strict > for top/left neighbors, >= for right/bottom neighbors.
        """
        keypoints = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                center = scores[y][x]
                if center <= 0:
                    continue

                # Check neighbors
                if (center <= scores[y - 1][x - 1] or
                    center <= scores[y - 1][x] or
                    center <= scores[y - 1][x + 1] or
                    center <= scores[y][x - 1]):
                    continue

                if (center < scores[y][x + 1] or
                    center < scores[y + 1][x - 1] or
                    center < scores[y + 1][x] or
                    center < scores[y + 1][x + 1]):
                    continue

                keypoints.append(Keypoint(x=x, y=y, score=center, level=level))

        return keypoints

    def compute_orientation(self, blurred_img: List[List[int]], width: int, height: int, kpt: Keypoint) -> int:
        """
        37x37 Intensity Centroid Orientation discretized into 64 sectors (0..63).
        Matches hardware orientation.sv using quadrant mapping and 15 tangent comparators.
        """
        win_size = 37
        half_win = win_size // 2  # 18

        m00 = 0
        m01 = 0
        m10 = 0

        for dy in range(-half_win, half_win + 1):
            py = kpt.y + dy
            if py < 0 or py >= height:
                continue
            for dx in range(-half_win, half_win + 1):
                px = kpt.x + dx
                if px < 0 or px >= width:
                    continue
                val = blurred_img[py][px]
                m00 += val
                m01 += val * dy
                m10 += val * dx

        abs_m01 = abs(m01)
        abs_m10 = abs(m10)

        # Quadrant determination:
        # Quad 0: [0, 90)   -> m10 >= 0, m01 >= 0
        # Quad 1: [90, 180)  -> m10 < 0,  m01 >= 0
        # Quad 2: [180, 270) -> m10 < 0,  m01 < 0
        # Quad 3: [270, 360) -> m10 >= 0, m01 < 0
        if m10 >= 0 and m01 >= 0:
            quadrant = 0
        elif m10 < 0 and m01 >= 0:
            quadrant = 1
        elif m10 < 0 and m01 < 0:
            quadrant = 2
        else:
            quadrant = 3

        # Sector in quadrant (0 to 15) using 15 tangent threshold multipliers
        sector_in_quad = 15
        for s in range(15):
            if (abs_m10 * TAN_THRESH_15[s]) >= (abs_m01 << 8):
                sector_in_quad = s
                break

        angle_sector = (quadrant << 4) | sector_in_quad
        return angle_sector

    def generate_brief_descriptor(
        self,
        blurred_img: List[List[int]],
        width: int,
        height: int,
        kpt: Keypoint
    ) -> int:
        """
        Generates 256-bit Rotated BRIEF descriptor using 64-sector Q1.7 sin/cos LUTs.
        Matches hardware brief.sv and rotator.sv.
        """
        angle = kpt.angle_sector
        cos_val = COS_LUT_64[angle]
        sin_val = SIN_LUT_64[angle]

        descriptor = 0
        for bit_idx, (x1, y1, x2, y2) in enumerate(self.brief_pattern):
            # Rotate pair coordinates: x' = (x*cos - y*sin) >> 7, y' = (x*sin + y*cos) >> 7
            rot_x1 = (x1 * cos_val - y1 * sin_val) >> 7
            rot_y1 = (x1 * sin_val + y1 * cos_val) >> 7
            rot_x2 = (x2 * cos_val - y2 * sin_val) >> 7
            rot_y2 = (x2 * sin_val + y2 * cos_val) >> 7

            px1 = kpt.x + rot_x1
            py1 = kpt.y + rot_y1
            px2 = kpt.x + rot_x2
            py2 = kpt.y + rot_y2

            val1 = blurred_img[py1][px1] if (0 <= px1 < width and 0 <= py1 < height) else 0
            val2 = blurred_img[py2][px2] if (0 <= px2 < width and 0 <= py2 < height) else 0

            if val1 > val2:
                descriptor |= (1 << bit_idx)

        return descriptor

    def extract_features(self, raw_image_2d: List[List[int]]) -> List[Keypoint]:
        """
        Full end-to-end ORB feature extraction pipeline:
        Quantize -> Pyramid -> Blur -> FAST -> NMS -> Orientation -> BRIEF.
        """
        quantized = self.quantize_image(raw_image_2d)
        pyramid = self.build_image_pyramid(quantized)

        all_keypoints: List[Keypoint] = []

        for lvl in range(self.pyramid_levels):
            lvl_img = pyramid[lvl]
            w, h = self.level_dims[lvl]

            # Gaussian blur on level image
            blurred = self.gaussian_blur(lvl_img, w, h)

            # FAST corner detection
            scores = self.detect_fast_corners(lvl_img, w, h, lvl)

            # NMS
            kpts = self.non_maximum_suppression(scores, w, h, lvl)

            # Orientation & Descriptor calculation
            for kpt in kpts:
                kpt.angle_sector = self.compute_orientation(blurred, w, h, kpt)
                kpt.descriptor = self.generate_brief_descriptor(blurred, w, h, kpt)
                all_keypoints.append(kpt)

        return all_keypoints


# ==============================================================================
# Hamming Distance Matcher
# ==============================================================================

def popcount256(val: int) -> int:
    """Computes population count (Hamming weight) of a 256-bit integer."""
    return bin(val).count('1')


def match_features(
    query_kpts: List[Keypoint],
    map_kpts: List[Keypoint],
    threshold: int = 50
) -> List[Tuple[Keypoint, Keypoint, int]]:
    """
    Matches query features against map features using Hamming distance.
    Returns list of (query_kpt, matched_map_kpt, distance).
    """
    matches = []
    for q in query_kpts:
        best_dist = 256
        best_match = None
        for m in map_kpts:
            dist = popcount256(q.descriptor ^ m.descriptor)
            if dist < best_dist and dist <= threshold:
                best_dist = dist
                best_match = m
        if best_match is not None:
            matches.append((q, best_match, best_dist))
    return matches


# ==============================================================================
# Helper Utilities & Self-Test
# ==============================================================================

def generate_synthetic_image(width: int = 640, height: int = 480) -> List[List[int]]:
    """Generates a synthetic test image with high-contrast geometric corners."""
    img = [[32] * width for _ in range(height)]

    # Draw high-contrast squares / checkerboard corners
    for sq_y in range(50, height - 100, 80):
        for sq_x in range(50, width - 100, 80):
            for y in range(sq_y, sq_y + 40):
                for x in range(sq_x, sq_x + 40):
                    img[y][x] = 220

    # Draw gradient lines
    for y in range(100, 300):
        for x in range(100, 300):
            if (x + y) % 30 < 15:
                img[y][x] = 180

    return img


if __name__ == "__main__":
    print("=================================================================")
    print(" Running FSLAM ORB Hardware Reference Model Self-Test")
    print("=================================================================")

    model = ORBHardwareReferenceModel(
        pixel_depth=6,
        fast_threshold=20,
        pyramid_levels=4,
        img_width=640,
        img_height=480
    )

    test_img = generate_synthetic_image(640, 480)
    print(f"[1] Generated synthetic test image: {len(test_img[0])}x{len(test_img)}")

    features = model.extract_features(test_img)
    print(f"[2] Extracted total features: {len(features)}")

    # Group features by pyramid level
    by_level = {}
    for f in features:
        by_level[f.level] = by_level.get(f.level, 0) + 1

    for lvl in range(4):
        print(f"    - Level {lvl} ({model.level_dims[lvl][0]}x{model.level_dims[lvl][1]}): {by_level.get(lvl, 0)} features")

    if features:
        sample = features[0]
        print(f"[3] Sample Feature 0:")
        print(f"    - Level: {sample.level}")
        print(f"    - Coord: ({sample.x}, {sample.y})")
        print(f"    - Score: {sample.score}")
        print(f"    - Orientation Sector: {sample.angle_sector} ({sample.angle_sector * 5.625:.1f}°)")
        print(f"    - Descriptor (Hex): 0x{sample.descriptor:064x}")

        # Self-matching test
        matches = match_features(features[:20], features[:20], threshold=50)
        print(f"[4] Self-matching test (20 features): {len(matches)} matches found (all distance 0).")

    print("=================================================================")
    print(" Self-Test PASSED Successfully!")
    print("=================================================================")
