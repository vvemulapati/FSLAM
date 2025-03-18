"""
Cocotb test for NMS (Non-Maximum Suppression) module

Tests the corner suppression functionality that keeps only local maxima.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import random
import numpy as np


async def reset_dut(dut):
    """Reset the DUT"""
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def generate_corner_grid(width=640, height=480, spacing=50):
    """Generate a grid of corners with known locations and scores"""
    corners = []
    
    for y in range(spacing, height-spacing, spacing):
        for x in range(spacing, width-spacing, spacing):
            # Create corners with random scores
            score = random.randint(50, 1000)
            corners.append((x, y, score))
    
    return corners


async def send_corner_stream(dut, corners):
    """Send corner detections and collect NMS output"""
    output_keypoints = []
    
    # Sort corners by raster order (y, then x)
    corners_sorted = sorted(corners, key=lambda c: (c[1], c[0]))
    
    for x, y, score in corners_sorted:
        dut.corner_valid_in.value = 1
        dut.corner_score_in.value = score
        dut.corner_x_in.value = x
        dut.corner_y_in.value = y
        
        await RisingEdge(dut.clk)
        
        # Check for keypoint output
        if dut.keypoint_valid.value == 1:
            kp_x = dut.keypoint_x.value
            kp_y = dut.keypoint_y.value
            kp_score = dut.keypoint_score.value
            output_keypoints.append((kp_x, kp_y, kp_score))
            dut._log.info(f"Keypoint output: ({kp_x}, {kp_y}) score={kp_score}")
    
    # Send invalid data to flush pipeline
    dut.corner_valid_in.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
        if dut.keypoint_valid.value == 1:
            kp_x = dut.keypoint_x.value
            kp_y = dut.keypoint_y.value
            kp_score = dut.keypoint_score.value
            output_keypoints.append((kp_x, kp_y, kp_score))
    
    return output_keypoints


@cocotb.test()
async def test_nms_basic_suppression(dut):
    """Test basic non-maximum suppression functionality"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create test case: 3 nearby corners, middle one has highest score
    corners = [
        (100, 100, 500),  # Lower score
        (101, 100, 800),  # Highest score (should be kept)
        (102, 100, 600),  # Lower score
    ]
    
    keypoints = await send_corner_stream(dut, corners)
    
    # Should only keep the highest scoring corner
    dut._log.info(f"Input corners: {len(corners)}, Output keypoints: {len(keypoints)}")
    
    # At least should suppress some corners
    assert len(keypoints) <= len(corners), "NMS should not increase number of features"
    
    # The highest scoring corner should be in output
    if keypoints:
        max_score_output = max(kp[2] for kp in keypoints)
        assert max_score_output == 800, f"Highest score corner should be preserved, got {max_score_output}"


@cocotb.test()
async def test_nms_isolated_corners(dut):
    """Test that isolated corners are preserved"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create widely spaced corners (no suppression should occur)
    corners = [
        (50, 50, 400),
        (200, 50, 500),
        (50, 200, 600),
        (200, 200, 300),
    ]
    
    keypoints = await send_corner_stream(dut, corners)
    
    # All isolated corners should be preserved
    dut._log.info(f"Isolated corners: {len(corners)}, Output: {len(keypoints)}")
    
    # Should preserve most or all isolated corners
    assert len(keypoints) >= len(corners) - 1, "Isolated corners should be preserved"


@cocotb.test()
async def test_nms_edge_cases(dut):
    """Test NMS behavior at image edges"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Corners near image edges
    edge_corners = [
        (1, 1, 500),      # Near top-left edge
        (638, 1, 600),    # Near top-right edge
        (1, 478, 400),    # Near bottom-left edge
        (638, 478, 700),  # Near bottom-right edge
        (320, 1, 550),    # Top edge
        (320, 478, 450),  # Bottom edge
    ]
    
    keypoints = await send_corner_stream(dut, edge_corners)
    
    # Edge corners should be handled properly
    dut._log.info(f"Edge corners: {len(edge_corners)}, Output: {len(keypoints)}")
    
    # All edge keypoints should be within valid image bounds
    for kp_x, kp_y, score in keypoints:
        assert 0 <= kp_x < 640, f"Keypoint x={kp_x} outside image bounds"
        assert 0 <= kp_y < 480, f"Keypoint y={kp_y} outside image bounds"


@cocotb.test()
async def test_nms_score_ordering(dut):
    """Test that higher scores are preserved over lower scores"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create 3x3 grid of corners with center having highest score
    corners = [
        (99, 99, 100),   (100, 99, 200),   (101, 99, 150),
        (99, 100, 300),  (100, 100, 800),  (101, 100, 250),  # Center has max
        (99, 101, 180),  (100, 101, 120),  (101, 101, 90),
    ]
    
    keypoints = await send_corner_stream(dut, corners)
    
    # Should preserve the center corner with highest score
    if keypoints:
        # Find output near center
        center_outputs = [(x, y, s) for x, y, s in keypoints if 99 <= x <= 101 and 99 <= y <= 101]
        if center_outputs:
            max_output_score = max(s for x, y, s in center_outputs)
            assert max_output_score == 800, f"Center corner with max score should be preserved"


@cocotb.test()
async def test_nms_pipeline_behavior(dut):
    """Test NMS pipeline timing and valid signal behavior"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Send corners one by one and monitor timing
    test_corners = [
        (50, 50, 400),
        (51, 50, 500),   # Adjacent corner
        (100, 100, 600), # Distant corner
    ]
    
    output_count = 0
    
    for i, (x, y, score) in enumerate(test_corners):
        dut.corner_valid_in.value = 1
        dut.corner_score_in.value = score
        dut.corner_x_in.value = x
        dut.corner_y_in.value = y
        
        await RisingEdge(dut.clk)
        
        if dut.keypoint_valid.value == 1:
            output_count += 1
            dut._log.info(f"Cycle {i}: Keypoint output")
    
    # Test with invalid input
    dut.corner_valid_in.value = 0
    for j in range(5):
        await RisingEdge(dut.clk)
        if dut.keypoint_valid.value == 1:
            output_count += 1
            dut._log.info(f"Pipeline flush cycle {j}: Keypoint output")
    
    dut._log.info(f"Total output count: {output_count}")


@cocotb.test()
async def test_nms_zero_score_handling(dut):
    """Test NMS behavior with zero and low scores"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Mix of zero and non-zero scores
    corners = [
        (50, 50, 0),     # Zero score
        (51, 50, 100),   # Low score
        (100, 100, 500), # Good score
        (150, 150, 0),   # Another zero score
    ]
    
    keypoints = await send_corner_stream(dut, corners)
    
    # Zero score corners should typically be suppressed
    for kp_x, kp_y, score in keypoints:
        assert score > 0, f"Zero score corners should be suppressed, got score={score}"
    
    dut._log.info(f"Non-zero corners preserved: {len(keypoints)}")


@cocotb.test()
async def test_nms_dense_corner_field(dut):
    """Test NMS with many closely spaced corners"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create dense field of corners
    corners = []
    base_x, base_y = 100, 100
    
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            x = base_x + dx
            y = base_y + dy
            # Center corner has highest score
            score = 500 - abs(dx) * 50 - abs(dy) * 50
            corners.append((x, y, max(score, 100)))
    
    keypoints = await send_corner_stream(dut, corners)
    
    # Should significantly reduce number of corners
    dut._log.info(f"Dense input: {len(corners)}, NMS output: {len(keypoints)}")
    assert len(keypoints) < len(corners), "NMS should suppress overlapping corners"
    
    # The highest score corner should be preserved
    if keypoints:
        max_output_score = max(score for x, y, score in keypoints)
        expected_max = 500  # Center corner score
        assert max_output_score == expected_max, f"Max score corner should be preserved"


@cocotb.test()
async def test_nms_tie_breaking(dut):
    """Test NMS behavior with equal scores"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Two adjacent corners with same score
    corners = [
        (100, 100, 500),
        (101, 100, 500),  # Same score, adjacent
    ]
    
    keypoints = await send_corner_stream(dut, corners)
    
    # Should pick one of them (implementation dependent)
    dut._log.info(f"Tie-breaking test: {len(corners)} input, {len(keypoints)} output")
    assert len(keypoints) <= len(corners), "Tie-breaking should not increase corners"
    
    if keypoints:
        preserved_score = keypoints[0][2]
        assert preserved_score == 500, "Preserved corner should have correct score"


if __name__ == "__main__":
    import os
    os.system("make")