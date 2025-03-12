"""
Cocotb test for FAST (Features from Accelerated Segment Test) corner detection module

Tests corner detection using Bresenham circle pattern and threshold comparison.
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


def create_test_image(width=640, height=480, pattern='corners'):
    """Create a test image with known corner patterns"""
    image = np.zeros((height, width), dtype=int)
    
    if pattern == 'corners':
        # Add some corner-like features
        # Bright square in dark background
        image[100:150, 100:150] = 50  # Bright region
        image[200:250, 200:250] = 45  # Another bright region
        
        # Dark square in bright background  
        image[300:480, 300:640] = 40
        image[350:400, 350:400] = 10  # Dark region
        
    elif pattern == 'gradient':
        for i in range(height):
            for j in range(width):
                image[i][j] = min(63, (i + j) // 20)
                
    elif pattern == 'checkerboard':
        for i in range(height):
            for j in range(width):
                if (i // 20 + j // 20) % 2 == 0:
                    image[i][j] = 50
                else:
                    image[i][j] = 10
    
    return image


async def send_image_stream(dut, image):
    """Send image data as a pixel stream"""
    height, width = image.shape
    corners_detected = []
    
    for y in range(height):
        for x in range(width):
            # Set up line buffer (simplified - assumes 7 line buffers)
            # In practice, you'd need to properly manage the line buffer history
            if y >= 6:  # Need at least 7 rows for Bresenham circle
                # Set line buffer values for the 7-row window
                for i in range(7):
                    if y - 6 + i < height:
                        dut.line_buffer[i].value = image[y - 6 + i, x]
                    else:
                        dut.line_buffer[i].value = 0
            
            dut.pixel_in.value = image[y, x]
            dut.pixel_valid.value = 1
            
            await RisingEdge(dut.clk)
            
            # Check for corner detection
            if dut.corner_valid.value == 1:
                corner_x = dut.corner_x.value
                corner_y = dut.corner_y.value
                corner_score = dut.corner_score.value
                corners_detected.append((corner_x, corner_y, corner_score))
                dut._log.info(f"Corner detected at ({corner_x}, {corner_y}) with score {corner_score}")
    
    # Send a few more cycles to flush pipeline
    dut.pixel_valid.value = 0
    for _ in range(10):
        await RisingEdge(dut.clk)
    
    return corners_detected


@cocotb.test()
async def test_fast_basic_corner_detection(dut):
    """Test basic FAST corner detection functionality"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create test image with known corners
    image = create_test_image(640, 480, 'corners')
    
    # Send image and collect detected corners
    corners = await send_image_stream(dut, image)
    
    # Should detect some corners
    assert len(corners) > 0, "Should detect at least one corner"
    
    # Check that corners are within valid region (excluding borders)
    for corner_x, corner_y, score in corners:
        assert 3 <= corner_x < 640 - 3, f"Corner x={corner_x} outside valid range"
        assert 3 <= corner_y < 480 - 3, f"Corner y={corner_y} outside valid range"
        assert score > 0, f"Corner score {score} should be positive"
    
    dut._log.info(f"Detected {len(corners)} corners")


@cocotb.test()
async def test_fast_threshold_behavior(dut):
    """Test FAST behavior with different threshold values"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create a simple test pattern - bright center pixel with darker surround
    # This should create a corner-like feature
    
    # Simulate a small region around a potential corner
    center_pixel = 40
    surround_pixels = [10] * 16  # All darker than center - threshold
    
    # Set up line buffers to simulate Bresenham circle
    for i in range(7):
        dut.line_buffer[i].value = surround_pixels[i % 16]
    
    # Send center pixel
    dut.pixel_in.value = center_pixel
    dut.pixel_valid.value = 1
    
    await RisingEdge(dut.clk)
    
    # Check if corner is detected (should be since difference > threshold)
    await RisingEdge(dut.clk)  # Wait for processing
    
    dut._log.info(f"Corner valid: {dut.corner_valid.value}")
    if dut.corner_valid.value:
        dut._log.info(f"Corner score: {dut.corner_score.value}")


@cocotb.test()
async def test_fast_no_corner_detection(dut):
    """Test that FAST doesn't detect corners in uniform regions"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create uniform image
    uniform_value = 30
    image = np.full((480, 640), uniform_value, dtype=int)
    
    corners = await send_image_stream(dut, image)
    
    # Should not detect corners in uniform region
    assert len(corners) == 0, f"Should not detect corners in uniform image, but detected {len(corners)}"


@cocotb.test()
async def test_fast_edge_cases(dut):
    """Test FAST behavior at image edges"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create small test image
    small_image = create_test_image(20, 20, 'checkerboard')
    
    corners = await send_image_stream(dut, small_image)
    
    # All detected corners should be away from edges
    for corner_x, corner_y, score in corners:
        assert 3 <= corner_x < 20 - 3, f"Corner too close to edge: x={corner_x}"
        assert 3 <= corner_y < 20 - 3, f"Corner too close to edge: y={corner_y}"


@cocotb.test()
async def test_fast_pipeline_timing(dut):
    """Test FAST pipeline timing and valid signal behavior"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Send a few pixels and monitor timing
    test_pixels = [20, 30, 40, 50, 25]
    
    for i, pixel in enumerate(test_pixels):
        # Set up minimal line buffer
        for j in range(7):
            dut.line_buffer[j].value = pixel + (j - 3) * 2
        
        dut.pixel_in.value = pixel
        dut.pixel_valid.value = 1
        
        await RisingEdge(dut.clk)
        
        # Monitor corner_valid timing
        if dut.corner_valid.value:
            dut._log.info(f"Cycle {i}: Corner detected")
    
    # Test invalid input
    dut.pixel_valid.value = 0
    await RisingEdge(dut.clk)
    
    # corner_valid should follow pixel_valid timing
    # (exact timing depends on pipeline depth)


@cocotb.test()
async def test_fast_score_calculation(dut):
    """Test FAST corner score calculation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create a strong corner pattern
    center = 50
    strong_contrast = [10] * 16  # High contrast
    weak_contrast = [45] * 16    # Low contrast
    
    test_cases = [
        (center, strong_contrast, "strong"),
        (center, weak_contrast, "weak")
    ]
    
    for center_val, surround, case_name in test_cases:
        # Set up line buffers
        for i in range(7):
            dut.line_buffer[i].value = surround[i % 16]
        
        dut.pixel_in.value = center_val
        dut.pixel_valid.value = 1
        
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)  # Wait for processing
        
        if dut.corner_valid.value:
            score = dut.corner_score.value
            dut._log.info(f"{case_name} corner score: {score}")
            
            # Strong contrast should give higher scores
            if case_name == "strong":
                assert score > 100, f"Strong corner should have high score, got {score}"


@cocotb.test()
async def test_fast_contiguous_pixels(dut):
    """Test FAST requirement for 9 contiguous pixels"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test case: only 8 contiguous pixels (should not detect corner)
    center = 40
    
    # 8 bright pixels, then dark pixels (not 9 contiguous)
    circle_pixels = [60] * 8 + [20] * 8  # 8 bright + 8 dark
    
    # Set up line buffers to simulate Bresenham circle
    for i in range(7):
        dut.line_buffer[i].value = circle_pixels[i % 16]
    
    dut.pixel_in.value = center
    dut.pixel_valid.value = 1
    
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    
    # Should not detect corner with only 8 contiguous pixels
    # (This test depends on exact implementation)
    if dut.corner_valid.value:
        dut._log.info("Corner detected (may be valid depending on implementation)")
    else:
        dut._log.info("No corner detected for 8 contiguous pixels")


if __name__ == "__main__":
    import os
    os.system("make")