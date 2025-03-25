"""
Cocotb test for orientation calculation module

Tests the intensity centroid-based orientation calculation with 64-sector discretization.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import random
import numpy as np
import math


async def reset_dut(dut):
    """Reset the DUT"""
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def create_oriented_pattern(size=37, angle_degrees=0):
    """Create a test pattern with known orientation"""
    pattern = np.zeros((size, size), dtype=int)
    center = size // 2
    
    # Create gradient pattern oriented at specified angle
    angle_rad = math.radians(angle_degrees)
    
    for i in range(size):
        for j in range(size):
            # Vector from center
            dx = j - center
            dy = i - center
            
            # Project onto orientation direction
            projection = dx * math.cos(angle_rad) + dy * math.sin(angle_rad)
            
            # Create intensity based on projection
            intensity = max(0, min(63, int(32 + projection * 2)))
            pattern[i, j] = intensity
    
    return pattern


def create_circular_pattern(size=37, bright_sector_start=0, bright_sector_end=90):
    """Create circular pattern with bright sector at specified angle range"""
    pattern = np.zeros((size, size), dtype=int)
    center = size // 2
    
    for i in range(size):
        for j in range(size):
            dx = j - center
            dy = i - center
            
            if dx == 0 and dy == 0:
                pattern[i, j] = 32
                continue
                
            # Calculate angle
            angle = math.degrees(math.atan2(dy, dx))
            if angle < 0:
                angle += 360
            
            # Bright sector
            if bright_sector_start <= angle <= bright_sector_end:
                pattern[i, j] = 60
            else:
                pattern[i, j] = 10
    
    return pattern


async def send_windowed_pixels(dut, pattern, keypoint_x=18, keypoint_y=18):
    """Send pixel data for orientation calculation"""
    size = len(pattern)
    
    # Set keypoint location
    dut.keypoint_x.value = keypoint_x
    dut.keypoint_y.value = keypoint_y
    
    # Start orientation calculation
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    
    # Send pixel data in raster order
    for y in range(size):
        for x in range(size):
            dut.pixel_valid.value = 1
            dut.pixel_in.value = pattern[y, x]
            dut.pixel_x.value = keypoint_x - size//2 + x
            dut.pixel_y.value = keypoint_y - size//2 + y
            
            await RisingEdge(dut.clk)
    
    # Wait for completion
    dut.pixel_valid.value = 0
    timeout = 0
    while timeout < 1000:
        await RisingEdge(dut.clk)
        if dut.orientation_valid.value == 1:
            break
        timeout += 1
    
    assert timeout < 1000, "Orientation calculation timed out"
    return dut.orientation_out.value


@cocotb.test()
async def test_orientation_basic(dut):
    """Test basic orientation calculation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test horizontal gradient (should give 0 degrees)
    horizontal_pattern = create_oriented_pattern(37, 0)
    orientation = await send_windowed_pixels(dut, horizontal_pattern)
    
    dut._log.info(f"Horizontal pattern orientation: {orientation} (sector)")
    
    # Convert sector to degrees for interpretation
    degrees = (orientation * 360) / 64
    dut._log.info(f"Horizontal pattern orientation: {degrees} degrees")
    
    # Should be close to 0 degrees (or 360 degrees)
    assert orientation == 0 or orientation >= 60, f"Horizontal pattern should give ~0 degrees, got sector {orientation}"


@cocotb.test()
async def test_orientation_vertical(dut):
    """Test vertical orientation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test vertical gradient (should give 90 degrees = sector 16)
    vertical_pattern = create_oriented_pattern(37, 90)
    orientation = await send_windowed_pixels(dut, vertical_pattern)
    
    degrees = (orientation * 360) / 64
    dut._log.info(f"Vertical pattern orientation: {degrees} degrees")
    
    # Should be close to 90 degrees (sector 16)
    expected_sector = 16  # 90 degrees
    assert abs(orientation - expected_sector) <= 2, f"Vertical pattern should give ~sector 16, got {orientation}"


@cocotb.test()
async def test_orientation_diagonal(dut):
    """Test diagonal orientations"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    # Test 45-degree diagonal
    await reset_dut(dut)
    diagonal_pattern = create_oriented_pattern(37, 45)
    orientation = await send_windowed_pixels(dut, diagonal_pattern)
    
    degrees = (orientation * 360) / 64
    dut._log.info(f"45-degree pattern orientation: {degrees} degrees")
    
    # Should be close to 45 degrees (sector 8)
    expected_sector = 8  # 45 degrees
    assert abs(orientation - expected_sector) <= 2, f"45-degree pattern should give ~sector 8, got {orientation}"


@cocotb.test()
async def test_orientation_sectors(dut):
    """Test all major orientation sectors"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    # Test major angles
    test_angles = [0, 45, 90, 135, 180, 225, 270, 315]
    
    for angle in test_angles:
        await reset_dut(dut)
        
        pattern = create_oriented_pattern(37, angle)
        orientation = await send_windowed_pixels(dut, pattern)
        
        calculated_degrees = (orientation * 360) / 64
        dut._log.info(f"Input: {angle}°, Output: sector {orientation} ({calculated_degrees}°)")
        
        # Verify orientation is reasonable (within discretization error)
        expected_sector = int((angle * 64) / 360) % 64
        error = min(abs(orientation - expected_sector), 
                   64 - abs(orientation - expected_sector))  # Handle wrap-around
        
        assert error <= 3, f"Angle {angle}° error too large: expected ~{expected_sector}, got {orientation}"


@cocotb.test()
async def test_orientation_circular_sectors(dut):
    """Test orientation with circular bright sectors"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    # Test bright sectors at different angles
    sector_tests = [
        (0, 30),      # Right side bright
        (90, 120),    # Bottom bright  
        (180, 210),   # Left side bright
        (270, 300),   # Top bright
    ]
    
    for start_angle, end_angle in sector_tests:
        await reset_dut(dut)
        
        pattern = create_circular_pattern(37, start_angle, end_angle)
        orientation = await send_windowed_pixels(dut, pattern)
        
        mid_angle = (start_angle + end_angle) / 2
        calculated_degrees = (orientation * 360) / 64
        
        dut._log.info(f"Bright sector {start_angle}-{end_angle}°, centroid at {mid_angle}°")
        dut._log.info(f"Output: sector {orientation} ({calculated_degrees}°)")


@cocotb.test()
async def test_orientation_uniform_pattern(dut):
    """Test orientation with uniform pattern (should give consistent result)"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Uniform pattern should have centroid at center
    uniform_pattern = np.full((37, 37), 30, dtype=int)
    orientation = await send_windowed_pixels(dut, uniform_pattern)
    
    dut._log.info(f"Uniform pattern orientation: sector {orientation}")
    
    # Uniform pattern might give arbitrary orientation due to numerical effects
    # Main test is that it completes without error


@cocotb.test()
async def test_orientation_busy_signal(dut):
    """Test busy signal behavior during orientation calculation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Initially not busy
    assert dut.busy.value == 0, "Should not be busy initially"
    
    # Create test pattern
    pattern = create_oriented_pattern(37, 30)
    
    dut.keypoint_x.value = 18
    dut.keypoint_y.value = 18
    
    # Start calculation
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    
    # Should be busy now
    await RisingEdge(dut.clk)
    assert dut.busy.value == 1, "Should be busy after start"
    
    # Send pixel data while monitoring busy
    for y in range(37):
        for x in range(37):
            dut.pixel_valid.value = 1
            dut.pixel_in.value = pattern[y, x]
            dut.pixel_x.value = 18 - 18 + x
            dut.pixel_y.value = 18 - 18 + y
            
            await RisingEdge(dut.clk)
            
            # Should remain busy during processing
            if dut.orientation_valid.value == 0:
                assert dut.busy.value == 1, "Should be busy while processing"
    
    # Wait for completion
    dut.pixel_valid.value = 0
    while dut.busy.value == 1:
        await RisingEdge(dut.clk)
    
    # Should not be busy when done
    assert dut.busy.value == 0, "Should not be busy when complete"
    assert dut.orientation_valid.value == 1, "Should have valid orientation when done"


@cocotb.test()
async def test_orientation_edge_keypoint(dut):
    """Test orientation calculation for keypoints near edges"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test keypoint near edge of window
    edge_pattern = create_oriented_pattern(37, 60)
    
    # Keypoint near edge of pattern
    orientation = await send_windowed_pixels(dut, edge_pattern, keypoint_x=25, keypoint_y=25)
    
    degrees = (orientation * 360) / 64
    dut._log.info(f"Edge keypoint orientation: sector {orientation} ({degrees}°)")
    
    # Should complete successfully
    assert dut.orientation_valid.value == 1, "Edge keypoint should produce valid orientation"


@cocotb.test()
async def test_orientation_multiple_keypoints(dut):
    """Test multiple orientation calculations in sequence"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    patterns_and_angles = [
        (create_oriented_pattern(37, 0), 0),
        (create_oriented_pattern(37, 45), 45),
        (create_oriented_pattern(37, 90), 90),
    ]
    
    for i, (pattern, expected_angle) in enumerate(patterns_and_angles):
        await reset_dut(dut)
        
        orientation = await send_windowed_pixels(dut, pattern)
        calculated_degrees = (orientation * 360) / 64
        
        dut._log.info(f"Pattern {i}: expected {expected_angle}°, got {calculated_degrees}°")
        
        # Verify each calculation
        expected_sector = int((expected_angle * 64) / 360) % 64
        error = min(abs(orientation - expected_sector), 
                   64 - abs(orientation - expected_sector))
        assert error <= 3, f"Pattern {i} orientation error too large"


if __name__ == "__main__":
    import os
    os.system("make")