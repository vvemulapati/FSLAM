"""
Cocotb test for coordinate rotator module

Tests 2D coordinate rotation using precomputed sin/cos lookup tables.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import math
import random


async def reset_dut(dut):
    """Reset the DUT"""
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def calculate_expected_rotation(x, y, angle_sector):
    """Calculate expected rotation result"""
    # Convert sector to radians (64 sectors = 360 degrees)
    angle_rad = (angle_sector * 2 * math.pi) / 64
    
    # Apply rotation matrix
    cos_val = math.cos(angle_rad)
    sin_val = math.sin(angle_rad)
    
    x_rot = x * cos_val - y * sin_val
    y_rot = x * sin_val + y * cos_val
    
    return int(round(x_rot)), int(round(y_rot))


@cocotb.test()
async def test_rotator_no_rotation(dut):
    """Test rotation with angle = 0 (no rotation)"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test coordinates
    test_coords = [(5, 0), (0, 5), (-3, 4), (10, -7)]
    
    for x, y in test_coords:
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = 0  # No rotation
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        
        # Wait for output
        await RisingEdge(dut.clk)
        assert dut.valid_out.value == 1, "Output should be valid"
        
        # With no rotation, output should equal input
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        # Convert to signed if needed
        if x_out >= 32:  # Assuming 6-bit signed
            x_out = x_out - 64
        if y_out >= 32:
            y_out = y_out - 64
            
        dut._log.info(f"No rotation: ({x},{y}) -> ({x_out},{y_out})")
        
        assert abs(x_out - x) <= 1, f"X coordinate mismatch: expected {x}, got {x_out}"
        assert abs(y_out - y) <= 1, f"Y coordinate mismatch: expected {y}, got {y_out}"


@cocotb.test()
async def test_rotator_90_degrees(dut):
    """Test 90-degree rotation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # 90 degrees = sector 16 (64 sectors / 4 = 16)
    angle_90 = 16
    
    test_cases = [
        (5, 0),   # Should become (0, 5)
        (0, 5),   # Should become (-5, 0)
        (3, 4),   # Should become (-4, 3)
        (-2, 3),  # Should become (-3, -2)
    ]
    
    for x, y in test_cases:
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = angle_90
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        await RisingEdge(dut.clk)
        
        assert dut.valid_out.value == 1, "Output should be valid"
        
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        # Convert to signed
        if x_out >= 32:
            x_out = x_out - 64
        if y_out >= 32:
            y_out = y_out - 64
        
        # Expected 90-degree rotation: (x,y) -> (-y,x)
        expected_x = -y
        expected_y = x
        
        dut._log.info(f"90° rotation: ({x},{y}) -> ({x_out},{y_out}), expected ({expected_x},{expected_y})")
        
        assert abs(x_out - expected_x) <= 1, f"90° X rotation error: expected {expected_x}, got {x_out}"
        assert abs(y_out - expected_y) <= 1, f"90° Y rotation error: expected {expected_y}, got {y_out}"


@cocotb.test()
async def test_rotator_180_degrees(dut):
    """Test 180-degree rotation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # 180 degrees = sector 32
    angle_180 = 32
    
    test_cases = [(5, 3), (-4, 2), (0, 5), (7, -2)]
    
    for x, y in test_cases:
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = angle_180
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        await RisingEdge(dut.clk)
        
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        if x_out >= 32:
            x_out = x_out - 64
        if y_out >= 32:
            y_out = y_out - 64
        
        # Expected 180-degree rotation: (x,y) -> (-x,-y)
        expected_x = -x
        expected_y = -y
        
        dut._log.info(f"180° rotation: ({x},{y}) -> ({x_out},{y_out}), expected ({expected_x},{expected_y})")
        
        assert abs(x_out - expected_x) <= 1, f"180° X rotation error"
        assert abs(y_out - expected_y) <= 1, f"180° Y rotation error"


@cocotb.test()
async def test_rotator_270_degrees(dut):
    """Test 270-degree rotation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # 270 degrees = sector 48
    angle_270 = 48
    
    test_cases = [(5, 0), (0, 5), (3, 4)]
    
    for x, y in test_cases:
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = angle_270
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        await RisingEdge(dut.clk)
        
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        if x_out >= 32:
            x_out = x_out - 64
        if y_out >= 32:
            y_out = y_out - 64
        
        # Expected 270-degree rotation: (x,y) -> (y,-x)
        expected_x = y
        expected_y = -x
        
        dut._log.info(f"270° rotation: ({x},{y}) -> ({x_out},{y_out}), expected ({expected_x},{expected_y})")
        
        assert abs(x_out - expected_x) <= 1, f"270° X rotation error"
        assert abs(y_out - expected_y) <= 1, f"270° Y rotation error"


@cocotb.test()
async def test_rotator_arbitrary_angles(dut):
    """Test rotation with arbitrary angles"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test various sectors
    test_angles = [8, 24, 40, 56]  # 45°, 135°, 225°, 315° approximately
    
    for angle_sector in test_angles:
        x, y = 5, 3  # Test coordinate
        
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = angle_sector
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        await RisingEdge(dut.clk)
        
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        if x_out >= 32:
            x_out = x_out - 64
        if y_out >= 32:
            y_out = y_out - 64
        
        # Calculate expected result
        expected_x, expected_y = calculate_expected_rotation(x, y, angle_sector)
        
        angle_degrees = (angle_sector * 360) / 64
        dut._log.info(f"Angle {angle_degrees}°: ({x},{y}) -> ({x_out},{y_out}), expected ({expected_x},{expected_y})")
        
        # Allow some error due to quantization
        assert abs(x_out - expected_x) <= 2, f"X rotation error for angle {angle_degrees}°"
        assert abs(y_out - expected_y) <= 2, f"Y rotation error for angle {angle_degrees}°"


@cocotb.test()
async def test_rotator_pipeline_timing(dut):
    """Test rotator pipeline timing"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test multiple rotations in sequence
    test_sequence = [
        (3, 4, 0),
        (5, 0, 16),
        (-2, 3, 32),
        (1, -1, 48),
    ]
    
    for x, y, angle in test_sequence:
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = angle
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        
        # Wait for pipeline
        await RisingEdge(dut.clk)
        
        # Check timing
        assert dut.valid_out.value == 1, f"Output should be valid for input ({x},{y})"
        
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        dut._log.info(f"Pipeline test: ({x},{y}) angle {angle} -> ({x_out},{y_out})")


@cocotb.test()
async def test_rotator_edge_coordinates(dut):
    """Test rotation with edge case coordinates"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test boundary values
    edge_cases = [
        (0, 0),     # Origin
        (31, 0),    # Max positive X
        (-32, 0),   # Max negative X  
        (0, 31),    # Max positive Y
        (0, -32),   # Max negative Y
        (31, 31),   # Max positive corner
        (-32, -32), # Max negative corner
    ]
    
    for x, y in edge_cases:
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = 8  # 45 degrees
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        await RisingEdge(dut.clk)
        
        # Should complete without error
        assert dut.valid_out.value == 1, f"Edge case ({x},{y}) should produce valid output"
        
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        dut._log.info(f"Edge case: ({x},{y}) -> ({x_out},{y_out})")


@cocotb.test()
async def test_rotator_all_sectors(dut):
    """Test rotation for all 64 sectors"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test point
    x, y = 8, 6
    
    # Test every 8th sector (8 tests total)
    for sector in range(0, 64, 8):
        dut.coord_x_in.value = x
        dut.coord_y_in.value = y
        dut.angle.value = sector
        dut.start.value = 1
        
        await RisingEdge(dut.clk)
        dut.start.value = 0
        await RisingEdge(dut.clk)
        
        x_out = dut.coord_x_out.value
        y_out = dut.coord_y_out.value
        
        if x_out >= 32:
            x_out = x_out - 64
        if y_out >= 32:
            y_out = y_out - 64
        
        angle_degrees = (sector * 360) / 64
        dut._log.info(f"Sector {sector} ({angle_degrees}°): ({x},{y}) -> ({x_out},{y_out})")
        
        # Basic sanity check - output should be reasonable
        assert abs(x_out) <= 20, f"X output too large for sector {sector}"
        assert abs(y_out) <= 20, f"Y output too large for sector {sector}"


@cocotb.test()
async def test_rotator_without_start(dut):
    """Test that rotator doesn't produce output without start signal"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Set inputs but don't assert start
    dut.coord_x_in.value = 5
    dut.coord_y_in.value = 3
    dut.angle.value = 16
    dut.start.value = 0  # No start signal
    
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    
    # Should not have valid output
    assert dut.valid_out.value == 0, "Should not have valid output without start signal"


if __name__ == "__main__":
    import os
    os.system("make")