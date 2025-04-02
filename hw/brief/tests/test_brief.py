"""
Cocotb test for BRIEF (Binary Robust Independent Elementary Features) module

Tests the rotation-invariant descriptor generation functionality.
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


def create_test_window(size=37, pattern='gradient'):
    """Create a test window with known pattern"""
    window = np.zeros((size, size), dtype=int)
    
    if pattern == 'gradient':
        for i in range(size):
            for j in range(size):
                window[i][j] = min(63, (i + j) * 2)  # 6-bit values
    elif pattern == 'checkerboard':
        for i in range(size):
            for j in range(size):
                window[i][j] = 63 if (i + j) % 2 == 0 else 0
    elif pattern == 'center_bright':
        center = size // 2
        for i in range(size):
            for j in range(size):
                dist = abs(i - center) + abs(j - center)
                window[i][j] = max(0, 63 - dist * 2)
    
    return window


def create_brief_pattern():
    """Create a test BRIEF pattern (256 coordinate pairs)"""
    pattern = []
    window_center = 18  # 37//2
    
    for i in range(256):
        # Generate random but valid coordinates within a 27x27 region
        # (to ensure rotated coordinates stay within 37x37)
        x1 = random.randint(-13, 13)
        y1 = random.randint(-13, 13)
        x2 = random.randint(-13, 13)
        y2 = random.randint(-13, 13)
        pattern.append([x1, y1, x2, y2])
    
    return pattern


async def load_window_buffer(dut, window):
    """Load test data into the window buffer"""
    # In a real implementation, this would interface with the window buffer
    # For simulation, we'll assume the window is already loaded
    size = len(window)
    for i in range(size):
        for j in range(size):
            # Set window buffer values (this is implementation-specific)
            # In actual testbench, you'd drive the window_buffer signal
            pass


@cocotb.test()
async def test_brief_basic_functionality(dut):
    """Test basic BRIEF descriptor generation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create test window and pattern
    window = create_test_window(37, 'gradient')
    pattern = create_brief_pattern()
    
    # Set up test inputs
    dut.keypoint_x.value = 18  # Center of 37x37 window
    dut.keypoint_y.value = 18
    dut.orientation.value = 0   # No rotation
    dut.window_valid.value = 1
    
    # Load BRIEF pattern (simplified - would need proper interface)
    for i in range(min(256, len(pattern))):
        # In real testbench, drive pattern_coords signal
        pass
    
    # Start BRIEF generation
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    
    # Wait for completion
    timeout = 0
    while timeout < 1000:  # Timeout after 1000 cycles
        await RisingEdge(dut.clk)
        if dut.descriptor_valid.value == 1:
            break
        timeout += 1
    
    assert timeout < 1000, "BRIEF generation timed out"
    assert dut.descriptor_valid.value == 1, "Descriptor should be valid"
    assert dut.busy.value == 0, "Module should not be busy when done"
    
    # Check descriptor length
    descriptor = dut.descriptor.value
    dut._log.info(f"Generated descriptor: {bin(descriptor)}")
    
    # Descriptor should be a 256-bit value
    assert descriptor.bit_length() <= 256, "Descriptor should be 256 bits or less"


@cocotb.test()
async def test_brief_with_rotation(dut):
    """Test BRIEF descriptor generation with different orientations"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test different orientations
    orientations = [0, 16, 32, 48]  # Different sectors in 64-sector space
    descriptors = []
    
    for orientation in orientations:
        # Reset for new test
        await RisingEdge(dut.clk)
        
        # Set up inputs
        dut.keypoint_x.value = 18
        dut.keypoint_y.value = 18
        dut.orientation.value = orientation
        dut.window_valid.value = 1
        
        # Start generation
        dut.start.value = 1
        await RisingEdge(dut.clk)
        dut.start.value = 0
        
        # Wait for completion
        timeout = 0
        while timeout < 1000:
            await RisingEdge(dut.clk)
            if dut.descriptor_valid.value == 1:
                break
            timeout += 1
        
        assert timeout < 1000, f"BRIEF generation timed out for orientation {orientation}"
        
        descriptor = dut.descriptor.value
        descriptors.append(descriptor)
        dut._log.info(f"Orientation {orientation}: descriptor = {hex(descriptor)}")
        
        # Wait a few cycles before next test
        for _ in range(5):
            await RisingEdge(dut.clk)
    
    # Descriptors should be different for different orientations (unless symmetric pattern)
    # This is a basic sanity check
    unique_descriptors = len(set(descriptors))
    assert unique_descriptors >= 2, "Different orientations should produce different descriptors"


@cocotb.test()
async def test_brief_busy_signal(dut):
    """Test the busy signal behavior"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Initially not busy
    assert dut.busy.value == 0, "Should not be busy initially"
    
    # Set up inputs
    dut.keypoint_x.value = 18
    dut.keypoint_y.value = 18
    dut.orientation.value = 0
    dut.window_valid.value = 1
    
    # Start generation
    dut.start.value = 1
    await RisingEdge(dut.clk)
    
    # Should be busy now
    assert dut.busy.value == 1, "Should be busy after start"
    
    dut.start.value = 0
    
    # Wait for completion while monitoring busy signal
    while dut.busy.value == 1:
        await RisingEdge(dut.clk)
        assert dut.descriptor_valid.value == 0, "descriptor_valid should be 0 while busy"
    
    # After completion
    assert dut.busy.value == 0, "Should not be busy when done"
    assert dut.descriptor_valid.value == 1, "descriptor_valid should be asserted when done"


@cocotb.test()
async def test_brief_invalid_window(dut):
    """Test behavior with invalid window"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Set up inputs with invalid window
    dut.keypoint_x.value = 18
    dut.keypoint_y.value = 18
    dut.orientation.value = 0
    dut.window_valid.value = 0  # Invalid window
    
    # Try to start generation
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0
    
    # Should not start processing with invalid window
    await RisingEdge(dut.clk)
    assert dut.busy.value == 0, "Should not be busy with invalid window"


@cocotb.test()
async def test_brief_multiple_requests(dut):
    """Test multiple BRIEF generation requests"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test multiple keypoints
    keypoints = [(10, 10), (18, 18), (25, 25)]
    
    for i, (kx, ky) in enumerate(keypoints):
        # Set up inputs
        dut.keypoint_x.value = kx
        dut.keypoint_y.value = ky
        dut.orientation.value = i * 8  # Different orientations
        dut.window_valid.value = 1
        
        # Start generation
        dut.start.value = 1
        await RisingEdge(dut.clk)
        dut.start.value = 0
        
        # Wait for completion
        timeout = 0
        while timeout < 1000:
            await RisingEdge(dut.clk)
            if dut.descriptor_valid.value == 1:
                break
            timeout += 1
        
        assert timeout < 1000, f"BRIEF generation timed out for keypoint {i}"
        
        descriptor = dut.descriptor.value
        dut._log.info(f"Keypoint {i} ({kx},{ky}): descriptor = {hex(descriptor)}")
        
        # Wait for valid to deassert
        await RisingEdge(dut.clk)


@cocotb.test()
async def test_brief_edge_coordinates(dut):
    """Test BRIEF generation near window edges"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test keypoints near edges (within valid range)
    edge_keypoints = [(5, 5), (5, 30), (30, 5), (30, 30)]
    
    for i, (kx, ky) in enumerate(edge_keypoints):
        # Set up inputs
        dut.keypoint_x.value = kx
        dut.keypoint_y.value = ky
        dut.orientation.value = 0
        dut.window_valid.value = 1
        
        # Start generation
        dut.start.value = 1
        await RisingEdge(dut.clk)
        dut.start.value = 0
        
        # Wait for completion
        timeout = 0
        while timeout < 1000:
            await RisingEdge(dut.clk)
            if dut.descriptor_valid.value == 1:
                break
            timeout += 1
        
        assert timeout < 1000, f"BRIEF generation timed out for edge keypoint {i}"
        
        descriptor = dut.descriptor.value
        dut._log.info(f"Edge keypoint {i} ({kx},{ky}): descriptor = {hex(descriptor)}")


if __name__ == "__main__":
    import os
    os.system("make")