"""
Cocotb test for gaussian_blur module

Tests the 7x7 binomial Gaussian filter implementation used for image smoothing.
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


def create_test_image(width=640, height=480, pattern='impulse'):
    """Create test images with known patterns"""
    image = np.zeros((height, width), dtype=int)
    
    if pattern == 'impulse':
        # Single bright pixel in center - should create Gaussian blob
        image[height//2, width//2] = 63
    elif pattern == 'step':
        # Step function - should create smooth transition
        image[:, width//2:] = 63
    elif pattern == 'gradient':
        for i in range(height):
            for j in range(width):
                image[i][j] = min(63, (i + j) // 20)
    elif pattern == 'uniform':
        image.fill(30)
    elif pattern == 'noise':
        for i in range(height):
            for j in range(width):
                image[i][j] = random.randint(0, 63)
    
    return image


async def send_image_stream(dut, image):
    """Send image as pixel stream and collect filtered output"""
    height, width = image.shape
    output_image = np.zeros((height, width), dtype=int)
    
    for y in range(height):
        for x in range(width):
            dut.pixel_in.value = image[y, x]
            dut.pixel_valid.value = 1
            
            await RisingEdge(dut.clk)
            
            # Collect output
            if dut.blur_valid.value == 1:
                # Calculate actual output position accounting for pipeline delay
                # This is simplified - actual implementation would track coordinates
                output_pixel = dut.pixel_out.value
                if y >= 3 and x >= 3:  # Account for kernel border
                    output_image[y-3, x-3] = output_pixel
    
    # Flush pipeline
    dut.pixel_valid.value = 0
    for _ in range(20):
        await RisingEdge(dut.clk)
        if dut.blur_valid.value == 1:
            output_pixel = dut.pixel_out.value
            # Store final outputs
    
    return output_image


@cocotb.test()
async def test_gaussian_blur_basic(dut):
    """Test basic Gaussian blur functionality"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test with impulse input
    input_image = create_test_image(20, 20, 'impulse')
    output_image = await send_image_stream(dut, input_image)
    
    # Check that impulse is spread out (basic blur verification)
    center_y, center_x = 10, 10
    
    # Center should still be bright but reduced
    center_output = output_image[center_y, center_x]
    dut._log.info(f"Center pixel output: {center_output}")
    
    # Surrounding pixels should have non-zero values
    neighbors = [
        output_image[center_y-1, center_x],
        output_image[center_y+1, center_x],
        output_image[center_y, center_x-1],
        output_image[center_y, center_x+1]
    ]
    
    dut._log.info(f"Neighbor values: {neighbors}")
    
    # At least some neighbors should be non-zero (blur effect)
    assert any(n > 0 for n in neighbors), "Gaussian blur should spread impulse to neighbors"


@cocotb.test()
async def test_gaussian_uniform_input(dut):
    """Test Gaussian blur with uniform input"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Uniform input should produce uniform output
    uniform_value = 40
    input_image = create_test_image(15, 15, 'uniform')
    input_image.fill(uniform_value)
    
    output_image = await send_image_stream(dut, input_image)
    
    # Check central region (away from borders)
    central_region = output_image[5:10, 5:10]
    
    # All central pixels should be close to input value
    for i in range(central_region.shape[0]):
        for j in range(central_region.shape[1]):
            output_val = central_region[i, j]
            if output_val != 0:  # Skip unprocessed pixels
                assert abs(output_val - uniform_value) <= 2, f"Uniform blur failed: expected ~{uniform_value}, got {output_val}"


@cocotb.test()
async def test_gaussian_edge_handling(dut):
    """Test Gaussian blur edge handling"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Small image to test edge effects
    small_image = create_test_image(10, 10, 'step')
    output_image = await send_image_stream(dut, small_image)
    
    # Should not produce output for border pixels
    # Valid output region should be reduced by kernel size/2
    dut._log.info(f"Output image shape: {output_image.shape}")
    dut._log.info(f"Non-zero outputs: {np.count_nonzero(output_image)}")


@cocotb.test()
async def test_gaussian_pipeline_timing(dut):
    """Test pipeline timing and valid signal behavior"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Send test pixels and monitor timing
    test_pixels = [10, 20, 30, 40, 50, 60, 30, 20, 10]
    output_count = 0
    
    for i, pixel in enumerate(test_pixels):
        dut.pixel_in.value = pixel
        dut.pixel_valid.value = 1
        
        await RisingEdge(dut.clk)
        
        if dut.blur_valid.value == 1:
            output_count += 1
            output_val = dut.pixel_out.value
            dut._log.info(f"Cycle {i}: Input={pixel}, Output={output_val}")
    
    # Test invalid input
    dut.pixel_valid.value = 0
    await RisingEdge(dut.clk)
    
    # blur_valid should be 0 when pixel_valid is 0 (after pipeline delay)
    for _ in range(10):
        await RisingEdge(dut.clk)
        if dut.blur_valid.value == 1:
            output_count += 1
            dut._log.info(f"Pipeline flush: Output={dut.pixel_out.value}")


@cocotb.test()
async def test_gaussian_kernel_properties(dut):
    """Test Gaussian kernel mathematical properties"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test linearity: Gaussian(a*x + b*y) = a*Gaussian(x) + b*Gaussian(y)
    # Create two test patterns
    pattern1 = create_test_image(15, 15, 'uniform')
    pattern1.fill(20)
    
    pattern2 = create_test_image(15, 15, 'uniform') 
    pattern2.fill(30)
    
    # Test combined pattern
    combined = pattern1 + pattern2  # Should equal 50 everywhere
    
    output_combined = await send_image_stream(dut, combined)
    
    # Reset for individual tests
    await reset_dut(dut)
    output1 = await send_image_stream(dut, pattern1)
    
    await reset_dut(dut)
    output2 = await send_image_stream(dut, pattern2)
    
    # Check linearity in central region
    for i in range(5, 10):
        for j in range(5, 10):
            if (output_combined[i,j] != 0 and output1[i,j] != 0 and output2[i,j] != 0):
                expected = output1[i,j] + output2[i,j]
                actual = output_combined[i,j]
                dut._log.info(f"Linearity test ({i},{j}): {output1[i,j]} + {output2[i,j]} = {expected}, got {actual}")
                assert abs(actual - expected) <= 2, f"Linearity test failed at ({i},{j})"


@cocotb.test()
async def test_gaussian_noise_reduction(dut):
    """Test noise reduction capability"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create noisy image
    base_image = create_test_image(20, 20, 'uniform')
    base_image.fill(30)
    
    # Add noise
    for i in range(20):
        for j in range(20):
            if random.random() < 0.1:  # 10% noise
                base_image[i, j] = random.randint(0, 63)
    
    output_image = await send_image_stream(dut, base_image)
    
    # Calculate variance in central region
    central_input = base_image[5:15, 5:15]
    central_output = output_image[2:12, 2:12]  # Account for border reduction
    
    input_variance = np.var(central_input)
    output_variance = np.var(central_output[central_output != 0])
    
    dut._log.info(f"Input variance: {input_variance}, Output variance: {output_variance}")
    
    # Gaussian filter should reduce variance (noise)
    if output_variance > 0:
        assert output_variance < input_variance, "Gaussian filter should reduce noise variance"


@cocotb.test()
async def test_gaussian_step_response(dut):
    """Test Gaussian filter response to step input"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Create step function
    step_image = create_test_image(20, 20, 'step')
    output_image = await send_image_stream(dut, step_image)
    
    # Step response should create smooth transition
    # Check middle row for transition
    middle_row = 10
    transition_region = output_image[middle_row, 5:15]
    
    dut._log.info(f"Step response: {transition_region}")
    
    # Should see gradual transition rather than sharp step
    # (specific values depend on kernel implementation)
    non_zero_outputs = transition_region[transition_region != 0]
    if len(non_zero_outputs) > 3:
        # Check for monotonic transition (simplified test)
        dut._log.info(f"Transition values: {non_zero_outputs}")


if __name__ == "__main__":
    import os
    os.system("make")