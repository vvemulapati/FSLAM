"""
Cocotb test for bilinear_interpolator module

Tests the bilinear interpolation functionality used for image scaling
with various fractional coordinates and pixel values.
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


@cocotb.test()
async def test_bilinear_interpolation_basic(dut):
    """Test basic bilinear interpolation functionality"""
    
    # Start clock
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset DUT
    await reset_dut(dut)
    
    # Test case 1: Simple interpolation
    dut.pixel_00.value = 100  # Top-left
    dut.pixel_01.value = 120  # Top-right
    dut.pixel_10.value = 140  # Bottom-left
    dut.pixel_11.value = 160  # Bottom-right
    
    dut.frac_x.value = 2  # x fraction = 2/5
    dut.frac_y.value = 3  # y fraction = 3/5
    dut.valid_in.value = 1
    
    await RisingEdge(dut.clk)
    
    # Wait for output
    await RisingEdge(dut.clk)
    
    # Check if valid output is asserted
    assert dut.valid_out.value == 1, "valid_out should be asserted"
    
    # Calculate expected result manually
    # w00 = (5-2) * (5-3) = 3 * 2 = 6
    # w01 = 2 * (5-3) = 2 * 2 = 4  
    # w10 = (5-2) * 3 = 3 * 3 = 9
    # w11 = 2 * 3 = 6
    # weighted_sum = 100*6 + 120*4 + 140*9 + 160*6 = 600 + 480 + 1260 + 960 = 3300
    # result = 3300 / 25 = 132
    
    expected = 132
    actual = dut.pixel_out.value
    
    dut._log.info(f"Expected: {expected}, Actual: {actual}")
    assert abs(actual - expected) <= 1, f"Interpolation result mismatch: expected {expected}, got {actual}"


@cocotb.test()
async def test_corner_cases(dut):
    """Test corner cases of interpolation"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test case: frac_x = 0, frac_y = 0 (should return top-left pixel)
    dut.pixel_00.value = 50
    dut.pixel_01.value = 100
    dut.pixel_10.value = 150
    dut.pixel_11.value = 200
    
    dut.frac_x.value = 0
    dut.frac_y.value = 0
    dut.valid_in.value = 1
    
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    
    assert dut.valid_out.value == 1
    # w00 = 5*5 = 25, others = 0, result = 50*25/25 = 50
    assert dut.pixel_out.value == 50, f"Corner case failed: expected 50, got {dut.pixel_out.value}"
    
    # Test case: frac_x = 5, frac_y = 5 (should return bottom-right pixel)
    dut.frac_x.value = 5
    dut.frac_y.value = 5
    
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    
    assert dut.valid_out.value == 1
    # w11 = 5*5 = 25, others = 0, result = 200*25/25 = 200
    assert dut.pixel_out.value == 200, f"Corner case failed: expected 200, got {dut.pixel_out.value}"


@cocotb.test()
async def test_pipeline_behavior(dut):
    """Test pipeline behavior with continuous data"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Test multiple interpolations in sequence
    test_data = [
        {"pixels": [10, 20, 30, 40], "frac": [1, 1], "expected": 22},
        {"pixels": [0, 255, 128, 64], "frac": [3, 2], "expected": 115},
        {"pixels": [100, 100, 100, 100], "frac": [2, 4], "expected": 100},
    ]
    
    for i, data in enumerate(test_data):
        dut.pixel_00.value = data["pixels"][0]
        dut.pixel_01.value = data["pixels"][1]
        dut.pixel_10.value = data["pixels"][2]
        dut.pixel_11.value = data["pixels"][3]
        dut.frac_x.value = data["frac"][0]
        dut.frac_y.value = data["frac"][1]
        dut.valid_in.value = 1
        
        await RisingEdge(dut.clk)
        
        # Wait for pipeline delay
        await RisingEdge(dut.clk)
        
        if dut.valid_out.value == 1:
            actual = dut.pixel_out.value
            expected = data["expected"]
            dut._log.info(f"Test {i}: Expected {expected}, Actual {actual}")
            assert abs(actual - expected) <= 2, f"Pipeline test {i} failed"


@cocotb.test()
async def test_invalid_input(dut):
    """Test behavior with invalid input"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Set up pixel values
    dut.pixel_00.value = 50
    dut.pixel_01.value = 100
    dut.pixel_10.value = 150
    dut.pixel_11.value = 200
    dut.frac_x.value = 2
    dut.frac_y.value = 3
    
    # Test with valid_in = 0
    dut.valid_in.value = 0
    
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    
    # valid_out should be 0 when valid_in is 0
    assert dut.valid_out.value == 0, "valid_out should be 0 when valid_in is 0"


@cocotb.test()
async def test_random_interpolation(dut):
    """Test with random values to verify interpolation properties"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    # Generate random test cases
    for _ in range(10):
        # Random pixel values
        p00 = random.randint(0, 255)
        p01 = random.randint(0, 255)
        p10 = random.randint(0, 255)
        p11 = random.randint(0, 255)
        
        # Random fractional coordinates
        fx = random.randint(0, 5)
        fy = random.randint(0, 5)
        
        dut.pixel_00.value = p00
        dut.pixel_01.value = p01
        dut.pixel_10.value = p10
        dut.pixel_11.value = p11
        dut.frac_x.value = fx
        dut.frac_y.value = fy
        dut.valid_in.value = 1
        
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        
        if dut.valid_out.value == 1:
            result = dut.pixel_out.value
            
            # Verify result is within reasonable bounds
            min_pixel = min(p00, p01, p10, p11)
            max_pixel = max(p00, p01, p10, p11)
            
            assert min_pixel <= result <= max_pixel, f"Interpolated value {result} outside bounds [{min_pixel}, {max_pixel}]"
            
            dut._log.info(f"Random test: pixels=[{p00},{p01},{p10},{p11}], frac=[{fx},{fy}], result={result}")


if __name__ == "__main__":
    import os
    os.system("make")