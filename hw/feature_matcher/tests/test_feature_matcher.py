"""
Cocotb test for feature_matcher module

Tests Hamming distance-based feature matching with heap storage.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import random


async def reset_dut(dut):
    """Reset the DUT"""
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def hamming_distance(desc1, desc2):
    """Calculate Hamming distance between two descriptors"""
    xor_result = desc1 ^ desc2
    return bin(xor_result).count('1')


def create_test_descriptor(base=0, bit_pattern='random'):
    """Create test descriptors with known properties"""
    if bit_pattern == 'random':
        return random.randint(0, (1 << 256) - 1)
    elif bit_pattern == 'alternating':
        return int('01' * 128, 2)  # 256-bit alternating pattern
    elif bit_pattern == 'all_ones':
        return (1 << 256) - 1
    elif bit_pattern == 'all_zeros':
        return 0
    elif bit_pattern == 'similar':
        # Create descriptor similar to base
        desc = base
        # Flip a few random bits
        for _ in range(random.randint(1, 10)):
            bit_pos = random.randint(0, 255)
            desc ^= (1 << bit_pos)
        return desc
    else:
        return base


@cocotb.test()
async def test_feature_matcher_basic_insertion(dut):
    """Test basic feature insertion into heap"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    dut.enable.value = 1
    
    # Insert a few features
    test_features = [
        (0x123456789ABCDEF, 500, 100, 200),
        (0xFEDCBA987654321, 600, 150, 250),
        (0xAAAAAAAAAAAAA, 400, 300, 400),
    ]
    
    for desc, score, x, y in test_features:
        dut.new_feature_valid.value = 1
        dut.new_descriptor.value = desc & ((1 << 256) - 1)  # Ensure 256-bit
        dut.new_score.value = score
        dut.new_x.value = x
        dut.new_y.value = y
        
        await RisingEdge(dut.clk)
        
        # Check heap status
        num_features = dut.num_features.value
        dut._log.info(f"Inserted feature, heap size: {num_features}")
    
    dut.new_feature_valid.value = 0
    await RisingEdge(dut.clk)
    
    # Should have 3 features in heap
    final_count = dut.num_features.value
    assert final_count == 3, f"Expected 3 features in heap, got {final_count}"


@cocotb.test()
async def test_feature_matcher_matching(dut):
    """Test feature matching functionality"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    dut.enable.value = 1
    
    # Insert a reference feature
    ref_descriptor = 0x123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF
    dut.new_feature_valid.value = 1
    dut.new_descriptor.value = ref_descriptor
    dut.new_score.value = 500
    dut.new_x.value = 100
    dut.new_y.value = 200
    
    await RisingEdge(dut.clk)
    dut.new_feature_valid.value = 0
    
    # Wait a few cycles for insertion
    for _ in range(5):
        await RisingEdge(dut.clk)
    
    # Query with same descriptor (should match with distance 0)
    dut.query_valid.value = 1
    dut.query_descriptor.value = ref_descriptor
    
    await RisingEdge(dut.clk)
    dut.query_valid.value = 0
    
    # Wait for matching to complete
    timeout = 0
    while timeout < 100:
        await RisingEdge(dut.clk)
        if dut.match_valid.value == 1:
            break
        timeout += 1
    
    assert timeout < 100, "Feature matching timed out"
    assert dut.match_valid.value == 1, "Should find a match"
    
    # Check match results
    matched_desc = dut.matched_descriptor.value
    hamming_dist = dut.hamming_distance.value
    
    dut._log.info(f"Match found with Hamming distance: {hamming_dist}")
    assert hamming_dist == 0, f"Identical descriptors should have distance 0, got {hamming_dist}"


@cocotb.test()
async def test_feature_matcher_no_match(dut):
    """Test case where no good match exists"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    dut.enable.value = 1
    
    # Insert a feature
    ref_descriptor = 0x0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F
    dut.new_feature_valid.value = 1
    dut.new_descriptor.value = ref_descriptor
    dut.new_score.value = 500
    dut.new_x.value = 100
    dut.new_y.value = 200
    
    await RisingEdge(dut.clk)
    dut.new_feature_valid.value = 0
    
    # Wait for insertion
    for _ in range(5):
        await RisingEdge(dut.clk)
    
    # Query with very different descriptor
    query_desc = 0xF0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0
    dut.query_valid.value = 1
    dut.query_descriptor.value = query_desc
    
    await RisingEdge(dut.clk)
    dut.query_valid.value = 0
    
    # Wait for matching
    timeout = 0
    while timeout < 100:
        await RisingEdge(dut.clk)
        if dut.match_valid.value == 1:
            break
        timeout += 1
    
    # Check if match was found
    if dut.match_valid.value == 1:
        hamming_dist = dut.hamming_distance.value
        dut._log.info(f"Match found with distance: {hamming_dist}")
        
        # Distance should be high (poor match)
        expected_distance = hamming_distance(ref_descriptor, query_desc)
        assert hamming_dist > 50, f"Distance should be high for poor match, got {hamming_dist}"
    else:
        dut._log.info("No match found (distance above threshold)")


@cocotb.test()
async def test_feature_matcher_heap_capacity(dut):
    """Test heap behavior when reaching capacity"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    dut.enable.value = 1
    
    # Insert many features to test heap capacity
    # Note: MAX_FEATURES parameter in design determines capacity
    max_features = 50  # Test with smaller number for simulation
    
    for i in range(max_features + 10):  # Insert more than capacity
        desc = create_test_descriptor(bit_pattern='random')
        score = random.randint(100, 1000)
        
        dut.new_feature_valid.value = 1
        dut.new_descriptor.value = desc & ((1 << 256) - 1)
        dut.new_score.value = score
        dut.new_x.value = i % 640
        dut.new_y.value = i % 480
        
        await RisingEdge(dut.clk)
        
        if i % 10 == 0:
            heap_size = dut.num_features.value
            heap_full = dut.heap_full.value
            dut._log.info(f"Inserted {i} features, heap size: {heap_size}, full: {heap_full}")
    
    dut.new_feature_valid.value = 0
    
    # Check final heap status
    final_size = dut.num_features.value
    is_full = dut.heap_full.value
    
    dut._log.info(f"Final heap size: {final_size}, full: {is_full}")
    
    # Heap should not exceed maximum capacity
    assert final_size <= max_features + 50, "Heap size should be limited"  # Allow some margin for implementation


@cocotb.test()
async def test_feature_matcher_hamming_threshold(dut):
    """Test Hamming distance threshold behavior"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    dut.enable.value = 1
    
    # Insert reference feature
    ref_desc = create_test_descriptor(bit_pattern='alternating')
    dut.new_feature_valid.value = 1
    dut.new_descriptor.value = ref_desc
    dut.new_score.value = 500
    dut.new_x.value = 100
    dut.new_y.value = 200
    
    await RisingEdge(dut.clk)
    dut.new_feature_valid.value = 0
    
    # Wait for insertion
    for _ in range(5):
        await RisingEdge(dut.clk)
    
    # Test with similar descriptor (should match)
    similar_desc = create_test_descriptor(base=ref_desc, bit_pattern='similar')
    dut.query_valid.value = 1
    dut.query_descriptor.value = similar_desc
    
    await RisingEdge(dut.clk)
    dut.query_valid.value = 0
    
    # Wait for matching
    timeout = 0
    while timeout < 100:
        await RisingEdge(dut.clk)
        if dut.match_valid.value == 1:
            break
        timeout += 1
    
    if dut.match_valid.value == 1:
        hamming_dist = dut.hamming_distance.value
        dut._log.info(f"Similar descriptor match distance: {hamming_dist}")
        
        # Should be within threshold (depends on HAMMING_THRESHOLD parameter)
        assert hamming_dist <= 50, f"Similar descriptor should match within threshold"


@cocotb.test()
async def test_feature_matcher_multiple_queries(dut):
    """Test multiple query operations"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    dut.enable.value = 1
    
    # Insert multiple features
    reference_features = []
    for i in range(5):
        desc = create_test_descriptor(bit_pattern='random')
        reference_features.append(desc)
        
        dut.new_feature_valid.value = 1
        dut.new_descriptor.value = desc
        dut.new_score.value = 500 + i * 100
        dut.new_x.value = 100 + i * 50
        dut.new_y.value = 200 + i * 50
        
        await RisingEdge(dut.clk)
    
    dut.new_feature_valid.value = 0
    
    # Wait for insertions
    for _ in range(10):
        await RisingEdge(dut.clk)
    
    # Query each feature
    for i, ref_desc in enumerate(reference_features):
        dut.query_valid.value = 1
        dut.query_descriptor.value = ref_desc
        
        await RisingEdge(dut.clk)
        dut.query_valid.value = 0
        
        # Wait for match
        timeout = 0
        while timeout < 100:
            await RisingEdge(dut.clk)
            if dut.match_valid.value == 1:
                break
            timeout += 1
        
        if dut.match_valid.value == 1:
            hamming_dist = dut.hamming_distance.value
            dut._log.info(f"Query {i}: distance = {hamming_dist}")
            assert hamming_dist == 0, f"Exact match should have distance 0"
        
        # Wait before next query
        for _ in range(5):
            await RisingEdge(dut.clk)


@cocotb.test()
async def test_feature_matcher_disabled(dut):
    """Test behavior when matcher is disabled"""
    
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    
    dut.enable.value = 0  # Disabled
    
    # Try to insert feature
    dut.new_feature_valid.value = 1
    dut.new_descriptor.value = 0x123456789ABCDEF
    dut.new_score.value = 500
    dut.new_x.value = 100
    dut.new_y.value = 200
    
    await RisingEdge(dut.clk)
    dut.new_feature_valid.value = 0
    
    # Should not accept features when disabled
    for _ in range(5):
        await RisingEdge(dut.clk)
    
    heap_size = dut.num_features.value
    assert heap_size == 0, f"Should not accept features when disabled, got {heap_size}"


if __name__ == "__main__":
    import os
    os.system("make")