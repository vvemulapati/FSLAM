# File: test_pipelined_adder.py
# Cocotb testbench for an N-input pipelined adder with parameterized width and stages

import cocotb
import random
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


def get_param_int(param):
    """
    Retrieve a numeric parameter value from the DUT in a simulator-agnostic way.
    """
    try:
        return int(param.value.integer)
    except AttributeError:
        return int(param.value)


async def reset_dut(dut, cycles=2):
    """
    Reset the DUT by asserting 'rst' for a few clock cycles.
    """
    dut.rst.value = 1
    dut.valid_in.value = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_pipelined_n_input_adder(dut):
    """
    Test a pipelined adder with N inputs, parameterized width, and pipeline stages.
    - Feeds random vectors of N inputs each cycle.
    - Verifies outputs appear after correct latency and match the sum of all inputs.
    """
    # Extract parameters
    WIDTH = get_param_int(dut.DATA_WIDTH)
    STAGES = get_param_int(dut.NUM_STAGES)
    N_INPUTS = get_param_int(dut.N_INPUTS)

    # Start clock
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # Reset DUT
    await reset_dut(dut)

    # Number of test vectors
    NUM_TESTS = 50
    expected_queue = []
    match_count = 0
    total_cycles = NUM_TESTS + STAGES + 10

    for cycle in range(total_cycles):
        if cycle < NUM_TESTS:
            # Generate random inputs
            inputs = [random.randint(0, 2**WIDTH - 1) for _ in range(N_INPUTS)]
            # Drive each input, handle array bus or individual ports
            for idx, val in enumerate(inputs):
                try:
                    # Array port support
                    dut.in_data[idx].value = val
                except Exception:
                    # Fallback to individual ports in0, in1, ...
                    port = getattr(dut, f"in{idx}")
                    port.value = val
            dut.valid_in.value = 1
            expected_queue.append(inputs)
        else:
            dut.valid_in.value = 0

        # Advance clock
        await RisingEdge(dut.clk)

        # Check output when valid
        if int(dut.valid_out.value) == 1:
            assert expected_queue, "No expected data in queue"
            inputs = expected_queue.pop(0)
            # Compute expected sum of up to N_INPUTS
            exp_sum = sum(inputs)
            got = int(dut.sum.value)
            assert got == exp_sum, (
                f"Mismatch at cycle {cycle}: inputs={inputs}, "
                f"expected sum={exp_sum}, got={got}"
            )
            match_count += 1

    # Verify all test vectors produced outputs
    assert match_count == NUM_TESTS, (
        f"Expected {NUM_TESTS} results, got {match_count}"
    )
    dut._log.info(
        f"N-input pipelined adder passed {NUM_TESTS} tests "
        f"(N_INPUTS={N_INPUTS}, WIDTH={WIDTH}, STAGES={STAGES})."
    )

