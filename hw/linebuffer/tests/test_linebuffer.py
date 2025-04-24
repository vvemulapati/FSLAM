# test_linebuffer.py
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer


@cocotb.test()
async def test_linebuffer_basic(dut):
    """
    Simple sanity check:
    - Feed N = 2*LINE_WIDTH random pixels in.
    - Expect the first LINE_WIDTH cycles: out_valid == 0.
    - For the next LINE_WIDTH cycles: out_valid == 1,
      and out_data[i] == in_data[i - LINE_WIDTH].
    """

    # parameters from the DUT
    LINE_WIDTH = int(dut.LINE_WIDTH.value)

    # start clock @100 MHz
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # reset
    dut.rst.value = 1
    dut.in_valid.value = 0
    for i in range(10):
        await RisingEdge(dut.clk)
    dut.rst.value = 0

    await RisingEdge(dut.clk)

    # prepare a long random stream
    total_samples = 2 * LINE_WIDTH
    stimulus = [random.randint(0, 2**int(dut.DATA_WIDTH.value) - 1)
                for _ in range(total_samples)]

    # capture outputs
    received = []

    # drive the DUT
    for cycle, px in enumerate(stimulus):
        dut.in_valid.value = 1
        dut.in_data.value  = px
        await RisingEdge(dut.clk)

        # record output
        received.append((int(dut.out_valid.value), int(dut.out_data.value)))

    # drain any remaining
    dut.in_valid.value = 0
    await Timer(10, units="ns")

    received.pop(0) # Ignore the first cycle
    # --- check behavior ---
    # First LINE_WIDTH cycles: out_valid should be 0
    for i in range(LINE_WIDTH):
        v, d = received[i]
        assert v == 0, f"Cycle {i}: expected out_valid=0, got {v}"

    # Next LINE_WIDTH cycles: out_valid == 1, data delayed
    for i in range(LINE_WIDTH, 2*LINE_WIDTH - 1):
        v, d = received[i]
        exp = stimulus[i - LINE_WIDTH]
        assert v == 1, f"Cycle {i}: expected out_valid=1, got {v}"
        assert d == exp, f"Cycle {i}: expected out_data={exp}, got {d}"

    cocotb.log.info("linebuffer passed basic delay test")

