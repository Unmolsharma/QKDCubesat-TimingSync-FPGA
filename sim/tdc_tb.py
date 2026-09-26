"""Shared cocotb helpers for the TDC testbenches.

All times inside the testbench are integer femtoseconds (the build uses a
1ns/1fs timescale) to avoid float rounding when placing hits a few ps apart.

Everything these tests measure is SIMULATED against the behavioural delay-line
model in rtl/tdc_delay_line.v. None of it describes real hardware.
"""

import os
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadOnly, RisingEdge, Timer
from cocotb.utils import get_sim_time

PS = 1000                 # fs per ps
NS = 1_000_000            # fs per ns
T_CLK_PS = int(os.environ.get("TDC_T_CLK_PS", "4000"))
T_CLK = T_CLK_PS * PS     # fs
HIT_W_DEFAULT = 12 * NS   # > chain (4.2 ns) + T_clk (4 ns); see README


def now_fs() -> int:
    return int(get_sim_time(unit="fs"))


def out_dir() -> Path:
    """Where tests drop CSVs for the Python analysis (sim/out/<config>)."""
    d = Path(os.environ.get("TDC_OUT", Path(__file__).parent / "out" / "adhoc"))
    d.mkdir(parents=True, exist_ok=True)
    return d


def config_name() -> str:
    return os.environ.get("TDC_CONFIG", "default")


def val(sig) -> int:
    return int(sig.value)


def start_clock(sig):
    Clock(sig, T_CLK, unit="fs").start()


async def pulse_at(hit, t_rise_fs: int, width_fs: int):
    """Drive a single hit pulse at an absolute time (fs)."""
    dt = t_rise_fs - now_fs()
    assert dt > 0, "pulse scheduled in the past"
    await Timer(dt, unit="fs")
    hit.value = 1
    await Timer(width_fs, unit="fs")
    hit.value = 0


async def pulse_train(hit, pulses):
    """pulses: iterable of (t_rise_fs, width_fs), sorted, non-overlapping."""
    for t_rise, width in pulses:
        await pulse_at(hit, t_rise, width)


async def edge_time(clk) -> int:
    """Wait for the next rising clock edge and return its time (fs)."""
    await RisingEdge(clk)
    return now_fs()


async def sample(clk):
    """Rising edge, then ReadOnly so register outputs are settled."""
    await RisingEdge(clk)
    await ReadOnly()
    return now_fs()
