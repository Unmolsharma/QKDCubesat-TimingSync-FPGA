"""UART readout tests on the full board wrapper (tdc_arty_top).

The UART line is decoded in Python at 2 Mbaud, and the frames are parsed with
the SAME FrameParser that scripts/capture.py uses on the lab PC. Every frame
is compared with the timestamps seen on tdc_top's internal valid/ready
interface.

Build parameters for this suite (run_tests.py): FIFO_AW=3 (8 entries, so
bursts overflow quickly), STATUS_PERIOD=50000 clk (200 us), RO_DIV_BITS=4.

ALL RESULTS ARE SIMULATED.
"""

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge, Timer

from tdc_tb import NS, PS, now_fs, out_dir, pulse_train, val

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from capture import T_FINE, T_STATUS, T_TS, FrameParser  # noqa: E402

BIT_FS = 125 * 4 * NS          # CLKS_PER_BIT * 4 ns
US = 1000 * NS


class UartTB:
    def __init__(self, dut):
        self.dut = dut
        self.rx_bytes = bytearray()
        self.beats = []            # (coarse, fine) accepted by the readout
        self.framing_errors = 0

    async def start(self, sw):
        # Tests share one simulation, so every test starts by holding the
        # board in reset (clears FIFO, counters and any half-sent UART byte),
        # attaches the monitors, then releases reset.
        d = self.dut
        d.CLK100MHZ.value = 0
        d.ck_rst.value = 0                          # CPU_RESET pressed
        d.sw.value = sw
        d.ja.value = 0
        await Timer(300, unit="ns")                 # MMCM stand-in locks at 100 ns
        await ClockCycles(d.clk, 20)
        assert val(d.uart_rxd_out) == 1, "UART not idle in reset"
        cocotb.start_soon(self._uart_rx())
        cocotb.start_soon(self._beat_monitor())
        await RisingEdge(d.clk)
        d.ck_rst.value = 1
        await ClockCycles(d.clk, 20)
        assert val(d.u_tdc.rst) == 0

    async def _uart_rx(self):
        tx = self.dut.uart_rxd_out
        while True:
            await FallingEdge(tx)
            await Timer(BIT_FS + BIT_FS // 2, unit="fs")   # middle of data bit 0
            b = 0
            for i in range(8):
                b |= val(tx) << i
                await Timer(BIT_FS, unit="fs")
            if val(tx) != 1:
                self.framing_errors += 1
            self.rx_bytes.append(b)

    async def _beat_monitor(self):
        t = self.dut.u_tdc
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            if val(t.m_valid) and val(t.m_ready):
                raw = val(t.m_raw)
                self.beats.append((raw >> 9, raw & 0x1FF))

    def frames(self):
        p = FrameParser()
        fr = p.feed(bytes(self.rx_bytes))
        return fr, p


@cocotb.test()
async def test_uart_timestamp_burst(dut):
    """30 hits 40 ns apart in timestamp mode: the 8-deep FIFO overflows. Frames
    must be well-formed, match the accepted timestamps exactly and in order,
    and accepted + dropped must equal 30 (reported in a STATUS frame)."""
    tb = UartTB(dut)
    await tb.start(sw=0b0010)                         # external hit, timestamp mode
    t0 = now_fs() + 100 * NS
    n = 30
    await cocotb.start_soon(pulse_train(dut.ja, [(t0 + k * 40 * NS, 12 * NS) for k in range(n)]))
    await Timer(900, unit="us")                       # drain + at least one STATUS frame

    frames, p = tb.frames()
    ts = [(f["coarse"], f["fine"]) for t, f in frames if t == T_TS]
    st = [f for t, f in frames if t == T_STATUS]
    dut._log.info("SIMULATED UART burst: %d bytes, %d TS frames, %d STATUS frames, "
                  "last status %s, parser resyncs %d", len(tb.rx_bytes), len(ts), len(st),
                  st[-1] if st else None, p.resyncs)
    assert tb.framing_errors == 0 and p.resyncs == 0 and p.junk == 0
    assert ts == tb.beats, "UART timestamps differ from the TDC output stream"
    assert st, "no STATUS frame"
    assert st[-1]["drop"] > 0, "burst should have overflowed the 8-deep FIFO"
    assert len(ts) + st[-1]["drop"] == n, f"{len(ts)} sent + {st[-1]['drop']} dropped != {n}"
    assert st[-1]["short"] == 0 and st[-1]["sat"] == 0

    # Run the real capture.py CLI offline on the captured bytes: the CSV it
    # writes must contain exactly the fine codes that were sent.
    import capture
    raw = out_dir() / "uart_burst.bin"
    raw.write_bytes(bytes(tb.rx_bytes))
    csv = out_dir() / "uart_burst_codes.csv"
    capture.main(["--raw-in", str(raw), "--out", str(csv),
                  "--ts-out", str(out_dir() / "uart_burst_ts.csv")])
    got = [int(x) for x in csv.read_text().splitlines() if x and not x.startswith("#")]
    assert got == [f for _c, f in ts]


@cocotb.test()
async def test_uart_fine_mode(dut):
    """Fine-code mode, hits spaced so nothing is dropped: every hit arrives as
    a 2-byte FINE frame with the right code."""
    tb = UartTB(dut)
    await tb.start(sw=0b0000)
    t0 = now_fs() + 100 * NS
    pulses = [(t0 + k * 6 * US + (k * 977 * PS) % (4000 * PS), 12 * NS) for k in range(12)]
    await cocotb.start_soon(pulse_train(dut.ja, pulses))
    await Timer(250, unit="us")

    frames, p = tb.frames()
    fines = [f["fine"] for t, f in frames if t == T_FINE]
    st = [f for t, f in frames if t == T_STATUS]
    dut._log.info("SIMULATED UART fine mode: %d FINE frames %s", len(fines), fines)
    assert tb.framing_errors == 0 and p.resyncs == 0
    assert len(fines) == 12
    assert fines == [f for _c, f in tb.beats]
    assert st and st[-1]["drop"] == 0


@cocotb.test()
async def test_ring_osc_source(dut):
    """sw0 = 1 selects the on-chip ring oscillator (simulation stand-in). Hits
    must reach the TDC and come out on the UART. The RO rate far exceeds the
    UART rate here, so drops are expected and must be counted."""
    tb = UartTB(dut)
    await tb.start(sw=0b0001)
    await Timer(300, unit="us")
    frames, p = tb.frames()
    fines = [f["fine"] for t, f in frames if t == T_FINE]
    st = [f for t, f in frames if t == T_STATUS]
    dut._log.info("SIMULATED ring-osc source: %d timestamps accepted by readout, "
                  "%d FINE frames received, last status %s", len(tb.beats), len(fines),
                  st[-1] if st else None)
    assert len(fines) > 10
    assert fines == [f for _c, f in tb.beats][:len(fines)]
    assert st and st[-1]["drop"] > 0 and st[-1]["short"] == 0
