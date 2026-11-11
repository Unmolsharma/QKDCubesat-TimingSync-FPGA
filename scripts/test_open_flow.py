#!/usr/bin/env python3
"""
Tests for scripts/open_flow.py. No FPGA tools needed:

    python scripts/test_open_flow.py

The checker's job is to refuse a broken chain, so most of these break something on
purpose. Several are regressions for mistakes this flow really made or nearly made.
"""

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

import open_flow as flow  # noqa: E402


def fake_netlist(top_has_ring=True):
    """Just enough of a Yosys netlist for the repository's real XDC files."""
    cells = {
        "u_mmcm": {
            "type": "MMCME2_BASE",
            "parameters": {"CLKFBOUT_MULT_F": "10.000000", "CLKOUT0_DIVIDE_F": "4.000000",
                           "CLKOUT1_DIVIDE": "00000000000000000000000000010100",
                           "DIVCLK_DIVIDE": "00000000000000000000000000000001"},
            "connections": {"CLKOUT0": [10], "CLKOUT1": [14]},
        },
        "u_bufg": {"type": "BUFG", "parameters": {}, "connections": {"I": [10], "O": [11]}},
        "u_bufg_ser": {"type": "BUFG", "parameters": {}, "connections": {"I": [14], "O": [15]}},
        "u_bufg_src": {"type": "BUFG", "parameters": {}, "connections": {"I": [12], "O": [13]}},
    }
    netnames = {"clk": {"bits": [11]}, "clk_mmcm": {"bits": [10]}, "clk_src": {"bits": [13]},
                "clk_ser": {"bits": [15]}}
    if top_has_ring:
        cells["u_ro.u_ro_nand"] = {"type": "LUT2", "parameters": {}, "connections": {"O": [20]}}
        netnames["u_ro.ro"] = {"bits": [20]}
    ports = {
        "CLK100MHZ": {"bits": [1]}, "ck_rst": {"bits": [2]}, "sw": {"bits": [3, 4, 5, 6]},
        "ja": {"bits": [7]}, "jb": {"bits": [12]}, "led": {"bits": [30, 31, 32, 33]},
        "uart_rxd_out": {"bits": [34]}, "emu_out": {"bits": [35]}, "uart_txd_in": {"bits": [36]},
    }
    data = {"modules": {"top": {"cells": cells, "ports": ports, "netnames": netnames}}}
    return flow.Netlist(data, "top")


class XdcReading(unittest.TestCase):
    def test_continuation_lines_are_joined(self):
        # Regression: the second line of set_clock_groups was read as its own statement.
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "a.xdc"
            path.write_text("## comment\nset_clock_groups -asynchronous -group a \\\n    -group b\nfoo\n")
            statements = list(flow.xdc_statements(path))
        self.assertEqual(statements[0], (2, "set_clock_groups -asynchronous -group a -group b"))
        self.assertEqual(statements[1], (4, "foo"))

    def test_every_statement_in_the_repository_xdc_has_a_rule(self):
        for xdc in ("tdc_arty_a7.xdc", "m1_arty_a7.xdc"):
            periods = {"clk100": 10.0}
            for _number, statement in flow.xdc_statements(ROOT / "constraints" / xdc):
                status, _lines, _note = flow.translate_statement(statement, fake_netlist(), periods)
                self.assertIn(status, (flow.APPLIED, flow.TRANSLATED, flow.UNNEEDED, flow.DROPPED, flow.ABSENT))

    def test_unknown_statement_stops_the_build(self):
        with self.assertRaises(flow.FlowError):
            flow.translate_statement("set_input_delay 3 -clock a [get_ports b]", fake_netlist(), {})

    def test_serial_clock_period_from_the_integer_divider(self):
        periods = {"clk100": 10.0}
        status, lines, _note = flow.translate_statement(
            "create_generated_clock -name clk_ser [get_pins u_mmcm/CLKOUT1]", fake_netlist(), periods)
        self.assertEqual(status, flow.TRANSLATED)
        self.assertEqual(lines, ["create_clock -name clk_ser -period 20.000 [get_nets {clk_ser}]"])
        self.assertAlmostEqual(periods["clk_ser"], 20.0)

    def test_max_delay_between_clocks_is_reported_not_honored(self):
        status, lines, _note = flow.translate_statement(
            "set_max_delay -datapath_only -from [get_clocks clk_sys] -to [get_clocks clk_ser] 4.000",
            fake_netlist(), {})
        self.assertEqual((status, lines), (flow.DROPPED, []))


class Translation(unittest.TestCase):
    def translate(self, statement, periods=None):
        return flow.translate_statement(statement, fake_netlist(), periods if periods is not None else {})

    def test_one_bit_vector_port_loses_its_index(self):
        # nextpnr sees `input [0:0] ja` as plain "ja".
        status, lines, _ = self.translate("set_property -dict {PACKAGE_PIN G13 IOSTANDARD LVCMOS33} [get_ports {ja[0]}]")
        self.assertEqual(status, flow.APPLIED)
        self.assertIn("[get_ports {ja}]", lines[0])

    def test_real_vector_port_keeps_its_index(self):
        _, lines, _ = self.translate("set_property -dict {PACKAGE_PIN A8 IOSTANDARD LVCMOS33} [get_ports {sw[0]}]")
        self.assertIn("[get_ports {sw[0]}]", lines[0])

    def test_output_drive_settings_survive(self):
        _, lines, _ = self.translate(
            "set_property -dict {PACKAGE_PIN B11 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports emu_out]")
        self.assertIn("SLEW FAST DRIVE 12", lines[0])

    def test_generated_clock_period_comes_from_the_mmcm(self):
        periods = {"clk100": 10.0}
        status, lines, _ = self.translate("create_generated_clock -name clk_sys [get_pins u_mmcm/CLKOUT0]", periods)
        self.assertEqual(status, flow.TRANSLATED)
        self.assertAlmostEqual(periods["clk_sys"], 4.0)
        self.assertIn("-period 4.000 [get_nets {clk}]", lines[0])

    def test_port_clock_also_constrains_the_buffered_net(self):
        _, lines, _ = self.translate("create_clock -name clk_src -period 10.000 [get_ports {jb[0]}]")
        self.assertIn("[get_nets {clk_src}]", lines[1])

    def test_ring_clock_is_absent_from_a_top_without_a_ring(self):
        status, lines, _ = flow.translate_statement(
            "create_clock -name ro_clk -period 2.500 [get_pins u_ro/u_ro_nand/O]", fake_netlist(False), {})
        self.assertEqual((status, lines), (flow.ABSENT, []))

    def test_false_path_through_the_chain_is_not_called_a_false_path(self):
        status, lines, note = self.translate(
            "set_false_path -through [get_pins -of_objects [get_cells -hier "
            "-filter {REF_NAME == CARRY4}] -filter {REF_PIN_NAME == CYINIT}]")
        self.assertEqual(status, flow.TRANSLATED)
        self.assertIn("set_multicycle_path", lines[0])
        self.assertIn("NOT a real false path", note)


def chain_names(channel=None, carries=48):
    prefix = "u_tdc.u_fe." if channel is None else f"u_multi.ch[{channel}].u_fe."
    names = []
    for k in range(carries):
        names.append(f"{prefix}u_dl.g_c4[{k}].u_carry4")
        names.extend(f"{prefix}u_dl.g_c4[{k}].g_ff[{b}].u_cap" for b in range(4))
    return names


class Placement(unittest.TestCase):
    REGION = (12, 50, 12, 99)

    def test_same_mapping_as_the_vivado_script(self):
        placement = flow.chain_placement(chain_names(), self.REGION)
        self.assertEqual(len(placement), 240)
        self.assertEqual(placement["u_tdc.u_fe.u_dl.g_c4[0].u_carry4"], "SLICE_X12Y50/CARRY4")
        self.assertEqual(placement["u_tdc.u_fe.u_dl.g_c4[47].u_carry4"], "SLICE_X12Y97/CARRY4")
        self.assertEqual(placement["u_tdc.u_fe.u_dl.g_c4[3].g_ff[2].u_cap"], "SLICE_X12Y53/CFF")

    def test_other_cells_are_left_alone(self):
        placement = flow.chain_placement(chain_names() + ["u_uart.foo", "$auto$carry4"], self.REGION)
        self.assertEqual(len(placement), 240)

    def test_channel_selects_the_column(self):
        names = chain_names(0) + chain_names(1)
        placement = flow.chain_placement(names, (12, 50, 13, 99))
        self.assertEqual(placement["u_multi.ch[1].u_fe.u_dl.g_c4[0].u_carry4"], "SLICE_X13Y50/CARRY4")

    def test_pblock_width_must_match_channel_count(self):
        with self.assertRaisesRegex(flow.FlowError, "one per channel"):
            flow.chain_placement(chain_names(0) + chain_names(1), self.REGION)

    def test_chain_must_fit_in_the_rows(self):
        with self.assertRaisesRegex(flow.FlowError, "outside pblock rows"):
            flow.chain_placement(chain_names(), (12, 50, 12, 60))

    def test_no_chain_found_is_an_error(self):
        with self.assertRaises(flow.FlowError):
            flow.chain_placement(["u_uart.foo"], self.REGION)

    def test_only_the_root_loses_its_di_tie(self):
        root = "x.u_dl.g_c4[0].u_carry4"
        next_up = "x.u_dl.g_c4[1].u_carry4"
        netlist = flow.Netlist({"modules": {"top": {"ports": {}, "netnames": {}, "cells": {
            root: {"type": "CARRY4", "attributes": {}, "connections": {
                "CYINIT": [99], "DI": ["0", "0", "0", "0"], "S": ["1", "1", "1", "1"]}},
            next_up: {"type": "CARRY4", "attributes": {}, "connections": {
                "CYINIT": ["0"], "DI": ["0", "0", "0", "0"], "S": ["1", "1", "1", "1"]}},
        }}}}, "top")
        flow.apply_placement(netlist, {root: "SLICE_X12Y50/CARRY4", next_up: "SLICE_X12Y51/CARRY4"})
        self.assertEqual(netlist.cells[root]["connections"]["DI"], ["x", "0", "0", "0"])
        self.assertEqual(netlist.cells[next_up]["connections"]["DI"], ["0", "0", "0", "0"])
        self.assertEqual(netlist.cells[root]["attributes"]["BEL"], "SLICE_X12Y50/CARRY4")

    def test_root_di_is_only_dropped_when_it_really_is_a_dont_care(self):
        root = "x.u_dl.g_c4[0].u_carry4"
        netlist = flow.Netlist({"modules": {"top": {"ports": {}, "netnames": {}, "cells": {
            root: {"type": "CARRY4", "attributes": {}, "connections": {
                "CYINIT": [99], "DI": ["0", "0", "0", "0"], "S": [5, "1", "1", "1"]}},
        }}}}, "top")
        with self.assertRaises(flow.FlowError):
            flow.apply_placement(netlist, {root: "SLICE_X12Y50/CARRY4"})


class GoodChain:
    """A small routed design and FASM that should pass, for tests to break."""

    def __init__(self, carries=3):
        self.placement = flow.chain_placement(chain_names(carries=carries), (12, 50, 12, 99))
        self.grid = {f"SLICE_X12Y{50 + k}": (f"CLBLM_L_X10Y{50 + k}", "CLBLM_L", "SLICEM", "X0Y1")
                     for k in range(carries)}
        self.cells = {"u_bufg": {"type": "BUFGCTRL", "attributes": {}, "connections": {"O": [1]}}}
        lines = []
        for name, bel in self.placement.items():
            k = int(flow.TAP_RE.search(name).group(1))
            attributes = {"NEXTPNR_BEL": bel}
            if bel.endswith("/CARRY4"):
                connections = {"CO3": [100 + k]}
                if k > 0:
                    connections["CIN"] = [100 + k - 1]
                self.cells[name] = {"type": "CARRY4", "attributes": attributes, "connections": connections}
            else:
                self.cells[name] = {"type": "SLICE_FFX", "attributes": attributes, "connections": {"CK": [1]}}
        for k in range(carries):
            prefix = f"CLBLM_L_X10Y{50 + k}.SLICEM_X0"
            lines.append(f"{prefix}.PRECYINIT.{'AX' if k == 0 else 'CIN'}")
            for letter in "ABCD":
                lines.append(f"{prefix}.{letter}FFMUX.CY")
                lines.append(f"{prefix}.{letter}LUT.INIT[63:0] = 64'b" + "10" * 32)
            for letter, imux in zip("ABCD", (7, 15, 32, 40)):
                lines.append(f"CLBLM_L_X10Y{50 + k}.CLBLM_M_{letter}1.CLBLM_IMUX{imux}")
                lines.append(f"INT_L_X10Y{50 + k}.IMUX_L{imux}.VCC_WIRE")
        self.fasm = "\n".join(lines) + "\n"

    def check(self):
        return flow.check_chain({"cells": self.cells}, self.placement, self.fasm, self.grid)


class ChainCheck(unittest.TestCase):
    def assert_fails(self, chain, expected):
        ok, log = chain.check()
        self.assertFalse(ok, "\n".join(log))
        self.assertTrue(any(expected in line and "FAIL" in line for line in log), "\n".join(log))

    def test_good_chain_passes(self):
        ok, log = GoodChain().check()
        self.assertTrue(ok, "\n".join(log))

    def test_cell_moved_by_the_placer(self):
        chain = GoodChain()
        chain.cells["u_tdc.u_fe.u_dl.g_c4[1].g_ff[0].u_cap"]["attributes"]["NEXTPNR_BEL"] = "SLICE_X99Y1/AFF"
        self.assert_fails(chain, "cells moved")

    def test_chain_cell_gone_after_packing(self):
        # What a splitting carry packer does: the CARRY4s are replaced by new cells.
        chain = GoodChain()
        del chain.cells["u_tdc.u_fe.u_dl.g_c4[1].u_carry4"]
        self.assert_fails(chain, "missing after packing")

    def test_tap_reaches_its_ff_through_general_routing(self):
        chain = GoodChain()
        chain.fasm = chain.fasm.replace("CLBLM_L_X10Y51.SLICEM_X0.BFFMUX.CY\n", "CLBLM_L_X10Y51.SLICEM_X0.BFFMUX.BX\n")
        self.assert_fails(chain, "FF D mux")

    def test_tap_also_leaves_the_slice(self):
        chain = GoodChain()
        chain.fasm += "CLBLM_L_X10Y51.SLICEM_X0.COUTMUX.CY\n"
        self.assert_fails(chain, "OUTMUX")

    def test_hit_does_not_enter_through_ax(self):
        chain = GoodChain()
        chain.fasm = chain.fasm.replace("X10Y50.SLICEM_X0.PRECYINIT.AX", "X10Y50.SLICEM_X0.PRECYINIT.C0")
        self.assert_fails(chain, "PRECYINIT")

    def test_chain_restarts_instead_of_cascading(self):
        chain = GoodChain()
        chain.fasm = chain.fasm.replace("X10Y51.SLICEM_X0.PRECYINIT.CIN", "X10Y51.SLICEM_X0.PRECYINIT.AX")
        self.assert_fails(chain, "PRECYINIT")

    def test_cascade_net_broken(self):
        chain = GoodChain()
        chain.cells["u_tdc.u_fe.u_dl.g_c4[1].u_carry4"]["connections"]["CIN"] = [555]
        self.assert_fails(chain, "carry cascade")

    def test_s_input_not_tied_high(self):
        # DI matters as soon as S is not 1: the line would stop passing the edge.
        chain = GoodChain()
        chain.fasm = chain.fasm.replace("INT_L_X10Y50.IMUX_L7.VCC_WIRE\n", "")
        self.assert_fails(chain, "S=1")

    def test_s_lut_with_the_wrong_function(self):
        chain = GoodChain()
        chain.fasm = chain.fasm.replace(
            "X10Y51.SLICEM_X0.CLUT.INIT[63:0] = 64'b" + "10" * 32, "X10Y51.SLICEM_X0.CLUT.INIT[63:0] = 64'b" + "01" * 32)
        self.assert_fails(chain, "S=1")

    def test_capture_ffs_on_two_clocks(self):
        chain = GoodChain()
        chain.cells["u_tdc.u_fe.u_dl.g_c4[1].g_ff[0].u_cap"]["connections"]["CK"] = [2]
        self.assert_fails(chain, "one global clock")

    def test_chain_crossing_a_clock_region(self):
        chain = GoodChain()
        chain.grid["SLICE_X12Y52"] = ("CLBLM_L_X10Y52", "CLBLM_L", "SLICEM", "X0Y2")
        self.assert_fails(chain, "spans clock regions")


class RingCheck(unittest.TestCase):
    STAGES = ["u_ro.u_ro_nand"] + [f"u_ro.g_inv[{i}].u_ro_inv" for i in range(1, 5)]

    def ring(self, buffered=False):
        cells = {}
        for index, name in enumerate(self.STAGES):
            cells[name] = {
                "port_directions": {"I0": "input", "O": "output"},
                "connections": {"I0": [100 + (index - 1) % 5], "O": [100 + index]},
            }
        if buffered:
            cells["bufg"] = {"port_directions": {"I": "input", "O": "output"},
                             "connections": {"I": [100], "O": [200]}}
            cells[self.STAGES[1]]["connections"]["I0"] = [200]
        return {"cells": cells}

    def test_clean_ring(self):
        self.assertEqual(flow.check_ring(self.ring(), "tdc_arty_top"), "")

    def test_buffer_inside_the_ring_is_caught(self):
        # Regression: Yosys's clkbufmap did exactly this.
        self.assertIn("not driven directly", flow.check_ring(self.ring(buffered=True), "tdc_arty_top"))

    def test_top_without_a_ring(self):
        self.assertIsNone(flow.check_ring({"cells": {}}, "m1_arty_top"))


class LogParsing(unittest.TestCase):
    HISTOGRAM = (
        "Info: Slack histogram:\n"
        "Info:  legend: * represents 17 endpoint(s)\n"
        "Info:          + represents [1,17) endpoint(s)\n"
        "Info: [ -3239,    708) |*******+\n"
        "Info: [   708,   4655) |********\n"
    )

    def test_slack_comes_from_the_bins_not_the_legend(self):
        # Regression: this read +0.001 ns from "[1,17)" and reported a false 250 MHz.
        self.assertEqual(flow.worst_slack_ps(self.HISTOGRAM), -3239)

    def test_last_histogram_wins(self):
        self.assertEqual(flow.worst_slack_ps("Slack histogram:\n[ -9, 1) |*\nSlack histogram:\n[ 5, 6) |*\n"), 5)

    def test_no_histogram(self):
        self.assertIsNone(flow.worst_slack_ps("nothing here"))

    def test_chain_path_is_recognised_as_the_false_path(self):
        chain = "Critical path report for clock 'clk' (posedge -> posedge):\n  Net u_dl.g_c4[3] \n 1 ns logic, 2 ns routing\n"
        self.assertTrue(flow.chain_path_is_critical(chain))

    def test_real_path_is_not_flagged(self):
        real = (
            "Critical path report for clock 'clk' (posedge -> posedge):\n  Net u_nco.x\n 1 ns logic, 2 ns routing\n"
            "cross-domain report mentioning u_dl.g_c4[3]\n"
        )
        self.assertFalse(flow.chain_path_is_critical(real))


class Mmcm(unittest.TestCase):
    def test_counters_decoded(self):
        fasm = (
            "T.MMCME2_ADV.CLKFBOUT_CLKOUT1_HIGH_TIME[5:0] = 6'b000101\n"
            "T.MMCME2_ADV.CLKFBOUT_CLKOUT1_LOW_TIME[5:0] = 6'b000101\n"
            "T.MMCME2_ADV.CLKOUT0_CLKOUT1_HIGH_TIME[5:0] = 6'b000010\n"
            "T.MMCME2_ADV.CLKOUT0_CLKOUT1_LOW_TIME[5:0] = 6'b000010\n"
        )
        self.assertEqual(flow.mmcm_from_fasm(fasm), {"M": 10, "O": 4})

    def test_missing_counters(self):
        self.assertIsNone(flow.mmcm_from_fasm(""))


if __name__ == "__main__":
    unittest.main()
