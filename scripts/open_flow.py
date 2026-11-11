#!/usr/bin/env python3
"""
open_flow.py -- helper steps for the open-source build (scripts/open_build.sh).

Standard library only. Two subcommands, run in this order:

  constrain  turn the Vivado XDC plus the chain placement into what
             nextpnr-xilinx 0.9.8 accepts, recording what happens to every line
  check      after routing, prove from the routed netlist and FASM what was built

nextpnr-xilinx's XDC parser understands set_property on ports, create_clock and
set_multicycle_path; everything else is skipped. Fixed placement of a logic cell
is a BEL attribute on that cell in the netlist ("SLICE_X12Y50/AFF"), so that is
what the chain placement becomes. The mapping mirrors constraints/tdc_chain_place.tcl.
"""

import argparse
import csv
import json
import re
import sys
from pathlib import Path

BANNER = (
    "*** PRELIMINARY: open-source flow (Yosys + nextpnr-xilinx). Not a Vivado result, ***\n"
    "*** not measured on hardware. Do not quote these numbers as TDC performance.     ***\n"
)

# Setup relaxation on the capture FFs that stands in for the false path through
# the chain: 20 x 4 ns comfortably exceeds nextpnr's own estimate for the path.
CHAIN_MULTICYCLE = 20

TAP_RE = re.compile(r"(?:^|\.)u_dl\.g_c4\[(\d+)\]\.(?:(u_carry4)|g_ff\[(\d+)\]\.u_cap)$")
CHANNEL_RE = re.compile(r"ch\[(\d+)\]")
PBLOCK_RE = re.compile(r"SLICE_X(\d+)Y(\d+):SLICE_X(\d+)Y(\d+)")
FF_BELS = "ABCD"


class FlowError(Exception):
    pass


# ------------------------------------------------------------- XDC parsing ----
def xdc_statements(path):
    """Yield (first line number, statement): no comments, continuation lines joined."""
    pending, first = "", 0
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.strip()
        if not pending and (not line or line.startswith("#")):
            continue
        first = first if pending else number
        pending += line
        if pending.endswith("\\"):
            pending = pending[:-1]
            continue
        yield first, pending
        pending = ""
    if pending:
        yield first, pending


def ports_in(statement):
    """Port names from [get_ports ...]: `[get_ports {a b[*]}]` gives [a, b[*]]."""
    match = re.search(r"\[get_ports\s+(\{[^}]*\}|\S+?)\]", statement)
    if not match:
        return []
    return match.group(1).strip("{}").split()


class Netlist:
    """Read access to a Yosys JSON netlist."""

    def __init__(self, data, top):
        self.data = data
        self.module = data["modules"][top]
        self.cells = self.module["cells"]
        self.ports = self.module["ports"]

    def port_name(self, name):
        """Name of a port bit as nextpnr sees it: a 1-bit vector has no index."""
        match = re.fullmatch(r"(\w+)\[(\d+)\]", name)
        base, index = (match.group(1), int(match.group(2))) if match else (name, None)
        if base not in self.ports:
            return None
        width = len(self.ports[base]["bits"])
        if index is None:
            return base if width == 1 else None
        if index >= width:
            return None
        return base if width == 1 else f"{base}[{index}]"

    def net_of_bit(self, bit):
        """Shortest visible net name carrying this bit."""
        best = None
        for name, net in self.module["netnames"].items():
            if bit in net["bits"] and not net.get("hide_name"):
                label = name if len(net["bits"]) == 1 else f"{name}[{net['bits'].index(bit)}]"
                if best is None or len(label) < len(best):
                    best = label
        return best

    def clock_net_of_pin(self, cell, pin):
        """Net a cell output clocks through, following one BUFG."""
        if cell not in self.cells:
            return None
        bit = self.cells[cell]["connections"][pin][0]
        return self.net_of_bit(self.buffered(bit))

    def buffered(self, bit):
        for cell in self.cells.values():
            if cell["type"] == "BUFG" and cell["connections"]["I"] == [bit]:
                return cell["connections"]["O"][0]
        return bit

    def mmcm_clkout_period_ns(self, cell, pin, input_period_ns):
        """Period of MMCM output CLKOUT0 (fractional divider) or CLKOUT1..6 (integer)."""
        params = self.cells[cell]["parameters"]
        mult = float(params["CLKFBOUT_MULT_F"])
        if pin == "CLKOUT0":
            out_div = float(params["CLKOUT0_DIVIDE_F"])
        else:
            out_div = int(params.get(f"{pin}_DIVIDE", "1"), 2)
        in_div = int(params.get("DIVCLK_DIVIDE", "1"), 2)
        return input_period_ns * in_div * out_div / mult


def pblock_range(xdc_paths):
    """The chain's slice range: the pblock line build.tcl and the placement use."""
    for path in xdc_paths:
        for _number, statement in xdc_statements(path):
            if statement.startswith("resize_pblock pb_tdc_chain"):
                match = PBLOCK_RE.search(statement)
                if not match:
                    raise FlowError(f"pb_tdc_chain range is not a SLICE range: {statement}")
                return tuple(int(v) for v in match.groups())
    raise FlowError("no `resize_pblock pb_tdc_chain` in the XDC files")


# ------------------------------------------------------------ XDC translate ----
# Status words used in constraints.rpt:
APPLIED = "APPLIED"          # nextpnr or Yosys receives it as written
TRANSLATED = "TRANSLATED"    # same intent, different mechanism
UNNEEDED = "NOT NEEDED"      # no such check or optimisation in this flow
DROPPED = "NOT HONORED"      # dropped, and the intent is not enforced
ABSENT = "NOT IN TOP"        # target does not exist in this top


def translate_statement(statement, netlist, clock_periods):
    """One Vivado XDC statement -> (status, nextpnr XDC lines, note)."""
    s = statement
    if s.startswith("set_property -dict") and "get_ports" in s:
        name = ports_in(s)[0]
        port = netlist.port_name(name)
        if port is None:
            return ABSENT, [], f"port {name}"
        dict_part = re.search(r"-dict\s+(\{[^}]*\})", s).group(1)
        return APPLIED, [f"set_property -dict {dict_part} [get_ports {{{port}}}]"], ""

    if s.startswith("create_clock"):
        period = float(re.search(r"-period\s+([\d.]+)", s).group(1))
        name = re.search(r"-name\s+(\S+)", s).group(1)
        clock_periods[name] = period
        if "get_ports" in s:
            port = netlist.port_name(ports_in(s)[0])
            if port is None:
                return ABSENT, [], f"clock {name}"
            lines = [f"create_clock -name {name} -period {period:.3f} [get_ports {{{port}}}]"]
            # The constraint must also sit on the net behind the buffer.
            bits = netlist.ports[re.sub(r"\[\d+\]$", "", port)]["bits"]
            buffered = netlist.net_of_bit(netlist.buffered(bits[0]))
            if buffered and netlist.buffered(bits[0]) != bits[0]:
                lines.append(f"create_clock -name {name} -period {period:.3f} [get_nets {{{buffered}}}]")
            return APPLIED, lines, ""
        pin = re.search(r"\[get_pins\s+(\S+?)/(\w+)/(\w+)\]", s)
        cell, pin_name = f"{pin.group(1)}.{pin.group(2)}", pin.group(3)
        net = netlist.clock_net_of_pin(cell, pin_name)
        if net is None:
            return ABSENT, [], f"clock {name}: no cell {cell}"
        return (
            TRANSLATED,
            [f"create_clock -name {name} -period {period:.3f} [get_nets {{{net}}}]"],
            f"on net {net} (get_pins is not supported)",
        )

    if s.startswith("create_generated_clock"):
        name = re.search(r"-name\s+(\S+)", s).group(1)
        pin = re.search(r"\[get_pins\s+(\w+)/(\w+)\]", s)
        cell, pin_name = pin.group(1), pin.group(2)
        net = netlist.clock_net_of_pin(cell, pin_name)
        period = netlist.mmcm_clkout_period_ns(cell, pin_name, clock_periods["clk100"])
        clock_periods[name] = period
        return (
            TRANSLATED,
            [f"create_clock -name {name} -period {period:.3f} [get_nets {{{net}}}]"],
            f"period computed from the MMCM parameters, on net {net}",
        )

    if s.startswith("set_max_delay"):
        return DROPPED, [], ("no max-delay constraints in nextpnr: the clk_sys <-> clk_ser crossings are "
                             "listed separately in sta.rpt ('Paths between clocks') and not checked")

    if s.startswith("set_clock_groups"):
        return UNNEEDED, [], "nextpnr only reports the delay of paths between clocks (`Max delay` in nextpnr.log)"

    if s.startswith("set_property ALLOW_COMBINATORIAL_LOOPS"):
        return UNNEEDED, [], "there is no LUTLP-1 DRC here; the ring is built as written (checked after routing)"

    if s.startswith("set_false_path"):
        if "CYINIT" in s:
            return (
                TRANSLATED,
                [f"set_multicycle_path {CHAIN_MULTICYCLE} -setup -to "
                 "[get_cells -hier -filter {NAME =~ *u_dl.g_c4*u_cap}]"],
                f"NOT a real false path: set_false_path is unsupported, so the capture FFs get a "
                f"x{CHAIN_MULTICYCLE} setup multicycle. It stops the path steering placement, but "
                "nextpnr's Fmax line is still computed from it; see 'Timing' in the summary",
            )
        return UNNEEDED, [], "nextpnr does not time paths from or to unconstrained ports"

    if "DONT_TOUCH" in s:
        return DROPPED, [], "Yosys and nextpnr ignore it; the chain and sync stage are checked after routing"
    if "ASYNC_REG" in s:
        return DROPPED, [], ("no MTBF handling and no placement next to the first stage (distance is "
                             "reported); synth -nosrl keeps synchronisers out of SRLs")

    if s.startswith("create_pblock") or s.startswith("add_cells_to_pblock"):
        return TRANSLATED, [], "the chain cells get BEL attributes on the netlist instead"
    if s.startswith("resize_pblock"):
        return TRANSLATED, [], "its corner sets the chain's column and first row, as in tdc_chain_place.tcl"
    if "IS_SOFT" in s:
        return TRANSLATED, [], "BEL attributes are hard"
    if "EXCLUDE_PLACEMENT" in s:
        return DROPPED, [], "other logic may use free sites in the column (counted after routing)"

    if "current_design" in s:
        return DROPPED, [], "bitstream settings are not passed on: uncompressed bitstream, no voltage DRC"

    raise FlowError(f"no translation rule for XDC statement: {s}")


def channel_of(cell_name):
    match = CHANNEL_RE.search(cell_name)
    return int(match.group(1)) if match else 0


def chain_placement(cell_names, region):
    """Cell -> BEL, the same mapping as constraints/tdc_chain_place.tcl.

    CARRY4 #k of channel c goes to SLICE_X<x0+c>Y<y0+k>, BEL CARRY4; the capture
    FF for CO[b] goes to the same slice, BEL {A,B,C,D}FF.
    """
    x0, y0, x1, y1 = region
    found = [(name, TAP_RE.search(name)) for name in cell_names]
    found = [(name, match) for name, match in found if match]
    if not found:
        raise FlowError("no delay-line cells found in the netlist (expected ...u_dl.g_c4[k]...)")
    channels = 1 + max(channel_of(name) for name, _ in found)
    if x1 - x0 + 1 != channels:
        raise FlowError(f"pb_tdc_chain must be exactly {channels} slice column(s) wide, "
                        f"one per channel (got X{x0}..X{x1})")
    placement = {}
    for name, match in found:
        k = int(match.group(1))
        y = y0 + k
        if y > y1:
            raise FlowError(f"{name}: row Y{y} is outside pblock rows Y{y0}..Y{y1}")
        site = f"SLICE_X{x0 + channel_of(name)}Y{y}"
        bel = "CARRY4" if match.group(2) else f"{FF_BELS[int(match.group(3))]}FF"
        placement[name] = f"{site}/{bel}"
    return placement


def apply_placement(netlist, placement):
    """Stamp BEL attributes; returns notes on any connection that had to change."""
    notes = []
    for name, bel in placement.items():
        netlist.cells[name]["attributes"]["BEL"] = bel
    for name in placement:
        cell = netlist.cells[name]
        if cell["type"] != "CARRY4" or cell["connections"]["CYINIT"][0] in ("0", "1"):
            continue
        # The chain root: S[0]=1 and DI[0]=0 each become a feed-through LUT in the
        # same LUT pair, both wanting physical pin A1, so nextpnr cannot route them.
        # With S=1 the carry mux selects CI and never looks at DI, so drop that tie.
        if cell["connections"]["DI"][0] != "0" or cell["connections"]["S"][0] != "1":
            raise FlowError(f"{name}: expected DI[0]=0 and S[0]=1 on the chain root")
        cell["connections"]["DI"][0] = "x"
        notes.append(f"{name}: DI[0] tie to 0 removed (don't-care with S[0]=1; carry logic unchanged)")
    return notes


def check_sites_exist(placement, tilegrid_path):
    grid = load_site_grid(tilegrid_path)
    for name, bel in placement.items():
        site = bel.split("/")[0]
        if site not in grid:
            raise FlowError(f"{name}: site {site} does not exist on this device")


def cmd_constrain(args):
    netlist = Netlist(json.loads(Path(args.netlist).read_text()), args.top)

    lines, report, clock_periods = [], [], {}
    for path in args.xdc:
        for number, statement in xdc_statements(path):
            status, out, note = translate_statement(statement, netlist, clock_periods)
            lines.extend(out)
            report.append((status, f"{Path(path).name}:{number}", statement, note))

    region = pblock_range(args.xdc)
    placement = chain_placement(netlist.cells, region)
    check_sites_exist(placement, args.tilegrid)
    notes = apply_placement(netlist, placement)

    Path(args.out_netlist).write_text(json.dumps(netlist.data))
    Path(args.out_xdc).write_text("# generated by scripts/open_flow.py; do not edit\n" + "\n".join(lines) + "\n")

    sites = sorted({bel.split("/")[0] for bel in placement.values()})
    state = {
        "placement": placement,
        "region": region,
        "netlist_notes": notes,
        "pins": pin_requests(args.xdc, netlist),
        "periods": clock_periods,
        "statuses": [status for status, *_ in report],
    }
    Path(args.out_state).write_text(json.dumps(state, indent=1))

    with open(args.out_report, "w") as out:
        out.write(BANNER + "\n")
        out.write("What became of each Vivado constraint in this flow\n\n")
        out.write(f"Chain placement: {len(placement)} BEL attributes, sites {sites[0]} .. {sites[-1]}\n")
        for note in notes:
            out.write(f"Netlist change:  {note}\n")
        out.write("\n")
        for status, where, statement, note in report:
            out.write(f"{status:<12} {where}\n    {statement}\n")
            if note:
                out.write(f"    -> {note}\n")
    print(f"== constraints: {len(placement)} BELs, {len(lines)} XDC lines, see {args.out_report}")


def pin_requests(xdc_paths, netlist):
    """port -> {PACKAGE_PIN, IOSTANDARD, ...} as asked for by the XDC."""
    pins = {}
    for path in xdc_paths:
        for _number, statement in xdc_statements(path):
            if not (statement.startswith("set_property -dict") and "get_ports" in statement):
                continue
            port = netlist.port_name(ports_in(statement)[0])
            if port is None:
                continue
            fields = re.search(r"-dict\s+\{([^}]*)\}", statement).group(1).split()
            pins[port] = dict(zip(fields[0::2], fields[1::2]))
    return pins


# ------------------------------------------------------------------- check ----
def load_site_grid(tilegrid_path):
    """site -> (tile, tile type, site type, clock region) from prjxray's tilegrid."""
    grid = {}
    for tile, info in json.loads(Path(tilegrid_path).read_text()).items():
        for site, site_type in info.get("sites", {}).items():
            grid[site] = (tile, info.get("type"), site_type, info.get("clock_region"))
    return grid


def slice_feature_prefix(site, grid):
    """FASM prefix of a slice, e.g. CLBLM_L_X10Y50.SLICEM_X0."""
    tile, _tile_type, site_type, _region = grid[site]
    x = int(re.match(r"SLICE_X(\d+)Y", site).group(1))
    return f"{tile}.{site_type}_X{x % 2}"


def lut_pin_prefix(site, grid):
    """Tile-level LUT pin prefix: CLBLM_M, CLBLM_L, CLBLL_LL or CLBLL_L."""
    _tile, tile_type, site_type, _region = grid[site]
    x = int(re.match(r"SLICE_X(\d+)Y", site).group(1))
    if tile_type.startswith("CLBLM"):
        return "CLBLM_M" if site_type == "SLICEM" else "CLBLM_L"
    return "CLBLL_LL" if x % 2 == 0 else "CLBLL_L"


def lut_input_is_vcc(fasm, tile, tile_type, pin_prefix, letter):
    """Is pin <letter>1 of this LUT driven from the interconnect tile's VCC_WIRE?"""
    pattern = re.compile(rf"^{re.escape(tile)}\.{pin_prefix}_{letter}1\.\w*?IMUX(?:_[LR])?(\d+)$", re.M)
    match = pattern.search(fasm)
    if not match:
        return False
    side = "L" if tile_type.endswith("_L") else "R"
    coords = re.search(r"_(X\d+Y\d+)$", tile).group(1)
    imux = f"IMUX_L{match.group(1)}" if side == "L" else f"IMUX{match.group(1)}"
    return f"INT_{side}_{coords}.{imux}.VCC_WIRE" in fasm


def check_chain(routed, placement, fasm, grid):
    """Everything tdc_chain_place.tcl guarantees, plus what Vivado does implicitly.

    Returns (ok, report lines). The physical questions are answered from the FASM,
    which is what the bitstream is made from.
    """
    log, ok = [], True
    cells = routed["cells"]
    features = set(line.split(" =")[0].strip() for line in fasm.splitlines())

    # A cell the packer split or renamed has simply gone: that is a failure.
    missing = sorted(name for name in placement if name not in cells)
    if missing:
        log.append(f"FAIL: {len(missing)} chain cells missing after packing, e.g. {missing[0]}")
        return False, log

    wrong = sorted(n for n, bel in placement.items() if cells[n]["attributes"].get("NEXTPNR_BEL") != bel)
    log.append(f"BEL honored: {len(placement) - len(wrong)}/{len(placement)} cells at the requested BEL")
    if wrong:
        ok = False
        got = cells[wrong[0]]["attributes"].get("NEXTPNR_BEL")
        log.append(f"FAIL: {len(wrong)} cells moved, e.g. {wrong[0]} at {got}, asked {placement[wrong[0]]}")

    # One clock for every tap, from a global buffer: the bins assume a common sample.
    clocks = {tuple(cells[n]["connections"]["CK"]) for n, b in placement.items() if not b.endswith("/CARRY4")}
    drivers = {name: cell for name, cell in cells.items() if cell["type"] == "BUFGCTRL"}
    from_global = len(clocks) == 1 and any(
        drivers[d]["connections"].get("O") == list(next(iter(clocks))) for d in drivers)
    log.append(f"{'PASS' if from_global else 'FAIL'}: all capture FFs on one global clock")
    ok = ok and from_global

    by_channel = {}
    for name, bel in placement.items():
        by_channel.setdefault(channel_of(name), {})[name] = bel

    all_regions = set()
    for channel, chain in sorted(by_channel.items()):
        log.append(f"=== channel {channel} ===")
        carries = sorted((int(TAP_RE.search(n).group(1)), b.split("/")[0])
                         for n, b in chain.items() if b.endswith("/CARRY4"))
        rows = [int(site.split("Y")[1]) for _k, site in carries]
        columns = {site.split("Y")[0] for _k, site in carries}
        regions = {grid[site][3] for _k, site in carries}
        all_regions |= regions
        log.append(f"carry4={len(carries)} ffs={len(chain) - len(carries)} "
                   f"sites={carries[0][1]}..{carries[-1][1]} region={sorted(regions)}")
        if len(regions) != 1:
            ok = False
            log.append(f"FAIL: chain spans clock regions {sorted(regions)}")
        if len(columns) != 1:
            ok = False
            log.append(f"FAIL: chain spans columns {sorted(columns)}")
        if rows != list(range(rows[0], rows[0] + len(rows))):
            ok = False
            log.append("FAIL: gap in the chain's rows")

        problems = {"FF D mux = CY (dedicated CO path)": 0, "PRECYINIT": 0, "S=1": 0, "tap leaves through OUTMUX": 0, "carry cascade": 0}
        for k, site in carries:
            prefix = slice_feature_prefix(site, grid)
            tile, tile_type, _site_type, _region = grid[site]
            pin_prefix = lut_pin_prefix(site, grid)
            # Root enters through the slice's AX pin, the rest from the slice below.
            expected = "PRECYINIT.AX" if k == 0 else "PRECYINIT.CIN"
            if f"{prefix}.{expected}" not in features:
                problems["PRECYINIT"] += 1
            for letter in FF_BELS:
                # CO -> FF by the slice's dedicated mux, not through general routing.
                if f"{prefix}.{letter}FFMUX.CY" not in features:
                    problems["FF D mux = CY (dedicated CO path)"] += 1
                if f"{prefix}.{letter}OUTMUX.CY" in features:
                    problems["tap leaves through OUTMUX"] += 1
                # S=1: the LUT passes A1 and A1 is tied to VCC (or the LUT is constant 1).
                init = re.search(rf"^{re.escape(prefix)}\.{letter}LUT\.INIT\[63:0\] = 64'b([01]+)$", fasm, re.M)
                bits = init.group(1).zfill(64) if init else ""
                constant_one = bits == "1" * 64
                passes_a1 = bits == "10" * 32 and lut_input_is_vcc(fasm, tile, tile_type, pin_prefix, letter)
                if not (constant_one or passes_a1):
                    problems["S=1"] += 1
            if k < len(carries) - 1:
                # Cascade net CO[3] -> next CI must be a dedicated connection.
                net = routed_carry_net(routed, cells, chain, k)
                if net is None:
                    problems["carry cascade"] += 1
        for what, count in problems.items():
            status = "PASS" if count == 0 else "FAIL"
            log.append(f"{status}: {what}: {count} problems")
            ok = ok and count == 0

    if len(all_regions) != 1:
        ok = False
        log.append(f"FAIL: channels are in different clock regions: {sorted(all_regions)}")
    return ok, log


def routed_carry_net(routed, cells, chain, k):
    """The net from CARRY4 #k CO3 to CARRY4 #k+1 CIN; None if they are not connected."""
    def by_index(index):
        for name in chain:
            match = TAP_RE.search(name)
            if match.group(2) and int(match.group(1)) == index:
                return cells[name]
        return None

    this, above = by_index(k), by_index(k + 1)
    if this is None or above is None:
        return None
    out, into = this["connections"].get("CO3"), above["connections"].get("CIN")
    if not out or out != into:
        return None
    return out[0]


def bit_users(routed):
    users = {}
    for name, cell in routed["cells"].items():
        for port, bits in cell["connections"].items():
            if cell.get("port_directions", {}).get(port) == "input":
                for bit in bits:
                    users.setdefault(bit, []).append((name, port))
    return users


def check_sync_stage(routed, placement):
    """DONT_TOUCH stand-in: every capture FF must feed exactly one second-stage FF."""
    cells = routed["cells"]
    users = bit_users(routed)
    bad, worst, worst_cell = 0, 0, ""
    for name, bel in placement.items():
        if bel.endswith("/CARRY4"):
            continue
        sinks = users.get(cells[name]["connections"]["Q"][0], [])
        if len(sinks) != 1 or cells[sinks[0][0]]["type"] != "SLICE_FFX" or sinks[0][1] != "D":
            bad += 1
            continue
        sink_bel = cells[sinks[0][0]]["attributes"].get("NEXTPNR_BEL", "")
        here = re.search(r"X(\d+)Y(\d+)", bel)
        there = re.search(r"X(\d+)Y(\d+)", sink_bel)
        if there:
            distance = abs(int(here[1]) - int(there[1])) + abs(int(here[2]) - int(there[2]))
            if distance > worst:
                worst, worst_cell = distance, sinks[0][0]
    return bad, worst, worst_cell


def check_foreign_cells(routed, placement, region):
    """EXCLUDE_PLACEMENT stand-in: other logic inside the reserved pblock slices."""
    x0, y0, x1, y1 = region
    # LUTs the packer made to drive the chain's own S and DI pins belong to the chain.
    chain_inputs = set()
    for name in placement:
        cell = routed["cells"][name]
        if cell["type"] == "CARRY4":
            for port, bits in cell["connections"].items():
                if re.fullmatch(r"(S|DI)\d", port):
                    chain_inputs.update(bits)
    foreign = []
    for name, cell in routed["cells"].items():
        match = re.fullmatch(r"SLICE_X(\d+)Y(\d+)/(\w+)", cell["attributes"].get("NEXTPNR_BEL", ""))
        if not match or name in placement or not (x0 <= int(match[1]) <= x1 and y0 <= int(match[2]) <= y1):
            continue
        outputs = [bit for port, bits in cell["connections"].items()
                   if cell.get("port_directions", {}).get(port) == "output" for bit in bits]
        if not (outputs and set(outputs) <= chain_inputs):
            foreign.append((name, cell["type"], match[0]))
    return foreign


def check_pins(routed, pins, package_pins_path):
    """Requested pin and I/O standard against what was placed."""
    with open(package_pins_path, newline="") as handle:
        site_of_pin = {row["pin"]: row["site"] for row in csv.DictReader(handle)}
    problems = []
    for port, wanted in pins.items():
        cell = routed["cells"].get(port)
        if cell is None:
            problems.append(f"{port}: no pad cell after packing (unused port removed?)")
            continue
        site = cell["attributes"].get("NEXTPNR_BEL", "").split("/")[0]
        expected = site_of_pin.get(wanted["PACKAGE_PIN"])
        if site != expected:
            problems.append(f"{port}: at {site}, pin {wanted['PACKAGE_PIN']} is {expected}")
        got = cell["attributes"].get("IOSTANDARD")
        if got != wanted["IOSTANDARD"]:
            problems.append(f"{port}: IOSTANDARD {got}, wanted {wanted['IOSTANDARD']}")
    return problems


def check_output_drive(routed, pins, fasm, package_pins_path, grid):
    """SLEW / DRIVE requests (only the M1 pulse output has them): are they in the FASM?"""
    with open(package_pins_path, newline="") as handle:
        rows = {row["pin"]: row for row in csv.DictReader(handle)}
    problems = []
    for port, wanted in pins.items():
        if "SLEW" not in wanted and "DRIVE" not in wanted:
            continue
        row = rows[wanted["PACKAGE_PIN"]]
        y = int(re.search(r"Y(\d+)$", row["site"]).group(1))
        prefix = f"{row['tile']}.IOB_Y{y % 2}."
        lines = [line for line in fasm.splitlines() if line.startswith(prefix)]
        if wanted.get("SLEW") == "FAST" and not any(".SLEW.FAST" in line for line in lines):
            problems.append(f"{port}: SLEW FAST not found in the FASM")
        if "DRIVE" in wanted and not any(
            re.search(rf"\.DRIVE\.[\w]*\bI?{wanted['DRIVE']}\b|\.DRIVE\.\w*I{wanted['DRIVE']}(_|$)", line)
            for line in lines
        ):
            problems.append(f"{port}: DRIVE {wanted['DRIVE']} not found in the FASM")
    return problems


def check_ring(routed, top):
    """The ring must be five LUTs in one loop, each driving the next directly."""
    if top != "tdc_arty_top":
        return None
    cells = routed["cells"]
    stages = ["u_ro.u_ro_nand"] + [f"u_ro.g_inv[{i}].u_ro_inv" for i in range(1, 5)]
    for name in stages:
        if name not in cells:
            return f"ring stage missing after synthesis: {name}"
    driver = {}
    for name, cell in cells.items():
        for port, bits in cell["connections"].items():
            if cell.get("port_directions", {}).get(port) == "output":
                for bit in bits:
                    driver[bit] = name
    for index, name in enumerate(stages):
        previous = stages[index - 1]
        drivers = {driver.get(bit) for port, bits in cells[name]["connections"].items()
                   if cells[name].get("port_directions", {}).get(port) == "input" for bit in bits}
        if previous not in drivers:
            return (f"ring stage {name} is not driven directly by {previous} "
                    f"(got {sorted(str(d) for d in drivers)}); a buffer in the loop changes the ring")
    return ""


def mmcm_from_fasm(fasm):
    """f_out = 100 MHz * M / (D * O) from the counters in the FASM, or None."""
    def counter(name, field):
        match = re.search(rf"MMCME2_ADV\.{name}_{field}\[5:0\] = 6'b([01]+)", fasm)
        return int(match.group(1), 2) if match else None

    high_fb, low_fb = counter("CLKFBOUT_CLKOUT1", "HIGH_TIME"), counter("CLKFBOUT_CLKOUT1", "LOW_TIME")
    high_0, low_0 = counter("CLKOUT0_CLKOUT1", "HIGH_TIME"), counter("CLKOUT0_CLKOUT1", "LOW_TIME")
    if None in (high_fb, low_fb, high_0, low_0):
        return None
    return {"M": high_fb + low_fb, "O": high_0 + low_0}


def worst_slack_ps(log_text):
    """Lowest slack in the last post-route histogram.

    The histogram is per endpoint and does apply the multicycle factor, so unlike
    nextpnr's 'Max frequency' line it is not hidden by the path through the chain.
    Its first bin starts at the minimum slack over all endpoints in all domains.
    """
    sections = log_text.split("Slack histogram:")
    if len(sections) < 2:
        return None
    # Bins look like "[ -3239,    708) |***"; the legend above them also has a "[1,17)".
    match = re.search(r"\[\s*(-?\d+)\s*,\s*-?\d+\)\s*\|", sections[-1])
    return int(match.group(1)) if match else None


def chain_path_is_critical(log_text):
    """Is nextpnr's post-route critical path for clk the (false) path through the chain?

    The path is chosen by largest raw arrival, so it hides every real path behind it.
    """
    sections = log_text.split("Critical path report for clock 'clk'")
    if len(sections) < 2:
        return False
    return "u_dl.g_c4" in sections[-1].split("ns routing")[0]


def utilization_lines(report):
    wanted = ["SLICE_LUTX", "SLICE_FFX", "CARRY4", "RAMB36E1_RAMB36E1", "RAMB18E1_RAMB18E1",
              "DSP48E1_DSP48E1", "BUFGCTRL", "MMCME2_ADV_MMCME2_ADV"]
    util = report.get("utilization", {})
    return [f"  {name:<24} {util[name]['used']:>6} / {util[name]['available']}"
            for name in wanted if name in util and util[name]["used"]]


def cmd_check(args):
    state = json.loads(Path(args.state).read_text())
    routed_data = json.loads(Path(args.routed).read_text())
    # nextpnr's written netlist has a single module, named "top" whatever the design.
    routed = next(iter(routed_data["modules"].values()))
    fasm = Path(args.fasm).read_text()
    log_text = Path(args.nextpnr_log).read_text()
    report = json.loads(Path(args.nextpnr_report).read_text())
    grid = load_site_grid(args.tilegrid)
    placement, pins, region = state["placement"], state["pins"], state["region"]

    # Fatal: the chain is the reason this design exists. No bitstream without it.
    chain_ok, chain_log = check_chain(routed, placement, fasm, grid)
    sync_bad, sync_worst, sync_cell = check_sync_stage(routed, placement)
    if sync_bad:
        chain_ok = False
        chain_log.append(f"FAIL: {sync_bad} capture FFs do not feed exactly one second-stage FF")
    else:
        chain_log.append("PASS: every capture FF feeds exactly one second-stage FF")
    chain_log.append(f"second-stage FF distance from its capture FF: worst {sync_worst} sites "
                     f"(ASYNC_REG adjacency not enforced here)")
    ring_problem = check_ring(routed, args.top)
    if ring_problem:
        chain_ok = False
        chain_log.append(f"FAIL: {ring_problem}")

    pin_problems = check_pins(routed, pins, args.package_pins)
    pin_problems += check_output_drive(routed, pins, fasm, args.package_pins, grid)
    mmcm = mmcm_from_fasm(fasm)
    mmcm_ok = mmcm is not None and mmcm["M"] == 10 and mmcm["O"] == 4
    foreign = check_foreign_cells(routed, placement, region)
    overall_ok = chain_ok and not pin_problems and mmcm_ok

    with open(args.out_chain, "w") as out:
        out.write(BANNER + "\n")
        out.write("TDC carry-chain placement, checked from the routed netlist and FASM\n\n")
        out.write("\n".join(chain_log) + "\n")
        out.write(f"foreign cells in reserved pblock slices: {len(foreign)}\n")
        for name, cell_type, bel in foreign[:40]:
            out.write(f"  {bel:<24} {cell_type:<12} {name}\n")
        out.write("RESULT: PASS\n" if chain_ok else "RESULT: FAIL\n")

    # ---- summary ----
    period = state["periods"].get("clk_sys", 4.0)
    slack = worst_slack_ps(log_text)
    fmax_line = report.get("fmax", {}).get("clk", {})
    chain_shadow = chain_path_is_critical(log_text)
    counts = {status: state["statuses"].count(status) for status in sorted(set(state["statuses"]))}

    lines = [BANNER]
    lines.append(f"top             {args.top}")
    lines.append(f"part            {args.part}")
    lines.append(f"lut_file        {args.lut or 'ideal (none)'}")
    lines.append(f"seed            {args.seed}")
    lines.append(f"tools           {args.tools}")
    lines.append("")
    lines.append(f"STRUCTURE       {'PASS' if overall_ok else 'FAIL (no bitstream)'}  "
                 "(what was built; says nothing about speed)")
    if slack is not None and slack < 0:
        lines.append(f"TIMING          NOT MET by {-slack / 1000.0:.3f} ns in nextpnr's model. Bitstream still "
                     "written, as build.tcl does; do not expect reliable behaviour at 250 MHz.")
    lines.append(f"chain placement {'PASS' if chain_ok else 'FAIL'}  (chain_placement.rpt)")
    lines.append(f"I/O pins        {'PASS' if not pin_problems else 'FAIL'}  ({len(pins)} requested)")
    for problem in pin_problems:
        lines.append(f"    {problem}")
    if mmcm:
        lines.append(f"MMCM in FASM    {'PASS' if mmcm_ok else 'FAIL'}  M={mmcm['M']} O={mmcm['O']} -> "
                     f"{100.0 * mmcm['M'] / mmcm['O']:.1f} MHz from 100 MHz")
    else:
        lines.append("MMCM in FASM    FAIL  counters not found")
    lines.append(f"constraints     {counts}  (constraints.rpt)")
    lines.append(f"reserved column {len(foreign)} foreign cells (EXCLUDE_PLACEMENT is not honored)")
    lines.append(f"sync stage      worst {sync_worst} sites from its capture FF (ASYNC_REG is not honored)")
    lines.append("")
    lines.append("Timing (nextpnr's model; hold and clock skew are NOT analysed)")
    if slack is not None:
        lines.append(f"  worst setup slack    {slack / 1000.0:+.3f} ns over all endpoints; "
                     f"capture FFs relaxed x{CHAIN_MULTICYCLE}")
        lines.append(f"  estimated Fmax       {1000.0 / (period - slack / 1000.0):.1f} MHz, "
                     f"if that endpoint is on clk_sys ({period:.3f} ns)")
    if fmax_line:
        note = ""
        if chain_shadow:
            note = "  <- IGNORE: this is the chain false path, not the design"
        lines.append(f"  nextpnr fmax report  {fmax_line.get('achieved', 0):.1f} MHz{note}")
    lines.append("")
    lines.append("Limits (see docs/OPEN_BUILD.md)")
    lines.append("  chain delay / coverage  NOT REPORTED: no equivalent of Vivado's chain_delay.rpt.")
    lines.append("                          Only sat_cnt on hardware or Vivado STA can answer it.")
    lines.append("  Vivado parity           not established; bins will differ from a Vivado build,")
    lines.append("                          so calibrate any bitstream from this flow separately.")
    lines.append("")
    lines.append("Resources")
    lines.extend(utilization_lines(report))
    if state["netlist_notes"]:
        lines.append("")
        lines.append("Changes made to the netlist for this flow (rtl/ and the XDC are untouched)")
        lines.extend(f"  {note}" for note in state["netlist_notes"])
    text = "\n".join(lines) + "\n"
    Path(args.out_summary).write_text(text)
    print(text)

    if not overall_ok:
        raise FlowError(f"post-route checks FAILED, no bitstream: see {args.out_summary}")


# -------------------------------------------------------------------- main ----
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("constrain")
    p.add_argument("--top", required=True)
    p.add_argument("--xdc", action="append", required=True)
    p.add_argument("--netlist", required=True)
    p.add_argument("--tilegrid", required=True)
    p.add_argument("--out-netlist", required=True)
    p.add_argument("--out-xdc", required=True)
    p.add_argument("--out-state", required=True)
    p.add_argument("--out-report", required=True)
    p.set_defaults(func=cmd_constrain)

    p = sub.add_parser("check")
    for name in ("top", "part", "seed", "state", "routed", "fasm", "nextpnr-log", "nextpnr-report",
                 "tilegrid", "package-pins", "tools", "out-chain", "out-summary"):
        p.add_argument(f"--{name}", required=True)
    p.add_argument("--lut", default="")
    p.set_defaults(func=cmd_check)

    args = parser.parse_args(argv)
    try:
        args.func(args)
    except FlowError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
