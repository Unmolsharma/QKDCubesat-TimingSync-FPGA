#!/usr/bin/env python3
"""
open_sta.py -- per-endpoint setup timing from nextpnr's SDF (open-source flow).

nextpnr reports one path per clock, picked by largest raw arrival, so one long path
hides everything behind it and set_false_path doesn't exist. This reads the SDF nextpnr
writes (the same delays it uses itself), and does a longest-path analysis that:

  * reports every failing register group, not just the worst path
  * cuts exactly the one intentional false path: the chain root's CYINIT
  * times only same-clock paths; paths between clocks are listed but don't count

Same model limits as nextpnr: setup only, ideal clocks, no hold, and FF clock-to-Q is
100 ps where real -1 silicon is about 450 ps. PRELIMINARY, see docs/OPEN_BUILD.md.

    python scripts/open_sta.py <design.sdf> <routed.json> <design.xdc> [--top N] [--paths N]
"""

import argparse
import json
import re
import sys
from collections import defaultdict, deque
from pathlib import Path

CHAIN_ROOT_RE = re.compile(r"(?:^|\.)u_dl\.g_c4\[0\]\.u_carry4$")
LUT_NAME_RE = re.compile(r"\[\d+\]")


def unescape(name):
    return re.sub(r"\\(.)", r"\1", name)


def split_pin(token):
    """'cell\\/name/PORT' -> (cell, port), splitting on the last unescaped slash."""
    match = re.match(r"^(.*)(?<!\\)/([^/]*)$", token)
    if not match:
        return unescape(token), ""
    return unescape(match.group(1)), unescape(match.group(2))


def triple_max(text):
    return max(float(v) for v in text.strip("()").split(":") if v)


class Sdf:
    """Edges and checks from an SDF. Delays in ps."""

    def __init__(self, text):
        self.interconnect = []             # (from cell, from port, to cell, to port, ps)
        self.iopath = []                   # (cell, from port, to port, ps)
        self.setup = []                    # (cell, data port, clock port, ps)
        cell = None
        for line in text.splitlines():
            line = line.strip()
            if line.startswith("(INSTANCE"):
                cell = unescape(line[len("(INSTANCE"):].rstrip(")").strip())
            elif line.startswith("(INTERCONNECT"):
                fields = line.split()
                src, dst = split_pin(fields[1]), split_pin(fields[2])
                self.interconnect.append((src[0], src[1], dst[0], dst[1], triple_max(fields[3])))
            elif line.startswith("(IOPATH"):
                fields = line.split()
                self.iopath.append((cell, unescape(fields[1]), unescape(fields[2]), triple_max(fields[3])))
            elif line.startswith("(SETUPHOLD"):
                match = re.match(r"\(SETUPHOLD \((\w+) (\S+)\) \((\w+) (\S+)\) (\S+)", line)
                self.setup.append((cell, unescape(match.group(2)), unescape(match.group(4)),
                                   triple_max(match.group(5))))


def clock_periods(xdc_text, netnames):
    """clock net bit -> (clock name, period ns) from the generated nextpnr XDC."""
    periods = {}
    for match in re.finditer(r"create_clock -name (\S+) -period ([\d.]+) \[get_(?:nets|ports) \{([^}]+)\}\]", xdc_text):
        name, period, net = match.group(1), float(match.group(2)), match.group(3)
        base = re.match(r"^(.*?)(?:\[(\d+)\])?$", net)
        entry = netnames.get(net) or netnames.get(base.group(1))
        if entry is None:
            continue
        index = int(base.group(2)) if base.group(2) and net not in netnames else 0
        periods[entry["bits"][index]] = (name, period)
    return periods


class Sta:
    def __init__(self, sdf, routed, xdc_text):
        self.cells = routed["cells"]
        self.netnames = routed["netnames"]
        self.periods = clock_periods(xdc_text, self.netnames)
        self.bit_name = {}
        for name, net in sorted(self.netnames.items(), key=lambda item: (bool(item[1].get("hide_name")), len(item[0]))):
            for index, bit in enumerate(net["bits"]):
                self.bit_name.setdefault(bit, name if len(net["bits"]) == 1 else f"{name}[{index}]")

        self.clock_ports = {(cell, port) for cell, _d, port, _s in sdf.setup}
        self.graph = defaultdict(list)     # node -> [(next node, ps, kind)]
        self.starts = []                   # (output node, clock port cell/port, ps)
        self.ends = defaultdict(list)      # node -> [(clock bit, setup ps)]

        for src_cell, src_port, dst_cell, dst_port, ps in sdf.interconnect:
            self.graph[(src_cell, src_port)].append(((dst_cell, dst_port), ps, "net"))
        for cell, from_port, to_port, ps in sdf.iopath:
            if (cell, from_port) in self.clock_ports or from_port in ("CK", "CLK"):
                clock = self.bit_of(cell, from_port)
                if clock is not None:
                    self.starts.append(((cell, to_port), clock, ps))
                continue
            # The one intentional false path: the hit entering the chain.
            if from_port == "CYINIT" and CHAIN_ROOT_RE.search(cell):
                continue
            self.graph[(cell, from_port)].append(((cell, to_port), ps, "cell"))
        for cell, data_port, clock_port, ps in sdf.setup:
            clock = self.bit_of(cell, clock_port)
            if clock is not None:
                self.ends[(cell, data_port)].append((clock, ps))
        self.order = self.topological_order()

    def bit_of(self, cell, port):
        info = self.cells.get(cell)
        if info is None:
            return None
        bits = info["connections"].get(port)
        return bits[0] if bits else None

    def topological_order(self):
        """Kahn's algorithm, dropping back edges so the ring oscillator can't stall it."""
        nodes = set(self.graph)
        for edges in self.graph.values():
            nodes.update(edge[0] for edge in edges)
        pending = {node: 0 for node in nodes}
        for edges in self.graph.values():
            for target, _ps, _kind in edges:
                pending[target] += 1
        queue = deque(node for node, count in pending.items() if count == 0)
        order = []
        remaining = set(nodes)
        while remaining:
            while queue:
                node = queue.popleft()
                if node not in remaining:
                    continue
                remaining.discard(node)
                order.append(node)
                for target, _ps, _kind in self.graph.get(node, ()):
                    pending[target] -= 1
                    if pending[target] == 0:
                        queue.append(target)
            if remaining:                  # a loop: break it at an arbitrary node
                queue.append(min(remaining))
        return order

    def label(self, cell):
        """A register's RTL name: its output net, else the cell name."""
        info = self.cells.get(cell, {})
        for port in ("Q", "P[0]", "DOADO[0]", "DOBDO[0]"):
            bits = info.get("connections", {}).get(port)
            if bits and bits[0] in self.bit_name:
                return self.bit_name[bits[0]]
        return cell

    def analyse(self):
        """Longest path per launching clock. Returns [(slack ps, record)], worst first."""
        by_clock = defaultdict(list)
        for node, clock, ps in self.starts:
            by_clock[clock].append((node, ps))

        results, crossing = [], []
        for launch, starts in by_clock.items():
            arrival, previous = {}, {}
            for node, ps in starts:
                if ps > arrival.get(node, -1.0):
                    arrival[node] = ps
                    previous[node] = None
            for node in self.order:
                if node not in arrival:
                    continue
                for target, ps, _kind in self.graph.get(node, ()):
                    value = arrival[node] + ps
                    if value > arrival.get(target, -1.0):
                        arrival[target] = value
                        previous[target] = node
            for node, checks in self.ends.items():
                if node not in arrival:
                    continue
                for capture, setup in checks:
                    if capture != launch:
                        crossing.append((arrival[node], launch, capture, node, previous))
                        continue
                    if capture not in self.periods:
                        continue
                    name, period = self.periods[capture]
                    slack = period * 1000.0 - arrival[node] - setup
                    results.append((slack, name, node, previous))
        results.sort(key=lambda record: record[0])
        return results, crossing

    def path(self, node, previous):
        steps = []
        while node is not None:
            steps.append(node)
            node = previous.get(node)
        return list(reversed(steps))


def group_name(label):
    """u_nco.u_lf.tavg[10] -> u_nco.u_lf.tavg"""
    return LUT_NAME_RE.sub("", label)


def report(sta, results, crossing, top, paths):
    lines = ["*** PRELIMINARY: nextpnr's delay model, setup only, ideal clocks, no hold. ***", ""]
    if not results:
        return ["no timed paths"], None
    worst = results[0][0]
    for name in sorted({record[1] for record in results}):
        in_clock = [record for record in results if record[1] == name]
        failing = sum(1 for record in in_clock if record[0] < 0)
        lines.append(f"clock {name:<10} worst slack {in_clock[0][0] / 1000.0:+.3f} ns   "
                     f"{failing} of {len(in_clock)} endpoints failing")
    lines.append("")

    groups = {}
    for slack, name, node, _previous in results:
        group = group_name(sta.label(node[0]))
        entry = groups.setdefault(group, [slack, 0, name])
        entry[1] += 1
    lines.append(f"Worst register groups (top {top}):")
    lines.append(f"  {'slack ns':>9}  {'endpoints':>9}  group")
    for group, (slack, count, name) in sorted(groups.items(), key=lambda item: item[1][0])[:top]:
        lines.append(f"  {slack / 1000.0:+9.3f}  {count:>9}  {group}  [{name}]")
    lines.append("")

    shown = set()
    printed = 0
    for slack, name, node, previous in results:
        group = group_name(sta.label(node[0]))
        if group in shown:
            continue
        shown.add(group)
        steps = sta.path(node, previous)
        lines.append(f"Path to {sta.label(node[0])} ({node[1]})  slack {slack / 1000.0:+.3f} ns  [{name}]")
        start = steps[0]
        lines.append(f"    from {sta.label(start[0])}")
        cells = [step for step in steps if step[1] in ("O", "O5", "O6", "CO3", "OUT")
                 or re.match(r"^(O\d|CO\d|P\[)", step[1])]
        lines.append(f"    {len(steps)} pins, {len(cells)} logic stages")
        printed += 1
        if printed >= paths:
            break
    lines.append("")

    pairs = {}
    for arrival, launch, capture, _node, _previous in crossing:
        key = (launch, capture)
        pairs[key] = max(pairs.get(key, 0.0), arrival)
    if pairs:
        lines.append("Paths between clocks (not counted; synchronised by design):")
        for (launch, capture), arrival in sorted(pairs.items(), key=lambda item: -item[1]):
            lines.append(f"  {sta.bit_name.get(launch, launch)} -> {sta.bit_name.get(capture, capture)}: "
                         f"{arrival / 1000.0:.2f} ns")
    return lines, worst


def analyse_files(sdf_path, routed_path, xdc_path):
    sdf = Sdf(Path(sdf_path).read_text())
    routed = next(iter(json.loads(Path(routed_path).read_text())["modules"].values()))
    sta = Sta(sdf, routed, Path(xdc_path).read_text())
    return (sta, *sta.analyse())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sdf")
    parser.add_argument("routed")
    parser.add_argument("xdc")
    parser.add_argument("--top", type=int, default=30)
    parser.add_argument("--paths", type=int, default=10)
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    sta, results, crossing = analyse_files(args.sdf, args.routed, args.xdc)
    lines, _worst = report(sta, results, crossing, args.top, args.paths)
    text = "\n".join(lines) + "\n"
    if args.out:
        Path(args.out).write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
