#!/usr/bin/env bash
# =============================================================================
# open_build.sh  --  open-source build for the Arty A7-100T (Yosys + nextpnr-xilinx)
# -----------------------------------------------------------------------------
# PRELIMINARY FLOW. scripts/build.tcl (Vivado) stays the reference build; nothing
# here touches it, and results from this flow must never be quoted as Vivado or
# hardware results. See docs/OPEN_BUILD.md.
#
# Run from WSL or Linux. Same argument style as build.tcl:
#
#   bash scripts/open_build.sh                          (tdc_arty_top)
#   bash scripts/open_build.sh top=m1_arty_top
#   bash scripts/open_build.sh lut=calib/calib_lut.mem
#   bash scripts/open_build.sh seed=3
#
# Uses Docker by default (the image is built on first use). To use tools already
# on PATH instead, set OPEN_NATIVE=1 and also CHIPDB and PRJXRAY_DB.
#
# Outputs, kept apart from Vivado's build/ files:
#   build/open/<top>/<top>_PRELIMINARY.bit     only if the chain checks pass
#   build/open/<top>/reports/summary.txt       start here
#   build/open/<top>/reports/constraints.rpt   every XDC line and what became of it
#   build/open/<top>/reports/chain_placement.rpt, yosys.log, nextpnr.log, nextpnr.json
#
# Exit status: 0 = bitstream written; non-zero = no bitstream.
# =============================================================================
set -euo pipefail

IMAGE=qkd-openxc7:0.9.8
PART=xc7a100tcsg324-1
REQUIRED_NEXTPNR=0.9.8

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

TOP=tdc_arty_top
LUT=""
SEED=1
for arg in "$@"; do
    case "$arg" in
        top=*)  TOP="${arg#top=}" ;;
        lut=*)  LUT="${arg#lut=}" ;;
        seed=*) SEED="${arg#seed=}" ;;
        *) echo "unknown argument: $arg (expected top=, lut= or seed=)" >&2; exit 2 ;;
    esac
done
case "$TOP" in
    tdc_arty_top|m1_arty_top) ;;
    *) echo "unsupported top: $TOP (expected tdc_arty_top or m1_arty_top)" >&2; exit 2 ;;
esac

# ---- outside the container: re-run this script inside it ----
if [ "${OPEN_NATIVE:-0}" != 1 ] && [ "${OPEN_IN_CONTAINER:-0}" != 1 ]; then
    command -v docker >/dev/null || {
        echo "docker not found. See docs/OPEN_BUILD.md, or set OPEN_NATIVE=1." >&2; exit 1; }
    if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        echo "== building $IMAGE (first use, several minutes)"
        docker build -t "$IMAGE" -f "$ROOT/docker/openxc7.Dockerfile" "$ROOT/docker"
    fi
    # Mount at the same path so paths in the logs and reports are the real ones.
    exec docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e OPEN_IN_CONTAINER=1 \
        -v "$ROOT:$ROOT" -w "$ROOT" "$IMAGE" \
        bash "$ROOT/scripts/open_build.sh" "$@"
fi

# ---- the build ----
: "${CHIPDB:?set CHIPDB to the xc7a100t chip database (.bin)}"
: "${PRJXRAY_DB:?set PRJXRAY_DB to prjxray-db/artix7}"

version="$(nextpnr-xilinx --version 2>&1 | sed -n 's/.*(Version \(.*\)).*/\1/p' | head -n1)"
if [ "$version" != "$REQUIRED_NEXTPNR" ]; then
    # Not a formality: other versions either reject the chain placement or
    # quietly split the chain into pieces (docs/OPEN_BUILD.md).
    echo "nextpnr-xilinx is version '$version'; this flow is only tested with $REQUIRED_NEXTPNR." >&2
    [ "${OPEN_ALLOW_ANY_VERSION:-0}" = 1 ] || exit 1
    echo "OPEN_ALLOW_ANY_VERSION=1: continuing anyway; rely on chain_placement.rpt." >&2
fi

OUT="$ROOT/build/open/$TOP"
RPT="$OUT/reports"
rm -rf "$OUT"
mkdir -p "$RPT"
cd "$ROOT"

FLOW=(python3 "$ROOT/scripts/open_flow.py")
XDCS=(--xdc "$ROOT/constraints/tdc_arty_a7.xdc")
if [ "$TOP" = m1_arty_top ]; then
    XDCS+=(--xdc "$ROOT/constraints/m1_arty_a7.xdc")
fi

echo "== PRELIMINARY open-source build: part=$PART top=$TOP lut=${LUT:-<ideal linear>} seed=$SEED"

# ---- 1. synthesis ----
# -nosrl: Yosys ignores ASYNC_REG, and an SRL would swallow a synchroniser chain.
# -noclkbuf: otherwise Yosys puts a BUFG on the ring oscillator's output, inside the
# ring. The tops instantiate the BUFGs they want, so nothing else depends on it.
CHPARAM=""
if [ -n "$LUT" ]; then
    case "$LUT" in /*) ;; *) LUT="$ROOT/$LUT" ;; esac
    [ -f "$LUT" ] || { echo "LUT file not found: $LUT" >&2; exit 1; }
    CHPARAM="chparam -set LUT_FILE \"$LUT\" $TOP"
fi
cat > "$OUT/synth.ys" <<EOF
read_verilog -DSYNTHESIS=1 -defer $ROOT/rtl/*/*.v
$CHPARAM
synth_xilinx -flatten -arch xc7 -nosrl -noclkbuf -top $TOP
write_json $OUT/$TOP.synth.json
tee -o $RPT/utilization_synth.rpt stat
EOF
echo "== yosys"
yosys -q -l "$RPT/yosys.log" -s "$OUT/synth.ys" >/dev/null 2>&1 || {
    tail -n 5 "$RPT/yosys.log" >&2; echo "yosys failed, see $RPT/yosys.log" >&2; exit 1; }

# ---- 2. constraints: Vivado XDC -> nextpnr XDC + BEL attributes on the netlist ----
"${FLOW[@]}" constrain --top "$TOP" "${XDCS[@]}" \
    --netlist "$OUT/$TOP.synth.json" --tilegrid "$PRJXRAY_DB/xc7a100t/tilegrid.json" \
    --out-netlist "$OUT/$TOP.placed.json" --out-xdc "$OUT/$TOP.xdc" \
    --out-state "$OUT/constraints.json" --out-report "$RPT/constraints.rpt"

# ---- 3. place and route ----
# --timing-allow-fail: as in build.tcl, a timing miss is reported, not fatal.
echo "== nextpnr-xilinx"
nextpnr-xilinx --chipdb "$CHIPDB" --xdc "$OUT/$TOP.xdc" --json "$OUT/$TOP.placed.json" \
    --write "$OUT/$TOP.routed.json" --fasm "$OUT/$TOP.fasm" --report "$RPT/nextpnr.json" \
    --seed "$SEED" --timing-allow-fail -q -l "$RPT/nextpnr.log" >/dev/null 2>&1 || {
    grep -E '^ERROR' "$RPT/nextpnr.log" >&2 || tail -n 5 "$RPT/nextpnr.log" >&2
    echo "nextpnr-xilinx failed, no bitstream: see $RPT/nextpnr.log" >&2; exit 1; }

# ---- 4. check what was actually built; no bitstream if the chain is wrong ----
"${FLOW[@]}" check --top "$TOP" --part "$PART" --seed "$SEED" --lut "${LUT:-}" \
    --state "$OUT/constraints.json" --routed "$OUT/$TOP.routed.json" --fasm "$OUT/$TOP.fasm" \
    --nextpnr-log "$RPT/nextpnr.log" --nextpnr-report "$RPT/nextpnr.json" \
    --tilegrid "$PRJXRAY_DB/xc7a100t/tilegrid.json" \
    --package-pins "$PRJXRAY_DB/$PART/package_pins.csv" \
    --tools "$(yosys -V | cut -d' ' -f1-2), nextpnr-xilinx $version" \
    --out-chain "$RPT/chain_placement.rpt" --out-summary "$RPT/summary.txt"

# ---- 5. bitstream ----
echo "== bitstream"
fasm2frames --part "$PART" --db-root "$PRJXRAY_DB" "$OUT/$TOP.fasm" > "$OUT/$TOP.frames" 2>"$RPT/fasm2frames.log"
xc7frames2bit --part_file "$PRJXRAY_DB/$PART/part.yaml" --part_name "$PART" \
    --frm_file "$OUT/$TOP.frames" --output_file "$OUT/${TOP}_PRELIMINARY.bit"

echo "== done (PRELIMINARY: open-source flow, not Vivado, not measured on hardware)"
echo "   $OUT/${TOP}_PRELIMINARY.bit"
echo "   $RPT/summary.txt"
