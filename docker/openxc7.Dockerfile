# =============================================================================
# openxc7.Dockerfile  --  pinned toolchain for the open-source build
# -----------------------------------------------------------------------------
# Built automatically by scripts/open_build.sh (image tag qkd-openxc7:0.9.8).
# See docs/OPEN_BUILD.md for why these exact versions.
#
#   base image    regymm/openxc7 (community image, not published by openXC7):
#                 Yosys 0.68, prjxray + prjxray-db, pypy3, bbasm. Pinned by digest.
#   nextpnr       openXC7/nextpnr-xilinx tag 0.9.8, rebuilt here. The base image
#                 ships 0.8.2-81-g1743d0f4, whose carry packer breaks this design:
#                 it rejects a BEL on a flip-flop, and any CO[i] with a non-carry
#                 user ends the chain. 0.9.8 keeps each CARRY4 whole.
#   chip database xc7a100t, generated here so it always matches this nextpnr.
#
# Needs about 6 GB of disk and takes several minutes.
# =============================================================================
FROM regymm/openxc7@sha256:35c910739e3a4b40850b85de31ebc8954985a66e982009f09123b70dbb0e75da

# Tag 0.9.8 = commit cbabb65139c3d82f0a05b670b5869e1b512b69e8 (checked below).
RUN cd /nextpnr-xilinx \
 && git fetch --depth 1 origin tag 0.9.8 \
 && git checkout -f 0.9.8 \
 && test "$(git rev-parse HEAD)" = cbabb65139c3d82f0a05b670b5869e1b512b69e8 \
 && git submodule update --init --recursive --depth 1 \
 && cmake -DARCH=xilinx -DBUILD_GUI=OFF -DCMAKE_BUILD_TYPE=Release . \
 && make -j"$(nproc)" nextpnr-xilinx bbasm \
 && install -m755 nextpnr-xilinx bbasm /usr/local/bin/ \
 && nextpnr-xilinx --version

RUN mkdir -p /chipdb \
 && cd /nextpnr-xilinx/xilinx \
 && pypy3 python/bbaexport.py --device xc7a100tcsg324-1 --bba /tmp/xc7a100t.bba \
 && bbasm -l /tmp/xc7a100t.bba /chipdb/xc7a100t.bin \
 && rm -f /tmp/xc7a100t.bba

ENV CHIPDB=/chipdb/xc7a100t.bin \
    PRJXRAY_DB=/nextpnr-xilinx/xilinx/external/prjxray-db/artix7
