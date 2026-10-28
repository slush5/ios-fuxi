#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
: "${CLANG:?Set CLANG to an AArch64-capable clang}"
: "${LLD:?Set LLD to ld.lld}"
: "${OBJCOPY:?Set OBJCOPY to llvm-objcopy}"
: "${READELF:?Set READELF to llvm-readelf}"
: "${NM:?Set NM to llvm-nm}"
: "${MKBOOTIMG:?Set MKBOOTIMG to AOSP mkbootimg.py}"
# FUXI_BOARD=fuxi (default, Xiaomi 13) or salami (OnePlus 11: -DFUXI_SALAMI=1, own out dir).
board=${FUXI_BOARD:-fuxi}
[[ $board == fuxi || $board == salami ]] || { echo 'FUXI_BOARD must be fuxi or salami' >&2; exit 1; }
out=../out/$board-probe
mkdir -p "$out"
flags=(--target=aarch64-none-elf -O2 -ffreestanding -fno-builtin -nostdlib
       -fno-pic -fno-stack-protector -mgeneral-regs-only -mstrict-align -Wall -Wextra)
if [[ $board == salami ]]; then flags+=(-DFUXI_SALAMI=1); fi
# Expected ABL SSPP0 format (gpu-handoff.h FG_ABL_SSPP_FORMAT), once a probe boot has measured it.
if [[ -n ${FUXI_ABL_SSPP_FORMAT:-} ]]; then
 [[ $FUXI_ABL_SSPP_FORMAT =~ ^0x[0-9a-fA-F]{1,8}$ ]] || { echo 'FUXI_ABL_SSPP_FORMAT must look like 0x237ff' >&2; exit 1; }
 flags+=("-DFG_ABL_SSPP_FORMAT=${FUXI_ABL_SSPP_FORMAT}u")
fi
"$CLANG" "${flags[@]}" -c start.S -o "$out/start.o"
"$CLANG" "${flags[@]}" -c probe.c -o "$out/probe.o"
if "$READELF" -r "$out/probe.o" | grep -q R_AARCH64_ABS; then
 echo 'Absolute C relocation: Image would not run at the ABL load address' >&2
 exit 1
fi
"$LLD" -T probe.ld --build-id=none "$out/start.o" "$out/probe.o" -o "$out/probe.elf"
"$OBJCOPY" -O binary "$out/probe.elf" "$out/Image"
size=$("$NM" "$out/probe.elf" | awk '$3=="_end" {print $1}')
python3 - "$out/Image" "$size" <<'PY'
import struct, sys
with open(sys.argv[1], 'r+b') as f:
    size = int(sys.argv[2], 16)
    f.truncate(size)
    f.seek(16)
    assert struct.unpack('<Q', f.read(8))[0] == size
PY
python3 "$MKBOOTIMG" --header_version 4 --kernel "$out/Image" \
 --os_version 16.0.0 --os_patch_level 2026-08 -o "$out/$board-probe-boot.img"
ls -lh "$out/Image" "$out/$board-probe-boot.img"
