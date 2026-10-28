#!/usr/bin/env bash
set -euo pipefail
payload=$(realpath "${1:?Usage: build.sh FUXI_PAYLOAD}")
cd "$(dirname "$0")"
: "${CLANG:?Set CLANG to an AArch64-capable clang}"
: "${LLD:?Set LLD to ld.lld}"
: "${OBJCOPY:?Set OBJCOPY to llvm-objcopy}"
: "${READELF:?Set READELF to llvm-readelf}"
: "${MKBOOTIMG:?Set MKBOOTIMG to AOSP mkbootimg.py}"
# FUXI_BOARD=fuxi (default, Xiaomi 13) or salami (OnePlus 11: -DFUXI_SALAMI=1, own out dir).
board=${FUXI_BOARD:-fuxi}
[[ $board == fuxi || $board == salami ]] || { echo 'FUXI_BOARD must be fuxi or salami' >&2; exit 1; }
# FUXI_SALAMI_MDSS_STAGE=N (salami only, one digit, probe.c default 0): the staged
# MDSS probe cap of the embedded probe; display requests of a higher risk class
# STALL. A set cap gets its own out dir and image name (-sN).
stage=
if [[ -n ${FUXI_SALAMI_MDSS_STAGE:-} ]]; then
 [[ $board == salami ]] || { echo 'FUXI_SALAMI_MDSS_STAGE is for FUXI_BOARD=salami only' >&2; exit 1; }
 [[ $FUXI_SALAMI_MDSS_STAGE =~ ^[0-9]$ ]] || { echo 'FUXI_SALAMI_MDSS_STAGE must be one digit 0..9' >&2; exit 1; }
 stage=-s$FUXI_SALAMI_MDSS_STAGE
fi
# FUXI_SALAMI_TOUCH_AUTO=0 or 1 (salami only, probe.c default 1): the loader's S3908
# touch auto power right before the XNU handoff. 1 builds the default image; 0 gets
# its own out dir and image name (-ta0).
touch=
if [[ -n ${FUXI_SALAMI_TOUCH_AUTO:-} ]]; then
 [[ $board == salami ]] || { echo 'FUXI_SALAMI_TOUCH_AUTO is for FUXI_BOARD=salami only' >&2; exit 1; }
 [[ $FUXI_SALAMI_TOUCH_AUTO =~ ^[01]$ ]] || { echo 'FUXI_SALAMI_TOUCH_AUTO must be 0 or 1' >&2; exit 1; }
 [[ $FUXI_SALAMI_TOUCH_AUTO == 1 ]] || touch=-ta0
fi
# FUXI_BOOT_LOGO=/abs/mask FUXI_BOOT_LOGO_W=W FUXI_BOOT_LOGO_H=H (salami only): the
# loader fills the ABL splash black and draws this W*H-byte 8-bit coverage mask
# (255 = logo) centred as grey, with no loader text (loader.c boot_logo_draw); pair
# it with an ADT from tools/mkadt-salami.py --no-fbcon. The mask is Apple artwork and
# never enters git: only the ignored out dir receives its bytes (incbin). A mask that
# is not exactly W*H bytes is refused. Own out dir and image name (-logo).
# At the handoff the loader appends " sweetlogo_pa=0x<PA> sweetlogo_w=W sweetlogo_h=H"
# to the XNU command line (PA = Image load address + the mask offset printed below)
# unless the line would pass 607 characters or another loader.c boot_logo_args check
# fails; then nothing is appended, the boot goes on and monitor request 0x48 says why.
logo=
if [[ -n ${FUXI_BOOT_LOGO:-}${FUXI_BOOT_LOGO_W:-}${FUXI_BOOT_LOGO_H:-} ]]; then
 [[ $board == salami ]] || { echo 'FUXI_BOOT_LOGO is for FUXI_BOARD=salami only' >&2; exit 1; }
 [[ ${FUXI_BOOT_LOGO:-} =~ ^/[A-Za-z0-9._/+-]+$ && -f ${FUXI_BOOT_LOGO:-} ]] ||
  { echo 'FUXI_BOOT_LOGO must be an existing absolute file path (letters, digits and ._/+- only)' >&2; exit 1; }
 [[ ${FUXI_BOOT_LOGO_W:-} =~ ^[1-9][0-9]{0,3}$ && ${FUXI_BOOT_LOGO_H:-} =~ ^[1-9][0-9]{0,3}$ ]] ||
  { echo 'FUXI_BOOT_LOGO needs FUXI_BOOT_LOGO_W and FUXI_BOOT_LOGO_H (decimal pixel counts)' >&2; exit 1; }
 (( FUXI_BOOT_LOGO_W <= 1080 && FUXI_BOOT_LOGO_H <= 2412 )) ||
  { echo 'FUXI_BOOT_LOGO is larger than the 1080x2412 salami splash' >&2; exit 1; }
 logo_bytes=$(wc -c < "$FUXI_BOOT_LOGO")
 logo_bytes=${logo_bytes//[[:space:]]/}
 (( logo_bytes == FUXI_BOOT_LOGO_W * FUXI_BOOT_LOGO_H )) ||
  { echo "FUXI_BOOT_LOGO is $logo_bytes bytes, not ${FUXI_BOOT_LOGO_W}x${FUXI_BOOT_LOGO_H} = $((FUXI_BOOT_LOGO_W * FUXI_BOOT_LOGO_H))" >&2; exit 1; }
 logo=-logo
fi
# FUXI_NO_GPU_FW=1 is the only way to build without GPU firmware: empty slots, no pins, and
# stage_gpu_firmware() then never publishes the handoff magic, so the GPU lane stays off.
no_gpu_fw=${FUXI_NO_GPU_FW:-0}
[[ $no_gpu_fw == 0 || $no_gpu_fw == 1 ]] || { echo 'FUXI_NO_GPU_FW must be 0 or 1' >&2; exit 1; }
if [[ $no_gpu_fw == 1 ]]; then
 if [[ -n ${FUXI_GMU_FIRMWARE:-}${FUXI_SQE_FIRMWARE:-}${FUXI_GPU_ZAP_MBN:-} ]]; then
  echo 'FUXI_NO_GPU_FW=1 conflicts with FUXI_GMU_FIRMWARE/FUXI_SQE_FIRMWARE/FUXI_GPU_ZAP_MBN; unset them' >&2; exit 1
 fi
 FUXI_GMU_FIRMWARE='' FUXI_SQE_FIRMWARE='' FUXI_GPU_ZAP_MBN=''
 inputs=("$MKBOOTIMG")
else
 # The pins below are the Xiaomi set; salami needs its own (OnePlus-signed zap) first.
 [[ $board == fuxi ]] || { echo 'FUXI_BOARD=salami has no pinned GPU firmware yet: set FUXI_NO_GPU_FW=1' >&2; exit 1; }
 # GPU-bearing payloads must never silently produce a GPU-disabled image.
 : "${FUXI_GMU_FIRMWARE:?Set the pinned GMU firmware absolute path}"
 : "${FUXI_SQE_FIRMWARE:?Set the pinned sheng-firmware/a740_sqe.fw absolute path}"
 : "${FUXI_GPU_ZAP_MBN:?Set the pinned ZAP MBN absolute path}"
 inputs=("$MKBOOTIMG" "$FUXI_GMU_FIRMWARE" "$FUXI_SQE_FIRMWARE" "$FUXI_GPU_ZAP_MBN")
fi
for input in "${inputs[@]}"; do
 [[ $input == /* && -f $input ]] || { echo 'Packing inputs must be existing absolute file paths' >&2; exit 1; }
done
out=../out/$board-loader$stage$logo$touch
mkdir -p "$out"
# Reject a payload made from a different board, truncated segments, or unsafe RAM.
python3 - "$payload" "$board" <<'PY'
import struct,sys
b=open(sys.argv[1],'rb').read()
assert len(b)>=680 and b[:8]==b'SWEETXNU'
# Native Mac builds embed their DEVELOPMENT_ARM64_FUXI object path; hosted CI
# (darwin-xnu-build) builds do not, so accept an xnu-8796 kernel with Fuxi drivers.
assert b'DEVELOPMENT_ARM64_FUXI' in b or (b'xnu-8796' in b and b'FUXI KEYS:' in b), 'Payload is not a FUXI kernel'
# Only a FUXI_SALAMI kernel (fuxi_keys.c, 0114) carries this string: the kernel's
# board (volume map, panel height) must be the board this loader is built for.
salami=b'FUXI KEYS: salami volume map' in b
assert salami==(sys.argv[2]=='salami'), 'payload kernel board does not match FUXI_BOARD'
version,n=struct.unpack_from('<II',b,8)
assert version==1 and 0<n<=32 and len(b)>=680+32*n
assert struct.unpack_from('<Q',b,56)[0]==0x8c0000000
assert struct.unpack_from('<Q',b,64)[0] in (0x40000000,0x13e000000)
for i in range(n):
 va,vs,off,sz=struct.unpack_from('<4Q',b,680+32*i)
 assert off<=len(b) and sz<=len(b)-off and sz<=vs
PY
# Required pinned firmware input; only the ignored build directory receives bytes.
# With FUXI_NO_GPU_FW=1 all three paths are empty: empty slots, no pin is consulted.
python3 - "$payload" "${FUXI_GMU_FIRMWARE:-}" "${FUXI_SQE_FIRMWARE:-}" "${FUXI_GPU_ZAP_MBN:-}" "$out/gpu-fw.S" <<'FWPY'
import hashlib, json, pathlib, struct, sys
payload, gmu, sqe, zap, dst = sys.argv[1:]
reserved = True  # Both supported windows exclude top-of-bank scratch.
pinned = {
 "gpu": "1a2a419c39046d3141fc5fed5aa7f971de2db40cc7a1d89693c3e26fad64dd98",
 "sqe": "6d38046654333a6a5a4ee77ab67abc24e3c0c5c68237513afe98e94da8fc9fcc",
 "zap": "86e4c3f61d4ddbe581405f4cc496182cf385bee39aaefec61639a31d3619fd35",
}
lines = ['.section .gpu_fw,"a"']
for src, name in ((gmu, 'gpu'), (sqe, 'sqe'), (zap, 'zap')):
    lines += ['.balign 4', f'.global _{name}_fw_start', f'_{name}_fw_start:']
    if src:
        path = pathlib.Path(src).resolve(strict=True)
        size = path.stat().st_size
        assert path.is_file() and 4 < size <= 0x100000 and size % 4 == 0, 'invalid firmware size'
        if reserved:
            assert hashlib.sha256(path.read_bytes()).hexdigest() == pinned[name], 'M6 requires the device-verified firmware'
        if name == 'sqe':
            assert size - 4 <= 0x20000, 'SQE exceeds reserved buffer'
        if name == 'zap':
            assert size <= 0x10000, 'zap MBN exceeds parser bound'
        lines.append('.incbin ' + json.dumps(str(path)))
    lines += [f'.global _{name}_fw_end', f'_{name}_fw_end:']
pathlib.Path(dst).write_text('\n'.join(lines) + '\n')
FWPY
# incbin avoids C absolute pointers in an Image loaded at an arbitrary PA.
printf '.section .payload,"a"\n.incbin "%s"\n' "$payload" > "$out/payload.S"
flags=(--target=aarch64-none-elf -O2 -ffreestanding -fno-builtin -nostdlib
       -fno-pic -fno-stack-protector -mgeneral-regs-only -mstrict-align -Wall -Wextra -Werror -DFUXI=1 -DFUXI_XNU_LOADER=1 -I..)
if [[ $board == salami ]]; then flags+=(-DFUXI_SALAMI=1); fi
if [[ -n $stage ]]; then flags+=("-DFUXI_SALAMI_MDSS_STAGE=$FUXI_SALAMI_MDSS_STAGE"); fi
if [[ -n ${FUXI_SALAMI_TOUCH_AUTO:-} ]]; then flags+=("-DFUXI_SALAMI_TOUCH_AUTO=$FUXI_SALAMI_TOUCH_AUTO"); fi
if [[ -n $logo ]]; then
 flags+=(-DFUXI_BOOT_LOGO=1 "-DFUXI_BOOT_LOGO_W=$FUXI_BOOT_LOGO_W" "-DFUXI_BOOT_LOGO_H=$FUXI_BOOT_LOGO_H")
fi
# Expected ABL SSPP0 format (gpu-handoff.h FG_ABL_SSPP_FORMAT), once a probe boot has measured it.
if [[ -n ${FUXI_ABL_SSPP_FORMAT:-} ]]; then
 [[ $FUXI_ABL_SSPP_FORMAT =~ ^0x[0-9a-fA-F]{1,8}$ ]] || { echo 'FUXI_ABL_SSPP_FORMAT must look like 0x237ff' >&2; exit 1; }
 flags+=("-DFG_ABL_SSPP_FORMAT=${FUXI_ABL_SSPP_FORMAT}u")
fi
for source in start.S loader.c ../console.c ../fuxi-probe/probe.c; do
 name=$(basename "${source%.*}")
 "$CLANG" "${flags[@]}" -c "$source" -o "$out/$name.o"
 if [[ $source == *.c ]] && "$READELF" -r "$out/$name.o" | grep -q R_AARCH64_ABS; then
  echo "Absolute C relocation in $source" >&2; exit 1
 fi
done
"$CLANG" "${flags[@]}" -c "$out/payload.S" -o "$out/payload.o"
"$CLANG" "${flags[@]}" -c "$out/gpu-fw.S" -o "$out/gpu-fw.o"
# The boot logo mask, incbin like the payload; .rodata.* lands in loader.ld's .rodata.
# 16 KiB aligned (one XNU page): XNU maps it from its page start (0115
# fbcon_logo_probe) and reads it with aligned 32-bit loads.
logo_obj=()
if [[ -n $logo ]]; then
 printf '.section .rodata.boot_logo,"a"\n.balign 16384\n.global _boot_logo_start\n_boot_logo_start:\n.incbin "%s"\n.global _boot_logo_end\n_boot_logo_end:\n' \
  "$FUXI_BOOT_LOGO" > "$out/logo.S"
 "$CLANG" "${flags[@]}" -c "$out/logo.S" -o "$out/logo.o"
 logo_obj=("$out/logo.o")
fi
"$LLD" -T loader.ld --build-id=none "$out/start.o" "$out/loader.o" "$out/console.o" "$out/probe.o" "${logo_obj[@]}" "$out/gpu-fw.o" "$out/payload.o" -o "$out/loader.elf"
python3 - "$out/loader.elf" "$READELF" "$no_gpu_fw" <<'GPUCHECK'
import re, subprocess, sys
sections = subprocess.check_output([sys.argv[2], '-SW', sys.argv[1]], text=True)
match = re.search(r'\.gpu_fw\s+PROGBITS\s+[0-9a-fA-F]+\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)', sections)
if sys.argv[3] == '1':
    # No-firmware mode: the section must be absent or empty, never a partial set.
    assert not match or int(match[1], 16) == 0, 'FUXI_NO_GPU_FW=1 image must carry no GPU firmware'
    print('NO GPU FIRMWARE: empty .gpu_fw, GPU handoff magic is never published')
else:
    assert match and int(match[1], 16) == 0x25ac8, 'loader .gpu_fw must be exactly 0x25ac8 bytes'
    print('Verified loader .gpu_fw size 0x25ac8')
GPUCHECK
"$OBJCOPY" -O binary "$out/loader.elf" "$out/Image"
python3 - "$out/Image" <<'PY'
import struct,sys
from pathlib import Path
b=Path(sys.argv[1]).read_bytes()
assert struct.unpack_from('<Q',b,16)[0]==len(b), 'Image size must cover embedded payload'
assert b[56:60]==b'ARM\x64'
PY
# A logo Image carries the mask once and the marker tools/run-ios-salami.py checks; others neither.
python3 - "$out/Image" "${FUXI_BOOT_LOGO:-}" "$out/loader.elf" "$READELF" <<'LOGOPY'
import subprocess, sys
from pathlib import Path
image = Path(sys.argv[1]).read_bytes()
marker = b'FUXI BOOT LOGO:'
if sys.argv[2]:
    mask = Path(sys.argv[2]).read_bytes()
    assert image.count(mask) == 1, 'Image must carry the boot logo mask exactly once'
    assert image.count(marker) == 1, 'Image lacks the boot logo marker'
    # loader.ld links at 0 and objcopy starts the Image there: a link address is an
    # Image offset, and at run time load PA + offset (PC-relative C references).
    syms = {}
    for line in subprocess.check_output([sys.argv[4], '-sW', sys.argv[3]], text=True).splitlines():
        f = line.split()
        if len(f) == 8 and f[7] in ('_head', '_boot_logo_start', '_boot_logo_end'):
            syms[f[7]] = int(f[1], 16)
    assert syms.get('_head') == 0 and len(syms) == 3, 'loader.elf must link _head at 0'
    off = syms['_boot_logo_start']
    assert syms['_boot_logo_end'] - off == len(mask) and image[off:off + len(mask)] == mask, 'mask symbol off its bytes'
    assert off % 16384 == 0, f'mask Image offset {off:#x} is not 16 KiB aligned (logo.S .balign 16384)'
    print('Boot logo: mask and marker embedded, no loader text')
    print(f'Boot logo: mask at Image offset {off:#x} (16 KiB aligned), {len(mask)} bytes; XNU gets sweetlogo_pa = load PA + {off:#x}')
else:
    assert marker not in image, 'Image carries a boot logo marker without FUXI_BOOT_LOGO'
LOGOPY
# A GPU-less image never takes the name of the firmware-bearing one runners expect.
if [[ $no_gpu_fw == 1 ]]; then suffix=-nogpufw; else suffix=; fi
python3 "$MKBOOTIMG" --header_version 4 --kernel "$out/Image" \
 --os_version 16.0.0 --os_patch_level 2026-08 -o "$out/$board-xnu-boot$stage$logo$touch$suffix.img"
ls -lh "$out/Image" "$out/$board-xnu-boot$stage$logo$touch$suffix.img"
