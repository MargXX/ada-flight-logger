#!/usr/bin/env bash
# swd_dump.sh
# Reads the flash chip over SWD, using flash_dump_swd on the Pico.
# Writes a raw binary image, 256 bytes per page, in page order.
#
# Usage: tools/swd_dump.sh OUT.bin [PAGES]
#   PAGES defaults to 65536, the whole chip. It is rounded up to 512 pages.
#   Run from the repo root after `alr build`. Read-only for the flash.

set -euo pipefail

OUT=${1:?usage: tools/swd_dump.sh OUT.bin [PAGES]}
PAGES=${2:-65536}
CHUNK=512
CHUNKS=$(( (PAGES + CHUNK - 1) / CHUNK ))
ELF=bin/flash_dump_swd

# find the mailbox symbols in the built program
NM=$(alr exec -- which arm-eabi-nm)
addr() { "$NM" "$ELF" | awk -v s="$1" '$3 == s { print "0x" $1 }'; }
START=$(addr swd_start_page)
COUNT=$(addr swd_page_count)
REQUEST=$(addr swd_request)
DONE=$(addr swd_done)
BUFFER=$(addr swd_buffer)

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# load the reader, then ask for one 128 KB chunk at a time
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" \
  -c "program $ELF verify reset exit"

cat > "$TMP/dump.tcl" <<EOF
init
for {set i 0} {\$i < $CHUNKS} {incr i} {
  set n [expr {\$i + 1}]
  mww $START [expr {\$i * $CHUNK}]
  mww $COUNT $CHUNK
  mww $REQUEST \$n
  set tries 0
  while {[lindex [read_memory $DONE 32 1] 0] != \$n} {
    sleep 10
    incr tries
    if {\$tries > 500} { echo "timeout on chunk \$i"; shutdown error }
  }
  dump_image $TMP/chunk_[format %04d \$i].bin $BUFFER [expr {$CHUNK * 256}]
}
shutdown
EOF

openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" \
  -f "$TMP/dump.tcl"
cat "$TMP"/chunk_*.bin > "$OUT"
echo "wrote $OUT, $(( $(wc -c < "$OUT") / 256 )) pages"
