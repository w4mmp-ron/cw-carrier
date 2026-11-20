#!/bin/bash
# Build the cw-carrier Pi package from a normal mscc release .deb.
#
#   wsl bash /mnt/c/Users/Ron/.grok/cw-carrier/deb/build-cw-deb.sh BASE.deb VERSION
#   e.g. ... /mnt/c/Users/Ron/.grok/worktrees/installers/rpi/mscc_1.0.102_arm64.deb 1.0.102+cw1
#
# Takes the normal mscc package as it is (same install scripts, same name "mscc") and:
#   - replaces ms-sdr, sdrcore-recv, sdrcore-trans with ../binaries/ (the cw-carrier builds)
#   - adds the cw-carrier Black Pill files from ../pill-firmware/release/ next to the normal
#     ones (the normal ones stay, so the radio can go back)
#   - sets Version and adds a line to the description
# The .deb goes to ../installers/ (never overwritten). Then it lists every file that differs
# from BASE.deb (md5), so you can check only the expected files changed.
#
# Version: <base>+cw<n>. apt sorts 1.0.102+cw1 above 1.0.102 and below 1.0.103, so the next
# normal mscc installs over it.
set -euo pipefail

BASE="${1:?usage: build-cw-deb.sh BASE.deb VERSION}"
VER="${2:?usage: build-cw-deb.sh BASE.deb VERSION}"
R="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$R/installers"
T=/tmp/cw-deb
NAME="mscc_${VER}_arm64.deb"

mkdir -p "$OUT"
if [[ -e "$OUT/$NAME" ]]; then
  echo "NOT built: $OUT/$NAME already exists (use a new VERSION)" >&2
  exit 1
fi

rm -rf "$T"
mkdir -p "$T"
dpkg-deb -R "$BASE" "$T/pkg"
dpkg-deb -R "$BASE" "$T/old"

for f in ms-sdr sdrcore-recv sdrcore-trans; do
  install -m 0755 "$R/binaries/$f" "$T/pkg/usr/share/mscc/binaries/$f"
done
for f in "$R"/pill-firmware/release/BlackPill-25MHz-*-app.bin; do
  case "$(basename "$f")" in *-b[0-9]*-*) continue ;; esac   # skip -b2 rebuild copies
  install -m 0644 "$f" "$T/pkg/usr/share/mscc/firmware/black-pill/"
done

C="$T/pkg/DEBIAN/control"
sed -i "s/^Version: .*/Version: $VER/" "$C"
sed -i "s/^Description: \(.*\)$/Description: \1 - cw-carrier build/" "$C"
sed -i "/^Description:/a\\ cw-carrier build (github.com/w4mmp-ron/cw-carrier): the Black Pill (3.90+) makes\\n the CW carrier itself and the servers stop the radio streams when not needed.\\n Off unless ~/.local/mscc/mscc.ini has CW_CARRIER=1; otherwise as the normal mscc.\\n ." "$C"

dpkg-deb --root-owner-group -b "$T/pkg" "$T/$NAME" >/dev/null
cp "$T/$NAME" "$OUT/$NAME"
echo "built: $OUT/$NAME ($(stat -c %s "$OUT/$NAME") bytes)"

# file-by-file check against the base package
(cd "$T/old" && find . -type f | sort | xargs md5sum) > "$T/old.md5"
rm -rf "$T/new"; dpkg-deb -R "$OUT/$NAME" "$T/new"
(cd "$T/new" && find . -type f | sort | xargs md5sum) > "$T/new.md5"
echo "files: base $(wc -l < "$T/old.md5"), new $(wc -l < "$T/new.md5")"
echo "differences (< base, > new):"
diff "$T/old.md5" "$T/new.md5" | grep '^[<>]' | awk '{print $1, $3}' || true
