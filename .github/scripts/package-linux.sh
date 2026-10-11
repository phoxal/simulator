#!/usr/bin/env bash
# Relocatable development archive; graphical support qualification is separate.
set -euo pipefail
binary=${1:?compiled executable required}
engine=${2:?provisioned engine root required}
package=${3:?new absolute package directory required}
version=${4:?application version required}
source_revision=${5:?source revision required}
source_archive_sha256=${6:?exact source archive checksum required}
[[ "$package" = /* && ! -e "$package" ]] || { echo 'Use a new absolute package destination' >&2; exit 1; }
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+([-.][a-zA-Z0-9.-]+)?$ ]] || exit 1
[[ "$source_revision" =~ ^[0-9a-f]{40}$ && "$source_archive_sha256" =~ ^[0-9a-f]{64}$ ]] || exit 1
[[ $(uname -s) = Linux ]] || exit 1
case "$package" in
  *-x86_64-linux) expected_machine='Advanced Micro Devices X86-64' ;;
  *-aarch64-linux) expected_machine=AArch64 ;;
  *) echo 'Package name must identify a supported architecture' >&2; exit 1 ;;
esac
readelf -h "$binary" | grep -F "Machine:" | grep -F "$expected_machine" >/dev/null
working_tree=uncommitted-development
if [ "$(git rev-parse HEAD)" = "$source_revision" ] && [ -z "$(git status --porcelain)" ]; then working_tree=clean-release; fi
provenance=$(printf 'source_revision=%s\nsource_archive_sha256=%s\nworking_tree=%s\n' "$source_revision" "$source_archive_sha256" "$working_tree")
mkdir -p "$package/bin" "$package/lib" "$package/share"
cp "$binary" "$package/bin/phoxal-simulator"
# A bare SONAME plus RUNPATH falls back to ld.so.cache/default directories if
# the private engine disappears. A slash-containing DT_NEEDED resolves this
# one engine directly, while retaining ordinary dynamic linking.
patchelf --replace-needed libmujoco.so.3.12.0 "\$ORIGIN/../lib/libmujoco.so.3.12.0" "$package/bin/phoxal-simulator"
patchelf --remove-rpath "$package/bin/phoxal-simulator"
cp -a "$engine/lib/." "$package/lib/"
cp "$engine/LICENSE" "$package/share/MuJoCo-LICENSE"
cp "$engine/THIRD_PARTY_NOTICES" "$package/share/MuJoCo-THIRD_PARTY_NOTICES"
cp "$engine/provenance.txt" "$package/share/MuJoCo-provenance.txt"
cp LICENSE "$package/share/Phoxal-LICENSE"
cp resources/icons/LICENSE "$package/share/Lucide-LICENSE"
cp .github/resources/phoxal-mark.png "$package/share/phoxal-mark.png"
cp .github/resources/mujoco-rs-LICENSE "$package/share/mujoco-rs-LICENSE"
cat > "$package/share/phoxal-simulator.desktop.in" <<DESKTOP
[Desktop Entry]
Type=Application
Name=Phoxal Simulator
Exec=/absolute/installation/path/bin/phoxal-simulator
Icon=/absolute/installation/path/share/phoxal-mark.png
Categories=Development;Science;
Terminal=false
DESKTOP
printf '%s\n' "$provenance" > "$package/share/application-provenance.txt"
[ "$("$package/bin/phoxal-simulator" --version)" = "phoxal-simulator $version" ] || { echo 'Compiled executable/version mismatch' >&2; exit 1; }
shasum -a 256 "$package/bin/phoxal-simulator" "$package/lib/libmujoco.so.3.12.0" > "$package.provenance.txt"
tar -czf "$package.tar.gz" -C "$(dirname "$package")" "$(basename "$package")"
shasum -a 256 "$package.tar.gz" >> "$package.provenance.txt"
