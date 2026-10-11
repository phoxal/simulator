#!/usr/bin/env bash
# CI/contributor provisioning, never application startup.
set -euo pipefail
target=${1:?target required}
root=${2:?absolute destination required}
[[ "$root" = /* ]] || { echo 'Destination must be absolute' >&2; exit 1; }
case "$target" in
  aarch64-apple-darwin|x86_64-apple-darwin)
    archive=mujoco-3.12.0-macos-universal2.dmg
    checksum=8410882d724c3637b935dc0482b0de90efed44b17bbcfcb4a165ee46285a4865 ;;
  aarch64-unknown-linux-gnu)
    archive=mujoco-3.12.0-linux-aarch64.tar.gz
    checksum=08fd5627a2ef7d5a42580c40e014ab2c1a644f082010c584ca361a3ed8cad838 ;;
  x86_64-unknown-linux-gnu)
    archive=mujoco-3.12.0-linux-x86_64.tar.gz
    checksum=a9367911e6d5eaeade17c2197304687421c1fc932cdf7bcd4cb8cfaf0374dcb2 ;;
  *) echo "Unqualified target: $target" >&2; exit 1 ;;
esac
mkdir -p "$root"
url="https://github.com/google-deepmind/mujoco/releases/download/3.12.0/$archive"
if [[ ! -f "$root/$archive" ]]; then
  curl --fail --location --retry 3 "$url" --output "$root/$archive.part"
  mv "$root/$archive.part" "$root/$archive"
fi
printf '%s  %s\n' "$checksum" "$root/$archive" | shasum -a 256 --check
if [[ "$archive" = *.dmg ]]; then
  mount=$(mktemp -d "$root/mount.XXXXXX")
  hdiutil attach "$root/$archive" -readonly -nobrowse -mountpoint "$mount"
  trap 'hdiutil detach "$mount"; rmdir "$mount"' EXIT
  mkdir -p "$root/lib"
  ditto "$mount/mujoco.framework" "$root/lib/mujoco.framework"
  cp "$mount/LICENSE" "$mount/THIRD_PARTY_NOTICES" "$root/"
  ln -sfn mujoco.framework/Versions/A/libmujoco.3.12.0.dylib "$root/lib/libmujoco.dylib"
  ln -sfn mujoco.framework/Versions/A/libmujoco.3.12.0.dylib "$root/lib/libmujoco.3.12.0.dylib"
else
  tar -xzf "$root/$archive" -C "$root" --strip-components=1
fi
printf 'version=3.12.0\ntarget=%s\nurl=%s\narchive_sha256=%s\n' "$target" "$url" "$checksum" > "$root/provenance.txt"
printf 'MUJOCO_DYNAMIC_LINK_DIR=%s/lib\n' "$root"
