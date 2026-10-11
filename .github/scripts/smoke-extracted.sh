#!/usr/bin/env bash
# CLI smoke uses the actual default world owner and native renderer, never GUI.
set -euo pipefail
artifact=${1:?final archive}
root=${2:?new absolute extraction root}
[[ "$root" = /* && ! -e "$root" ]] || exit 1
mkdir -p "$root"
# The caller's loader hints must not make a broken private package pass.
unset MUJOCO_DYNAMIC_LINK_DIR MUJOCO_STATIC_LINK_DIR PKG_CONFIG_PATH LD_LIBRARY_PATH LD_PRELOAD DYLD_LIBRARY_PATH DYLD_FRAMEWORK_PATH DYLD_FALLBACK_LIBRARY_PATH DYLD_FALLBACK_FRAMEWORK_PATH DYLD_INSERT_LIBRARIES
if [ "$(uname -s)" = Darwin ]; then
  mkdir "$root/volume"
  mounted=false
  trap 'if $mounted; then hdiutil detach "$root/volume"; fi' EXIT
  hdiutil attach "$artifact" -readonly -nobrowse -mountpoint "$root/volume"
  mounted=true
  ditto "$root/volume/Phoxal Simulator.app" "$root/Phoxal Simulator.app"
  hdiutil detach "$root/volume"
  mounted=false
  executable="$root/Phoxal Simulator.app/Contents/MacOS/phoxal-simulator"
  engine="$root/Phoxal Simulator.app/Contents/Frameworks/mujoco.framework"
else
  tar -xzf "$artifact" -C "$root"
  package=$(basename "$artifact" .tar.gz)
  executable="$root/$package/bin/phoxal-simulator"
  engine="$root/$package/lib/libmujoco.so.3.12.0"
fi
# Run away from the checkout and with no robot/supervisor dependency.
(cd "$root"; env -i PATH=/usr/bin:/bin:/usr/sbin:/sbin "$executable" --version; env -i PATH=/usr/bin:/bin:/usr/sbin:/sbin "$executable" smoke)
mv "$engine" "$root/removed-private-engine"
if (cd "$root"; env -i PATH=/usr/bin:/bin:/usr/sbin:/sbin "$executable" smoke) > "$root/missing-private-engine.log" 2>&1; then
  echo 'Application ran without private engine; ambient fallback suspected' >&2
  exit 1
fi
grep -i mujoco "$root/missing-private-engine.log" >/dev/null || { echo 'Failure did not identify missing private engine' >&2; exit 1; }
printf 'Actual default native smoke passed; missing private engine failed closed.\n'
