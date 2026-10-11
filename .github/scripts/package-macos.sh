#!/usr/bin/env bash
# Assemble a private development bundle; release signing/notarization is separate.
set -euo pipefail
binary=${1:?compiled executable required}
engine=${2:?provisioned engine root required}
app=${3:?absolute new app path required}
version=${4:?application version required}
source_revision=${5:?source revision required}
source_archive_sha256=${6:?exact source archive checksum required}
[[ "$app" = /* && "$app" = *.app && ! -e "$app" ]] || { echo 'Use a new absolute .app destination' >&2; exit 1; }
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+([-.][a-zA-Z0-9.-]+)?$ ]] || exit 1
[[ "$source_revision" =~ ^[0-9a-f]{40}$ ]] || exit 1
[[ "$source_archive_sha256" =~ ^[0-9a-f]{64}$ ]] || exit 1
[[ $(uname -s) = Darwin && $(lipo -archs "$binary") = arm64 ]] || exit 1
working_tree=uncommitted-development
if [ "$(git rev-parse HEAD)" = "$source_revision" ] && [ -z "$(git status --porcelain)" ]; then working_tree=clean-release; fi
provenance=$(printf 'source_revision=%s\nsource_archive_sha256=%s\nworking_tree=%s\n' "$source_revision" "$source_archive_sha256" "$working_tree")
mkdir -p "$app/Contents/MacOS" "$app/Contents/Frameworks" "$app/Contents/Resources"
cp "$binary" "$app/Contents/MacOS/phoxal-simulator"
# Source-built binaries carry the selected link directory as an absolute
# contributor rpath (see build.rs). Normalize to the bundle's own layout by
# removing every rpath except the Frameworks entry, then fail closed unless
# exactly that entry remains, so the packaged executable can resolve its
# private engine only through the bundled copy and never an ambient source
# checkout.
executable="$app/Contents/MacOS/phoxal-simulator"
while rpath=$(otool -l "$executable" | grep '^ *path ' | sed 's/^ *path //; s/ (offset .*)$//' | grep -vx '@executable_path/../Frameworks' | head -1) && [ -n "$rpath" ]; do
  install_name_tool -delete_rpath "$rpath" "$executable"
done
remaining=$(otool -l "$executable" | grep '^ *path ' | sed 's/^ *path //; s/ (offset .*)$//' | sort -u)
if [ "$remaining" != '@executable_path/../Frameworks' ]; then
  echo "Packaged executable must keep only the bundle Frameworks rpath; found: $remaining" >&2
  exit 1
fi
ditto "$engine/lib/mujoco.framework" "$app/Contents/Frameworks/mujoco.framework"
cp "$engine/LICENSE" "$app/Contents/Resources/MuJoCo-LICENSE"
cp "$engine/THIRD_PARTY_NOTICES" "$app/Contents/Resources/MuJoCo-THIRD_PARTY_NOTICES"
cp "$engine/provenance.txt" "$app/Contents/Resources/MuJoCo-provenance.txt"
cp LICENSE "$app/Contents/Resources/Phoxal-LICENSE"
cp resources/icons/LICENSE "$app/Contents/Resources/Lucide-LICENSE"
cp .github/resources/mujoco-rs-LICENSE "$app/Contents/Resources/mujoco-rs-LICENSE"
icon_work=$(mktemp -d "$(dirname "$app")/icon.XXXXXX")
mkdir "$icon_work/Phoxal.iconset"
for size in 16 32 128 256 512; do
  sips -z "$size" "$size" .github/resources/phoxal-mark.png --out "$icon_work/Phoxal.iconset/icon_${size}x${size}.png" >/dev/null
  doubled=$((size * 2))
  sips -z "$doubled" "$doubled" .github/resources/phoxal-mark.png --out "$icon_work/Phoxal.iconset/icon_${size}x${size}@2x.png" >/dev/null
done
iconutil -c icns "$icon_work/Phoxal.iconset" -o "$app/Contents/Resources/Phoxal.icns"
cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
<key>CFBundleIdentifier</key><string>com.phoxal.simulator</string>
<key>CFBundleName</key><string>Phoxal Simulator</string>
<key>CFBundleDisplayName</key><string>Phoxal Simulator</string>
<key>CFBundleExecutable</key><string>phoxal-simulator</string>
<key>CFBundleIconFile</key><string>Phoxal.icns</string>
<key>CFBundlePackageType</key><string>APPL</string>
<key>CFBundleShortVersionString</key><string>$version</string>
<key>CFBundleVersion</key><string>$version</string>
<key>NSHighResolutionCapable</key><true/>
<key>LSApplicationCategoryType</key><string>public.app-category.developer-tools</string>
</dict></plist>
PLIST
plutil -lint "$app/Contents/Info.plist"
# Re-sign copied native bytes only with a local ad-hoc signature. This proves
# runnable packaging, never Developer ID, hardened runtime or notarization.
# Ad-hoc signatures only: a development tool distributed as complete
# applications, with no Developer ID, notarization or stapling pipeline.
codesign --force --sign - "$app/Contents/Frameworks/mujoco.framework"
codesign --force --sign - "$app/Contents/MacOS/phoxal-simulator"
printf '%s\n' "$provenance" > "$app/Contents/Resources/application-provenance.txt"
codesign --force --sign - "$app"
codesign --verify --deep --strict "$app"
# Finalized code hashes belong outside signed resources, avoiding a circular
# bundle seal and distinguishing these bytes from the unsigned build product.
{
  printf 'source_revision=%s\nsource_archive_sha256=%s\nsigning=ad-hoc-development\n' "$source_revision" "$source_archive_sha256"
  shasum -a 256 "$binary" "$app/Contents/MacOS/phoxal-simulator" "$app/Contents/Frameworks/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib"
} > "$app.provenance.txt"
[ "$("$app/Contents/MacOS/phoxal-simulator" --version)" = "phoxal-simulator $version" ] || { echo 'Compiled executable/version mismatch' >&2; exit 1; }
