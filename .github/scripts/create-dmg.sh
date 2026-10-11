#!/usr/bin/env bash
# Assemble from a copy; this does not change the input application or its signature.
set -euo pipefail
app=${1:?app required}
dmg=${2:?new absolute DMG required}
[[ $(uname -s) = Darwin && "$app" = *.app && "$dmg" = /* && ! -e "$dmg" ]] || exit 1
stage=$(mktemp -d "$(dirname "$dmg")/dmg-stage.XXXXXX")
ditto "$app" "$stage/Phoxal Simulator.app"
ln -s /Applications "$stage/Applications"
cp "$app/Contents/Resources/application-provenance.txt" "$stage/Source-provenance.txt"
printf 'Drag Phoxal Simulator into Applications.\nReplace only while the application is closed.\nNotices and exact source provenance are inside the application.\n' > "$stage/Installation.txt"
printf 'Ad-hoc application, not Developer ID signed or notarized.\n' > "$stage/Signature.txt"
hdiutil create -volname 'Phoxal Simulator' -srcfolder "$stage" -format UDZO -ov "$dmg"
hdiutil verify "$dmg"
