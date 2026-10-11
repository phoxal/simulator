#!/usr/bin/env bash
# Narrow CI policy: one exact source and the three final downloadable assets.
set -euo pipefail
mode=${1:?guard or seal}
version=${2:?Cargo version}
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || exit 1
tag="phoxal-simulator-v$version"
case "$mode" in
  guard)
    sha=${3:?exact commit}
    [[ "$sha" =~ ^[0-9a-f]{40}$ ]] || exit 1
    # API errors propagate: unavailable/auth failures are never "no release".
    record=$(gh api --paginate "repos/$GITHUB_REPOSITORY/releases?per_page=100" --jq ".[] | select(.tag_name == \"$tag\") | [.draft, .target_commitish] | @tsv")
    if [ -n "$record" ]; then
      expected=$(printf 'true\t%s' "$sha")
      [ "$record" = "$expected" ] || { echo "Refusing published or different-source $tag (including zero-asset releases)" >&2; exit 1; }
    fi
    # Exact-SHA checkouts may have no local tags. Origin is authoritative.
    remote_status=0
    remote_tags=$(git ls-remote --exit-code origin "refs/tags/$tag" "refs/tags/$tag^{}") || remote_status=$?
    case "$remote_status" in
      0)
        # Annotated tags resolve through their peeled commit, lightweight tags directly.
        remote_sha=$(printf '%s\n' "$remote_tags" | awk -v ref="refs/tags/$tag" '$2 == ref { direct=$1 } $2 == ref "^{}" { peeled=$1 } END { print peeled ? peeled : direct }')
        [ "$remote_sha" = "$sha" ] || { echo 'Remote tag belongs to different source' >&2; exit 1; }
        ;;
      2) ;; # Successful query with no matching tag.
      *) echo 'Cannot establish authoritative remote tag identity' >&2; exit "$remote_status" ;;
    esac
    ;;
  seal)
    input=${3:?downloaded artifact directory}
    output=${4:?new flat output directory}
    [ ! -e "$output" ] || exit 1
    mkdir -p "$output"
    for name in "phoxal-simulator-$version-aarch64-apple-darwin.dmg" "phoxal-simulator-$version-x86_64-linux.tar.gz" "phoxal-simulator-$version-aarch64-linux.tar.gz"; do
      matches=$(find "$input" -type f -name "$name")
      [ -n "$matches" ] && [ "$(printf '%s\n' "$matches" | wc -l | tr -d ' ')" = 1 ] || { echo "Missing or duplicate final asset $name" >&2; exit 1; }
      recorded=$(cat "$(dirname "$matches")/final-checksum.txt")
      actual=$(cd "$(dirname "$matches")"; shasum -a 256 "$name")
      [ "$recorded" = "$actual" ] || { echo "Final build checksum mismatch $name" >&2; exit 1; }
      cp "$matches" "$output/$name"
    done
    # Reject unexpected downloadable bytes rather than publishing an open glob.
    count=$(find "$input" -type f ! -name final-checksum.txt | wc -l | tr -d ' ')
    [ "$count" = 3 ] || { echo 'Unexpected release asset set' >&2; exit 1; }
    (cd "$output"; shasum -a 256 ./*.dmg ./*.tar.gz | sed 's|  \./|  |' > SHA256SUMS.txt; shasum -a 256 -c SHA256SUMS.txt)
    ;;
  *) exit 1 ;;
esac
