#!/bin/sh
set -eu
# One owned shell blocks on its own FIFO, without a busy loop or sleep descendants.
mkfifo "$1.wait"
exec 3<> "$1.wait"
case "$2" in
    'exit 0') trap 'exit 0' TERM ;;
    'exit 7') trap 'exit 7' TERM ;;
    ':') trap ':' TERM ;;
    *) exit 2 ;;
esac
printf '%s' '{"schema":"phoxal/supervisor-ready/v0","execution":"execution-1"}' > "$1.tmp"
mv "$1.tmp" "$1"
while :; do
    IFS= read -r _ <&3 || :
done
