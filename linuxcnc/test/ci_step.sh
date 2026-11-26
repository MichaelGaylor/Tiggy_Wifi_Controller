#!/bin/bash
# For the build server: runs a command, and shows the end of its output on
# the run's summary page as well as in the log, so that the result of a test
# can be read without opening (or being allowed to open) the full log.
#
#   ci_step.sh "title" command [arguments...]

title="$1"
shift
log="$(mktemp)"
"$@" > "$log" 2>&1
status=$?
cat "$log"

level=notice
lines=40
if [ $status -ne 0 ]; then
    level=error
    lines=100
fi
tail -n $lines "$log" | sed -e 's/%/%25/g' -e 's/\r//g' | split -l 20 - "$log.part."
n=0
for part in "$log".part.*; do
    [ -f "$part" ] || continue
    n=$((n + 1))
    echo "::$level title=$title ($n)::$(awk 'BEGIN { ORS = "%0A" } { print }' "$part")"
done
rm -f "$log" "$log".part.*
exit $status
