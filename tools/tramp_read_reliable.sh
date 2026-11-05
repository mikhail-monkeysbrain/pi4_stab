#!/usr/bin/env bash
set -u

CMD="${1:-s}"
MAX_ATTEMPTS=3

case "$CMD" in
    r|v|s) ;;
    *)
        echo "Usage: $0 r|v|s" >&2
        exit 2
        ;;
esac

cd "$(dirname "$0")/.." || exit 2

for ((i=1; i<=MAX_ATTEMPTS; i++)); do
    OUTPUT=$(sudo ./tools/tramp_gpiomem "$CMD" 2>&1)
    RC=$?

    if [ "$RC" -eq 0 ] &&
       grep -q 'VALID TRAMP FRAME at offset 0' <<< "$OUTPUT" &&
       grep -q 'Framing errors: 0' <<< "$OUTPUT" &&
       grep -q 'RX bytes: 16' <<< "$OUTPUT"; then
        echo "SUCCESS attempt=$i"
        echo "$OUTPUT"
        exit 0
    fi

    echo "FAILED attempt=$i"
    echo "$OUTPUT"

    if [ "$i" -lt "$MAX_ATTEMPTS" ]; then
        sleep 0.2
    fi
done

echo "ERROR: No valid response after $MAX_ATTEMPTS attempts" >&2
exit 1
