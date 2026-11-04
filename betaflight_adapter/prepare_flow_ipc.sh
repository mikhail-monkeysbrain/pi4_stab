#!/bin/bash

SOCK=/tmp/jtzero_bf_imu.sock

if [ ! -S "$SOCK" ]; then
    exit 0
fi

if /usr/bin/fuser -s "$SOCK" 2>/dev/null; then
    echo "BF_IPC_ERROR: socket is owned by active process"
    exit 1
fi

echo "BF_IPC_CLEANUP: removing stale socket"
rm -f -- "$SOCK"
