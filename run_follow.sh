#!/bin/bash
# Run follow.py, restarting it when its video ends (exit 3, e.g. the drone switched
# H.264 <-> H.265). While it restarts the ESP32 gets no commands and centres the
# sticks within 300 ms, so the drone holds. 'quit' (exit 0) or an error stops the loop.
#   ./run_follow.sh --out capture/flight2 --distance ...
cd "$(dirname "$0")"
while true; do
    .venv/bin/python -u follow.py "$@"
    code=$?
    [ $code -eq 3 ] || exit $code
    echo "$(date +%T) restarting follow.py (video ended)"
    sleep 1
done
