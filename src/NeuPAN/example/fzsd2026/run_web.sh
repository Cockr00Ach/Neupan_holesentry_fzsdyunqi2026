#!/bin/bash
# Start the NeuPAN web view ("RViz in the browser") for the fzsd2026 sentry.
#
#   ./run_web.sh                 # http://127.0.0.1:8088/
#   ./run_web.sh --port 8090     # another port
#
# It only needs the planner topics, so it works with ./run_sim.sh (or with the real
# robot once the bridge publishes /neupan_status, /neupan_obstacles, /neupan_plan, ...).
set +u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd "$HERE/../../../.." && pwd)

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

exec python3 "$HERE/web/web_server.py" "$@"
