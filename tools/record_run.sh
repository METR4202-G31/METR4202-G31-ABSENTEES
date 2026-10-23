#!/usr/bin/env bash
# Record one exploration run so it can be analysed afterwards.
#
# Saves, in ~/metr4202_runs/<date>_<label>/ (and as a .tar.gz of that folder):
#   launch.log        everything project.launch.py prints (all nodes)
#   bag/              rosbag of what the robot saw and decided: map, scans,
#                     odometry, TF, costmaps, plans, velocity commands,
#                     explorer goals/markers/status, Nav2 behaviour tree log
#   info.txt          your notes, branch, commit, uncommitted file list
#   uncommitted.diff  exact code changes not yet committed
#   params/           the parameter files that were actually installed
#   exploration_world.sdf (+ .spawn.json)  sim only: the generated world
#
# On the real robot run this on the laptop, exactly like project.launch.py.
#
# Usage (start Gazebo or the robot bringup first, as usual):
#   tools/record_run.sh sim   "short label"
#   tools/record_run.sh robot "short label"
# Then press Ctrl-C ONCE when you want to stop. It shuts everything down
# cleanly and writes the archive; that takes 10-30 s.

set -u

MODE="${1:-}"
LABEL="${2:-run}"
case "$MODE" in
    sim) USE_SIM_TIME=true ;;
    robot) USE_SIM_TIME=false ;;
    *)
        echo "Usage: $0 sim|robot \"short label\""
        exit 1
        ;;
esac

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
SAFE_LABEL="$(printf '%s' "$LABEL" | tr -c 'A-Za-z0-9_-' '_')"
RUNS_DIR="$HOME/metr4202_runs"
NAME="${STAMP}_${SAFE_LABEL}"
OUT="$RUNS_DIR/$NAME"
mkdir -p "$OUT/params"

if ! command -v ros2 > /dev/null; then
    echo "ros2 not found: source /opt/ros/humble/setup.bash and install/setup.bash first."
    exit 1
fi

# ---- notes and code snapshot --------------------------------------------------
echo "Describe this run: world command or arena, what you changed, what to look for."
echo "Paste or type it, then press Enter on an empty line:"
NOTES=""
while IFS= read -r line; do
    [ -z "$line" ] && break
    NOTES+="$line"$'\n'
done

{
    echo "label: $LABEL"
    echo "mode: $MODE (use_sim_time=$USE_SIM_TIME)"
    echo "started: $(date -Iseconds)"
    echo "host: $(hostname)  TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-unset}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
    echo "branch: $(git -C "$REPO" rev-parse --abbrev-ref HEAD 2>/dev/null)"
    echo "commit: $(git -C "$REPO" rev-parse HEAD 2>/dev/null)"
    echo
    echo "notes:"
    printf '%s' "$NOTES"
    echo
    echo "uncommitted files:"
    git -C "$REPO" status --short 2>/dev/null
} > "$OUT/info.txt"
git -C "$REPO" diff HEAD > "$OUT/uncommitted.diff" 2>/dev/null

for pkg in simple_explorer slam_setup navigation_supervisor; do
    prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null)" || continue
    if [ -d "$prefix/share/$pkg/config" ]; then
        mkdir -p "$OUT/params/$pkg"
        cp "$prefix/share/$pkg/config/"*.yaml "$OUT/params/$pkg/" 2>/dev/null
    fi
done

# Simulation: keep the generated world (true walls + marker positions) so the
# map and the marker estimates can be checked against ground truth.
if [ "$MODE" = sim ]; then
    for f in /tmp/exploration_world.sdf /tmp/exploration_world.sdf.spawn.json; do
        [ -f "$f" ] && cp "$f" "$OUT/"
    done
fi

# ---- start recording and the stack ---------------------------------------------
# Small topics only (no camera images), so a 10-15 min run stays a few tens of MB.
TOPICS=(
    /tf /tf_static /odom /scan /scan_filled /map
    /cmd_vel /cmd_vel_nav /plan /local_plan
    /global_costmap/costmap /local_costmap/costmap
    /explore /explore/status /explore/blacklist /explore/frontiers
    /failed_regions /behavior_tree_log /aruco
)

# Plain, line-by-line output so the log file matches the terminal.
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export PYTHONUNBUFFERED=1

# Each child gets its own process group (setsid) so that Ctrl-C reaches only
# this script, which then stops the bag first and the stack second. Bash
# starts background commands with Ctrl-C (SIGINT) ignored; the python3 line
# restores it so the bag and launch shut down cleanly when we signal them.
RESTORE_INT=(python3 -c 'import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])')

setsid "${RESTORE_INT[@]}" ros2 bag record -o "$OUT/bag" "${TOPICS[@]}" \
    > "$OUT/bag_record.log" 2>&1 &
BAG_PID=$!

setsid "${RESTORE_INT[@]}" ros2 launch simple_explorer project.launch.py \
    use_sim_time:="$USE_SIM_TIME" > >(trap '' INT; exec tee "$OUT/launch.log") 2>&1 &
LAUNCH_PID=$!

STOP=0
trap 'STOP=1' INT TERM
echo
echo "Recording to $OUT"
echo "Press Ctrl-C once to stop."
while [ "$STOP" -eq 0 ] && kill -0 "$LAUNCH_PID" 2> /dev/null; do
    sleep 1
done
trap '' INT    # ignore further Ctrl-C while shutting down

# ---- shut down ---------------------------------------------------------------
wait_for() {   # wait_for <pid> <seconds>
    local i
    for ((i = 0; i < $2 * 2; i++)); do
        kill -0 "$1" 2> /dev/null || return 0
        sleep 0.5
    done
    return 1
}

echo
echo "Stopping the bag..."
kill -INT -- "-$BAG_PID" 2> /dev/null
wait_for "$BAG_PID" 15 || kill -KILL -- "-$BAG_PID" 2> /dev/null

echo "Stopping the stack (rviz can take a while)..."
kill -INT -- "-$LAUNCH_PID" 2> /dev/null
wait_for "$LAUNCH_PID" 30 || kill -KILL -- "-$LAUNCH_PID" 2> /dev/null
sleep 1

echo "stopped: $(date -Iseconds)" >> "$OUT/info.txt"
tar -czf "$RUNS_DIR/$NAME.tar.gz" -C "$RUNS_DIR" "$NAME"

echo
echo "Saved: $RUNS_DIR/$NAME.tar.gz ($(du -h "$RUNS_DIR/$NAME.tar.gz" | cut -f1))"
echo "To copy it to Windows:  cp \"$RUNS_DIR/$NAME.tar.gz\" /mnt/c/Users/<you>/Downloads/"
