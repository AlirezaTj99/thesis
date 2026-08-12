#!/bin/bash
# Go2 Robot Control — Interactive Menu
# Usage: bash ~/Desktop/go2_control.sh

# ── Network mode ─────────────────────────────────────────────────────────────
CONNECTION_MODE="wifi"
ORIN_IP="130.251.13.140"
ORIN_USER="unitree"
PC_IP="130.251.13.110"

# ── Environment setup ─────────────────────────────────────────────────────────
source /opt/ros/foxy/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/home/alireza/livox-sdk2/lib:$LD_LIBRARY_PATH
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"wlp8s0\" priority=\"default\" multicast=\"false\" /></Interfaces></General><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address=\"127.0.0.1\"/><Peer Address=\"192.168.123.18\"/></Peers></Discovery></Domain></CycloneDDS>"

# ── Route to robot internal subnet ───────────────────────────────────────────
if ! ip route show | grep -q "^192.168.123.0/24"; then
    echo "  Adding route to robot internal network (requires sudo)..."
    sudo ip route add 192.168.123.0/24 via "$ORIN_IP" dev wlp8s0
fi

clear
echo "============================================"
echo "        Go2 Robot Control Panel"
echo "============================================"
echo ""

# ── Start bridge on Orin (SSH) ────────────────────────────────────────────────
echo "  [1/3] Checking bridge on Orin (${ORIN_USER}@${ORIN_IP})..."
echo "        (Enter Orin SSH password/PIN when prompted)"
echo ""

ssh -o ConnectTimeout=10 "${ORIN_USER}@${ORIN_IP}" '
if [ -f /tmp/go2_bridge.pid ] && kill -0 $(cat /tmp/go2_bridge.pid) 2>/dev/null; then
    echo "  Bridge already running (pid $(cat /tmp/go2_bridge.pid))."
else
    setsid bash /home/unitree/go2_wifi_bridge.sh 130.251.13.110 > /tmp/go2_bridge.log 2>&1 < /dev/null &
    echo $! > /tmp/go2_bridge.pid
    echo "  Bridge launched (pid $(cat /tmp/go2_bridge.pid))."
fi
'

SSH_EXIT=$?
echo ""

if [ $SSH_EXIT -ne 0 ]; then
    echo "  ERROR: SSH to Orin failed. Check robot is on and Orin is reachable."
    read -p "  Press Enter to exit..." ; exit 1
fi

echo "  Waiting for bridge to initialize..."
sleep 6

# ── Check robot reachability ──────────────────────────────────────────────────
echo "  [2/3] Checking robot connection..."
if ! ping -c 1 -W 3 "$ORIN_IP" &>/dev/null; then
    echo ""
    echo "  ERROR: Cannot reach Orin at ${ORIN_IP}"
    read -p "  Press Enter to exit..." ; exit 1
fi
echo "        Orin reachable."
echo ""

# ── Ask where the robot is right now ─────────────────────────────────────────
echo ""
echo "  [3/4] Where is the robot right now?"
echo "        (it may have been shut down somewhere other than home)"
echo ""
python3 - << 'PYEOF'
import yaml, sys, os

# stdin is the heredoc — reopen the real terminal for interactive input
sys.stdin = open('/dev/tty')

wps_file  = os.path.expanduser('~/maps/waypoints.yaml')
pose_file = os.path.expanduser('~/maps/robot_initial_pose.yaml')

try:
    with open(wps_file) as f:
        data = yaml.safe_load(f)
    wps   = data.get('waypoints', {})
    names = list(wps.keys())

    for i, n in enumerate(names, 1):
        wp = wps[n]
        print(f"    {i}) {n}  (x={wp['x']}, y={wp['y']}, yaw={wp.get('yaw', 0)}°)")
    print("")

    choice = input("  Your choice: ").strip()

    if choice.isdigit():
        idx = int(choice) - 1
        name = names[idx] if 0 <= idx < len(names) else None
    else:
        matched = [n for n in names if n.lower() == choice.lower()]
        name = matched[0] if matched else None

    if name is None:
        print(f"  Not found — defaulting to home position (0, 0, 0°)")
        sys.exit(0)

    wp  = wps[name]
    x   = float(wp.get('x',   0))
    y   = float(wp.get('y',   0))
    yaw = float(wp.get('yaw', 0))

    with open(pose_file, 'w') as f:
        yaml.dump({'name': name, 'x': x, 'y': y, 'yaw_deg': yaw}, f)

    print(f"  Starting at: '{name}'  (x={x}, y={y}, yaw={yaw}°)")

except FileNotFoundError:
    print("  No waypoints.yaml found — using default position (0, 0, 0°)")
except Exception as e:
    print(f"  Error: {e} — using default position (0, 0, 0°)")
PYEOF
echo ""

# ── Open navigation stack + companion terminals ───────────────────────────────
echo "  [4/4] Opening navigation stack and companion terminals..."
NAV_SCRIPT="/home/alireza/thesis1/navigation_go2/go2_ws/src/go2_real/scripts/start_go2_real.sh"
MONITOR_SCRIPT="/home/alireza/thesis1/navigation_go2/go2_ws/src/go2_real/scripts/run_monitor_robot.sh"

_open_term() {
    local title="$1"
    local cmd="$2"
    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" &
    elif command -v xterm &>/dev/null; then
        xterm -title "$title" -e "bash -c '$cmd; exec bash'" &
    elif command -v konsole &>/dev/null; then
        konsole --title "$title" -e bash -c "$cmd; exec bash" &
    fi
    sleep 0.3
}

# Always kill old navigation and restart fresh so go2_bridge.py reloads with
# the latest code. start_go2_real.sh itself kills all stale processes first.
pkill -9 -f "go2_real.launch.py"    2>/dev/null
pkill -9 -f go2_bridge              2>/dev/null
pkill -9 -f obstacle_repulsion      2>/dev/null
pkill -9 -f velocity_combiner       2>/dev/null
pkill -9 -f pointcloud_to_laserscan 2>/dev/null
sleep 1
_open_term "Go2 Navigation" "bash '${NAV_SCRIPT}'"
echo "        Navigation stack starting..."

# Monitor (fallback — start_go2_real.sh also opens it, but open here in case
# the user runs go2_control.sh without navigation)
if ! pgrep -f "monitor_robot.py" >/dev/null 2>&1; then
    _open_term "Go2 Monitor" "bash '${MONITOR_SCRIPT}'"
    echo "        Monitor opened."
fi

echo ""
echo "  Robot ready. Launching control panel..."
echo "  (Navigation stack may take ~30s to be fully ready)"
echo ""
sleep 0.5

# ── Launch persistent Python controller ───────────────────────────────────────
exec python3 -u "$(dirname "$0")/go2_control.py"
