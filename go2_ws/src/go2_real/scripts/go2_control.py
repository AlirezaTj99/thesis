#!/usr/bin/env python3
"""
go2_control.py — Interactive Go2 control with navigation.
"""
import time
import json
import os
import subprocess
import yaml
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

GOAL_SH       = '/home/alireza/thesis1/navigation_go2/go2_ws/src/go2_real/scripts/go2_goal.sh'
WAYPOINTS_YML = os.path.expanduser('~/maps/waypoints.yaml')

MENU = """
--------------------------------------------
  POSTURE
    1) Stand up / Recover    (RecoveryStand)
    2) Lie down              (StandDown)
    3) Sit
    4) Balance stand         (for movement)

  MOVEMENT  (sends once — robot keeps moving)
    5) Move forward   (0.3 m/s)
    6) Move backward  (0.3 m/s)
    7) Strafe left    (0.2 m/s)
    8) Strafe right   (0.2 m/s)
    9) Turn left      (0.5 rad/s)
    0) Turn right     (0.5 rad/s)

  STOP
    s) STOP immediately  <-- use this after any move

  NAVIGATION
    n) Navigate to waypoint
    x) Cancel navigation

  FUN
    h) Hello (wave)
    t) Stretch
    d) Dance 1
    e) Dance 2

    q) Quit
--------------------------------------------"""

_NAV_PROCESSES = [
    'go2_real.launch.py',
    'go2_bridge',
    'obstacle_repulsion',
    'velocity_combiner',
    'pointcloud_to_laserscan',
    'bt_navigator',
    'controller_server',
    'planner_server',
    'recoveries_server',
    'lifecycle_manager',
    'slam_toolbox',
    'amcl',
    'map_server',
    'rviz2',
    'monitor_robot',
    'run_monitor_robot',
    'go2_goal_session',
    'start_go2_real',
]

_TERMINAL_TITLES = ['Go2 Navigation', 'Robot Monitor', 'Go2 Goal']


def _close_terminals():
    for title in _TERMINAL_TITLES:
        subprocess.run(['wmctrl', '-c', title], capture_output=True)
        subprocess.run(
            ['bash', '-c', f'xdotool search --name "{title}" windowkill 2>/dev/null'],
            capture_output=True,
        )


def _graceful_shutdown(node):
    """
    Safe Ctrl+C handler:
      1. Stop all motion
      2. Lay the robot flat to the ground (StandDown)
      3. Kill the navigation stack and close its terminals
    """
    print('\n\n  [Shutdown] Ctrl+C received — safely laying robot down...')
    try:
        print('  Stopping motion...')
        node.send(1003)          # Stop
        time.sleep(0.5)

        print('  Laying robot down (StandDown)...')
        node.send(1005)          # StandDown — robot lies flat
        time.sleep(4.0)          # Wait until fully on the ground

        print('  Robot is down and safe.')
    except Exception as e:
        print(f'  Warning: robot command failed ({e}) — proceeding with kill.')

    print('  Killing navigation stack...')
    for pat in _NAV_PROCESSES:
        subprocess.run(['pkill', '-9', '-f', pat], capture_output=True)

    print('  Closing terminals...')
    _close_terminals()

    print('  Done. Bye!')


def _load_waypoints():
    if not os.path.exists(WAYPOINTS_YML):
        return {}
    with open(WAYPOINTS_YML) as f:
        data = yaml.safe_load(f)
    return data.get('waypoints', {})


def _open_terminal(title, cmd):
    if subprocess.run(['which', 'gnome-terminal'], capture_output=True).returncode == 0:
        subprocess.Popen(['gnome-terminal', '--title', title, '--',
                          'bash', '-c', f'{cmd}; exec bash'])
    elif subprocess.run(['which', 'xterm'], capture_output=True).returncode == 0:
        subprocess.Popen(['xterm', '-title', title, '-e', f"bash -c '{cmd}; exec bash'"])
    elif subprocess.run(['which', 'konsole'], capture_output=True).returncode == 0:
        subprocess.Popen(['konsole', '--title', title, '-e',
                          'bash', '-c', f'{cmd}; exec bash'])
    else:
        print('  WARNING: no terminal emulator found.')


def _navigate(waypoints):
    if not waypoints:
        print('  ERROR: no waypoints found in ~/maps/waypoints.yaml')
        return
    names = list(waypoints.keys())
    print('\n  Choose destination:')
    for i, name in enumerate(names, 1):
        wp = waypoints[name]
        print(f'    {i}) {name:12s}  x={wp["x"]}  y={wp["y"]}  yaw={wp.get("yaw", 0)}°')
    print('    q) back')
    choice = input('\n  Your choice: ').strip().lower()
    if choice == 'q':
        return
    try:
        idx = int(choice) - 1
        if idx < 0 or idx >= len(names):
            print('  Invalid choice.')
            return
        target = names[idx]
    except ValueError:
        if choice in names:
            target = choice
        else:
            print('  Invalid choice.')
            return

    # Kill any existing navigation goal first
    subprocess.run(['pkill', '-f', 'go2_goal.py'], capture_output=True)
    time.sleep(0.5)

    cmd = f'bash {GOAL_SH} {target}'
    _open_terminal(f'Nav → {target}', cmd)
    print(f'  Navigating to "{target}" — check the Nav terminal for progress.')


class Go2Controller(Node):
    def __init__(self):
        super().__init__('go2_controller')
        self._pub = self.create_publisher(Request, '/api/sport/request_wifi', 10)

    def send(self, api_id: int, parameter: str = ''):
        msg = Request()
        msg.header.identity.api_id = api_id
        msg.parameter = parameter
        self._pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)

    def send_move(self, x: float, y: float, z: float):
        self.send(1008, json.dumps({'x': x, 'y': y, 'z': z}))


def main():
    rclpy.init()
    node = Go2Controller()
    waypoints = _load_waypoints()

    print('\n============================================')
    print('        Go2 Robot Control Panel')
    print('============================================')
    print('  Initializing — sending RecoveryStand...')

    rclpy.spin_once(node, timeout_sec=2.0)

    node.send(1006)
    print('  RecoveryStand sent. Waiting 2s...')
    time.sleep(2.0)

    node.send(1002)
    print('  BalanceStand sent. Robot ready.')
    if waypoints:
        print(f'  Waypoints loaded: {", ".join(waypoints.keys())}')
    print()

    try:
        while True:
            print(MENU)
            try:
                choice = input('  Your choice: ').strip().lower()
            except EOFError:
                break
            except KeyboardInterrupt:
                _graceful_shutdown(node)
                return

            print()
            if choice == '1':
                print('  >> Stand up / Recover...')
                node.send(1006)
            elif choice == '2':
                print('  >> Lying down...')
                node.send(1005)
            elif choice == '3':
                print('  >> Sitting...')
                node.send(1009)
            elif choice == '4':
                print('  >> Balance stand...')
                node.send(1002)
            elif choice == '5':
                print('  >> Moving forward...')
                node.send_move(0.3, 0.0, 0.0)
            elif choice == '6':
                print('  >> Moving backward...')
                node.send_move(-0.3, 0.0, 0.0)
            elif choice == '7':
                print('  >> Strafing left...')
                node.send_move(0.0, 0.2, 0.0)
            elif choice == '8':
                print('  >> Strafing right...')
                node.send_move(0.0, -0.2, 0.0)
            elif choice == '9':
                print('  >> Turning left...')
                node.send_move(0.0, 0.0, 0.5)
            elif choice == '0':
                print('  >> Turning right...')
                node.send_move(0.0, 0.0, -0.5)
            elif choice == 's':
                print('  >> STOPPING...')
                node.send(1003)
            elif choice == 'n':
                _navigate(waypoints)
            elif choice == 'x':
                print('  >> Cancelling navigation...')
                result = subprocess.run(['pkill', '-f', 'go2_goal.py'], capture_output=True)
                if result.returncode == 0:
                    print('  Navigation cancelled.')
                else:
                    print('  No navigation was running.')
            elif choice == 'h':
                print('  >> Hello!')
                node.send(1016)
            elif choice == 't':
                print('  >> Stretching...')
                node.send(1017)
            elif choice == 'd':
                print('  >> Dance 1!')
                node.send(1022)
            elif choice == 'e':
                print('  >> Dance 2!')
                node.send(1023)
            elif choice == 'q':
                print('  Sending stop before exit...')
                node.send(1003)
                print('  Bye!')
                break
            else:
                print('  Unknown command. Try again.')
                continue

            if choice not in ('n', 'x', 'q'):
                print('  Done.')
            print()
    except KeyboardInterrupt:
        # Ctrl+C pressed during a command (not at the input prompt)
        _graceful_shutdown(node)
        return
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
