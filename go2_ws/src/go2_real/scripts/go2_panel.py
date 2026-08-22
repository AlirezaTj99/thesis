#!/usr/bin/env python3
"""
go2_panel.py — All-in-one GUI control panel for Go2 robot.

Sections:
  • Posture      — stand / lie / sit / balance
  • Movement     — directional buttons + STOP
  • Navigation   — waypoint picker → go2_goal.py
  • Stair        — start / abort climb sequence
  • Mapping      — start / archive-or-extend / snapshot / stop
  • Waypoints    — add / update / remove  (floor_id + property)
  • Fun          — wave / stretch / dance
"""

import json
import math
import os
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from datetime import datetime

import yaml

# ── ROS2 (optional — needed for robot commands and TF) ───────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    import tf2_ros
    from std_msgs.msg import Bool
    from unitree_api.msg import Request
    from sensor_msgs.msg import BatteryState
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# ── Paths ─────────────────────────────────────────────────────────────────────
WAYPOINTS_FILE = os.path.expanduser('~/maps/waypoints.yaml')
RTABMAP_DB     = os.path.expanduser('~/maps/rtabmap.db')
MAPS_DIR       = os.path.expanduser('~/maps')
GOAL_SH        = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'go2_goal.sh')

_SETUP = (
    'source /opt/ros/foxy/setup.bash && '
    'source /home/alireza/thesis1/navigation_go2/go2_ws/src/unitree_ros2/'
    'cyclonedds_ws/install/setup.bash && '
    'source /home/alireza/thesis1/navigation_go2/go2_ws/install/setup.bash && '
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && '
    "export CYCLONEDDS_URI='"
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="wlp8s0" multicast="false"/>'
    '</Interfaces></General><Discovery>'
    '<MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>'
    '<Peers><Peer address="127.0.0.1"/><Peer address="192.168.123.18"/></Peers>'
    "</Discovery></Domain></CycloneDDS>'"
)


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class _RosNode(Node):
    def __init__(self):
        super().__init__('go2_panel')

        self._buf      = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, self)

        self._sport_pub = self.create_publisher(Request, '/api/sport/request_wifi', 10)
        self._stair_pub = self.create_publisher(Bool,    '/stair_controller/activate', 10)

        self._stair_climbing = False
        self.on_stair_done   = None   # set by GUI after construction

        self.create_subscription(Bool, '/stair_controller/done',
                                 self._stair_done_cb, 10)

        self.battery_text = 'N/A'
        bat_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(BatteryState, '/go2/battery',
                                 self._battery_cb, bat_qos)

    def _battery_cb(self, msg):
        self.battery_text = f'{msg.percentage * 100.0:.0f}%  {msg.voltage:.1f} V'

    def _stair_done_cb(self, msg):
        if msg.data:
            self._stair_climbing = False
            if self.on_stair_done:
                self.on_stair_done()

    def send(self, api_id, parameter=''):
        msg = Request()
        msg.header.identity.api_id = api_id
        msg.parameter = parameter
        self._sport_pub.publish(msg)

    def send_move(self, x, y, z):
        self.send(1008, json.dumps({'x': x, 'y': y, 'z': z}))

    def map_pose(self):
        """Return (x, y, yaw_deg) from map->base_link TF, or None on timeout."""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                t = self._buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                p = t.transform.translation
                q = t.transform.rotation
                yaw = math.degrees(
                    math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
                )
                return round(p.x, 3), round(p.y, 3), round(yaw, 1)
            except Exception:
                time.sleep(0.1)
        return None


# ── Map-start dialog ──────────────────────────────────────────────────────────

def _ask_map_start(parent):
    result = [None]
    dlg = tk.Toplevel(parent)
    dlg.title('Map already exists')
    dlg.resizable(False, False)
    dlg.grab_set()
    ttk.Label(dlg, text='rtabmap.db already exists.\nWhat do you want to do?',
              padding=(20, 14)).pack()
    bf = ttk.Frame(dlg, padding=(16, 0, 16, 14))
    bf.pack()

    def pick(val):
        result[0] = val
        dlg.destroy()

    ttk.Button(bf, text='Archive old & start fresh',
               command=lambda: pick('archive'), width=28).pack(pady=3)
    ttk.Button(bf, text='Extend existing map',
               command=lambda: pick('extend'),  width=28).pack(pady=3)
    ttk.Button(bf, text='Cancel',
               command=lambda: pick(None),      width=28).pack(pady=3)
    parent.wait_window(dlg)
    return result[0]


# ── Waypoint file helpers ─────────────────────────────────────────────────────

def _load():
    if not os.path.exists(WAYPOINTS_FILE):
        return {}
    with open(WAYPOINTS_FILE) as f:
        data = yaml.safe_load(f) or {}
    return data.get('waypoints', {})


def _dump(waypoints):
    os.makedirs(MAPS_DIR, exist_ok=True)
    with open(WAYPOINTS_FILE, 'w') as f:
        yaml.dump({'waypoints': waypoints}, f,
                  default_flow_style=False, allow_unicode=True)


# ── GUI ───────────────────────────────────────────────────────────────────────

class ControlPanel(tk.Tk):

    def __init__(self, ros_node):
        super().__init__()
        self._ros      = ros_node
        self._map_proc = None
        self._nav_proc = None

        self.title('Go2 Control Panel')
        self.resizable(False, False)
        self._build()
        self._refresh()
        self._poll_battery()
        if self._ros:
            self._ros.on_stair_done = self._on_stair_done

    # ── Build UI ──────────────────────────────────────────────────────────────

    def _build(self):
        P = dict(padx=8, pady=4)

        # Header
        hdr = ttk.Frame(self)
        hdr.pack(fill='x', padx=8, pady=(8, 0))
        ttk.Label(hdr, text='Go2 Control Panel', font=('', 11, 'bold')).pack(side='left')
        self._bat_lbl = ttk.Label(hdr, text='Battery: —', foreground='#666')
        self._bat_lbl.pack(side='right')
        ttk.Separator(self, orient='horizontal').pack(fill='x', padx=8, pady=6)

        # ── Row 1: Posture + Movement ──────────────────────────────────────
        top = ttk.Frame(self)
        top.pack(fill='x', **P)

        pf = ttk.LabelFrame(top, text='  Posture  ', padding=8)
        pf.pack(side='left', fill='y', padx=(0, 8))
        for label, api_id in [
            ('Stand Up / Recover', 1006),
            ('Lie Down',           1005),
            ('Sit',                1009),
            ('Balance Stand',      1002),
        ]:
            ttk.Button(pf, text=label, width=18,
                       command=lambda a=api_id, l=label: self._sport(a, l)
                       ).pack(pady=2)

        mf = ttk.LabelFrame(top, text='  Movement  ', padding=8)
        mf.pack(side='left', fill='y')

        r0 = ttk.Frame(mf); r0.pack()
        ttk.Button(r0, text='  Fwd  ', width=7,
                   command=lambda: self._move(0.3, 0.0, 0.0)).pack(side='left', padx=2, pady=2)

        r1 = ttk.Frame(mf); r1.pack()
        ttk.Button(r1, text=' Left ', width=7,
                   command=lambda: self._move(0.0, 0.2, 0.0)).pack(side='left', padx=2, pady=2)
        ttk.Button(r1, text=' STOP ', width=7,
                   command=lambda: self._sport(1003, 'STOP')).pack(side='left', padx=2, pady=2)
        ttk.Button(r1, text=' Right', width=7,
                   command=lambda: self._move(0.0, -0.2, 0.0)).pack(side='left', padx=2, pady=2)

        r2 = ttk.Frame(mf); r2.pack()
        ttk.Button(r2, text='TurnL ', width=7,
                   command=lambda: self._move(0.0, 0.0, 0.5)).pack(side='left', padx=2, pady=2)
        ttk.Button(r2, text='  Bk  ', width=7,
                   command=lambda: self._move(-0.3, 0.0, 0.0)).pack(side='left', padx=2, pady=2)
        ttk.Button(r2, text='TurnR ', width=7,
                   command=lambda: self._move(0.0, 0.0, -0.5)).pack(side='left', padx=2, pady=2)

        ttk.Separator(self, orient='horizontal').pack(fill='x', padx=8, pady=4)

        # ── Navigation ─────────────────────────────────────────────────────
        nf = ttk.LabelFrame(self, text='  Navigation  ', padding=8)
        nf.pack(fill='x', **P)
        nr = ttk.Frame(nf); nr.pack(fill='x')
        ttk.Label(nr, text='Waypoint:').pack(side='left', padx=(0, 6))
        self._nav_var = tk.StringVar()
        self._nav_cb  = ttk.Combobox(nr, textvariable=self._nav_var,
                                     state='readonly', width=16)
        self._nav_cb.pack(side='left', padx=(0, 8))
        ttk.Button(nr, text='Navigate',         width=12,
                   command=self._navigate).pack(side='left', padx=(0, 6))
        ttk.Button(nr, text='Cancel Navigation', width=18,
                   command=self._cancel_nav).pack(side='left')

        ttk.Separator(self, orient='horizontal').pack(fill='x', padx=8, pady=4)

        # ── Stair climbing ─────────────────────────────────────────────────
        sf = ttk.LabelFrame(self, text='  Stair Climbing  ', padding=8)
        sf.pack(fill='x', **P)
        sr = ttk.Frame(sf); sr.pack(fill='x')
        self._btn_climb = ttk.Button(sr, text='Start Stair Climb', width=20,
                                     command=self._start_climb)
        self._btn_climb.pack(side='left', padx=(0, 8))
        self._btn_abort = ttk.Button(sr, text='Abort Stair Climb', width=20,
                                     command=self._abort_climb, state='disabled')
        self._btn_abort.pack(side='left')
        self._stair_lbl = ttk.Label(sf, text='idle', foreground='#888')
        self._stair_lbl.pack(anchor='w', pady=(4, 0))

        ttk.Separator(self, orient='horizontal').pack(fill='x', padx=8, pady=4)

        # ── Mapping ────────────────────────────────────────────────────────
        mapf = ttk.LabelFrame(self, text='  Mapping  ', padding=8)
        mapf.pack(fill='x', **P)
        mr = ttk.Frame(mapf); mr.pack(fill='x')
        ttk.Button(mr, text='Start Mapping',  width=16,
                   command=self._start_mapping).pack(side='left', padx=(0, 6))
        ttk.Button(mr, text='Save Snapshot',  width=16,
                   command=self._save_map).pack(side='left', padx=(0, 6))
        ttk.Button(mr, text='Stop Mapping',   width=16,
                   command=self._stop_mapping).pack(side='left')
        self._map_lbl = ttk.Label(mapf, text='idle', foreground='#888')
        self._map_lbl.pack(anchor='w', pady=(4, 0))

        ttk.Separator(self, orient='horizontal').pack(fill='x', padx=8, pady=4)

        # ── Waypoints ──────────────────────────────────────────────────────
        wf = ttk.LabelFrame(self, text='  Waypoints  ', padding=8)
        wf.pack(fill='both', expand=True, **P)

        form = ttk.Frame(wf); form.pack(fill='x', pady=(0, 6))
        ttk.Label(form, text='Name:').grid(row=0, column=0, sticky='w', padx=(0, 4))
        self._name_var = tk.StringVar()
        ttk.Entry(form, textvariable=self._name_var, width=14).grid(
            row=0, column=1, padx=(0, 12))
        ttk.Label(form, text='Floor:').grid(row=0, column=2, sticky='w', padx=(0, 4))
        self._floor_var = tk.StringVar(value='0')
        ttk.Spinbox(form, textvariable=self._floor_var, from_=0, to=20, width=4).grid(
            row=0, column=3, padx=(0, 12))
        ttk.Label(form, text='Type:').grid(row=0, column=4, sticky='w', padx=(0, 4))
        self._type_var = tk.StringVar(value='floor')
        ttk.Radiobutton(form, text='floor', variable=self._type_var,
                        value='floor').grid(row=0, column=5)
        ttk.Radiobutton(form, text='stair', variable=self._type_var,
                        value='stair').grid(row=0, column=6, padx=(6, 0))

        ttk.Button(wf, text='+ Add Waypoint at Current Position',
                   command=self._add).pack(anchor='w', pady=(0, 6))

        cols = ('name', 'x', 'y', 'yaw', 'floor', 'type')
        self._tree = ttk.Treeview(wf, columns=cols, show='headings',
                                  height=5, selectmode='browse')
        for cid, label, w in [
            ('name', 'Name', 130), ('x', 'X', 70), ('y', 'Y', 70),
            ('yaw', 'Yaw', 58), ('floor', 'Floor', 46), ('type', 'Type', 56),
        ]:
            self._tree.heading(cid, text=label)
            self._tree.column(cid, width=w, anchor='center')
        self._tree.column('name', anchor='w')

        vsb = ttk.Scrollbar(wf, orient='vertical', command=self._tree.yview)
        self._tree.configure(yscrollcommand=vsb.set)
        self._tree.pack(side='left', fill='both', expand=True)
        vsb.pack(side='left', fill='y')

        ar = ttk.Frame(wf); ar.pack(fill='x', pady=(6, 0))
        ttk.Button(ar, text='Update Position', command=self._update).pack(
            side='left', padx=(0, 6))
        ttk.Button(ar, text='Remove',          command=self._remove).pack(
            side='left', padx=(0, 6))
        ttk.Button(ar, text='Reload',          command=self._refresh).pack(side='right')

        ttk.Separator(self, orient='horizontal').pack(fill='x', padx=8, pady=4)

        # ── Fun ────────────────────────────────────────────────────────────
        ff = ttk.LabelFrame(self, text='  Fun  ', padding=8)
        ff.pack(fill='x', **P)
        fr = ttk.Frame(ff); fr.pack()
        for label, api_id in [
            ('Hello (wave)', 1016),
            ('Stretch',      1017),
            ('Dance 1',      1022),
            ('Dance 2',      1023),
        ]:
            ttk.Button(fr, text=label, width=12,
                       command=lambda a=api_id, l=label: self._sport(a, l)
                       ).pack(side='left', padx=3)

        # ── Status bar ─────────────────────────────────────────────────────
        self._status = ttk.Label(self, text='Ready', relief='sunken', anchor='w')
        self._status.pack(fill='x', padx=8, pady=(4, 8))

    # ── Battery ───────────────────────────────────────────────────────────────

    def _poll_battery(self):
        if self._ros:
            self._bat_lbl.config(text=f'Battery: {self._ros.battery_text}')
        self.after(5000, self._poll_battery)

    # ── Sport API ─────────────────────────────────────────────────────────────

    def _sport(self, api_id, label=''):
        if self._ros is None:
            self._info('No ROS — command not sent.')
            return
        self._ros.send(api_id)
        self._info(f'Sent: {label}' if label else f'Sent API {api_id}')

    def _move(self, x, y, z):
        if self._ros is None:
            self._info('No ROS — command not sent.')
            return
        self._ros.send_move(x, y, z)
        dirs = []
        if x > 0:   dirs.append('Fwd')
        elif x < 0: dirs.append('Back')
        if y > 0:   dirs.append('Left')
        elif y < 0: dirs.append('Right')
        if z > 0:   dirs.append('TurnL')
        elif z < 0: dirs.append('TurnR')
        self._info(f'Move: {" ".join(dirs)}')

    # ── Navigation ────────────────────────────────────────────────────────────

    def _navigate(self):
        target = self._nav_var.get().strip()
        if not target:
            messagebox.showwarning('No waypoint', 'Select a waypoint first.')
            return
        self._info(f'Preparing to navigate to "{target}"…')

        def _run():
            if self._ros:
                self._ros.send(1006)   # RecoveryStand
                time.sleep(0.3)
                self._ros.send(1002)   # BalanceStand
            if self._nav_proc and self._nav_proc.poll() is None:
                self._nav_proc.terminate()
            self._nav_proc = subprocess.Popen(['bash', GOAL_SH, target])
            self.after(0, lambda: self._info(f'Navigating to "{target}"'))

        threading.Thread(target=_run, daemon=True).start()

    def _cancel_nav(self):
        subprocess.run(['pkill', '-f', 'go2_goal.py'], capture_output=True)
        if self._nav_proc:
            try: self._nav_proc.terminate()
            except Exception: pass
        self._info('Navigation cancelled.')

    # ── Stair climbing ────────────────────────────────────────────────────────

    def _start_climb(self):
        if self._ros is None:
            self._info('No ROS.'); return
        if self._ros._stair_climbing:
            self._info('Already climbing — abort first.'); return
        self._ros._stair_climbing = True
        self._ros._stair_pub.publish(Bool(data=True))
        self._btn_climb.config(state='disabled')
        self._btn_abort.config(state='normal')
        self._stair_lbl.config(text='climbing…', foreground='orange')
        self._info('Stair climb started  (approach → align → climb)')

    def _abort_climb(self):
        if self._ros is None:
            self._info('No ROS.'); return
        self._ros._stair_climbing = False
        self._ros._stair_pub.publish(Bool(data=False))
        self._ros.send(1003)   # Stop
        self._btn_climb.config(state='normal')
        self._btn_abort.config(state='disabled')
        self._stair_lbl.config(text='aborted', foreground='red')
        self._info('Stair climb aborted.')

    def _on_stair_done(self):
        # Called from ROS background thread — dispatch to tk main thread
        self.after(0, self._stair_done_ui)

    def _stair_done_ui(self):
        self._btn_climb.config(state='normal')
        self._btn_abort.config(state='disabled')
        self._stair_lbl.config(text='climb complete', foreground='green')
        self._info('Stair climb finished.')

    # ── Mapping ───────────────────────────────────────────────────────────────

    def _start_mapping(self):
        if self._map_proc and self._map_proc.poll() is None:
            messagebox.showwarning('Running', 'Mapping is already active.')
            return
        if os.path.exists(RTABMAP_DB):
            choice = _ask_map_start(self)
            if choice is None:
                return
            if choice == 'archive':
                ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
                dst = os.path.join(MAPS_DIR, f'rtabmap_{ts}.db')
                shutil.move(RTABMAP_DB, dst)
                self._info(f'Archived old map → rtabmap_{ts}.db')
        cmd = (f'{_SETUP} && ros2 launch go2_real go2_rtabmap_mapping.launch.py'
               ' > /tmp/mapping.log 2>&1')
        self._map_proc = subprocess.Popen(['bash', '-c', cmd])
        self._map_lbl.config(text='mapping in progress…', foreground='green')
        self._info('Mapping started — drive robot to build map. Restart go2_control.sh after to resume nav.')

    def _save_map(self):
        if not os.path.exists(RTABMAP_DB):
            messagebox.showwarning('No map', 'rtabmap.db not found — is mapping running?')
            return
        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
        dst = os.path.join(MAPS_DIR, f'rtabmap_{ts}.db')
        shutil.copy2(RTABMAP_DB, dst)
        mb = os.path.getsize(dst) / 1_048_576
        self._info(f'Saved: rtabmap_{ts}.db  ({mb:.1f} MB)')
        messagebox.showinfo('Map saved', f'{dst}\n({mb:.1f} MB)')

    def _stop_mapping(self):
        subprocess.run(['pkill', '-SIGTERM', '-f', 'go2_rtabmap_mapping'],
                       capture_output=True)
        subprocess.run(['pkill', '-SIGTERM', '-f', 'rtabmap_slam'],
                       capture_output=True)
        if self._map_proc:
            try: self._map_proc.terminate()
            except Exception: pass
        self._map_proc = None
        self._map_lbl.config(text='idle', foreground='#888')
        self._info('Mapping stopped.')

    # ── Waypoints ─────────────────────────────────────────────────────────────

    def _refresh(self):
        for row in self._tree.get_children():
            self._tree.delete(row)
        wps = _load()
        for name, wp in wps.items():
            self._tree.insert('', 'end', iid=name, values=(
                name,
                wp.get('x', 0.0), wp.get('y', 0.0), wp.get('yaw', 0.0),
                wp.get('floor_id', 0), wp.get('property', 'floor'),
            ))
        self._nav_cb['values'] = list(wps.keys())
        if wps and not self._nav_var.get():
            self._nav_cb.current(0)
        self._info(f'{len(wps)} waypoints loaded')

    def _selected(self):
        sel = self._tree.selection()
        return sel[0] if sel else None

    def _capture_pose(self):
        if self._ros is None:
            messagebox.showerror('No ROS', 'ROS2 is not available.')
            return None
        self._info('Reading map → base_link TF…')
        self.update()
        pose = self._ros.map_pose()
        if pose is None:
            messagebox.showerror('TF timeout',
                'Could not read robot position.\n'
                'Make sure the nav stack or mapping is running.')
        return pose

    def _add(self):
        name = self._name_var.get().strip().replace(' ', '_')
        if not name:
            messagebox.showwarning('Name required', 'Enter a waypoint name.')
            return
        wps = _load()
        if name in wps and not messagebox.askyesno(
                'Overwrite?', f'"{name}" already exists. Overwrite?'):
            return
        pose = self._capture_pose()
        if pose is None:
            return
        x, y, yaw = pose
        wps[name] = {
            'x': x, 'y': y, 'yaw': yaw,
            'floor_id': int(self._floor_var.get()),
            'property': self._type_var.get(),
        }
        _dump(wps)
        self._name_var.set('')
        self._refresh()
        self._info(f'Added "{name}":  x={x}  y={y}  yaw={yaw}')

    def _update(self):
        name = self._selected()
        if name is None:
            messagebox.showwarning('Select', 'Select a waypoint to update.')
            return
        pose = self._capture_pose()
        if pose is None:
            return
        x, y, yaw = pose
        wps = _load()
        if name not in wps:
            messagebox.showerror('Missing', f'"{name}" not found in file.')
            return
        wps[name].update({'x': x, 'y': y, 'yaw': yaw})
        _dump(wps)
        self._refresh()
        self._info(f'Updated "{name}":  x={x}  y={y}  yaw={yaw}')

    def _remove(self):
        name = self._selected()
        if name is None:
            messagebox.showwarning('Select', 'Select a waypoint to remove.')
            return
        if not messagebox.askyesno('Confirm', f'Remove waypoint "{name}"?'):
            return
        wps = _load()
        wps.pop(name, None)
        _dump(wps)
        self._refresh()
        self._info(f'Removed "{name}"')

    # ── Status bar ────────────────────────────────────────────────────────────

    def _info(self, msg):
        self._status.config(text=msg)
        self.update_idletasks()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ros_node = None
    if HAS_ROS:
        rclpy.init()
        ros_node = _RosNode()
        threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()

    app = ControlPanel(ros_node)
    app.mainloop()

    if HAS_ROS:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
