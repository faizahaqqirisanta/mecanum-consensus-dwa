#!/usr/bin/env python3
"""
Experiment Master CLI — haqqi_ta
Dijalankan di PC MASTER.

Fitur:
  - Heartbeat /experiment_state (String, 10 Hz)
  - Load scenario dari scenarios.yaml dengan preview sebelum kirim
  - Readiness check sebelum START (pose, path, state)
  - Status live per robot: pose, WP index, sisa jarak, fault, priority stop
  - Monitor live kontinu (hingga tekan Enter)
  - Waypoints via RViz: undo, review, simpan ke YAML
  - Fault injection control dari CLI
  - Trial counter otomatis
"""

import os
import sys
import math
import time
import threading
import yaml
import json
import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import String, Bool, Float32, Int32
from ament_index_python.packages import get_package_share_directory


ROBOT_NAMESPACES = ['robot1', 'robot2', 'robot3']
SEP  = '─' * 68
SEP2 = '═' * 68
DEFAULT_FORMATION_SPACING_M = 0.30
DEFAULT_FORMATION_RADIUS_M = 0.30
DEFAULT_SHARED_GOAL_TOLERANCE_M = 0.05


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _as_xy_yaw(raw):
    if raw is None:
        return None
    if isinstance(raw, (list, tuple)):
        x = float(raw[0])
        y = float(raw[1])
        yaw = math.radians(float(raw[2])) if len(raw) > 2 else 0.0
        return x, y, yaw
    x = float(raw['x'])
    y = float(raw['y'])
    yaw = (math.radians(float(raw['yaw_deg']))
           if 'yaw_deg' in raw else float(raw.get('theta', 0.0)))
    return x, y, yaw


def _scenario_gathering_point(scenario_data):
    gp = scenario_data.get('gathering_point')
    if isinstance(gp, dict) and 'x' in gp and 'y' in gp:
        return float(gp['x']), float(gp['y'])
    return None


def _face_gathering_yaw(scenario_data, x, y, fallback_yaw):
    mode = str(scenario_data.get('final_orientation_mode', '')).strip()
    if mode != 'face_gathering_point':
        return fallback_yaw, False
    gp = _scenario_gathering_point(scenario_data)
    if gp is None:
        return fallback_yaw, False
    dx = gp[0] - float(x)
    dy = gp[1] - float(y)
    if math.hypot(dx, dy) < 1e-3:
        return fallback_yaw, False
    return math.atan2(dy, dx), True


def _final_yaw_for_robot(scenario_data, robot_cfg, fallback_yaw=0.0):
    wps = robot_cfg.get('waypoints', [])
    gp_cfg = robot_cfg.get('goal_pose')
    parsed = _as_xy_yaw(wps[-1] if wps else gp_cfg)
    if parsed is None:
        return fallback_yaw, False
    x, y, yaw = parsed
    return _face_gathering_yaw(scenario_data, x, y, yaw)


def _robot_order_key(ns):
    try:
        return ROBOT_NAMESPACES.index(ns)
    except ValueError:
        return len(ROBOT_NAMESPACES)


def _truthy(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _formation_layout(scenario_data):
    return str(scenario_data.get(
        'final_formation_layout',
        scenario_data.get('formation_layout', 'line'))).strip().lower()


def _final_target_for_robot(robot_cfg):
    wps = robot_cfg.get('waypoints', [])
    gp_cfg = robot_cfg.get('goal_pose')
    return _as_xy_yaw(wps[-1] if wps else gp_cfg)


def _shared_goal_groups(scenario_data, robot_configs):
    tol = float(scenario_data.get(
        'shared_goal_tolerance_m', DEFAULT_SHARED_GOAL_TOLERANCE_M))
    groups = []

    for ns, robot_cfg in robot_configs.items():
        parsed = _final_target_for_robot(robot_cfg)
        if parsed is None:
            continue
        x, y, yaw = parsed
        for group in groups:
            if math.hypot(x - group['anchor_x'], y - group['anchor_y']) <= tol:
                group['robots'].append(ns)
                group['targets'][ns] = (x, y, yaw)
                n = len(group['targets'])
                group['anchor_x'] = sum(v[0] for v in group['targets'].values()) / n
                group['anchor_y'] = sum(v[1] for v in group['targets'].values()) / n
                break
        else:
            groups.append({
                'anchor_x': x,
                'anchor_y': y,
                'robots': [ns],
                'targets': {ns: (x, y, yaw)},
            })

    return [g for g in groups if len(g['robots']) >= 2]


def _formation_scalar_offsets(robots, spacing):
    ordered = sorted(robots, key=_robot_order_key)
    center = (len(ordered) - 1) / 2.0
    return {
        ns: (center - idx) * spacing
        for idx, ns in enumerate(ordered)
    }


def _apply_formation_offset(anchor_x, anchor_y, anchor_yaw, scalar, offset_mode):
    if offset_mode == 'fixed_y':
        return anchor_x, anchor_y + scalar
    if offset_mode == 'fixed_x':
        return anchor_x + scalar, anchor_y
    lx = -math.sin(anchor_yaw)
    ly = math.cos(anchor_yaw)
    return anchor_x + scalar * lx, anchor_y + scalar * ly


def _circle_formation_targets(scenario_data, robot_configs, robots,
                              anchor_x, anchor_y, label):
    ordered = sorted(robots, key=_robot_order_key)
    if len(ordered) < 2:
        return {}

    radius = float(scenario_data.get(
        'formation_radius',
        scenario_data.get('formation_spacing', DEFAULT_FORMATION_RADIUS_M)))
    start_angle = math.radians(float(
        scenario_data.get('formation_start_angle_deg', 90.0)))
    direction = str(scenario_data.get(
        'formation_direction', 'ccw')).strip().lower()
    step_sign = -1.0 if direction in ('cw', 'clockwise') else 1.0
    step = step_sign * (2.0 * math.pi / len(ordered))

    orientation_mode = str(scenario_data.get(
        'final_orientation_mode',
        scenario_data.get('shared_goal_final_orientation_mode',
                          'face_gathering_point'))).strip()

    targets = {}
    for idx, ns in enumerate(ordered):
        parsed = _final_target_for_robot(robot_configs[ns])
        if parsed is None:
            continue
        _, _, fallback_yaw = parsed
        raw_angle = scenario_data.get(f'formation_angle_{ns}')
        angle = (
            math.radians(float(raw_angle))
            if raw_angle is not None else start_angle + idx * step
        )
        raw_radius = scenario_data.get(f'formation_radius_{ns}')
        r = float(raw_radius) if raw_radius is not None else radius
        x = anchor_x + r * math.cos(angle)
        y = anchor_y + r * math.sin(angle)

        if orientation_mode in ('copy_anchor', 'anchor_yaw', 'copy_waypoint'):
            yaw = fallback_yaw
        elif orientation_mode in ('face_out', 'radial_out'):
            yaw = math.atan2(y - anchor_y, x - anchor_x)
        elif math.hypot(anchor_x - x, anchor_y - y) > 1e-3:
            yaw = math.atan2(anchor_y - y, anchor_x - x)
        else:
            yaw = fallback_yaw

        targets[ns] = (
            x, y, yaw,
            f'{label}/circle_{360.0 / len(ordered):.0f}deg')

    return targets


def _scenario_circle_formation_targets(scenario_data, robot_configs):
    if _formation_layout(scenario_data) not in (
            'circle', 'circular', 'radial', 'around_gathering_point'):
        return {}
    if not _truthy(scenario_data.get('formation_at_goal', False)):
        return {}
    gp = _scenario_gathering_point(scenario_data)
    if gp is None:
        return {}
    robots = [
        ns for ns, cfg in robot_configs.items()
        if _final_target_for_robot(cfg) is not None
    ]
    return _circle_formation_targets(
        scenario_data, robot_configs, robots, gp[0], gp[1],
        'final_formation')


def _shared_goal_formation_targets(scenario_data, robot_configs):
    """Return ns -> (x, y, yaw, label) for robots sharing one final goal."""
    scenario_targets = _scenario_circle_formation_targets(
        scenario_data, robot_configs)
    if scenario_targets:
        return scenario_targets

    if str(scenario_data.get('shared_goal_formation', 'auto')).lower() in ('false', '0', 'no', 'off'):
        return {}

    spacing = float(scenario_data.get(
        'formation_spacing', DEFAULT_FORMATION_SPACING_M))
    offset_mode = str(scenario_data.get(
        'formation_offset_mode',
        scenario_data.get('offset_mode', 'lateral'))).strip()
    orientation_mode = str(scenario_data.get(
        'shared_goal_final_orientation_mode',
        scenario_data.get('final_orientation_mode', 'face_anchor'))).strip()

    formation_targets = {}
    for group in _shared_goal_groups(scenario_data, robot_configs):
        anchor_x = group['anchor_x']
        anchor_y = group['anchor_y']
        anchor_yaw = next(iter(group['targets'].values()))[2]
        if _formation_layout(scenario_data) in (
                'circle', 'circular', 'radial', 'around_gathering_point'):
            formation_targets.update(_circle_formation_targets(
                scenario_data, robot_configs, group['robots'],
                anchor_x, anchor_y, 'shared_goal_formation'))
            continue

        scalars = _formation_scalar_offsets(group['robots'], spacing)

        for ns in group['robots']:
            _, _, fallback_yaw = group['targets'][ns]
            raw_scalar = scenario_data.get(f'formation_offset_{ns}')
            scalar = (
                float(raw_scalar)
                if raw_scalar is not None and abs(float(raw_scalar)) > 1e-6
                else scalars[ns]
            )
            x, y = _apply_formation_offset(
                anchor_x, anchor_y, anchor_yaw, scalar, offset_mode)

            if orientation_mode in ('copy_anchor', 'anchor_yaw'):
                yaw = fallback_yaw
            elif math.hypot(anchor_x - x, anchor_y - y) > 1e-3:
                yaw = math.atan2(anchor_y - y, anchor_x - x)
            else:
                yaw = fallback_yaw

            formation_targets[ns] = (
                x, y, yaw,
                f'shared_goal_formation/{offset_mode}')

    return formation_targets


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class ExperimentMasterCLI(Node):

    def __init__(self):
        super().__init__('experiment_master_cli')

        # ── State machine ──────────────────────────────────────────────────
        self._state           = 'STOP'
        self._state_lock      = threading.Lock()
        self._current_scenario = None   # nama skenario aktif
        self._trial_count     = 0

        # ── Monitoring data (diisi oleh subscriber callbacks) ──────────────
        self._data_lock      = threading.Lock()
        self._robot_poses    = {}   # ns → (x, y, theta)
        self._pose_time      = {}   # ns → float (wall clock, last update)
        self._goal_reached   = {}   # ns → bool
        self._pos_reached    = {}   # ns → bool  [FIX-ARRIVE-CLI] SAMPAI-posisi (sebelum rotasi)
        self._path_length    = {}   # ns → float
        self._remaining      = {}   # ns → float
        self._waypoint_index = {}   # ns → int  (-1 = belum ada WP)
        self._wp_count       = {}   # ns → int  (total WP dari skenario, set saat load)
        self._fault_active        = {}   # ns → bool
        self._priority_stop       = {}   # ns → bool
        self._consensus_prog      = {}   # ns → float
        self._mission_remaining   = {}   # ns → float  (dari mission_remaining_length)
        self._mission_total       = {}   # ns → float  (dari mission_total_length)
        # [MERGE-MONITOR] Telemetry tambahan agar CLI selengkap sync_monitor_node
        self._vmax_consensus      = {}   # ns → float  (vmax_consensus)
        self._dwa_vmax_eff        = {}   # ns → float  (dwa_vmax_eff, fisik)
        self._dwa_speed_mag       = {}   # ns → float  (dwa_speed_mag, fisik)
        self._dwa_mode            = {}   # ns → str    (dwa_mode)
        self._detection_enabled   = None # bool|None   (dari /coordination_debug)
        self._agent_failed        = {}   # ns → bool   (dari /coordination_debug)
        # [ARRIVE-TIME] Waktu tiba relatif terhadap START trial (detik)
        self._trial_start_time    = None # float|None
        self._pos_arrival_time    = {}   # ns → float  (saat position_reached True)
        self._goal_arrival_time   = {}   # ns → float  (saat goal_reached True)

        # ── RViz manual goal / AMCL correction ────────────────────────────
        self._pending_goal       = None
        self._pending_goal_event = threading.Event()
        self._pending_amcl       = None
        self._pending_amcl_event = threading.Event()

        # ── Publishers ────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/experiment_state', 10)
        self._scenario_pub = self.create_publisher(String, '/experiment_scenario', 10)
        self.create_timer(0.1, self._heartbeat)

        self._initialpose_pubs = {
            ns: self.create_publisher(
                PoseWithCovarianceStamped, f'/{ns}/initialpose', 10)
            for ns in ROBOT_NAMESPACES}

        self._goal_pubs = {
            ns: self.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
            for ns in ROBOT_NAMESPACES}

        self._waypoints_pubs = {
            ns: self.create_publisher(PoseArray, f'/{ns}/waypoints', 10)
            for ns in ROBOT_NAMESPACES}

        self._fault_trigger_pubs = {
            ns: self.create_publisher(Bool, f'/{ns}/fault_trigger', 10)
            for ns in ROBOT_NAMESPACES}

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(PoseStamped, '/goal_pose', self._rviz_goal_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._rviz_amcl_cb, 10)

        for ns in ROBOT_NAMESPACES:
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{ns}/amcl_pose',
                lambda msg, n=ns: self._pose_cb(msg, n), 10)
            self.create_subscription(
                Bool, f'/{ns}/goal_reached',
                lambda msg, n=ns: self._goal_cb(msg, n), 10)
            # [FIX-ARRIVE-CLI] SAMPAI-posisi dari DWA (posisi tercapai, sebelum rotasi-akhir)
            self.create_subscription(
                Bool, f'/{ns}/position_reached',
                lambda msg, n=ns: self._pos_reached_cb(msg, n), 10)
            self.create_subscription(
                Float32, f'/{ns}/path_length',
                lambda msg, n=ns: self._path_length_cb(msg, n), 10)
            self.create_subscription(
                Float32, f'/{ns}/remaining_length',
                lambda msg, n=ns: self._remaining_cb(msg, n), 10)
            self.create_subscription(
                Int32, f'/{ns}/waypoint_index',
                lambda msg, n=ns: self._wp_index_cb(msg, n), 10)
            self.create_subscription(
                Bool, f'/{ns}/fault_active',
                lambda msg, n=ns: self._fault_cb(msg, n), 10)
            self.create_subscription(
                Bool, f'/{ns}/priority_stop',
                lambda msg, n=ns: self._pstop_cb(msg, n), 10)
            self.create_subscription(
                Float32, f'/{ns}/consensus_progress',
                lambda msg, n=ns: self._prog_cb(msg, n), 10)
            self.create_subscription(
                Float32, f'/{ns}/mission_remaining_length',
                lambda msg, n=ns: self._mission_rem_cb(msg, n), 10)
            self.create_subscription(
                Float32, f'/{ns}/mission_total_length',
                lambda msg, n=ns: self._mission_total_cb(msg, n), 10)
            # [MERGE-MONITOR] Telemetry tambahan (selaras sync_monitor_node)
            self.create_subscription(
                Float32, f'/{ns}/vmax_consensus',
                lambda msg, n=ns: self._vcons_cb(msg, n), 10)
            self.create_subscription(
                Float32, f'/{ns}/dwa_vmax_eff',
                lambda msg, n=ns: self._veff_cb(msg, n), 10)
            self.create_subscription(
                Float32, f'/{ns}/dwa_speed_mag',
                lambda msg, n=ns: self._dmag_cb(msg, n), 10)
            self.create_subscription(
                String, f'/{ns}/dwa_mode',
                lambda msg, n=ns: self._dmode_cb(msg, n), 10)

        # [MERGE-MONITOR] Status deteksi agen gagal (satu JSON, bukan per-robot)
        self.create_subscription(
            String, '/coordination_debug', self._coord_debug_cb, 10)

        self.get_logger().info('ExperimentMasterCLI ready | state=STOP | heartbeat=10Hz')

    # ─────────────────────────────────────────────────────────────────────
    # Callbacks (thread-safe via _data_lock)
    # ─────────────────────────────────────────────────────────────────────

    def _heartbeat(self):
        msg = String()
        with self._state_lock:
            msg.data = self._state
        self._state_pub.publish(msg)

    def _pose_cb(self, msg, ns):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._data_lock:
            self._robot_poses[ns] = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y, yaw)
            self._pose_time[ns] = time.time()

    def _goal_cb(self, msg, ns):
        with self._data_lock:
            was = self._goal_reached.get(ns, False)
            self._goal_reached[ns] = bool(msg.data)
            # [ARRIVE-TIME] Catat waktu goal-penuh sekali (transisi False->True)
            if (bool(msg.data) and not was
                    and self._trial_start_time is not None
                    and ns not in self._goal_arrival_time):
                self._goal_arrival_time[ns] = time.time() - self._trial_start_time

    def _pos_reached_cb(self, msg, ns):
        # [FIX-ARRIVE-CLI] Status SAMPAI-posisi (posisi goal tercapai, rotasi-akhir mungkin lanjut)
        with self._data_lock:
            was = self._pos_reached.get(ns, False)
            self._pos_reached[ns] = bool(msg.data)
            # [ARRIVE-TIME] Catat waktu sampai-posisi sekali (transisi False->True)
            if (bool(msg.data) and not was
                    and self._trial_start_time is not None
                    and ns not in self._pos_arrival_time):
                self._pos_arrival_time[ns] = time.time() - self._trial_start_time

    def _path_length_cb(self, msg, ns):
        with self._data_lock:
            self._path_length[ns] = float(msg.data)

    def _remaining_cb(self, msg, ns):
        with self._data_lock:
            self._remaining[ns] = float(msg.data)

    def _wp_index_cb(self, msg, ns):
        with self._data_lock:
            self._waypoint_index[ns] = int(msg.data)

    def _fault_cb(self, msg, ns):
        with self._data_lock:
            self._fault_active[ns] = bool(msg.data)

    def _pstop_cb(self, msg, ns):
        with self._data_lock:
            self._priority_stop[ns] = bool(msg.data)

    def _prog_cb(self, msg, ns):
        with self._data_lock:
            self._consensus_prog[ns] = float(msg.data)

    def _mission_rem_cb(self, msg, ns):
        with self._data_lock:
            self._mission_remaining[ns] = float(msg.data)

    def _mission_total_cb(self, msg, ns):
        with self._data_lock:
            self._mission_total[ns] = float(msg.data)

    # [MERGE-MONITOR] Callback telemetry tambahan ──────────────────────────
    def _vcons_cb(self, msg, ns):
        with self._data_lock:
            self._vmax_consensus[ns] = float(msg.data)

    def _veff_cb(self, msg, ns):
        with self._data_lock:
            self._dwa_vmax_eff[ns] = float(msg.data)

    def _dmag_cb(self, msg, ns):
        with self._data_lock:
            self._dwa_speed_mag[ns] = float(msg.data)

    def _dmode_cb(self, msg, ns):
        with self._data_lock:
            self._dwa_mode[ns] = str(msg.data)

    def _coord_debug_cb(self, msg):
        # [MERGE-MONITOR] Status deteksi & daftar robot gagal dari consensus_node
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        failed = payload.get('failed', {}) or {}
        de = payload.get('detection_enabled', None)
        with self._data_lock:
            for n in ROBOT_NAMESPACES:
                fv = failed.get(n, None)
                self._agent_failed[n] = None if fv is None else bool(fv)
            self._detection_enabled = None if de is None else bool(de)

    def _rviz_goal_cb(self, msg):
        self._pending_goal = msg
        self._pending_goal_event.set()

    def _rviz_amcl_cb(self, msg):
        self._pending_amcl = msg
        self._pending_amcl_event.set()

    def _set_state_keep_running(self, new_state):
        """[FIX-KEEPRUN] Jangan turunkan state saat misi sedang RUNNING.
        Dipakai aksi kirim goal/waypoint manual: mengirim goal TIDAK boleh
        menjatuhkan RUNNING -> READY (penyebab logger berhenti merekam
        di tengah run). Emergency Stop / Reset / Load Scenario tetap memakai
        _set_state biasa."""
        with self._state_lock:
            cur = self._state
        if cur == 'RUNNING':
            self.get_logger().warn(
                f'[FIX-KEEPRUN] Abaikan set {new_state}: misi sedang RUNNING '
                f'(goal/waypoint dikirim tanpa mengubah /experiment_state).')
            return
        self._set_state(new_state)

    def _set_state(self, new_state):
        with self._state_lock:
            prev = self._state
            self._state = new_state
        if prev != new_state:
            self.get_logger().info(f'[STATE] {prev} → {new_state}')

    # ─────────────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────────────

    # Jeda antara publish initialpose dan waypoints agar AMCL sempat konvergen.
    # Terlalu pendek → path diplan dari pose lama → m_p fluktuatif.
    # 1.5s cukup untuk AMCL memproses initialpose di kondisi normal.
    _AMCL_SETTLE_S = 1.5

    def load_scenario(self, scenario_key: str, scenario_data: dict,
                      active_robots: list):
        """
        Publish initialpose + waypoints untuk tiap robot aktif.

        Format YAML yang didukung:
          robot1:
            start_pose: {x, y, yaw_deg}
            waypoints:
              - [x, y, yaw_deg]   ← list format

        Dua fase terpisah:
          Fase 1: publish semua initialpose → tunggu AMCL settle
          Fase 2: publish semua waypoints
        Pemisahan ini menghindari race condition (waypoints tiba sebelum
        AMCL melaporkan posisi baru → path tidak terbentuk / fluktuatif).
        """
        self._current_scenario = scenario_key
        scenario_msg = String()
        scenario_msg.data = scenario_key
        self._scenario_pub.publish(scenario_msg)

        # Kumpulkan konfigurasi semua robot dulu sebelum publish apapun
        robot_configs = {}
        for ns in active_robots:
            robot_cfg = (scenario_data.get(ns)
                         or scenario_data.get('robots', {}).get(ns))
            if robot_cfg is None:
                self.get_logger().warn(f'{ns} tidak ada dalam skenario, dilewati')
                continue
            ip_cfg = robot_cfg.get('start_pose') or robot_cfg.get('initial_pose')
            if ip_cfg is None:
                self.get_logger().warn(f'{ns}: tidak ada start_pose/initial_pose')
                continue
            robot_configs[ns] = robot_cfg
        formation_targets = _shared_goal_formation_targets(
            scenario_data, robot_configs)

        # ── FASE 1: Publish semua initialpose ────────────────────────────
        for ns, robot_cfg in robot_configs.items():
            ip_cfg = robot_cfg.get('start_pose') or robot_cfg.get('initial_pose')
            theta_ip = (math.radians(float(ip_cfg['yaw_deg']))
                        if 'yaw_deg' in ip_cfg
                        else float(ip_cfg.get('theta', 0.0)))

            ip_msg = PoseWithCovarianceStamped()
            ip_msg.header.frame_id      = 'map'
            ip_msg.header.stamp         = self.get_clock().now().to_msg()
            ip_msg.pose.pose.position.x = float(ip_cfg['x'])
            ip_msg.pose.pose.position.y = float(ip_cfg['y'])
            z, w = yaw_to_quat(theta_ip)
            ip_msg.pose.pose.orientation.z = z
            ip_msg.pose.pose.orientation.w = w
            ip_msg.pose.covariance[0]  = 0.04
            ip_msg.pose.covariance[7]  = 0.04
            ip_msg.pose.covariance[35] = 0.03
            self._initialpose_pubs[ns].publish(ip_msg)
            self.get_logger().info(
                f'  {ns}: initialpose → ({ip_cfg["x"]}, {ip_cfg["y"]})')

        # Tunggu AMCL memproses initialpose sebelum mengirim waypoints
        self.get_logger().info(
            f'  Menunggu {self._AMCL_SETTLE_S:.1f}s agar AMCL konvergen...')
        time.sleep(self._AMCL_SETTLE_S)

        # ── FASE 2: Publish semua waypoints ──────────────────────────────
        for ns, robot_cfg in robot_configs.items():
            ip_cfg  = robot_cfg.get('start_pose') or robot_cfg.get('initial_pose')
            wps_raw = robot_cfg.get('waypoints', [])
            gp_cfg  = robot_cfg.get('goal_pose')

            if wps_raw:
                wp_array = PoseArray()
                wp_array.header.frame_id = 'map'
                wp_array.header.stamp    = self.get_clock().now().to_msg()

                final_override_yaw, final_override_applied = _final_yaw_for_robot(
                    scenario_data, robot_cfg)
                formation_target = formation_targets.get(ns)

                for idx, wp in enumerate(wps_raw):
                    p = Pose()
                    x_wp, y_wp, yaw_wp = _as_xy_yaw(wp)
                    if idx == len(wps_raw) - 1 and formation_target is not None:
                        x_wp, y_wp, yaw_wp, _ = formation_target
                    p.position.x = x_wp
                    p.position.y = y_wp
                    if (idx == len(wps_raw) - 1
                            and formation_target is None
                            and final_override_applied):
                        yaw_wp = final_override_yaw
                    z_q, w_q = yaw_to_quat(yaw_wp)
                    p.orientation.z = z_q
                    p.orientation.w = w_q
                    wp_array.poses.append(p)

                self._waypoints_pubs[ns].publish(wp_array)

                with self._data_lock:
                    self._wp_count[ns] = len(wps_raw)

                last_wp = wps_raw[-1]
                if isinstance(last_wp, (list, tuple)):
                    final_x = float(last_wp[0])
                    final_y = float(last_wp[1])
                else:
                    final_x = float(last_wp['x'])
                    final_y = float(last_wp['y'])
                formation_label = ''
                if formation_target is not None:
                    final_x, final_y, final_yaw, formation_tag = formation_target
                    formation_label = (
                        f' {formation_tag} yaw={math.degrees(final_yaw):.1f}°')

                self.get_logger().info(
                    f'  {ns}: start=({ip_cfg["x"]}, {ip_cfg["y"]}) '
                    f'| {len(wps_raw)} waypoints '
                    f'→ final=({final_x:.2f},{final_y:.2f})'
                    + formation_label
                    + (f' yaw_face={math.degrees(final_override_yaw):.1f}°'
                       if final_override_applied and formation_target is None else ''))

            elif gp_cfg is not None:
                # Fallback: format lama (single goal_pose)
                x_gp, y_gp, theta_gp = _as_xy_yaw(gp_cfg)
                formation_target = formation_targets.get(ns)
                if formation_target is not None:
                    x_gp, y_gp, theta_gp, formation_tag = formation_target
                    face_applied = False
                else:
                    theta_gp, face_applied = _face_gathering_yaw(
                        scenario_data, x_gp, y_gp, theta_gp)
                gp_msg = PoseStamped()
                gp_msg.header.frame_id = 'map'
                gp_msg.header.stamp    = self.get_clock().now().to_msg()
                gp_msg.pose.position.x = x_gp
                gp_msg.pose.position.y = y_gp
                z, w = yaw_to_quat(theta_gp)
                gp_msg.pose.orientation.z = z
                gp_msg.pose.orientation.w = w
                self._goal_pubs[ns].publish(gp_msg)

                with self._data_lock:
                    self._wp_count[ns] = 1

                self.get_logger().info(
                    f'  {ns}: pose=({ip_cfg["x"]}, {ip_cfg["y"]}) '
                    f'→ goal=({x_gp}, {y_gp})'
                    + (f' {formation_tag} yaw={math.degrees(theta_gp):.1f}°'
                       if formation_target is not None else '')
                    + (f' yaw_face={math.degrees(theta_gp):.1f}°'
                       if face_applied else ''))
            else:
                self.get_logger().warn(f'{ns}: tidak ada waypoints maupun goal_pose')

        self._set_state('READY')

    def localized_robots(self, active_robots: list) -> list:
        """[FIX-LOCGATE] Robot dengan pose AMCL segar (<2s)."""
        st  = self.get_status()
        now = time.time()
        ready = []
        for ns in active_robots:
            last = st['pose_time'].get(ns, 0)
            if (now - last) < 2.0 and ns in st['poses']:
                ready.append(ns)
        return ready

    def missing_localized_robots(self, active_robots: list) -> list:
        """[FIX-LOCGATE2] Robot aktif yang pose AMCL-nya belum segar.

        Untuk eksperimen TA 3-robot, START dengan 1 robot saja belum localized
        tetap membuat metrik sinkronisasi/adaptasi jaringan tidak valid. Karena
        itu pose semua active_robots harus fresh; tidak ada force-start untuk
        kasus pose hilang/stale.
        """
        localized = set(self.localized_robots(active_robots))
        return [ns for ns in active_robots if ns not in localized]

    def start(self):
        with self._state_lock:
            self._trial_count += 1
        # [ARRIVE-TIME] Tandai awal trial & reset catatan waktu tiba
        with self._data_lock:
            self._trial_start_time = time.time()
            self._pos_arrival_time.clear()
            self._goal_arrival_time.clear()
        self._set_state('RUNNING')

    def emergency_stop(self):
        self._set_state('STOP')

    def reset(self):
        with self._data_lock:
            self._goal_reached.clear()
            self._pos_reached.clear()
            self._remaining.clear()
            self._mission_remaining.clear()
            self._mission_total.clear()
            # [ARRIVE-TIME] Reset catatan waktu tiba antar-trial
            self._pos_arrival_time.clear()
            self._goal_arrival_time.clear()
            self._trial_start_time = None
        self._set_state('READY')

    def get_status(self) -> dict:
        with self._state_lock:
            state    = self._state
            trial    = self._trial_count
            scenario = self._current_scenario
        with self._data_lock:
            return {
                'state'          : state,
                'trial'          : trial,
                'scenario'       : scenario,
                'poses'          : dict(self._robot_poses),
                'pose_time'      : dict(self._pose_time),
                'goal_reached'   : dict(self._goal_reached),
                'pos_reached'    : dict(self._pos_reached),
                'path_length'    : dict(self._path_length),
                'remaining'      : dict(self._remaining),
                'waypoint_index' : dict(self._waypoint_index),
                'wp_count'       : dict(self._wp_count),
                'fault_active'   : dict(self._fault_active),
                'priority_stop'  : dict(self._priority_stop),
                'consensus_prog'    : dict(self._consensus_prog),
                'mission_remaining' : dict(self._mission_remaining),
                'mission_total'     : dict(self._mission_total),
                # [MERGE-MONITOR] telemetry tambahan
                'vmax_consensus'    : dict(self._vmax_consensus),
                'dwa_vmax_eff'      : dict(self._dwa_vmax_eff),
                'dwa_speed_mag'     : dict(self._dwa_speed_mag),
                'dwa_mode'          : dict(self._dwa_mode),
                'detection_enabled' : self._detection_enabled,
                'agent_failed'      : dict(self._agent_failed),
                # [ARRIVE-TIME]
                'pos_arrival_time'  : dict(self._pos_arrival_time),
                'goal_arrival_time' : dict(self._goal_arrival_time),
                'trial_start_time'  : self._trial_start_time,
            }

    def check_readiness(self, active_robots: list) -> list:
        """Return list of (ok: bool, label: str) checks."""
        st  = self.get_status()
        now = time.time()
        checks = []

        sc_ok = st['scenario'] is not None
        checks.append((sc_ok, f'scenario dimuat: {st["scenario"] or "belum"}'))
        checks.append((st['state'] == 'READY',
                        f'state = {st["state"]} (harus READY)'))

        for ns in active_robots:
            last  = st['pose_time'].get(ns, 0)
            fresh = (now - last) < 2.0 and ns in st['poses']
            age   = f'{now - last:.1f}s lalu' if ns in st['pose_time'] else 'tidak ada'
            checks.append((fresh, f'{ns}: pose AMCL {"OK (" + age + ")" if fresh else "belum/stale (" + age + ")"}'))

        for ns in active_robots:
            pl = st['path_length'].get(ns)
            ok = pl is not None and float(pl) > 0.1
            label = f'OK {pl:.2f}m' if ok else 'belum terbentuk'
            checks.append((ok, f'{ns}: path {label}'))

        return checks

    def wait_for_rviz_goal(self, timeout=60.0):
        """Tunggu klik 2D Goal Pose di RViz. Return PoseStamped atau None."""
        self._pending_goal = None
        self._pending_goal_event.clear()
        if self._pending_goal_event.wait(timeout=timeout):
            return self._pending_goal
        return None

    def wait_for_rviz_amcl(self, timeout=60.0):
        """Tunggu klik 2D Pose Estimate di RViz. Return PoseWithCovarianceStamped atau None."""
        self._pending_amcl = None
        self._pending_amcl_event.clear()
        if self._pending_amcl_event.wait(timeout=timeout):
            return self._pending_amcl
        return None

    def send_amcl_pose(self, ns: str, pose_msg: PoseWithCovarianceStamped):
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        # RViz 2D Pose Estimate sering membawa covariance default 0.25
        # (sigma 0.5 m). Untuk arena kecil, itu terlalu lebar dan membuat
        # AMCL mulai trial dalam keadaan "tidak valid".
        pose_msg.pose.covariance[0]  = min(float(pose_msg.pose.covariance[0] or 0.04), 0.04)
        pose_msg.pose.covariance[7]  = min(float(pose_msg.pose.covariance[7] or 0.04), 0.04)
        pose_msg.pose.covariance[35] = min(float(pose_msg.pose.covariance[35] or 0.03), 0.03)
        self._initialpose_pubs[ns].publish(pose_msg)
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        self.get_logger().info(f'AMCL pose → {ns}: ({x:.2f}, {y:.2f})')

    def send_goal(self, ns: str, goal_msg: PoseStamped):
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp    = self.get_clock().now().to_msg()
        self._goal_pubs[ns].publish(goal_msg)
        self.get_logger().info(
            f'Manual goal → {ns}: ({goal_msg.pose.position.x:.2f},'
            f'{goal_msg.pose.position.y:.2f})')

    def send_waypoints(self, ns: str, waypoints: list):
        """Kirim daftar waypoints (list of PoseStamped) ke robot."""
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        for wp in waypoints:
            msg.poses.append(wp.pose)
        self._waypoints_pubs[ns].publish(msg)
        self.get_logger().info(f'Manual waypoints → {ns}: {len(waypoints)} titik')

    def trigger_fault(self, ns: str, active: bool):
        """Trigger atau clear fault injection secara manual."""
        msg = Bool()
        msg.data = active
        self._fault_trigger_pubs[ns].publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Utility
# ─────────────────────────────────────────────────────────────────────────────

def load_scenarios_yaml() -> dict:
    try:
        share = get_package_share_directory('haqqi_ta')
        path  = os.path.join(share, 'param', 'scenarios.yaml')
    except Exception:
        path = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', 'param', 'scenarios.yaml'))

    if not os.path.exists(path):
        print(f'  [WARN] scenarios.yaml tidak ditemukan di: {path}')
        return {}, ''

    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    scenarios = data.get('scenarios', {})
    # [MOD-MENUORDER] urutan nomor di menu: convoy(1), split(2), merge(3), crossing(4)
    desired_order = ['convoy', 'split', 'merge', 'crossing']
    ordered = {k: scenarios[k] for k in desired_order if k in scenarios}
    for k, v in scenarios.items():
        if k not in ordered:
            ordered[k] = v
    return ordered, path


# ─────────────────────────────────────────────────────────────────────────────
# Display helpers
# ─────────────────────────────────────────────────────────────────────────────

def _fmt_pose(p):
    if p is None:
        return '(  ---  ,  ---  )  ---°'
    return f'({p[0]:6.2f},{p[1]:6.2f}) {math.degrees(p[2]):4.0f}°'


def _render_header(st: dict, active_robots: list) -> str:
    scenario_str = st['scenario'] or 'belum dipilih'
    trial_str    = f'Trial #{st["trial"]}' if st['trial'] > 0 else 'Trial -'
    state_str    = st['state']
    now          = time.time()

    lines = [
        SEP2,
        f'  State: {state_str:<8}  │  Scenario: {scenario_str:<18}  │  {trial_str}',
        SEP,
    ]

    # [MERGE-MONITOR] Ringkasan sinkronisasi + status deteksi agen gagal
    p_vals = {}
    for ns in active_robots:
        mr = st['mission_remaining'].get(ns)
        mt = st['mission_total'].get(ns)
        if mr is not None and mt is not None and mt > 0.05:
            p_vals[ns] = max(0.0, min(1.0, 1.0 - mr / mt))
        else:
            p_vals[ns] = st['consensus_prog'].get(ns)
    valid_p = [v for v in p_vals.values() if v is not None]
    p_avg = sum(valid_p) / len(valid_p) if valid_p else None
    p_spread = (max(valid_p) - min(valid_p)) if len(valid_p) > 1 else None
    leader = lagger = None
    if len(valid_p) > 1:
        leader = max(active_robots, key=lambda n: p_vals[n] if p_vals[n] is not None else -1)
        lagger = min(active_robots, key=lambda n: p_vals[n] if p_vals[n] is not None else 2)
    p_avg_str  = f'{p_avg:.3f}'    if p_avg    is not None else ' --- '
    spread_str = f'{p_spread:.3f}' if p_spread is not None else ' --- '
    lead_str   = leader if (leader and p_spread is not None and p_spread > 0.05) else '-'
    lag_str    = lagger if (lagger and p_spread is not None and p_spread > 0.05) else '-'
    det = st.get('detection_enabled')
    det_str = '---' if det is None else ('ON' if det else 'OFF')
    failed_robots = [n for n in active_robots if st.get('agent_failed', {}).get(n)]
    failed_str = ', '.join(failed_robots) if failed_robots else 'none'
    lines.append(
        f'  sync │ p_avg={p_avg_str} spread={spread_str} '
        f'lead={lead_str} lag={lag_str} │ detection={det_str} failed=[{failed_str}]')
    lines.append(SEP)

    for ns in active_robots:
        pose    = st['poses'].get(ns)
        pt      = st['pose_time'].get(ns, 0)
        fresh   = (now - pt) < 2.0
        pstr    = _fmt_pose(pose) if (pose and fresh) else '(  ---  ,  ---  )  ???'
        stale   = '' if fresh or pt == 0 else '?'

        wpi     = st['waypoint_index'].get(ns, -1)
        wptotal = st['wp_count'].get(ns, 0)
        wp_str  = f'WP {wpi+1:1d}/{wptotal}' if wptotal > 0 else 'WP -/-'

        # Tampilkan mission remaining jika tersedia (tidak reset saat pindah WP)
        # Fallback ke per-segmen jika mission metric belum ada
        m_rem   = st['mission_remaining'].get(ns)
        m_tot   = st['mission_total'].get(ns)
        using_mission = (m_rem is not None and m_tot is not None
                         and m_tot > 0.1)
        if using_mission:
            rem_str = f'{m_rem:5.2f}m[M]'
        else:
            rem     = st['remaining'].get(ns)
            rem_str = f'{rem:5.2f}m   ' if rem is not None else ' ---    '

        prog     = st['consensus_prog'].get(ns)
        prog_tag = '[M]' if using_mission else '   '
        prog_str = f'p={prog:.2f}{prog_tag}' if prog is not None else 'p=---   '

        fault   = '!ON!' if st['fault_active'].get(ns) else ' OFF'
        pstop   = '!ON!' if st['priority_stop'].get(ns) else ' OFF'

        # [MERGE-MONITOR] Telemetry kecepatan + mode DWA (dari sync_monitor)
        vc = st.get('vmax_consensus', {}).get(ns)
        ve = st.get('dwa_vmax_eff', {}).get(ns)
        mg = st.get('dwa_speed_mag', {}).get(ns)
        md = st.get('dwa_mode', {}).get(ns)
        vc_str = f'{vc:.2f}' if vc is not None else ' -- '
        ve_str = f'{ve:.2f}' if ve is not None else ' -- '
        mg_str = f'{mg:.2f}' if mg is not None else ' -- '
        md_str = (md or '---')[:6]
        fail_str = ' FAIL' if st.get('agent_failed', {}).get(ns) else ''

        # [FIX-ARRIVE-CLI] [POS] = posisi sudah sampai (rotasi-akhir berjalan); [DONE] = goal penuh
        # [ARRIVE-TIME] tampilkan waktu tiba (detik dari START) saat status YES
        if st['goal_reached'].get(ns):
            gt = st.get('goal_arrival_time', {}).get(ns)
            reached = ' [DONE]' + (f' t={gt:.1f}s' if gt is not None else '')
        elif st.get('pos_reached', {}).get(ns):
            pt = st.get('pos_arrival_time', {}).get(ns)
            reached = ' [POS]' + (f' t={pt:.1f}s' if pt is not None else '')
        else:
            reached = ''

        lines.append(
            f'  {ns}{stale} │ {pstr} │ {wp_str} │ rem{rem_str} │ '
            f'{prog_str} │ vc{vc_str} ve{ve_str} mg{mg_str} {md_str:<6} │ '
            f'fault{fault} │ stop{pstop}{reached}{fail_str}')

    lines.append(SEP2)
    return '\n'.join(lines)


def _preview_scenario(key: str, sc: dict, active_robots: list):
    """Tampilkan ringkasan skenario sebelum dikirim ke robot."""
    print()
    print(f'  ┌─ Preview Skenario {"─"*46}┐')
    print(f'  │  Nama     : {key}')
    desc = sc.get('description', '')
    print(f'  │  Deskripsi: {desc}')
    print(f'  └{"─"*64}┘')
    print()

    robot_configs = {}
    for ns in active_robots:
        robot_cfg = sc.get(ns) or sc.get('robots', {}).get(ns)
        if robot_cfg is not None:
            robot_configs[ns] = robot_cfg
    formation_targets = _shared_goal_formation_targets(sc, robot_configs)

    for ns in active_robots:
        robot_cfg = robot_configs.get(ns)
        if robot_cfg is None:
            print(f'  {ns}: [tidak ada dalam skenario]')
            continue

        ip = robot_cfg.get('start_pose') or robot_cfg.get('initial_pose') or {}
        yaw_s = ip.get('yaw_deg', math.degrees(ip.get('theta', 0.0)))
        print(f'  {ns}:')
        print(f'    start     : ({float(ip.get("x",0)):.2f}, {float(ip.get("y",0)):.2f}, {yaw_s:.0f}°)')

        wps = robot_cfg.get('waypoints', [])
        gp  = robot_cfg.get('goal_pose')

        if wps:
            final_override_yaw, final_override_applied = _final_yaw_for_robot(sc, robot_cfg)
            formation_target = formation_targets.get(ns)
            for i, wp in enumerate(wps):
                is_final = (i == len(wps) - 1)
                label = 'FINAL' if is_final else 'intermediate'
                xw, yw, yaw = _as_xy_yaw(wp)
                formation_tag = None
                if is_final and formation_target is not None:
                    xw, yw, yaw, formation_tag = formation_target
                elif is_final and final_override_applied:
                    yaw = final_override_yaw
                yw_deg = math.degrees(yaw)
                tag = f'[{label}]'
                if formation_tag:
                    tag += f' [{formation_tag}]'
                if is_final and final_override_applied and formation_tag is None:
                    tag += ' [face_gathering_point]'
                print(f'    WP {i+1:2d}    : ({xw:.2f}, {yw:.2f}, {yw_deg:.0f}°)  {tag}')
        elif gp:
            xg, yg, yaw = _as_xy_yaw(gp)
            formation_target = formation_targets.get(ns)
            formation_tag = None
            if formation_target is not None:
                xg, yg, yaw, formation_tag = formation_target
                face_applied = False
            else:
                yaw, face_applied = _face_gathering_yaw(sc, xg, yg, yaw)
            tag = '[FINAL]'
            if formation_tag:
                tag += f' [{formation_tag}]'
            if face_applied:
                tag += ' [face_gathering_point]'
            print(f'    goal      : ({xg:.2f}, {yg:.2f}, {math.degrees(yaw):.0f}°)  {tag}')
        else:
            print(f'    [tidak ada waypoints/goal]')
        print()


# ─────────────────────────────────────────────────────────────────────────────
# Menu functions
# ─────────────────────────────────────────────────────────────────────────────

def _menu_load_scenario(cli_node, scenarios, active_robots):
    st = cli_node.get_status()
    if st['state'] == 'RUNNING':
        print('  [!] Trial masih RUNNING. STOP dulu sebelum Load Scenario baru.')
        print('      Urutan aman: [6] EMERGENCY STOP → [1] Load Scenario → [4] START Trial.')
        return False

    if not scenarios:
        print('  [!] Tidak ada skenario di scenarios.yaml.')
        return False

    print()
    for i, (key, sc) in enumerate(scenarios.items(), 1):
        desc = sc.get('description', '')
        print(f'  [{i}] {key}')
        if desc:
            print(f'       {desc}')
    print()

    choice = input('  Pilih nomor skenario (Enter = batal): ').strip()
    if not choice:
        return False
    try:
        idx = int(choice) - 1
        key = list(scenarios.keys())[idx]
        sc  = scenarios[key]
    except (ValueError, IndexError):
        print('  [!] Pilihan tidak valid.')
        return False

    # Preview dulu sebelum kirim
    _preview_scenario(key, sc, active_robots)

    confirm = input('  Kirim skenario ini ke robot? [Enter]=Ya  [n]=Batal: ').strip().lower()
    if confirm == 'n':
        print('  Dibatalkan.')
        return False

    print(f'\n  Mengirim skenario "{key}"...')
    cli_node.load_scenario(key, sc, active_robots)
    print()
    print('  Langkah selanjutnya:')
    print('  1. Initialpose preset sudah dikirim otomatis ke semua robot.')
    print('     Pantau RViz tiap robot: particle cloud harus mengecil.')
    print('  2. Jika AMCL perlu dikoreksi manual, gunakan:')
    print('     [8] Manual Goal/Waypoints → pilih robot → [5] Koreksi AMCL via RViz')
    print('     → klik "2D Pose Estimate" di RViz PC Master → diteruskan ke robot yang dipilih.')
    print('  3. Path hijau (global_path_node) muncul otomatis setelah AMCL konvergen.')
    print()
    print('  Gunakan [3] Readiness Check untuk memastikan robot siap sebelum START.')
    return True


def _menu_preview_scenario(scenarios, active_robots):
    if not scenarios:
        print('  [!] Tidak ada skenario di scenarios.yaml.')
        return
    print()
    for i, (key, sc) in enumerate(scenarios.items(), 1):
        desc = sc.get('description', '')
        print(f'  [{i}] {key}: {desc}')
    print()
    choice = input('  Preview skenario nomor (Enter = batal): ').strip()
    if not choice:
        return
    try:
        idx = int(choice) - 1
        key = list(scenarios.keys())[idx]
        sc  = scenarios[key]
    except (ValueError, IndexError):
        print('  [!] Pilihan tidak valid.')
        return
    _preview_scenario(key, sc, active_robots)


def _menu_readiness_check(cli_node, active_robots):
    """Tampilkan status kesiapan semua robot dan beri rekomendasi."""
    print()
    print(f'  READINESS CHECK')
    print(f'  {SEP[:50]}')

    checks = cli_node.check_readiness(active_robots)
    all_ok = all(ok for ok, _ in checks)
    fail_count = sum(1 for ok, _ in checks if not ok)

    for ok, label in checks:
        mark = '✓' if ok else '✗'
        print(f'  [{mark}] {label}')

    print(f'  {SEP[:50]}')
    if all_ok:
        print('  Semua checks OK. Robot siap untuk START.')
    else:
        _print_readiness_recommendations(checks, active_robots, fail_count)


def _print_readiness_recommendations(checks, active_robots, fail_count=None):
    """Cetak rekomendasi readiness yang sesuai arsitektur UDP multi-domain."""
    failed = [label for ok, label in checks if not ok]
    if fail_count is None:
        fail_count = len(failed)
    print(f'  {fail_count} check gagal.')
    print('  Pastikan:')

    pose_missing = [
        ns for ns in active_robots
        if any(label.startswith(f'{ns}: pose AMCL') for label in failed)
    ]
    path_missing = [
        ns for ns in active_robots
        if any(label.startswith(f'{ns}: path') for label in failed)
    ]
    all_robot_telemetry_missing = (
        len(pose_missing) == len(active_robots)
        and len(path_missing) == len(active_robots)
    )

    if all_robot_telemetry_missing:
        print('  → PC master belum menerima telemetry balik dari robot.')
        print('    Cek pc_master_ip pada robot*_haqqi.launch, udp_sender_node,')
        print('    jaringan/port UDP 9001-9003 dan 9031-9033, lalu relaunch robot.')
        print('    Jika tetap START, robot bisa jalan lokal tanpa koordinasi L4/L5.')
    else:
        if pose_missing:
            print('  → Set 2D Pose Estimate di RViz dan pastikan AMCL robot sudah publish.')
            print('    Jika RViz robot OK tapi CLI tetap stale, cek UDP robot→PC.')
        if path_missing:
            print('  → Tunggu global_path_node di robot merencanakan path (biasanya <5s).')
            print('    Jika path terlihat di RViz robot tapi CLI tetap kosong, cek UDP path telemetry.')
    if any('state' in label for label in failed):
        print('  → Jalankan [1] Load Scenario terlebih dahulu.')


def _menu_monitor_live(cli_node, active_robots):
    """Monitor live kontinu hingga tekan Enter."""
    print()
    print('  Monitoring live... (tekan Enter untuk berhenti)')
    print()

    stop_event = threading.Event()

    def _input_watcher():
        input()
        stop_event.set()

    t = threading.Thread(target=_input_watcher, daemon=True)
    t.start()

    first = True
    prev_lines = 0
    try:
        while not stop_event.is_set():
            st = cli_node.get_status()
            header = _render_header(st, active_robots)
            if not first:
                # Geser cursor ke atas untuk timpa baris sebelumnya.
                # [MERGE-MONITOR] hitung jumlah baris dinamis (header kini variabel).
                sys.stdout.write(f'\033[{prev_lines}A\033[J')
            sys.stdout.write(header + '\n')
            sys.stdout.flush()
            prev_lines = header.count('\n') + 1
            first = False
            time.sleep(0.4)
    except KeyboardInterrupt:
        pass

    print()


def _menu_manual_goal(cli_node, active_robots, yaml_path):
    """[8] Manual goal atau waypoints — RViz atau ketik, dengan undo & simpan."""
    print()
    print('  ── Manual Goal / Waypoints ───────────────────────────��──────')
    print('  Pilih robot:')
    for i, ns in enumerate(active_robots, 1):
        print(f'    [{i}] {ns}')
    ns_choice = input('  Robot (Enter=batal): ').strip()
    if not ns_choice:
        return
    try:
        ns = active_robots[int(ns_choice) - 1]
    except (ValueError, IndexError):
        print('  [!] Pilihan tidak valid.')
        return

    print(f'\n  Robot: {ns}')
    print('  Mode input:')
    print('  [1] Single goal via RViz')
    print('  [2] Waypoints via RViz (klik berulang)')
    print('  [3] Single goal ketik koordinat')
    print('  [4] Waypoints ketik koordinat')
    print('  [5] Koreksi AMCL pose via RViz (2D Pose Estimate)')
    mode = input('  Mode (Enter=batal): ').strip()
    if not mode:
        return

    if mode == '1':
        print(f'\n  Klik "2D Goal Pose" di RViz PC Master untuk {ns}...')
        goal = cli_node.wait_for_rviz_goal(timeout=60.0)
        if goal is None:
            print('  [!] Timeout. Tidak ada klik diterima.')
            return
        x, y = goal.pose.position.x, goal.pose.position.y
        confirm = input(f'  Goal: ({x:.2f}, {y:.2f}) — kirim? [Enter]=Ya [n]=Batal: ').strip()
        if confirm.lower() == 'n':
            return
        cli_node.send_goal(ns, goal)
        cli_node._set_state_keep_running('READY')
        print(f'  Goal dikirim ke {ns}.')

    elif mode == '2':
        print(f'\n  Kumpulkan waypoints untuk {ns} via RViz.')
        print('  Setiap klik "2D Goal Pose" = satu waypoint.')
        print('  Enter  = tambah | u = undo terakhir | l = lihat daftar | d = selesai')
        waypoints = []  # list of PoseStamped

        while True:
            n = len(waypoints) + 1
            print(f'  Menunggu waypoint {n} (30 detik)...')
            goal = cli_node.wait_for_rviz_goal(timeout=30.0)
            if goal is None:
                print('  Timeout — dianggap selesai.')
                break
            x, y = goal.pose.position.x, goal.pose.position.y
            action = input(
                f'  WP {n}: ({x:.2f}, {y:.2f}) '
                '— [Enter]=tambah  [n]=lewati  [u]=undo  [l]=lihat  [d]=selesai: '
            ).strip().lower()

            if action == 'd':
                break
            elif action == 'n':
                continue
            elif action == 'u':
                if waypoints:
                    removed = waypoints.pop()
                    rx, ry = removed.pose.position.x, removed.pose.position.y
                    print(f'  Undo: WP {len(waypoints)+1} ({rx:.2f}, {ry:.2f}) dihapus.')
                else:
                    print('  Tidak ada waypoint untuk di-undo.')
            elif action == 'l':
                if not waypoints:
                    print('  Daftar kosong.')
                else:
                    print(f'  Daftar {len(waypoints)} waypoint untuk {ns}:')
                    for i, wp in enumerate(waypoints, 1):
                        print(f'    {i}. ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})')
            else:
                waypoints.append(goal)
                print(f'  WP {len(waypoints)} ditambahkan: ({x:.2f}, {y:.2f})')

        if not waypoints:
            print('  Tidak ada waypoint.')
            return

        # Review final
        print(f'\n  Final waypoints untuk {ns}:')
        for i, wp in enumerate(waypoints, 1):
            label = '[FINAL]' if i == len(waypoints) else '[intermediate]'
            print(f'    {i}. ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}) {label}')

        confirm = input('\n  Kirim? [Enter]=Ya  [n]=Batal  [e]=Edit ulang: ').strip().lower()
        if confirm == 'n':
            print('  Dibatalkan.')
            return
        elif confirm == 'e':
            print('  Edit ulang: hapus waypoint terakhir satu per satu.')
            while waypoints:
                wp = waypoints[-1]
                x, y = wp.pose.position.x, wp.pose.position.y
                c = input(f'  Hapus WP {len(waypoints)} ({x:.2f}, {y:.2f})? [Enter]=hapus [k]=simpan: ').strip().lower()
                if c != 'k':
                    waypoints.pop()
                else:
                    break
            if not waypoints:
                print('  Tidak ada waypoint tersisa.')
                return

        cli_node.send_waypoints(ns, waypoints)
        cli_node._set_state_keep_running('READY')
        print(f'  {len(waypoints)} waypoints dikirim ke {ns}.')

        # Tawarkan simpan ke YAML
        if yaml_path:
            save = input('\n  Simpan sebagai skenario baru di scenarios.yaml? [y/n]: ').strip().lower()
            if save == 'y':
                _save_waypoints_to_yaml(yaml_path, ns, waypoints, active_robots)

    elif mode == '3':
        line = input('  Koordinat x y yaw_deg (contoh: 2.5 1.0 90): ').strip()
        if not line:
            return
        try:
            parts = line.split()
            x, y  = float(parts[0]), float(parts[1])
            theta = math.radians(float(parts[2])) if len(parts) > 2 else 0.0
        except (ValueError, IndexError):
            print('  [!] Format salah.')
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        z_q, w_q = yaw_to_quat(theta)
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = z_q
        goal.pose.orientation.w = w_q
        cli_node.send_goal(ns, goal)
        cli_node._set_state_keep_running('READY')
        print(f'  Goal ({x:.2f}, {y:.2f}) dikirim ke {ns}.')

    elif mode == '4':
        print('  Ketik x y yaw_deg per baris. Baris kosong = selesai.')
        waypoints = []
        while True:
            line = input(f'  WP {len(waypoints)+1} (Enter=selesai): ').strip()
            if not line:
                break
            try:
                parts = line.split()
                x, y  = float(parts[0]), float(parts[1])
                theta = math.radians(float(parts[2])) if len(parts) > 2 else 0.0
            except (ValueError, IndexError):
                print('  Format salah.')
                continue
            wp = PoseStamped()
            wp.header.frame_id = 'map'
            z_q, w_q = yaw_to_quat(theta)
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.orientation.z = z_q
            wp.pose.orientation.w = w_q
            waypoints.append(wp)
            print(f'  WP {len(waypoints)}: ({x:.2f}, {y:.2f}, {math.degrees(theta):.0f}°)')

        if not waypoints:
            print('  Tidak ada waypoint.')
            return
        cli_node.send_waypoints(ns, waypoints)
        cli_node._set_state_keep_running('READY')
        print(f'  {len(waypoints)} waypoint dikirim ke {ns}.')

        if yaml_path:
            save = input('\n  Simpan sebagai skenario baru di scenarios.yaml? [y/n]: ').strip().lower()
            if save == 'y':
                _save_waypoints_to_yaml(yaml_path, ns, waypoints, [ns])

    elif mode == '5':
        print(f'\n  Klik "2D Pose Estimate" di RViz PC Master untuk {ns} (60 detik)...')
        pose = cli_node.wait_for_rviz_amcl(timeout=60.0)
        if pose is None:
            print('  [!] Timeout. Tidak ada klik diterima.')
            return
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        confirm = input(
            f'  Pose: ({x:.2f}, {y:.2f}) — kirim ke {ns}? [Enter]=Ya [n]=Batal: '
        ).strip().lower()
        if confirm == 'n':
            print('  Dibatalkan.')
            return
        cli_node.send_amcl_pose(ns, pose)
        print(f'  AMCL pose dikirim ke {ns}.')
        print(f'  → udp_bridge_pc akan forward ke robot via UDP.')

    else:
        print('  [!] Mode tidak dikenali.')


def _save_waypoints_to_yaml(yaml_path, ns, waypoints, active_robots):
    """Simpan waypoints yang baru diklik ke scenarios.yaml."""
    key = input('  Nama skenario baru (contoh: crossing_v2): ').strip()
    if not key:
        print('  Dibatalkan.')
        return
    desc = input('  Deskripsi singkat: ').strip()

    robots_data = {
        ns: {
            'start_pose': {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0},
            'waypoints': [
                [round(wp.pose.position.x, 3),
                 round(wp.pose.position.y, 3),
                 round(math.degrees(math.atan2(
                     2*(wp.pose.orientation.w * wp.pose.orientation.z),
                     1 - 2*wp.pose.orientation.z**2)), 1)]
                for wp in waypoints
            ]
        }
    }
    # Ingatkan user bahwa start_pose masih default
    print('  CATATAN: start_pose diset ke (0,0,0) — edit manual di YAML setelah simpan.')

    try:
        with open(yaml_path, 'r') as f:
            full = yaml.safe_load(f) or {}
        if 'scenarios' not in full:
            full['scenarios'] = {}
        full['scenarios'][key] = {'description': desc}
        full['scenarios'][key].update(robots_data)
        with open(yaml_path, 'w') as f:
            yaml.dump(full, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        print(f'  Tersimpan ke {yaml_path} dengan key "{key}".')
        print(f'  Edit start_pose di YAML sebelum dipakai!')
    except Exception as e:
        print(f'  [!] Gagal simpan: {e}')


def _menu_fault_control(cli_node, active_robots):
    """[9] Kontrol fault injection dari CLI."""
    print()
    print('  FAULT INJECTION CONTROL')
    print(f'  {SEP[:50]}')

    st = cli_node.get_status()
    for ns in active_robots:
        fa = '!AKTIF!' if st['fault_active'].get(ns) else 'OFF'
        print(f'  {ns}: fault={fa}')

    print(f'  {SEP[:50]}')
    print('  [Catatan] Manual trigger hanya bekerja jika robot diluncurkan')
    print('  dengan fault_mode:=manual di robot*_haqqi.launch.py')
    print()
    print('  Pilih robot:')
    for i, ns in enumerate(active_robots, 1):
        print(f'    [{i}] {ns}')
    print('    [0] Kembali')

    ns_choice = input('  Pilihan: ').strip()
    if ns_choice == '0' or not ns_choice:
        return
    try:
        ns = active_robots[int(ns_choice) - 1]
    except (ValueError, IndexError):
        print('  [!] Pilihan tidak valid.')
        return

    print(f'\n  {ns} fault trigger:')
    print('  [1] Aktifkan fault   (kirim True  → /fault_trigger)')
    print('  [2] Nonaktifkan fault (kirim False → /fault_trigger)')
    print('  [0] Kembali')

    act_choice = input('  Pilihan: ').strip()
    if act_choice == '1':
        cli_node.trigger_fault(ns, True)
        print(f'  Fault ON dikirim ke {ns}.')
    elif act_choice == '2':
        cli_node.trigger_fault(ns, False)
        print(f'  Fault OFF dikirim ke {ns}.')
    else:
        print('  Kembali.')


# ─────────────────────────────────────────────────────────────────────────────
# Main menu loop
# ─────────────────────────────────────────────────────────────────────────────

def menu_loop(cli_node, scenarios, active_robots, yaml_path):
    print()
    print('╔════════════════════════════════════════════════════════════════════╗')
    print('║          EXPERIMENT MASTER CLI — haqqi_ta                        ║')
    print('║          /experiment_state heartbeat @ 10 Hz                     ║')
    print('╚════════════════════════════════════════════════════════════════════╝')

    while rclpy.ok():
        print()
        st = cli_node.get_status()
        print(_render_header(st, active_robots))
        print()
        print('  [1]  Load Scenario         [2]  Preview Scenario')
        print('  [3]  Readiness Check       [4]  START Trial')
        print('  [5]  Monitor Live          [6]  EMERGENCY STOP')
        print('  [7]  Reset Trial           [8]  Manual Goal / Waypoints')
        print('  [9]  Fault Injection       [0]  Keluar')
        print()

        choice = input('  Pilihan: ').strip()

        if choice == '1':
            _menu_load_scenario(cli_node, scenarios, active_robots)
            # Reload scenarios dari file setelah load (kalau ada perubahan YAML)
            scenarios, _ = load_scenarios_yaml()

        elif choice == '2':
            _menu_preview_scenario(scenarios, active_robots)

        elif choice == '3':
            _menu_readiness_check(cli_node, active_robots)

        elif choice == '4':
            st = cli_node.get_status()
            if st['state'] != 'READY':
                print(f'  [!] State harus READY sebelum START (sekarang: {st["state"]})')
                print('       Jalankan [1] Load Scenario terlebih dahulu.')
                continue

            # [FIX-LOCGATE2] Hard-block: semua robot aktif wajib punya pose
            # AMCL segar. Kalau hanya sebagian robot localized, trial masih bisa
            # menghasilkan DNF parsial dan metrik sinkronisasi/adaptasi jaringan
            # menjadi tidak valid. Ini TIDAK boleh dipaksa.
            missing_pose = cli_node.missing_localized_robots(active_robots)
            if missing_pose:
                print()
                print('  [BLOCK] START dibatalkan: pose AMCL belum segar untuk: ' + ', '.join(missing_pose))
                print('          Semua robot aktif wajib localized sebelum START.')
                print('          Jika dipaksa, trial berisiko DNF/log kosong parsial dan metrik TA tidak valid.')
                print('          Perbaiki dulu: pastikan AMCL tiap robot publish /{ns}/amcl_pose,')
                print('          set 2D Pose Estimate, cek UDP pose robot->PC (port 9001-9003),')
                print('          lalu jalankan [3] Readiness Check sebelum START.')
                continue

            # Pre-check sebelum START
            checks = cli_node.check_readiness(active_robots)
            fails  = [(ok, l) for ok, l in checks if not ok]
            if fails:
                print()
                print('  [!] Peringatan — beberapa check belum OK:')
                for _, l in fails:
                    print(f'      ✗ {l}')
                print()
                _print_readiness_recommendations(checks, active_robots, len(fails))
                print()
                confirm = input('  Paksa START meskipun belum ready? ketik y untuk lanjut, Enter=Batal: ').strip().lower()
                if confirm not in ('y', 'yes'):
                    print('  → START dibatalkan. Perbaiki telemetry/readiness dulu.')
                    continue

            cli_node.start()
            trial = cli_node.get_status()['trial']
            print(f'  → Trial #{trial} dimulai. Semua robot mulai bergerak.')

        elif choice == '5':
            _menu_monitor_live(cli_node, active_robots)

        elif choice == '6':
            cli_node.emergency_stop()
            print('  → EMERGENCY STOP. Robot berhenti dalam <100ms.')

        elif choice == '7':
            cli_node.reset()
            print('  → State direset ke READY. Semua node tetap berjalan.')

        elif choice == '8':
            _menu_manual_goal(cli_node, active_robots, yaml_path)

        elif choice == '9':
            _menu_fault_control(cli_node, active_robots)

        elif choice == '0':
            cli_node.emergency_stop()
            print('  Keluar. State diset ke STOP.')
            break


# ────────────────────────────────────────────────────────────────��────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # [FIX-SUBSET] Pilih subset robot aktif lewat CLI arg agar sesi tetap jalan
    # walau salah satu robot tidak dinyalakan. Robot yang TIDAK didaftarkan tidak
    # ikut dicek readiness/START sehingga tidak lagi memicu hard-block maupun
    # konfirmasi-ulang "belum ready".
    import argparse
    parser = argparse.ArgumentParser(description='Experiment Master CLI haqqi_ta')
    parser.add_argument(
        '--robots', type=str, default=None,
        help="Daftar robot aktif dipisah koma, mis. 'robot1,robot3'. "
             "Default: semua (robot1,robot2,robot3).")
    cli_args, _ = parser.parse_known_args()

    rclpy.init()
    cli_node = ExperimentMasterCLI()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(cli_node,), daemon=True)
    spin_thread.start()

    time.sleep(0.5)   # tunggu publisher siap

    scenarios, yaml_path = load_scenarios_yaml()

    # [FIX-SUBSET] Tentukan robot aktif dari --robots (default: semua).
    if cli_args.robots:
        requested = [r.strip() for r in cli_args.robots.split(',') if r.strip()]
        invalid   = [r for r in requested if r not in ROBOT_NAMESPACES]
        if invalid:
            print(f'  [!] Robot tidak dikenal diabaikan: {invalid} '
                  f'(valid: {list(ROBOT_NAMESPACES)})')
        active_robots = [r for r in ROBOT_NAMESPACES if r in requested]
        if not active_robots:
            print('  [!] Tidak ada robot valid di --robots; fallback ke semua.')
            active_robots = list(ROBOT_NAMESPACES)
    else:
        active_robots = list(ROBOT_NAMESPACES)

    if list(active_robots) != list(ROBOT_NAMESPACES):
        print(f'  Robot aktif sesi ini: {active_robots} '
              f'(subset — robot lain tidak dicek/diblok).')
    else:
        print(f'  Robot aktif sesi ini: {active_robots}')

    try:
        menu_loop(cli_node, scenarios, active_robots, yaml_path)
    except KeyboardInterrupt:
        cli_node.emergency_stop()
        print('\n  Interrupted. STOP dikirim.')
    finally:
        cli_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
