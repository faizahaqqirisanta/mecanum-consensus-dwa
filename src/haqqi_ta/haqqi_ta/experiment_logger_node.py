#!/usr/bin/env python3
"""
Experiment Logger Node — haqqi_ta
Support: Merekam semua metrik evaluasi ke file CSV

Metrik yang direkam (sesuai arsitektur TA):

INDIVIDUAL:
  - Goal precision: jarak akhir robot ke goal (m)
  - Cross-track error: deviasi dari global path (min, max, MSE)
  - Jarak tempuh aktual vs panjang global path

MULTI-AGENT:
  - Arrival time difference: selisih waktu tiba antar robot (≈0)
  - Profil kecepatan: vx, vy, w tiap robot per timestep
  - Minimum inter-robot distance (harus ≥ d_emergency)

FAULT & KOORDINASI:
  - Consensus convergence time setelah gangguan
  - Durasi stop-and-go per robot
  - Fault injection event (start, end, TTF aktual vs direncanakan)

Output: satu folder per trial berisi beberapa file CSV:
  experiment_YYYYMMDD_HHMMSS/
  ├── pose_log.csv          — pose + covariance semua robot (10 Hz)
  ├── velocity_log.csv      — cmd_vel semua robot (10 Hz)
  ├── consensus_log.csv     — p_i, p_bar, v_max per robot (10 Hz)
  ├── mission_log.csv       — mission remaining/total/progress per robot
  ├── local_plan_log.csv    — local trajectory DWA per snapshot
  ├── crosstrack_log.csv    — cross-track error per robot (10 Hz)
  ├── interrobot_log.csv    — jarak antar pasangan robot (10 Hz)
  ├── fault_event_log.csv   — event fault injection
  ├── stop_event_log.csv    — event stop-and-go
  ├── goal_result.csv       — hasil akhir: goal precision + arrival time
  └── experiment_summary.txt — ringkasan eksperimen

Timestamp convention:
  Semua kolom timestamp_s = now - t0, di mana t0 diset saat RUNNING.
  Setiap trial baru mendapat folder baru dan t0 baru.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import csv
import json
import math
import os
import time
import traceback
import yaml
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


ROBOT_NAMESPACES = ['robot1', 'robot2', 'robot3']

ALL_PAIRS = [
    ('robot1', 'robot2'),
    ('robot1', 'robot3'),
    ('robot2', 'robot3'),
]


class ExperimentLoggerNode(Node):
    def __init__(self):
        super().__init__('experiment_logger_node')

        # ── Parameter ─────────────────────────────────────────────────────
        self.declare_parameter('output_dir',       os.path.expanduser('~/experiment_logs'))
        self.declare_parameter('experiment_name',  'run')
        self.declare_parameter('log_rate',         10.0)   # Hz
        self.declare_parameter('csv_flush_period_s', 1.0)  # batch disk flush
        self.declare_parameter('scenario',         'merge')
        self.declare_parameter('arrival_mode',     'time_consensus')
        self.declare_parameter('coordination_mode', '')
        self.declare_parameter('d_emergency',      0.50)   # m
        self.declare_parameter('goal_tolerance',   0.10)   # m — error goal min 0.1
        self.declare_parameter('trial_timeout_s',  0.0)    # s — 0 = no timeout
        self.declare_parameter('goal_stable_time', 1.5)    # s
        self.declare_parameter('heading_goal_tolerance', 0.12)
        self.declare_parameter('auto_stop_on_all_goal', True)
        self.declare_parameter('fault_event_dedup_s', 0.75)
        self.declare_parameter('target_arrival_robot1', 0.0)
        self.declare_parameter('target_arrival_robot2', 0.0)
        self.declare_parameter('target_arrival_robot3', 0.0)
        self.declare_parameter('log_coordination_debug', False)
        self.declare_parameter('log_dwa_mode', False)
        self.declare_parameter('log_dynamic_obstacle_debug', True)
        self.declare_parameter('log_path_debug', True)
        self.declare_parameter('log_conflict_detail', True)
        self.declare_parameter('log_local_plan', True)

        self.output_dir      = self.get_parameter('output_dir').value
        self.exp_name        = self.get_parameter('experiment_name').value
        self.log_rate        = self.get_parameter('log_rate').value
        self.csv_flush_period_s = max(
            0.0,
            self._as_float(self.get_parameter('csv_flush_period_s').value, 1.0))
        self.scenario        = self.get_parameter('scenario').value
        self.arrival_mode    = str(self.get_parameter('arrival_mode').value).strip() or 'time_consensus'
        coord_mode           = self.get_parameter('coordination_mode').value
        if self.arrival_mode == 'arrival_offset':
            self.arrival_mode = 'arrival_offset_consensus'
        if self.arrival_mode == 'time_offset':
            self.arrival_mode = 'time_offset_consensus'
        if self.arrival_mode == 'scheduler':
            self.arrival_mode = 'time_consensus'
        self.coordination_mode = str(coord_mode).strip() or self.arrival_mode
        if self.coordination_mode == 'arrival_offset':
            self.coordination_mode = 'arrival_offset_consensus'
        if self.coordination_mode == 'time_offset':
            self.coordination_mode = 'time_offset_consensus'
        if self.coordination_mode == 'scheduler':
            self.coordination_mode = 'time_consensus'
        self.d_emergency     = self.get_parameter('d_emergency').value
        self.goal_tolerance  = self.get_parameter('goal_tolerance').value
        self.trial_timeout_s = self.get_parameter('trial_timeout_s').value
        self.goal_stable_time = self.get_parameter('goal_stable_time').value
        self.heading_goal_tolerance = self.get_parameter('heading_goal_tolerance').value
        self.auto_stop       = self.get_parameter('auto_stop_on_all_goal').value
        self.fault_event_dedup_s = max(0.0, self._as_float(
            self.get_parameter('fault_event_dedup_s').value, 0.75))
        self.target_arrival  = {
            'robot1': self._as_float(self.get_parameter('target_arrival_robot1').value),
            'robot2': self._as_float(self.get_parameter('target_arrival_robot2').value),
            'robot3': self._as_float(self.get_parameter('target_arrival_robot3').value),
        }
        # [FIX-ARRTGT] Jika launch arg target_arrival_robotN tidak diisi (<=0),
        # pakai arrival_schedule dari scenarios.yaml untuk skenario aktif sebagai
        # target kedatangan absolut (detik dari start) agar arrival_time_error_s
        # benar-benar terhitung (sebelumnya selalu N/A karena target default 0.0).
        self._arrival_target_explicit = any(
            v > 0.0 for v in self.target_arrival.values())
        if not self._arrival_target_explicit:
            self._load_arrival_schedule_targets()
        self.log_coordination_debug = self._as_bool(
            self.get_parameter('log_coordination_debug').value, False)
        self.log_dwa_mode = self._as_bool(
            self.get_parameter('log_dwa_mode').value, False)
        self.log_dynamic_obstacle_debug = self._as_bool(
            self.get_parameter('log_dynamic_obstacle_debug').value, True)
        self.log_path_debug = self._as_bool(
            self.get_parameter('log_path_debug').value, True)
        self.log_conflict_detail = self._as_bool(
            self.get_parameter('log_conflict_detail').value, True)
        self.log_local_plan = self._as_bool(
            self.get_parameter('log_local_plan').value, True)
        # ── State trial �����───────────────────────────────────────────────────
        # t0: ROS time saat trial dimulai; semua timestamp_s = now - t0
        self.t0              = None
        self._standby_warn_period_s = 5.0   # [FIX-STANDBYWARN] throttle warn tiap 5s
        self._last_standby_warn_t   = 0.0   # wall-clock terakhir warn dikirim
        self._last_nopose_warn_t    = 0.0   # [FIX-NOPOSE] wall-clock terakhir warn pose-hilang
        self._trial_count    = 0
        self.experiment_started    = False
        self.experiment_start_time = None   # = t0, dipertahankan untuk kompatibilitas
        self.experiment_ended      = False

        # ── State per robot ────────────────────────────────────────────────
        self.robot_pose      = {ns: None  for ns in ROBOT_NAMESPACES}
        # [FIX-SETTLED] Buffer pose "settled" (akhir run) untuk metrik presisi yang
        # robust thd jitter/drift AMCL: rata-ratakan pose dlm jendela terakhir.
        self.declare_parameter('settled_window_s', 3.0)
        self.declare_parameter('settled_min_samples', 3)
        self.settled_window_s    = max(0.5, self._as_float(
            self.get_parameter('settled_window_s').value, 3.0))
        self.settled_min_samples = max(1, int(
            self.get_parameter('settled_min_samples').value))
        self._settle_buf         = {ns: [] for ns in ROBOT_NAMESPACES}
        self.robot_cov       = {ns: None  for ns in ROBOT_NAMESPACES}
        self.robot_cmdvel    = {ns: None  for ns in ROBOT_NAMESPACES}
        self.robot_path      = {ns: []    for ns in ROBOT_NAMESPACES}
        self.robot_path_geometry = {
            ns: {'seg_lengths': [], 'cumulative': [0.0], 'point_count': 0}
            for ns in ROBOT_NAMESPACES
        }
        self.robot_goal      = {ns: None  for ns in ROBOT_NAMESPACES}
        self.robot_goal_reached      = {ns: False for ns in ROBOT_NAMESPACES}
        self.robot_arrival_time      = {ns: None  for ns in ROBOT_NAMESPACES}
        # [FIX-ARRIVE-LOG] Waktu SAMPAI-posisi (dari DWA /position_reached), TERPISAH dari
        # waktu goal penuh (posisi+heading) di robot_arrival_time.
        self.robot_position_arrival_time = {ns: None for ns in ROBOT_NAMESPACES}
        self.robot_path_length       = {ns: 0.0   for ns in ROBOT_NAMESPACES}
        self.mission_remaining       = {ns: None  for ns in ROBOT_NAMESPACES}
        self.mission_total           = {ns: None  for ns in ROBOT_NAMESPACES}
        self.robot_pos_success_time  = {ns: None  for ns in ROBOT_NAMESPACES}
        self.goal_inside_since       = {ns: None  for ns in ROBOT_NAMESPACES}
        # [FIX-GOALLATCH] Bekukan pose-saat-SAMPAI utk scoring; kebal teleport AMCL pasca-tiba.
        self.robot_latched_pose      = {ns: None  for ns in ROBOT_NAMESPACES}
        self._goal_latched           = {ns: False for ns in ROBOT_NAMESPACES}
        self.robot_last_good_pose    = {ns: None  for ns in ROBOT_NAMESPACES}
        self.goal_latch_band_m       = 0.60  # tolak update pose >0.6m dari goal (teleport)
        self.consensus_progress  = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self.consensus_vmax      = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self.priority_vmax       = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self.dwa_vmax_eff        = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self.omega_raw           = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self.omega_after_clamp   = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self.omega_global_limit  = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self.localization_hold   = {ns: False for ns in ROBOT_NAMESPACES}
        self.coordination_debug  = {}
        self.robot_priority_stop = {ns: False for ns in ROBOT_NAMESPACES}
        self.fault_active        = {ns: False for ns in ROBOT_NAMESPACES}
        self._fault_active_start = {ns: None  for ns in ROBOT_NAMESPACES}
        self._last_fault_event_time = {
            ns: {'START': None, 'END': None} for ns in ROBOT_NAMESPACES}
        self.robot_lane_offset   = {ns: 0.0   for ns in ROBOT_NAMESPACES}
        self._prev_pose      = {ns: None  for ns in ROBOT_NAMESPACES}
        self._prev_pose_time = {ns: None  for ns in ROBOT_NAMESPACES}
        self._pose_jump      = {ns: False for ns in ROBOT_NAMESPACES}
        self.robot_dwa_mode  = {ns: 'IDLE' for ns in ROBOT_NAMESPACES}
        # Field dari robot yang sebelumnya tidak di-bridge ke PC
        self.tracking_mode       = {ns: 'IDLE' for ns in ROBOT_NAMESPACES}
        self.heading_error       = {ns: 0.0    for ns in ROBOT_NAMESPACES}
        self.heading_error_time  = {ns: None   for ns in ROBOT_NAMESPACES}
        self.dwa_speed_mag       = {ns: 0.0    for ns in ROBOT_NAMESPACES}
        self.vmax_priority_robot = {ns: 0.0    for ns in ROBOT_NAMESPACES}
        self.priority_stop_robot = {ns: False  for ns in ROBOT_NAMESPACES}
        self.lane_offset_robot   = {ns: 0.0    for ns in ROBOT_NAMESPACES}
        self.crosstrack_stats    = {
            ns: self._new_crosstrack_stats() for ns in ROBOT_NAMESPACES
        }
        self.min_inter_dist      = {pair: float('inf') for pair in ALL_PAIRS}
        self._stop_start         = {ns: None for ns in ROBOT_NAMESPACES}
        self._last_csv_flush_wall = {}
        self._local_plan_seq = {ns: 0 for ns in ROBOT_NAMESPACES}
        self._goal_results_written = False

        # ── Buat folder trial pertama + CSV ────────────────────────────────
        self._init_trial_dir()
        self._init_csv_files()

        # ── Subscribers pose robot via amcl_pose ─────────────────────────
        pose_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        for ns in ROBOT_NAMESPACES:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{ns}/amcl_pose',
                lambda msg, n=ns: self._pose_cb(msg, n),
                pose_qos)

        # ── Subscriber /experiment_state heartbeat ────────────────────────
        self.create_subscription(
            String, '/experiment_state',
            self._experiment_state_cb, 10)

        # ── Subscriber /start_signal Bool ────────────────────────────────
        signal_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Bool, '/start_signal',
            self._start_signal_cb, signal_qos)

        # ── Subscribers data per robot ─────────────────────────────────────
        for ns in ROBOT_NAMESPACES:
            self.create_subscription(
                Twist,
                f'/{ns}/cmd_vel',
                lambda msg, n=ns: self.cmdvel_callback(msg, n), 10)

            self.create_subscription(
                Path,
                f'/{ns}/plan',
                lambda msg, n=ns: self.path_callback(msg, n), 10)

            if self.log_local_plan:
                self.create_subscription(
                    Path,
                    f'/{ns}/local_plan',
                    lambda msg, n=ns: self.local_plan_callback(msg, n), 10)

            self.create_subscription(
                PoseArray,
                f'/{ns}/waypoints',
                lambda msg, n=ns: self.waypoints_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/path_length',
                lambda msg, n=ns: self.path_length_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/mission_remaining_length',
                lambda msg, n=ns: self.mission_remaining_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/mission_total_length',
                lambda msg, n=ns: self.mission_total_callback(msg, n), 10)

            self.create_subscription(
                Bool,
                f'/{ns}/goal_reached',
                lambda msg, n=ns: self.goal_reached_callback(msg, n), 10)
            # [FIX-ARRIVE-LOG] SAMPAI-posisi dari DWA — waktu tiba = saat posisi, bukan rotasi.
            self.create_subscription(
                Bool,
                f'/{ns}/position_reached',
                lambda msg, n=ns: self.position_reached_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/consensus_progress',
                lambda msg, n=ns: self.consensus_progress_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/vmax_consensus',
                lambda msg, n=ns: self.consensus_vmax_callback(msg, n), 10)

            self.create_subscription(
                Bool,
                f'/{ns}/priority_stop',
                lambda msg, n=ns: self.priority_stop_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/vmax_priority',
                lambda msg, n=ns: self.priority_vmax_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/dwa_vmax_eff',
                lambda msg, n=ns: self.dwa_vmax_eff_callback(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/omega_raw',
                lambda msg, n=ns: self._omega_raw_cb(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/omega_after_clamp',
                lambda msg, n=ns: self._omega_after_clamp_cb(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/omega_global_limit',
                lambda msg, n=ns: self._omega_global_limit_cb(msg, n), 10)

            self.create_subscription(
                Bool,
                f'/{ns}/localization_hold_active',
                lambda msg, n=ns: self._localization_hold_cb(msg, n), 10)

            self.create_subscription(
                Float32,
                f'/{ns}/crossing_lane_offset',
                lambda msg, n=ns: self._lane_offset_cb(msg, n), 10)

            if self.log_dwa_mode:
                self.create_subscription(
                    String,
                    f'/{ns}/dwa_mode',
                    lambda msg, n=ns: self._dwa_mode_cb(msg, n), 10)

            if self.log_dynamic_obstacle_debug:
                self.create_subscription(
                    String,
                    f'/{ns}/dynamic_obstacle_debug',
                    self._dynamic_obstacle_debug_cb, 10)

            # ── Field baru yang di-bridge dari robot via consensus_node ──
            self.create_subscription(
                String, f'/{ns}/tracking_mode',
                lambda msg, n=ns: self.tracking_mode.__setitem__(n, str(msg.data)), 10)

            self.create_subscription(
                Float32, f'/{ns}/heading_error',
                lambda msg, n=ns: self.heading_error_callback(msg, n), 10)

            self.create_subscription(
                Float32, f'/{ns}/dwa_speed_mag',
                lambda msg, n=ns: self.dwa_speed_mag.__setitem__(n, float(msg.data)), 10)

            self.create_subscription(
                Float32, f'/{ns}/vmax_priority_robot',
                lambda msg, n=ns: self.vmax_priority_robot.__setitem__(n, float(msg.data)), 10)

            self.create_subscription(
                Bool, f'/{ns}/priority_stop_robot',
                lambda msg, n=ns: self.priority_stop_robot.__setitem__(n, bool(msg.data)), 10)

            self.create_subscription(
                Float32, f'/{ns}/lane_offset_robot',
                lambda msg, n=ns: self.lane_offset_robot.__setitem__(n, float(msg.data)), 10)

        self.create_subscription(
            String, '/coordination_debug',
            self._coordination_debug_cb, 10)

        for ns in ROBOT_NAMESPACES:
            self.create_subscription(
                String,
                f'/{ns}/fault_log',
                self.fault_log_callback, 10)

            self.create_subscription(
                Bool,
                f'/{ns}/fault_active',
                lambda msg, n=ns: self.fault_active_callback(msg, n), 10)

        self.create_subscription(
            String, '/conflict_zone_state',
            self._conflict_zone_state_cb, 10)

        if self.log_conflict_detail:
            self.create_subscription(
                String, '/conflict_zone_detail',
                self._conflict_zone_detail_cb, 10)

        if self.log_path_debug:
            self.create_subscription(
                String, '/path_debug',
                self._path_debug_cb, 10)

        # ── Timers ────────────────────────────────────────────────────────
        period = 1.0 / self.log_rate
        self.create_timer(period, self.log_loop)
        self.create_timer(1.0,    self.status_report)

        self.get_logger().info(
            f'Experiment Logger ready | scenario={self.scenario} | '
            f'arrival_mode={self.arrival_mode} | targets={self.target_arrival} | '
            f'{self.log_rate}Hz | auto_stop={self.auto_stop} | '
            f'debug_csv coord={self.log_coordination_debug}, '
            f'dwa_mode={self.log_dwa_mode}, '
            f'dynobs={self.log_dynamic_obstacle_debug}, '
            f'path_debug={self.log_path_debug}, '
            f'conflict_detail={self.log_conflict_detail}, '
            f'local_plan={self.log_local_plan}')

    @staticmethod
    def _as_float(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _as_bool(value, default=False):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            text = value.strip().lower()
            if text in ('true', '1', 'yes', 'y', 'on'):
                return True
            if text in ('false', '0', 'no', 'n', 'off'):
                return False
        return default

    # ═══════════════════════════════════════════════════════════════════════
    # INISIALISASI TRIAL
    # ═══════════════════════════════════════════════════════════════════════

    def _init_trial_dir(self):
        """Buat folder eksperimen baru untuk trial ini."""
        timestamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.exp_dir = os.path.join(
            self.output_dir,
            f'{self.exp_name}_{timestamp}')
        os.makedirs(self.exp_dir, exist_ok=True)
        self.get_logger().info(f'Logging ke: {self.exp_dir}')

    def _init_csv_files(self):
        """Buka semua file CSV dan tulis header."""

        def open_csv(filename, fieldnames):
            path = os.path.join(self.exp_dir, filename)
            f    = open(path, 'w', newline='')
            w    = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            return f, w

        self._f_pose, self._w_pose = open_csv('pose_log.csv', [
            'timestamp_s', 'robot',
            'x', 'y', 'theta',
            'sigma_x', 'sigma_y', 'r_safe_est',
            'sigma_valid',
            'pose_jump',
        ])

        self._f_vel, self._w_vel = open_csv('velocity_log.csv', [
            'timestamp_s', 'robot',
            'vx', 'vy', 'w',
            'speed',
            'omega_raw', 'omega_after_clamp', 'omega_global_limit',
            'localization_hold_active',
            'fault_active',
            # PC-side (apa yg priority_manager kirim):
            'priority_stop', 'vmax_consensus', 'vmax_priority',
            'dwa_vmax_eff', 'lane_offset',
            # Robot-side (dikonfirmasi diterima robot, via UDP bridge):
            'priority_stop_robot', 'vmax_priority_robot',
            'lane_offset_robot',
            # Data tracking baru (sebelumnya tidak di-bridge):
            'tracking_mode', 'heading_error_deg', 'speed_mag',
        ])

        self._f_cons, self._w_cons = open_csv('consensus_log.csv', [
            'timestamp_s',
            'coordination_mode',
            'p_robot1', 'p_robot2', 'p_robot3',
            'p_bar',
            'delta_p_robot1', 'delta_p_robot2', 'delta_p_robot3',
            'vmax_robot1', 'vmax_robot2', 'vmax_robot3',
            'ETA_robot1', 'ETA_robot2', 'ETA_robot3',
            'A_robot1', 'A_robot2', 'A_robot3',
            'offset_robot1', 'offset_robot2', 'offset_robot3',
            'q_robot1', 'q_robot2', 'q_robot3',
            'q_bar',
            'e_robot1', 'e_robot2', 'e_robot3',
            'v_consensus_robot1', 'v_consensus_robot2', 'v_consensus_robot3',
            'max_deviation', 'converged',
            # [FIX-OBS] Status deteksi agen gagal [M4] — bukti terekam untuk
            # ablation F1/F2 & pengukuran latency deteksi.
            'failed_robot1', 'failed_robot2', 'failed_robot3',
            'detection_enabled',
        ])

        if self.log_coordination_debug:
            self._f_coord, self._w_coord = open_csv('coordination_debug_log.csv', [
                'timestamp_s', 'coordination_mode',
                'ETA_robot1', 'ETA_robot2', 'ETA_robot3',
                'A_robot1', 'A_robot2', 'A_robot3',
                'offset_robot1', 'offset_robot2', 'offset_robot3',
                'q_robot1', 'q_robot2', 'q_robot3',
                'q_bar',
                'e_robot1', 'e_robot2', 'e_robot3',
                'v_consensus_robot1', 'v_consensus_robot2', 'v_consensus_robot3',
                'failed_robot1', 'failed_robot2', 'failed_robot3',
                'detection_enabled',
            ])
        else:
            self._f_coord, self._w_coord = None, None

        self._f_mission, self._w_mission = open_csv('mission_log.csv', [
            'timestamp_s', 'robot',
            'mission_remaining_m',
            'mission_total_m',
            'mission_progress',
            'waypoint_progress',
        ])

        self._f_ct, self._w_ct = open_csv('crosstrack_log.csv', [
            'timestamp_s', 'robot',
            'crosstrack_error_m',  # jarak tegak lurus robot ke path (m)
            'along_track_m',       # jarak kumulatif dari awal path ke titik terdekat (m)
            'path_progress',       # along_track / total_path_length (0–1)
            'heading_error_deg',   # selisih heading robot vs tangent path di titik terdekat (deg)
            'path_index',
        ])

        # path_log: simpan global path planned sekali saat pertama diterima per trial
        self._f_path, self._w_path = open_csv('path_log.csv', [
            'robot', 'point_index', 'x', 'y',
        ])
        self._path_logged = {ns: False for ns in ROBOT_NAMESPACES}

        if self.log_local_plan:
            self._f_local_plan, self._w_local_plan = open_csv('local_plan_log.csv', [
                'timestamp_s', 'robot', 'plan_seq',
                'point_index', 'point_count', 'frame_id',
                'x', 'y', 'theta',
            ])
        else:
            self._f_local_plan, self._w_local_plan = None, None

        self._f_ir, self._w_ir = open_csv('interrobot_log.csv', [
            'timestamp_s',
            'dist_r1_r2', 'dist_r1_r3', 'dist_r2_r3',
            'min_dist',
            'violation',
        ])

        self._f_fault, self._w_fault = open_csv('fault_event_log.csv', [
            'timestamp_s', 'event_type',
            'robot', 'ttf_planned_s', 'ttf_actual_s', 'mode',
            'fault_type',
            'pulse_index', 'planned_start_s', 'actual_start_s',
            'actual_end_s', 'duration_s',
        ])

        self._f_stop, self._w_stop = open_csv('stop_event_log.csv', [
            'timestamp_s', 'event_type',
            'robot', 'duration_s',
        ])

        if self.log_dwa_mode:
            self._f_dwa_mode, self._w_dwa_mode = open_csv('dwa_mode_log.csv', [
                'timestamp_s', 'robot', 'dwa_mode',
            ])
        else:
            self._f_dwa_mode, self._w_dwa_mode = None, None

        self._f_conflict, self._w_conflict = open_csv('conflict_log.csv', [
            'timestamp_s', 'zone', 'state', 'owner', 'wall_t',
        ])

        if self.log_conflict_detail:
            self._f_conflict_detail, self._w_conflict_detail = open_csv(
                'conflict_detail_log.csv', [
                    'timestamp_s', 'scenario', 'zone', 'state', 'owner', 'gap_remain_s',
                    'source', 'robot_pair',
                    'center_x', 'center_y', 'radius', 'detect_radius', 'hold_radius',
                    'clear_radius',
                    'd_robot1', 'd_robot2', 'd_robot3',
                    'eta_robot1', 'eta_robot2', 'eta_robot3',
                    'cmd_robot1', 'cmd_robot2', 'cmd_robot3',
                ])
        else:
            self._f_conflict_detail, self._w_conflict_detail = None, None

        if self.log_dynamic_obstacle_debug:
            self._f_dynobs, self._w_dynobs = open_csv('dynamic_obstacle_log.csv', [
                'timestamp_s', 'robot', 'peer_robot',
                'crossing_cmd', 'peer_crossing_cmd',
                'peer_x', 'peer_y', 'distance_to_peer',
                'blocks_path', 'bypass_active', 'bypass_x', 'bypass_y',
                'rejected_candidate_count_dyn', 'min_dynamic_obstacle_distance',
                'front_dist', 'front_blocked', 'peer_blocks_path',
                'peer_in_front_sector', 'holo_blk_reason', 'convoy_follow_peer',
            ])
        else:
            self._f_dynobs, self._w_dynobs = None, None

        if self.log_path_debug:
            self._f_path_debug, self._w_path_debug = open_csv('path_debug_log.csv', [
                'timestamp_s', 'robot',
                'point_count', 'source', 'age_s', 'has_forward_path',
                'auto_conflict_zone_enabled', 'auto_zone_count', 'active_zone_count',
            ])
        else:
            self._f_path_debug, self._w_path_debug = None, None

        self._f_goal, self._w_goal = open_csv('goal_result.csv', [
            'robot',
            'state_success',
            'position_success',
            'arrival_time_from_start_s',
            'full_goal_time_from_start_s',
            'position_success_time_s',
            'target_arrival_time_s',
            'arrival_time_error_s',
            'final_error_m',
            'final_heading_error_deg',
            'heading_success',
            'goal_x', 'goal_y',
            'final_x', 'final_y',
            'goal_precision_m',
            'settled_precision_m',
            'settled_pose_std_m',
            'settled_sample_count',
            'path_length_planned_m',
            'crosstrack_min_m', 'crosstrack_max_m', 'crosstrack_mse_m2',
        ])

        self.get_logger().info('Semua file CSV siap.')

    def _close_csv_files(self):
        """Tutup semua file CSV yang sedang terbuka."""
        files = [self._f_pose, self._f_vel, self._f_cons,
                 self._f_mission, self._f_ct, self._f_ir, self._f_fault,
                 self._f_stop, self._f_goal, self._f_dwa_mode,
                 self._f_conflict, self._f_conflict_detail,
                 self._f_coord, self._f_dynobs, self._f_path_debug,
                 self._f_path, self._f_local_plan]
        self._flush_files(files, key='close', force=True)
        for f in files:
            if f is None:
                continue
            try:
                f.close()
            except Exception:
                pass

    def _flush_files(self, files, key='default', force=False):
        """Batch CSV flush supaya logger tidak memaksa disk sync setiap callback."""
        now = time.time()
        if not force and self.csv_flush_period_s > 0.0:
            last = self._last_csv_flush_wall.get(key, 0.0)
            if now - last < self.csv_flush_period_s:
                return
        for f in files:
            if f is None:
                continue
            try:
                f.flush()
            except Exception:
                pass
        self._last_csv_flush_wall[key] = now

    def _snapshot_live_telemetry(self):
        """Ambil cache telemetry terakhir sebelum reset trial.

        Path/pose biasanya sudah datang saat READY, sebelum experiment_state
        berubah ke RUNNING. Jangan buang cache ini saat trial dimulai, karena
        beberapa topic seperti /robot*/plan tidak selalu dipublish ulang.
        """
        return {
            'robot_pose': {ns: (list(v) if v is not None else None)
                           for ns, v in self.robot_pose.items()},
            'robot_cov': {ns: (list(v) if v is not None else None)
                          for ns, v in self.robot_cov.items()},
            'robot_cmdvel': {ns: (list(v) if v is not None else None)
                             for ns, v in self.robot_cmdvel.items()},
            'robot_path': {ns: [list(p) for p in path]
                           for ns, path in self.robot_path.items()},
            'robot_path_geometry': {
                ns: {
                    'seg_lengths': list(geom.get('seg_lengths', [])),
                    'cumulative': list(geom.get('cumulative', [0.0])),
                    'point_count': int(geom.get('point_count', 0)),
                }
                for ns, geom in self.robot_path_geometry.items()
            },
            'robot_goal': {ns: (list(v) if v is not None else None)
                           for ns, v in self.robot_goal.items()},
            'robot_path_length': dict(self.robot_path_length),
            'mission_remaining': dict(self.mission_remaining),
            'mission_total': dict(self.mission_total),
            'consensus_progress': dict(self.consensus_progress),
            'consensus_vmax': dict(self.consensus_vmax),
            'priority_vmax': dict(self.priority_vmax),
            'dwa_vmax_eff': dict(self.dwa_vmax_eff),
            'omega_raw': dict(self.omega_raw),
            'omega_after_clamp': dict(self.omega_after_clamp),
            'omega_global_limit': dict(self.omega_global_limit),
            'localization_hold': dict(self.localization_hold),
            'fault_active': dict(self.fault_active),
            'robot_dwa_mode': dict(self.robot_dwa_mode),
            'tracking_mode': dict(self.tracking_mode),
            'heading_error': dict(self.heading_error),
            'heading_error_time': dict(self.heading_error_time),
            'dwa_speed_mag': dict(self.dwa_speed_mag),
            'vmax_priority_robot': dict(self.vmax_priority_robot),
            'priority_stop_robot': dict(self.priority_stop_robot),
            'lane_offset_robot': dict(self.lane_offset_robot),
        }

    def _restore_live_telemetry(self, snapshot):
        for ns in ROBOT_NAMESPACES:
            self.robot_pose[ns] = snapshot['robot_pose'].get(ns)
            self.robot_cov[ns] = snapshot['robot_cov'].get(ns)
            self.robot_cmdvel[ns] = snapshot['robot_cmdvel'].get(ns)
            self.robot_path[ns] = snapshot['robot_path'].get(ns, [])
            self.robot_path_geometry[ns] = snapshot.get(
                'robot_path_geometry', {}).get(ns) or self._build_path_geometry(
                    self.robot_path[ns])
            self.robot_goal[ns] = snapshot['robot_goal'].get(ns)
            self.robot_path_length[ns] = snapshot['robot_path_length'].get(ns, 0.0)
            self.mission_remaining[ns] = snapshot['mission_remaining'].get(ns)
            self.mission_total[ns] = snapshot['mission_total'].get(ns)
            self.consensus_progress[ns] = snapshot['consensus_progress'].get(ns, 0.0)
            self.consensus_vmax[ns] = snapshot['consensus_vmax'].get(ns, 0.0)
            self.priority_vmax[ns] = snapshot['priority_vmax'].get(ns, 0.0)
            self.dwa_vmax_eff[ns] = snapshot['dwa_vmax_eff'].get(ns, 0.0)
            self.omega_raw[ns] = snapshot['omega_raw'].get(ns, 0.0)
            self.omega_after_clamp[ns] = snapshot['omega_after_clamp'].get(ns, 0.0)
            self.omega_global_limit[ns] = snapshot['omega_global_limit'].get(ns, 0.0)
            self.localization_hold[ns] = snapshot['localization_hold'].get(ns, False)
            self.fault_active[ns] = snapshot['fault_active'].get(ns, False)
            self.robot_dwa_mode[ns] = snapshot['robot_dwa_mode'].get(ns, 'IDLE')
            self.tracking_mode[ns] = snapshot['tracking_mode'].get(ns, 'IDLE')
            self.heading_error[ns] = snapshot['heading_error'].get(ns, 0.0)
            self.heading_error_time[ns] = snapshot.get(
                'heading_error_time', {}).get(ns)
            self.dwa_speed_mag[ns] = snapshot['dwa_speed_mag'].get(ns, 0.0)
            self.vmax_priority_robot[ns] = snapshot['vmax_priority_robot'].get(ns, 0.0)
            self.priority_stop_robot[ns] = snapshot['priority_stop_robot'].get(ns, False)
            self.lane_offset_robot[ns] = snapshot['lane_offset_robot'].get(ns, 0.0)

    def _write_path_once(self, ns):
        path = self.robot_path.get(ns) or []
        if not path or self._path_logged.get(ns, False):
            return
        for i, (px, py) in enumerate(path):
            self._w_path.writerow({
                'robot': ns,
                'point_index': i,
                'x': f'{px:.4f}',
                'y': f'{py:.4f}',
            })
        self._f_path.flush()
        self._path_logged[ns] = True

    def _reset_trial_state(self):
        """Reset semua buffer per-trial ke kondisi bersih."""
        for ns in ROBOT_NAMESPACES:
            self.robot_pose[ns]             = None
            self.robot_cov[ns]              = None
            self.robot_cmdvel[ns]           = None
            self.robot_path[ns]             = []
            self.robot_path_geometry[ns]    = self._build_path_geometry([])
            self.robot_goal[ns]             = None
            self.robot_path_length[ns]      = 0.0
            self.mission_remaining[ns]      = None
            self.mission_total[ns]          = None
            self.robot_goal_reached[ns]     = False
            self.robot_arrival_time[ns]     = None
            self.robot_position_arrival_time[ns] = None  # [FIX-ARRIVE-LOG]
            self.robot_pos_success_time[ns] = None
            self.goal_inside_since[ns]      = None
            self.robot_latched_pose[ns]     = None   # [FIX-GOALLATCH]
            self._goal_latched[ns]          = False
            self.robot_last_good_pose[ns]   = None
            self.crosstrack_stats[ns]       = self._new_crosstrack_stats()
            self.consensus_progress[ns]     = 0.0
            self.consensus_vmax[ns]         = 0.0
            self.priority_vmax[ns]          = 0.0
            self.dwa_vmax_eff[ns]           = 0.0
            self.omega_raw[ns]              = 0.0
            self.omega_after_clamp[ns]      = 0.0
            self.omega_global_limit[ns]     = 0.0
            self.localization_hold[ns]      = False
            self.robot_priority_stop[ns]    = False
            self.fault_active[ns]           = False
            self._fault_active_start[ns]     = None
            self._last_fault_event_time[ns]  = {'START': None, 'END': None}
            self.robot_lane_offset[ns]      = 0.0
            self.robot_dwa_mode[ns]         = 'IDLE'
            self.tracking_mode[ns]          = 'IDLE'
            self.heading_error[ns]          = 0.0
            self.heading_error_time[ns]     = None
            self.dwa_speed_mag[ns]          = 0.0
            self.vmax_priority_robot[ns]    = 0.0
            self.priority_stop_robot[ns]    = False
            self.lane_offset_robot[ns]      = 0.0
            self._path_logged[ns]           = False
            self._local_plan_seq[ns]        = 0
            self._prev_pose[ns]             = None
            self._prev_pose_time[ns]        = None
            self._pose_jump[ns]             = False
            self._settle_buf[ns]            = []   # [FIX-SETTLED]
            self._stop_start[ns]            = None
        self.min_inter_dist = {pair: float('inf') for pair in ALL_PAIRS}
        self.coordination_debug = {}
        self._goal_results_written = False

    def _start_new_trial(self, source: str):
        """Tutup trial lama, buka folder/CSV baru, lalu reset semua buffer."""
        live_snapshot = self._snapshot_live_telemetry()
        if self._trial_count > 0:
            if not self.experiment_ended:
                self._write_goal_results()
                self._write_summary()
            self._close_csv_files()
            self._init_trial_dir()
            self._init_csv_files()
        self._reset_trial_state()
        self._restore_live_telemetry(live_snapshot)
        for ns in ROBOT_NAMESPACES:
            self._write_path_once(ns)
        self._trial_count += 1
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.experiment_start_time = self.t0
        self.experiment_ended   = False
        self.experiment_started = True
        self.get_logger().info(
            f'Logger: trial #{self._trial_count} dimulai via {source}, '
            f't0={self.t0:.2f}, folder={self.exp_dir}')

    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def _pose_cb(self, msg, ns):
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        th = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                        1.0 - 2.0*(q.y*q.y + q.z*q.z))
        now_wall  = time.time()
        new_pose  = [p.x, p.y, th]
        prev      = self._prev_pose[ns]
        prev_time = self._prev_pose_time[ns]
        if prev is not None and prev_time is not None:
            jump_dist  = math.hypot(p.x - prev[0], p.y - prev[1])
            dt         = max(now_wall - prev_time, 1e-3)
            jump_speed = jump_dist / dt
            self._pose_jump[ns] = (jump_speed > 2.0 and jump_dist > 0.30)
        self._prev_pose[ns]      = new_pose
        self._prev_pose_time[ns] = now_wall
        self.robot_pose[ns]  = new_pose
        # [FIX-GOALLATCH] Sebelum SAMPAI: simpan pose-bagus terakhir (bukan teleport).
        # Sesudah SAMPAI (latched): pose terkunci TIDAK diubah -> kebal drift/teleport AMCL.
        if not self._goal_latched[ns] and not self._pose_jump[ns]:
            self.robot_last_good_pose[ns] = new_pose
        # [FIX-SETTLED] Rekam pose non-teleport ke buffer rolling (jendela settled_window_s).
        if not self._pose_jump[ns]:
            buf = self._settle_buf[ns]
            buf.append((now_wall, new_pose[0], new_pose[1]))
            cutoff = now_wall - self.settled_window_s
            while len(buf) > self.settled_min_samples and buf[0][0] < cutoff:
                buf.pop(0)
        cov = list(msg.pose.covariance)
        self.robot_cov[ns] = cov

    def _start_signal_cb(self, msg):
        if bool(msg.data) and not self.experiment_started:
            self._start_new_trial('start_signal')
        elif not bool(msg.data):
            self.experiment_started = False

    def _load_arrival_schedule_targets(self):
        """[FIX-ARRTGT] Muat arrival_schedule scenario aktif sebagai target
        kedatangan absolut (detik dari start) untuk metrik arrival_time_error_s.
        Hanya dipakai bila launch arg target_arrival_robotN tidak di-set (<=0).
        Nilai arrival_schedule di scenarios.yaml memang sudah absolut (mis.
        convoy 75s, crossing r1=55/r2=60/r3=52), sehingga dipakai apa adanya
        sebagai t_tgt, bukan sebagai offset relatif seperti di consensus_node."""
        try:
            pkg_dir = get_package_share_directory('haqqi_ta')
            yaml_path = os.path.join(pkg_dir, 'param', 'scenarios.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            schedule = (data.get('scenarios', {})
                           .get(self.scenario, {})
                           .get('arrival_schedule', {}))
            loaded = {}
            for ns in ROBOT_NAMESPACES:
                val = self._as_float(schedule.get(ns, 0.0), 0.0)
                if val > 0.0:
                    self.target_arrival[ns] = val
                    loaded[ns] = val
            if loaded:
                self.get_logger().info(
                    f'[LOGGER] arrival_schedule dipakai sebagai target absolut '
                    f'scenario={self.scenario}: {loaded}')
            else:
                self.get_logger().warn(
                    f'[LOGGER] arrival_schedule kosong/tak ada untuk '
                    f'scenario={self.scenario}; arrival_time_error_s = N/A.')
        except Exception as e:
            self.get_logger().warn(
                f'[LOGGER] Gagal baca arrival_schedule scenario={self.scenario}: {e}')

    def _experiment_state_cb(self, msg):
        if msg.data == 'RUNNING' and not self.experiment_started:
            self._start_new_trial('experiment_state')
        elif msg.data in ('STOP', 'READY'):
            self.experiment_started = False

    def cmdvel_callback(self, msg, ns):
        self.robot_cmdvel[ns] = [msg.linear.x, msg.linear.y, msg.angular.z]

    def path_callback(self, msg, ns):
        path = []
        for pose in msg.poses:
            path.append([pose.pose.position.x, pose.pose.position.y])
        self.robot_path[ns] = path
        self.robot_path_geometry[ns] = self._build_path_geometry(path)
        if path:
            last = msg.poses[-1]
            q    = last.pose.orientation
            th   = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                              1.0 - 2.0*(q.y*q.y + q.z*q.z))
            self.robot_goal[ns] = [
                last.pose.position.x,
                last.pose.position.y,
                th]
            # Simpan global path ke path_log.csv sekali per trial saat path pertama tiba.
            # Path bisa berubah karena replan AMCL; hanya path pertama yang disimpan
            # sebagai "planned path" referensi bab 4.
            self._write_path_once(ns)

    def local_plan_callback(self, msg: Path, ns):
        if not self.experiment_started or self.experiment_ended or self.t0 is None:
            return
        if not self.log_local_plan or self._w_local_plan is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        t_rel = f'{now - self.t0:.4f}'
        seq = self._local_plan_seq[ns]
        self._local_plan_seq[ns] = seq + 1
        point_count = len(msg.poses)
        frame_id = msg.header.frame_id or ''

        if point_count == 0:
            self._w_local_plan.writerow({
                'timestamp_s': t_rel,
                'robot': ns,
                'plan_seq': seq,
                'point_index': -1,
                'point_count': 0,
                'frame_id': frame_id,
                'x': '',
                'y': '',
                'theta': '',
            })
        else:
            for i, ps in enumerate(msg.poses):
                q = ps.pose.orientation
                theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self._w_local_plan.writerow({
                    'timestamp_s': t_rel,
                    'robot': ns,
                    'plan_seq': seq,
                    'point_index': i,
                    'point_count': point_count,
                    'frame_id': frame_id,
                    'x': f'{ps.pose.position.x:.5f}',
                    'y': f'{ps.pose.position.y:.5f}',
                    'theta': f'{theta:.5f}',
                })
        self._flush_files([self._f_local_plan], key='local_plan')

    def waypoints_callback(self, msg: PoseArray, ns):
        if self.robot_path[ns]:
            return
        path = []
        for pose in msg.poses:
            path.append([pose.position.x, pose.position.y])
        if path:
            self.robot_path[ns] = path
            self.robot_path_geometry[ns] = self._build_path_geometry(path)
            if not self.robot_goal[ns]:
                last = msg.poses[-1]
                q  = last.orientation
                th = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                1.0 - 2.0*(q.y*q.y + q.z*q.z))
                self.robot_goal[ns] = [last.position.x, last.position.y, th]

    def path_length_callback(self, msg, ns):
        self.robot_path_length[ns] = float(msg.data)

    def mission_remaining_callback(self, msg, ns):
        self.mission_remaining[ns] = max(0.0, float(msg.data))

    def mission_total_callback(self, msg, ns):
        self.mission_total[ns] = max(0.0, float(msg.data))

    def goal_reached_callback(self, msg, ns):
        if not self.experiment_started or self.experiment_ended:
            return
        if bool(msg.data) and not self.robot_goal_reached[ns]:
            now = self.get_clock().now().nanoseconds / 1e9
            self.robot_goal_reached[ns] = True
            self.robot_arrival_time[ns] = now
            t_rel = now - (self.t0 or now)
            self.get_logger().info(
                f'[LOGGER] {ns} GOAL REACHED | t={t_rel:.2f}s dari start')

            if self.auto_stop and all(self.robot_goal_reached.values()):
                self.get_logger().info('[LOGGER] Semua robot selesai — menulis hasil akhir')
                self._write_goal_results()
                self._write_summary()
                self.experiment_ended = True

    def _try_latch_goal_pose(self, ns):
        """Latch pose saat sampai, tapi tolak kandidat yang jauh dari goal."""
        if self._goal_latched[ns]:
            return True
        goal = self.robot_goal[ns]
        candidates = []
        if self.robot_pose[ns] is not None and not self._pose_jump[ns]:
            candidates.append(self.robot_pose[ns])
        if self.robot_last_good_pose[ns] is not None:
            candidates.append(self.robot_last_good_pose[ns])
        for snap in candidates:
            if snap is None:
                continue
            if goal is None:
                self.robot_latched_pose[ns] = list(snap)
                self._goal_latched[ns] = True
                return True
            dist = math.hypot(snap[0] - goal[0], snap[1] - goal[1])
            if dist <= self.goal_latch_band_m:
                self.robot_latched_pose[ns] = list(snap)
                self._goal_latched[ns] = True
                return True
        return False

    def position_reached_callback(self, msg, ns):
        # [FIX-ARRIVE-LOG] Catat waktu SAMPAI-posisi sekali (transisi False->True).
        # Ini waktu-tiba sebenarnya menurut DWA (saat position_reached / _goal_arrival_time),
        # terpisah dari durasi rotasi-akhir menghadap pusat formasi.
        if not self.experiment_started or self.experiment_ended:
            return
        if bool(msg.data):
            if self.robot_position_arrival_time[ns] is None:
                now = self.get_clock().now().nanoseconds / 1e9
                self.robot_position_arrival_time[ns] = now
                t_rel = now - (self.t0 or now)
                self.get_logger().info(
                    f'[LOGGER] {ns} POSISI TERCAPAI | t={t_rel:.2f}s dari start '
                    f'(arrival_time, terpisah dari rotasi-akhir)')
            # [FIX-GOALLATCH] Kunci pose saat SAMPAI (AMCL masih bagus). Sesudah ini
            # scoring memakai pose terkunci ini -> kebal teleport AMCL pasca-tiba.
            if not self._goal_latched[ns]:
                self._try_latch_goal_pose(ns)
        else:
            # [FIX-GOALLATCH] Re-approach nyata (DWA lepas position_reached): lepas latch
            # supaya pose akhir mengikuti realita baru, bukan snapshot lama.
            if self._goal_latched[ns]:
                self._goal_latched[ns] = False
                self.robot_latched_pose[ns] = None
                self.get_logger().info(f'[LOGGER] {ns} re-approach -> goal-latch dilepas')

    def consensus_progress_callback(self, msg, ns):
        self.consensus_progress[ns] = float(msg.data)

    def consensus_vmax_callback(self, msg, ns):
        self.consensus_vmax[ns] = float(msg.data)

    def priority_vmax_callback(self, msg, ns):
        self.priority_vmax[ns] = float(msg.data)

    def dwa_vmax_eff_callback(self, msg, ns):
        self.dwa_vmax_eff[ns] = float(msg.data)

    def heading_error_callback(self, msg, ns):
        self.heading_error[ns] = float(msg.data)
        self.heading_error_time[ns] = self.get_clock().now().nanoseconds / 1e9

    def _omega_raw_cb(self, msg, ns):
        self.omega_raw[ns] = float(msg.data)

    def _omega_after_clamp_cb(self, msg, ns):
        self.omega_after_clamp[ns] = float(msg.data)

    def _omega_global_limit_cb(self, msg, ns):
        self.omega_global_limit[ns] = float(msg.data)

    def _localization_hold_cb(self, msg, ns):
        self.localization_hold[ns] = bool(msg.data)

    def priority_stop_callback(self, msg, ns):
        prev = self.robot_priority_stop[ns]
        curr = bool(msg.data)
        self.robot_priority_stop[ns] = curr

        if not self.experiment_started or self.experiment_ended:
            return

        now   = self.get_clock().now().nanoseconds / 1e9
        t_rel = f'{now - self.t0:.4f}'

        if curr and not prev:
            self._stop_start[ns] = now
            self._w_stop.writerow({
                'timestamp_s': t_rel,
                'event_type' : 'STOP',
                'robot'      : ns,
                'duration_s' : '',
            })
            self._f_stop.flush()

        elif not curr and prev and self._stop_start[ns] is not None:
            duration = now - self._stop_start[ns]
            self._w_stop.writerow({
                'timestamp_s': t_rel,
                'event_type' : 'RESUME',
                'robot'      : ns,
                'duration_s' : f'{duration:.4f}',
            })
            self._f_stop.flush()
            self._stop_start[ns] = None

    def _claim_fault_event(self, ns, event_type, now):
        last_by_type = self._last_fault_event_time.setdefault(
            ns, {'START': None, 'END': None})
        last = last_by_type.get(event_type)
        if last is not None and now - last < self.fault_event_dedup_s:
            return False
        last_by_type[event_type] = now
        return True

    def fault_active_callback(self, msg, ns):
        prev = bool(self.fault_active.get(ns, False))
        curr = bool(msg.data)
        self.fault_active[ns] = curr
        if curr == prev:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if curr:
            self._fault_active_start[ns] = now
        start = self._fault_active_start.get(ns)

        if not self.experiment_started or self.experiment_ended or self.t0 is None:
            if not curr:
                self._fault_active_start[ns] = None
            return

        t_rel = f'{now - self.t0:.4f}'
        if curr:
            if not self._claim_fault_event(ns, 'START', now):
                return
            self._w_fault.writerow({
                'timestamp_s'  : t_rel,
                'event_type'   : 'START',
                'robot'        : ns,
                'ttf_planned_s': '',
                'ttf_actual_s' : '',
                'mode'         : 'fault_active_edge',
                'fault_type'   : '',
                'pulse_index'  : '',
                'planned_start_s': '',
                'actual_start_s': t_rel,
                'actual_end_s' : '',
                'duration_s'   : '',
            })
            self._f_fault.flush()
        else:
            if not self._claim_fault_event(ns, 'END', now):
                self._fault_active_start[ns] = None
                return
            duration = (now - start) if start is not None else ''
            self._w_fault.writerow({
                'timestamp_s'  : t_rel,
                'event_type'   : 'END',
                'robot'        : ns,
                'ttf_planned_s': '',
                'ttf_actual_s' : '',
                'mode'         : 'fault_active_edge',
                'fault_type'   : '',
                'pulse_index'  : '',
                'planned_start_s': '',
                'actual_start_s': f'{start - self.t0:.4f}' if start is not None else '',
                'actual_end_s' : t_rel,
                'duration_s'   : f'{duration:.4f}' if isinstance(duration, float) else '',
            })
            self._f_fault.flush()
            self._fault_active_start[ns] = None

    def _lane_offset_cb(self, msg, ns):
        self.robot_lane_offset[ns] = float(msg.data)

    def _dwa_mode_cb(self, msg, ns):
        self.robot_dwa_mode[ns] = str(msg.data)

    def _coordination_debug_cb(self, msg):
        try:
            self.coordination_debug = json.loads(msg.data)
        except Exception:
            return
        if not self.experiment_started or self.experiment_ended or self.t0 is None:
            return
        if not self.log_coordination_debug or self._w_coord is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        t_rel = f'{now - self.t0:.4f}'
        row = self._coordination_row(t_rel)
        self._w_coord.writerow(row)
        self._flush_files([self._f_coord], key='coord')

    def _coordination_row(self, t_rel):
        payload = self.coordination_debug or {}
        robots = payload.get('robots', {})
        def _r(ns, key):
            val = robots.get(ns, {}).get(key, '')
            return '' if val is None else val
        row = {
            'timestamp_s': t_rel,
            'coordination_mode': payload.get('coordination_mode', self.coordination_mode),
            'q_bar': '' if payload.get('q_bar') is None else payload.get('q_bar'),
        }
        key_map = [
            ('ETA', 'ETA'), ('A', 'A'), ('offset', 'offset'),
            ('q', 'q'), ('e', 'e'), ('v_consensus', 'v_consensus'),
        ]
        for key, col in key_map:
            for ns in ROBOT_NAMESPACES:
                row[f'{col}_{ns}'] = _r(ns, key)
        # [FIX-OBS] Status deteksi agen gagal [M4] dari consensus_node.
        failed = payload.get('failed', {}) or {}
        for ns in ROBOT_NAMESPACES:
            fv = failed.get(ns, None)
            row[f'failed_{ns}'] = '' if fv is None else int(bool(fv))
        de = payload.get('detection_enabled', None)
        row['detection_enabled'] = '' if de is None else int(bool(de))
        return row

    def _dynamic_obstacle_debug_cb(self, msg):
        if not self.experiment_started or self.experiment_ended or self.t0 is None:
            return
        if not self.log_dynamic_obstacle_debug or self._w_dynobs is None:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        t_rel = f'{now - self.t0:.4f}'
        for row in payload.get('rows', []):
            self._w_dynobs.writerow({
                'timestamp_s': t_rel,
                'robot': row.get('robot', ''),
                'peer_robot': row.get('peer_robot', ''),
                'crossing_cmd': row.get('crossing_cmd', ''),
                'peer_crossing_cmd': row.get('peer_crossing_cmd', ''),
                'peer_x': row.get('peer_x', ''),
                'peer_y': row.get('peer_y', ''),
                'distance_to_peer': row.get('distance_to_peer', ''),
                'blocks_path': row.get('blocks_path', ''),
                'bypass_active': row.get('bypass_active', ''),
                'bypass_x': row.get('bypass_x', ''),
                'bypass_y': row.get('bypass_y', ''),
                'rejected_candidate_count_dyn': row.get('rejected_candidate_count_dyn', ''),
                'min_dynamic_obstacle_distance': row.get('min_dynamic_obstacle_distance', ''),
                'front_dist': row.get('front_dist', ''),
                'front_blocked': row.get('front_blocked', ''),
                'peer_blocks_path': row.get('peer_blocks_path', ''),
                'peer_in_front_sector': row.get('peer_in_front_sector', ''),
                'holo_blk_reason': row.get('holo_blk_reason', ''),
                'convoy_follow_peer': row.get('convoy_follow_peer', ''),
            })
        self._flush_files([self._f_dynobs], key='dynobs')

    def _path_debug_cb(self, msg):
        if not self.experiment_started or self.experiment_ended or self.t0 is None:
            return
        if not self.log_path_debug or self._w_path_debug is None:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        t_rel = f'{now - self.t0:.4f}'
        paths = payload.get('paths', {})
        for ns in ROBOT_NAMESPACES:
            entry = paths.get(ns, {})
            self._w_path_debug.writerow({
                'timestamp_s': t_rel,
                'robot': ns,
                'point_count': entry.get('point_count', ''),
                'source': entry.get('source', ''),
                'age_s': entry.get('age_s', ''),
                'has_forward_path': entry.get('has_forward_path', ''),
                'auto_conflict_zone_enabled': payload.get('auto_conflict_zone_enabled', ''),
                'auto_zone_count': payload.get('auto_zone_count', ''),
                'active_zone_count': payload.get('active_zone_count', ''),
            })
        self._flush_files([self._f_path_debug], key='path_debug')

    def _conflict_zone_state_cb(self, msg):
        if not self.experiment_started or self.experiment_ended:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        wall_t  = payload.get('t', '')
        t_rel   = f'{now - self.t0:.4f}'
        for entry in payload.get('zones', []):
            self._w_conflict.writerow({
                'timestamp_s': t_rel,
                'zone'       : entry.get('zone', ''),
                'state'      : entry.get('state', ''),
                'owner'      : entry.get('owner', ''),
                'wall_t'     : f'{wall_t:.4f}' if isinstance(wall_t, float) else str(wall_t),
            })
        self._flush_files([self._f_conflict], key='conflict')

    def _conflict_zone_detail_cb(self, msg):
        if not self.experiment_started or self.experiment_ended:
            return
        if not self.log_conflict_detail or self._w_conflict_detail is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        t_rel = f'{now - self.t0:.4f}'
        for entry in payload.get('zones', []):
            robots = entry.get('robots', {})
            def _d(ns):   return robots.get(ns, {}).get('d', '')
            def _eta(ns): return robots.get(ns, {}).get('eta', '')
            def _cmd(ns): return robots.get(ns, {}).get('cmd', '')
            self._w_conflict_detail.writerow({
                'timestamp_s' : t_rel,
                'scenario'    : self.scenario,
                'zone'        : entry.get('zone', ''),
                'state'       : entry.get('state', ''),
                'owner'       : entry.get('owner', ''),
                'gap_remain_s': entry.get('gap_remain_s', ''),
                'source'      : entry.get('source', ''),
                'robot_pair'  : entry.get('robot_pair', ''),
                'center_x'    : entry.get('center_x', ''),
                'center_y'    : entry.get('center_y', ''),
                'radius'      : entry.get('radius', ''),
                'detect_radius': entry.get('detect_radius', ''),
                'hold_radius' : entry.get('hold_radius', ''),
                'clear_radius': entry.get('clear_radius', ''),
                'd_robot1'    : _d('robot1'),
                'd_robot2'    : _d('robot2'),
                'd_robot3'    : _d('robot3'),
                'eta_robot1'  : _eta('robot1'),
                'eta_robot2'  : _eta('robot2'),
                'eta_robot3'  : _eta('robot3'),
                'cmd_robot1'  : _cmd('robot1'),
                'cmd_robot2'  : _cmd('robot2'),
                'cmd_robot3'  : _cmd('robot3'),
            })
        self._flush_files([self._f_conflict_detail], key='conflict_detail')

    def fault_log_callback(self, msg):
        if not self.experiment_started or self.experiment_ended:
            return
        now  = self.get_clock().now().nanoseconds / 1e9
        t_rel = f'{now - self.t0:.4f}'
        data = msg.data.split(',')
        if len(data) < 4:
            return

        event_type = data[0]
        robot      = data[1]

        if event_type == 'FAULT_START':
            if not self._claim_fault_event(robot, 'START', now):
                return
            self._w_fault.writerow({
                'timestamp_s'  : t_rel,
                'event_type'   : 'START',
                'robot'        : robot,
                'ttf_planned_s': data[3] if len(data) > 3 else '',
                'ttf_actual_s' : '',
                'mode'         : data[4] if len(data) > 4 else '',
                'fault_type'   : data[8] if len(data) > 8 else '',
                'pulse_index'  : data[5] if len(data) > 5 else '',
                'planned_start_s': data[6] if len(data) > 6 else '',
                'actual_start_s': data[7] if len(data) > 7 else '',
                'actual_end_s' : '',
                'duration_s'   : '',
            })
        elif event_type == 'FAULT_END':
            if not self._claim_fault_event(robot, 'END', now):
                return
            self._w_fault.writerow({
                'timestamp_s'  : t_rel,
                'event_type'   : 'END',
                'robot'        : robot,
                'ttf_planned_s': data[4] if len(data) > 4 else '',
                'ttf_actual_s' : data[3] if len(data) > 3 else '',
                'mode'         : '',
                'fault_type'   : data[9] if len(data) > 9 else '',
                'pulse_index'  : data[5] if len(data) > 5 else '',
                'planned_start_s': data[6] if len(data) > 6 else '',
                'actual_start_s': data[7] if len(data) > 7 else '',
                'actual_end_s' : data[8] if len(data) > 8 else '',
                'duration_s'   : data[3] if len(data) > 3 else '',
            })
        self._f_fault.flush()

    # ═══════════════════════════════════════════════════════════════════════
    # LOG LOOP — 10 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def log_loop(self):
        if not self.experiment_started:
            return
        if self.experiment_ended:
            return
        if self.t0 is None:
            return

        now   = self.get_clock().now().nanoseconds / 1e9
        t_rel = now - self.t0

        # Trial timeout
        if (self.trial_timeout_s > 0 and t_rel >= self.trial_timeout_s):
            self.get_logger().warn(
                f'[LOGGER] Trial timeout {self.trial_timeout_s:.0f}s — auto-stop')
            self._write_goal_results()
            self._write_summary()
            self.experiment_ended = True
            return

        t = f'{t_rel:.4f}'

        # ── 1. Pose log ──────────────────────────────────────────────────
        for ns in ROBOT_NAMESPACES:
            pose = self.robot_pose[ns]
            cov  = self.robot_cov[ns]
            if pose is None:
                continue
            sigma_x = math.sqrt(max(0, cov[0]))  if cov else 0.0
            sigma_y = math.sqrt(max(0, cov[7]))  if cov else 0.0
            r_safe  = max(0.20, min(0.55,
                         0.25 + 3.0 * math.sqrt(sigma_x**2 + sigma_y**2)))
            sigma_valid = int(sigma_x < 0.3 and sigma_y < 0.3)
            self._w_pose.writerow({
                'timestamp_s': t, 'robot': ns,
                'x': f'{pose[0]:.5f}', 'y': f'{pose[1]:.5f}',
                'theta': f'{pose[2]:.5f}',
                'sigma_x': f'{sigma_x:.6f}', 'sigma_y': f'{sigma_y:.6f}',
                'r_safe_est': f'{r_safe:.5f}',
                'sigma_valid': sigma_valid,
                'pose_jump'  : int(self._pose_jump[ns]),
            })

        # ── 1b. Position success — stabil selama goal_stable_time ─────────
        for ns in ROBOT_NAMESPACES:
            if self.robot_pos_success_time[ns] is not None:
                continue
            pose = self.robot_pose[ns]
            goal = self.robot_goal[ns]
            if pose is None or goal is None:
                continue
            dist = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
            if dist <= self.goal_tolerance:
                if self.goal_inside_since[ns] is None:
                    self.goal_inside_since[ns] = now
                elif (now - self.goal_inside_since[ns]) >= self.goal_stable_time:
                    self.robot_pos_success_time[ns] = self.goal_inside_since[ns]
                    # [FIX-GOALLATCH] Kunci pose juga via jalur pos_success stabil.
                    if not self._goal_latched[ns]:
                        self._try_latch_goal_pose(ns)
            else:
                self.goal_inside_since[ns] = None

        # ── 2. Velocity log ──────────────────────────────────────────────
        for ns in ROBOT_NAMESPACES:
            vel = self.robot_cmdvel[ns]
            if vel is None:
                continue
            speed = math.hypot(vel[0], vel[1])
            self._w_vel.writerow({
                'timestamp_s'  : t, 'robot': ns,
                'vx': f'{vel[0]:.5f}', 'vy': f'{vel[1]:.5f}',
                'w' : f'{vel[2]:.5f}',
                'speed'        : f'{speed:.5f}',
                'omega_raw'    : f'{self.omega_raw[ns]:.5f}',
                'omega_after_clamp': f'{self.omega_after_clamp[ns]:.5f}',
                'omega_global_limit': f'{self.omega_global_limit[ns]:.5f}',
                'localization_hold_active': int(self.localization_hold[ns]),
                'fault_active' : int(self.fault_active[ns]),
                'priority_stop': int(self.robot_priority_stop[ns]),
                'vmax_consensus': f'{self.consensus_vmax[ns]:.5f}',
                'vmax_priority' : f'{self.priority_vmax[ns]:.5f}',
                'dwa_vmax_eff'  : f'{self.dwa_vmax_eff[ns]:.5f}',
                # [FIX-LANEOFF] robot_lane_offset dari /{ns}/crossing_lane_offset tidak
                # di-bridge ke PC; pakai lane_offset_robot (dari UDP) sebagai fallback
                # agar kolom 'lane_offset' tidak selalu 0.
                'lane_offset'  : f'{self.robot_lane_offset[ns] or self.lane_offset_robot[ns]:.4f}',
                # Robot-side (dikonfirmasi diterima):
                'priority_stop_robot': int(self.priority_stop_robot[ns]),
                'vmax_priority_robot': f'{self.vmax_priority_robot[ns]:.5f}',
                'lane_offset_robot'  : f'{self.lane_offset_robot[ns]:.4f}',
                # Tracking detail baru:
                'tracking_mode'      : self.tracking_mode[ns],
                'heading_error_deg'  : f'{math.degrees(self.heading_error[ns]):.2f}',
                'speed_mag'          : f'{self.dwa_speed_mag[ns]:.5f}',
            })

        # ── 3. Consensus log ─────────────────────────────────────────────
        active_p = [self.consensus_progress[ns] for ns in ROBOT_NAMESPACES]
        p_bar    = sum(active_p) / len(active_p)
        deltas   = [p - p_bar for p in active_p]
        coord_row = self._coordination_row(t)
        # [FIX-CONVMETRIC] Metrik konvergensi konsisten dengan definisi tesis:
        # utamakan nilai otoritatif dari consensus_node (mengecualikan agen gagal
        # & memakai ambang param). Fallback lokal kini JUGA mengecualikan agen
        # gagal (payload 'failed') dan robot yang sudah sampai goal, bukan lagi
        # rata-rata mentah ketiga robot.
        _payload = self.coordination_debug or {}
        _has_node_metric = ('consensus_max_deviation' in _payload
                            or 'consensus_converged' in _payload)
        _node_dev = _payload.get('consensus_max_deviation', None)
        _node_conv = _payload.get('consensus_converged', None)
        if _has_node_metric:
            # [FIX-CONVLOG] consensus_node adalah sumber otoritatif. Jika node
            # mengirim None, artinya konvergensi multi-robot tidak terdefinisi
            # (<2 robot sedang berkoordinasi). Jangan fallback ke rata-rata lokal
            # 3 robot karena itu menghidupkan lagi artefak metrik lama.
            max_dev = (float(_node_dev) if _node_dev is not None else None)
            converged = (int(bool(_node_conv)) if _node_conv is not None else '')
        else:
            _failed = _payload.get('failed', {}) or {}
            _eval = [i for i, ns in enumerate(ROBOT_NAMESPACES)
                     if not bool(_failed.get(ns, False))
                     and self.robot_pos_success_time[ns] is None]
            if len(_eval) < 2:
                max_dev = None
                converged = ''
            else:
                max_dev = max(abs(deltas[i]) for i in _eval)
                converged = int(max_dev < 0.02)

        cons_row = {
            'timestamp_s'    : t,
            'coordination_mode': coord_row.get('coordination_mode', self.coordination_mode),
            'p_robot1'       : f'{active_p[0]:.5f}',
            'p_robot2'       : f'{active_p[1]:.5f}',
            'p_robot3'       : f'{active_p[2]:.5f}',
            'p_bar'          : f'{p_bar:.5f}',
            'delta_p_robot1' : f'{deltas[0]:.5f}',
            'delta_p_robot2' : f'{deltas[1]:.5f}',
            'delta_p_robot3' : f'{deltas[2]:.5f}',
            'vmax_robot1'    : f'{self.consensus_vmax["robot1"]:.5f}',
            'vmax_robot2'    : f'{self.consensus_vmax["robot2"]:.5f}',
            'vmax_robot3'    : f'{self.consensus_vmax["robot3"]:.5f}',
            'max_deviation'  : ('' if max_dev is None else f'{max_dev:.5f}'),
            'converged'      : converged,
        }
        for key in [
            'ETA_robot1', 'ETA_robot2', 'ETA_robot3',
            'A_robot1', 'A_robot2', 'A_robot3',
            'offset_robot1', 'offset_robot2', 'offset_robot3',
            'q_robot1', 'q_robot2', 'q_robot3',
            'q_bar',
            'e_robot1', 'e_robot2', 'e_robot3',
            'v_consensus_robot1', 'v_consensus_robot2', 'v_consensus_robot3',
            'failed_robot1', 'failed_robot2', 'failed_robot3',
            'detection_enabled',
        ]:
            cons_row[key] = coord_row.get(key, '')
        self._w_cons.writerow(cons_row)

        # ── 3b. Mission progress log ────────────────────────────────────
        for ns in ROBOT_NAMESPACES:
            mt = self.mission_total[ns]
            mr = self.mission_remaining[ns]
            if mt is not None and mr is not None and mt > 0.1:
                mission_p = max(0.0, min(1.0, 1.0 - mr / mt))
            else:
                mission_p = float('nan')
            self._w_mission.writerow({
                'timestamp_s'          : t,
                'robot'                : ns,
                'mission_remaining_m'  : f'{mr:.5f}' if mr is not None else 'nan',
                'mission_total_m'      : f'{mt:.5f}' if mt is not None else 'nan',
                'mission_progress'     : f'{mission_p:.5f}' if not math.isnan(mission_p) else 'nan',
                'waypoint_progress'    : f'{self.consensus_progress[ns]:.5f}',
            })

        # ── 4. Cross-track error log ─────────────────────────────────────
        for ns in ROBOT_NAMESPACES:
            pose = self.robot_pose[ns]
            path = self.robot_path[ns]
            if pose is None or len(path) < 2:
                continue
            robot_theta  = pose[2] if len(pose) > 2 else 0.0
            path_len_ref = self.robot_path_length[ns] or 1.0
            ct_err, along_m, herr_rad, path_idx = self._compute_crosstrack(
                pose[0], pose[1], robot_theta, path, ns)
            self._update_crosstrack_stats(ns, ct_err)
            self._w_ct.writerow({
                'timestamp_s'       : t, 'robot': ns,
                'crosstrack_error_m': f'{ct_err:.5f}',
                'along_track_m'     : f'{along_m:.4f}',
                'path_progress'     : f'{along_m / path_len_ref:.5f}',
                'heading_error_deg' : f'{math.degrees(herr_rad):.2f}',
                'path_index'        : path_idx,
            })

        # ── 5. Inter-robot distance log ──────────────────────────────────
        # [FIX-LOGSTOP] Setelah SEMUA robot sampai posisi-tujuan, berhenti
        # mencatat & melacak jarak antar-robot. Sampel pasca-misi (robot parkir
        # berdempetan + drift AMCL saat diam) bukan kejadian navigasi dan
        # mencemari statistik pelanggaran jarak / min_dist. Dengan ini statistik
        # keselamatan hanya mencerminkan fase transit (sebelum semua tiba).
        all_arrived = all(
            (self.robot_position_arrival_time[ns] is not None
             or self.robot_goal_reached[ns])
            for ns in ROBOT_NAMESPACES)
        dists     = {}
        violation = False
        for (ns_a, ns_b) in ALL_PAIRS:
            pa = self.robot_pose[ns_a]
            pb = self.robot_pose[ns_b]
            if pa is None or pb is None:
                dists[(ns_a, ns_b)] = float('nan')
                continue
            d = math.hypot(pa[0]-pb[0], pa[1]-pb[1])
            dists[(ns_a, ns_b)] = d
            if not all_arrived and d < self.min_inter_dist_val(ns_a, ns_b):
                self.min_inter_dist[(ns_a, ns_b)] = d
            if not all_arrived and d < self.d_emergency:
                violation = True

        if not all_arrived:
            min_all = min((v for v in dists.values() if not math.isnan(v)),
                          default=float('nan'))
            self._w_ir.writerow({
            'timestamp_s' : t,
            'dist_r1_r2'  : f'{dists.get(("robot1","robot2"), float("nan")):.5f}',
            'dist_r1_r3'  : f'{dists.get(("robot1","robot3"), float("nan")):.5f}',
            'dist_r2_r3'  : f'{dists.get(("robot2","robot3"), float("nan")):.5f}',
            'min_dist'    : f'{min_all:.5f}' if not math.isnan(min_all) else 'nan',
            'violation'   : int(violation),
        })

        # ── 6. DWA mode log ─────────────────────────────────────────────────
        if self.log_dwa_mode and self._w_dwa_mode is not None:
            for ns in ROBOT_NAMESPACES:
                self._w_dwa_mode.writerow({
                    'timestamp_s': t,
                    'robot'      : ns,
                    'dwa_mode'   : self.robot_dwa_mode[ns],
                })

        main_files = [
            self._f_pose, self._f_vel, self._f_cons, self._f_mission,
            self._f_ct, self._f_ir,
        ]
        if self.log_dwa_mode:
            main_files.append(self._f_dwa_mode)
        self._flush_files(main_files, key='main')

    def min_inter_dist_val(self, ns_a, ns_b):
        return self.min_inter_dist.get((ns_a, ns_b), float('inf'))

    # ═══════════════════════════════════════════════════════════════════════
    # CROSS-TRACK ERROR CALCULATION
    # ═══════════════════════════════════════════════════════════════════════

    def _build_path_geometry(self, path):
        seg_lengths = []
        cumulative = [0.0]
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            seg_lengths.append(math.hypot(x2 - x1, y2 - y1))
            cumulative.append(cumulative[-1] + seg_lengths[-1])
        return {
            'seg_lengths': seg_lengths,
            'cumulative': cumulative,
            'point_count': len(path),
        }

    @staticmethod
    def _new_crosstrack_stats():
        return {
            'count': 0,
            'min': float('inf'),
            'max': float('-inf'),
            'sum_sq': 0.0,
        }

    def _update_crosstrack_stats(self, ns, err):
        stats = self.crosstrack_stats[ns]
        stats['count'] += 1
        stats['min'] = min(stats['min'], err)
        stats['max'] = max(stats['max'], err)
        stats['sum_sq'] += err * err

    def _crosstrack_summary(self, ns):
        stats = self.crosstrack_stats.get(ns) or self._new_crosstrack_stats()
        count = int(stats.get('count', 0))
        if count <= 0:
            return float('nan'), float('nan'), float('nan')
        return (
            float(stats['min']),
            float(stats['max']),
            float(stats['sum_sq']) / count,
        )

    def _settled_pose(self, ns):
        """[FIX-SETTLED] Rata-rata pose dlm jendela 'settled' terakhir (robust thd
        jitter/drift AMCL). Return (mean_x, mean_y, std_m, n) atau None bila kosong."""
        buf = self._settle_buf.get(ns) or []
        if not buf:
            return None
        t_last = buf[-1][0]
        cutoff = t_last - self.settled_window_s
        sel = [(x, y) for (t, x, y) in buf if t >= cutoff]
        if len(sel) < self.settled_min_samples:
            sel = [(x, y) for (_, x, y) in buf[-self.settled_min_samples:]]
        n = len(sel)
        if n == 0:
            return None
        mx = sum(p[0] for p in sel) / n
        my = sum(p[1] for p in sel) / n
        var = sum((p[0] - mx) ** 2 + (p[1] - my) ** 2 for p in sel) / n
        return (mx, my, math.sqrt(max(0.0, var)), n)

    def _settled_precision(self, ns, goal):
        """[FIX-SETTLED] Presisi posisi memakai pose settled (rata-rata jendela).
        Return (precision_m, std_m, n)."""
        sp = self._settled_pose(ns)
        if sp is None or not goal:
            return float('nan'), float('nan'), 0
        mx, my, std, n = sp
        return math.hypot(mx - goal[0], my - goal[1]), std, n

    def _compute_crosstrack(self, rx, ry, robot_theta, path, ns=None):
        """Hitung cross-track error, along-track distance, dan heading error.

        Returns:
            crosstrack_m   : jarak tegak lurus robot ke path (m)
            along_track_m  : jarak kumulatif dari awal path ke titik terdekat (m)
            heading_err_rad: selisih heading robot vs tangent path (rad, [-pi,pi])
            best_idx       : indeks segmen path terdekat
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.0, 0

        geom = self.robot_path_geometry.get(ns) if ns else None
        if not geom or geom.get('point_count') != len(path):
            geom = self._build_path_geometry(path)
        seg_lengths = geom.get('seg_lengths', [])
        cumulative = geom.get('cumulative', [0.0])

        min_dist  = float('inf')
        best_idx  = 0
        best_t    = 0.0

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx*dx + dy*dy
            if seg_len_sq < 1e-9:
                continue
            t = ((rx-x1)*dx + (ry-y1)*dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            cx   = x1 + t*dx
            cy   = y1 + t*dy
            dist = math.hypot(rx-cx, ry-cy)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_t   = t

        # Along-track: panjang path dari awal sampai titik terdekat
        along_track = cumulative[best_idx] + best_t * seg_lengths[best_idx]

        # Heading error: heading robot vs tangent path di segmen terdekat
        x1, y1 = path[best_idx]
        x2, y2 = path[best_idx + 1]
        path_heading = math.atan2(y2 - y1, x2 - x1)
        heading_err  = math.atan2(
            math.sin(robot_theta - path_heading),
            math.cos(robot_theta - path_heading))

        return min_dist, along_track, heading_err, best_idx

    # ═══════════════════════════════════════════════════════════════════════
    # PENULISAN HASIL AKHIR
    # ═════════════════════════════��═════════════════════════════════════════

    def _write_goal_results(self):
        if self._goal_results_written:
            return
        t0 = self.t0 or 0.0

        for ns in ROBOT_NAMESPACES:
            # [FIX-GOALLATCH] Pakai pose terkunci saat SAMPAI bila ada (kebal teleport AMCL).
            pose  = (self.robot_latched_pose[ns]
                     if self.robot_latched_pose[ns] is not None
                     else self.robot_pose[ns])
            goal  = self.robot_goal[ns]
            arr_t = self.robot_arrival_time[ns]
            pos_arr_t = self.robot_position_arrival_time[ns]  # [FIX-ARRIVE-LOG]

            if pose is None:
                # [FIX-DNFROW] Jangan skip robot yang tidak punya pose. Kalau di-skip,
                # goal_result.csv kehilangan baris robot tsb dan analyzer bisa salah
                # menganggap robot "absen", bukan DNF. Tulis DNF eksplisit agar
                # success_rate dan active_robots tetap jujur.
                self._w_goal.writerow({
                    'robot'                      : ns,
                    'state_success'              : 'NO',
                    'position_success'           : 'NO',
                    'arrival_time_from_start_s'  : 'DNF',
                    'full_goal_time_from_start_s': 'DNF',
                    'position_success_time_s'    : 'DNF',
                    'target_arrival_time_s'      : f'{self.target_arrival.get(ns, 0.0):.4f}' if self.target_arrival.get(ns, 0.0) > 0 else 'N/A',
                    'arrival_time_error_s'       : 'N/A',
                    'final_error_m'              : 'nan',
                    'final_heading_error_deg'    : 'nan',
                    'heading_success'            : 'N/A',
                    'goal_x'                     : f'{goal[0]:.5f}' if goal else '',
                    'goal_y'                     : f'{goal[1]:.5f}' if goal else '',
                    'final_x'                    : '',
                    'final_y'                    : '',
                    'goal_precision_m'           : 'nan',
                    'settled_precision_m'        : 'nan',
                    'settled_pose_std_m'         : 'nan',
                    'settled_sample_count'       : '0',
                    'path_length_planned_m'      : f'{self.robot_path_length[ns]:.4f}',
                    'crosstrack_min_m'           : 'nan',
                    'crosstrack_max_m'           : 'nan',
                    'crosstrack_mse_m2'          : 'nan',
                })
                continue

            goal_prec = (math.hypot(pose[0]-goal[0], pose[1]-goal[1])
                         if goal else float('nan'))

            ct_min, ct_max, ct_mse = self._crosstrack_summary(ns)
            sp_prec, sp_std, sp_n = self._settled_precision(ns, goal)

            pos_ok   = (not math.isnan(goal_prec) and goal_prec <= self.goal_tolerance)
            # [FIX-SCORE-POS] Kriteria lulus = AKURASI POSISI (final rotation sudah
            # dihapus). Heading akhir hanya dicatat sbg info, bukan syarat lulus.
            state_ok = pos_ok
            pos_t = self.robot_pos_success_time[ns]

            # [FIX-ARRIVE-LOG] arrival_time_from_start_s = SAAT POSISI tercapai
            # (sinyal /position_reached dari DWA), TERPISAH dari rotasi-akhir.
            if pos_arr_t is not None and pos_arr_t >= t0:
                from_start_str = f'{pos_arr_t - t0:.4f}'
            elif pos_t is not None and pos_t >= t0:
                from_start_str = f'{pos_t - t0:.4f}'
            elif pos_ok:
                from_start_str = 'DNF_POS_OK'
            else:
                from_start_str = 'DNF'
            # full_goal_time_from_start_s = saat goal PENUH (posisi+heading) tercapai.
            # [FIX-SCORE-POS] full_goal_time_from_start_s = waktu SAMPAI-POSISI
            # (heading akhir tak lagi disyaratkan sejak final rotation dihapus).
            if pos_arr_t is not None and pos_arr_t >= t0:
                full_goal_str = f'{pos_arr_t - t0:.4f}'
            elif pos_t is not None and pos_t >= t0:
                full_goal_str = f'{pos_t - t0:.4f}'
            else:
                full_goal_str = 'DNF'

            if pos_t is not None and pos_t >= t0:
                pos_t_str = f'{pos_t - t0:.4f}'
            else:
                pos_t_str = 'DNF'

            t_tgt = self.target_arrival.get(ns, 0.0)
            if t_tgt > 0 and pos_arr_t is not None and pos_arr_t >= t0:
                err_str   = f'{(pos_arr_t - t0) - t_tgt:.4f}'
                t_tgt_str = f'{t_tgt:.4f}'
            elif t_tgt > 0 and pos_t is not None and pos_t >= t0:
                err_str   = f'{(pos_t - t0) - t_tgt:.4f}'
                t_tgt_str = f'{t_tgt:.4f}'
            elif t_tgt > 0 and arr_t is not None and arr_t >= t0:
                # [FIX-SCORE-POS] state_ok kini = pos_ok (tak menjamin arr_t!=None);
                # guard eksplisit arr_t agar tak TypeError saat fallback waktu-penuh.
                err_str   = f'{(arr_t - t0) - t_tgt:.4f}'
                t_tgt_str = f'{t_tgt:.4f}'
            else:
                err_str   = 'N/A'
                t_tgt_str = f'{t_tgt:.4f}' if t_tgt > 0 else 'N/A'

            # Di dekat goal, /robotX/heading_error dari DWA adalah referensi
            # paling jujur karena mengikuti yaw final /plan yang sedang dikejar
            # controller. robot_goal bisa berasal dari snapshot pre-start dan
            # pernah stale ke yaw 0 walau posisi formasi sudah benar.
            dwa_herr = self.heading_error.get(ns)
            dwa_herr_t = self.heading_error_time.get(ns)
            if (pos_ok and dwa_herr_t is not None and dwa_herr_t >= t0
                    and dwa_herr is not None
                    and math.isfinite(float(dwa_herr))):
                herr = float(dwa_herr)
                herr_deg_str = f'{math.degrees(abs(herr)):.2f}'
                heading_ok_str = (
                    'YES' if abs(herr) <= self.heading_goal_tolerance else 'NO')
            elif (goal is not None and pose is not None
                    and len(pose) > 2 and len(goal) > 2):
                herr = math.atan2(math.sin(goal[2] - pose[2]),
                                  math.cos(goal[2] - pose[2]))
                herr_deg_str = f'{math.degrees(abs(herr)):.2f}'
                heading_ok_str = (
                    'YES' if abs(herr) <= self.heading_goal_tolerance else 'NO')
            else:
                herr_deg_str = 'nan'
                heading_ok_str = 'N/A'

            self._w_goal.writerow({
                'robot'                      : ns,
                'state_success'              : 'YES' if state_ok else 'NO',
                'position_success'           : 'YES' if pos_ok else 'NO',
                'arrival_time_from_start_s'  : from_start_str,
                'full_goal_time_from_start_s': full_goal_str,
                'position_success_time_s'    : pos_t_str,
                'target_arrival_time_s'      : t_tgt_str,
                'arrival_time_error_s'       : err_str,
                'final_error_m'              : f'{goal_prec:.5f}' if not math.isnan(goal_prec) else 'nan',
                'final_heading_error_deg'    : herr_deg_str,
                'heading_success'            : heading_ok_str,
                'goal_x'                     : f'{goal[0]:.5f}' if goal else '',
                'goal_y'                     : f'{goal[1]:.5f}' if goal else '',
                'final_x'                    : f'{pose[0]:.5f}',
                'final_y'                    : f'{pose[1]:.5f}',
                'goal_precision_m'           : f'{goal_prec:.5f}' if not math.isnan(goal_prec) else 'nan',
                'settled_precision_m'        : f'{sp_prec:.5f}' if not math.isnan(sp_prec) else 'nan',
                'settled_pose_std_m'         : f'{sp_std:.5f}' if not math.isnan(sp_std) else 'nan',
                'settled_sample_count'       : str(sp_n),
                'path_length_planned_m'      : f'{self.robot_path_length[ns]:.4f}',
                'crosstrack_min_m'           : f'{ct_min:.5f}' if not math.isnan(ct_min) else 'nan',
                'crosstrack_max_m'           : f'{ct_max:.5f}' if not math.isnan(ct_max) else 'nan',
                'crosstrack_mse_m2'          : f'{ct_mse:.6f}' if not math.isnan(ct_mse) else 'nan',
            })
        self._f_goal.flush()
        self._goal_results_written = True
        self.get_logger().info('[LOGGER] goal_result.csv ditulis.')

    def _write_summary(self):
        now    = self.get_clock().now().nanoseconds / 1e9
        t0     = self.t0 or now
        dur    = now - t0

        arr = {}
        for ns in ROBOT_NAMESPACES:
            arr[ns] = self.robot_position_arrival_time.get(ns)
            if arr[ns] is None:
                arr[ns] = self.robot_pos_success_time.get(ns)
        arr_valid = [v - t0 for v in arr.values() if v is not None and v >= t0]
        arr_diff  = (max(arr_valid) - min(arr_valid)) if len(arr_valid) > 1 else float('nan')

        global_min = min(self.min_inter_dist.values()) if self.min_inter_dist else float('nan')

        lines = [
            '=' * 60,
            'EXPERIMENT SUMMARY — haqqi_ta',
            '=' * 60,
            f'Scenario     : {self.scenario}',
            f'Trial        : #{self._trial_count}',
            f'Duration     : {dur:.2f}s',
            f'Timestamp    : {datetime.now().isoformat()}',
            '',
            '── Arrival Time (Position) ───────────────────────────────',
        ]
        for ns in ROBOT_NAMESPACES:
            v    = arr[ns]
            # [FIX-SETTLED] Konsisten dgn goal_result: pakai pose terkunci (kebal teleport).
            pose = (self.robot_latched_pose[ns]
                    if self.robot_latched_pose[ns] is not None else self.robot_pose[ns])
            goal = self.robot_goal[ns]
            gp   = (math.hypot(pose[0]-goal[0], pose[1]-goal[1])
                    if pose and goal else float('nan'))
            pos_ok = (not math.isnan(gp) and gp <= self.goal_tolerance)
            if v is None:
                rel = 'DNF_POS_OK' if pos_ok else 'DNF'
            elif v < t0:
                rel = 'STALE'
            else:
                rel = f'{v - t0:.3f}s'
            pos_tag = f'  [pos_ok={pos_ok}, err={gp:.3f}m]' if not math.isnan(gp) else ''
            lines.append(f'  {ns}: {rel}{pos_tag}')
        lines.append(f'  Arrival Time Diff: {arr_diff:.4f}s'
                     if not math.isnan(arr_diff) else '  Arrival Time Diff: N/A')

        lines += ['', '── Goal Precision ────────────────────────────────────────']
        for ns in ROBOT_NAMESPACES:
            # [FIX-SETTLED] Pose terkunci + presisi settled (rata-rata jendela akhir).
            pose = (self.robot_latched_pose[ns]
                    if self.robot_latched_pose[ns] is not None else self.robot_pose[ns])
            goal = self.robot_goal[ns]
            if pose and goal:
                gp = math.hypot(pose[0]-goal[0], pose[1]-goal[1])
                sp_prec, sp_std, sp_n = self._settled_precision(ns, goal)
                if not math.isnan(sp_prec):
                    lines.append(f'  {ns}: {gp:.5f}m  [settled={sp_prec:.5f}m '
                                 f'std={sp_std:.4f}m n={sp_n}]')
                else:
                    lines.append(f'  {ns}: {gp:.5f}m')
            else:
                lines.append(f'  {ns}: N/A')

        lines += ['', '── Cross-Track Error ─────────────────────────────────────']
        for ns in ROBOT_NAMESPACES:
            ct_min, ct_max, mse = self._crosstrack_summary(ns)
            if not math.isnan(mse):
                lines.append(
                    f'  {ns}: min={ct_min:.5f}m  '
                    f'max={ct_max:.5f}m  '
                    f'MSE={mse:.6f}m²')
            else:
                lines.append(f'  {ns}: N/A')

        lines += [
            '',
            '── Inter-Robot Distance ──────────────────────────────────',
            f'  Global minimum: {global_min:.5f}m '
            f'(d_emergency={self.d_emergency}m)',
        ]
        for pair, d in self.min_inter_dist.items():
            status = '⚠ VIOLATION' if d < self.d_emergency else 'OK'
            lines.append(f'  {pair[0]}↔{pair[1]}: {d:.5f}m [{status}]')

        lines += ['', '=' * 60]

        summary_path = os.path.join(self.exp_dir, 'experiment_summary.txt')
        with open(summary_path, 'w') as f:
            f.write('\n'.join(lines))

        for line in lines:
            self.get_logger().info(line)

    # ═══════════════════════════════════════════════════════════════════════
    # STATUS REPORT — 1 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def status_report(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is not None and self.experiment_started:
            elapsed_str = f't={now - self.t0:.1f}s'
        else:
            elapsed_str = 'STANDBY'

        reached   = sum(1 for ns in ROBOT_NAMESPACES if self.robot_goal_reached[ns])
        fault_str = '+'.join(ns for ns in ROBOT_NAMESPACES if self.fault_active[ns]) or 'none'
        self.get_logger().info(
            f'[LOGGER] {elapsed_str} | '
            f'goal={reached}/3 | '
            f'fault={fault_str} | '
            f'trial=#{self._trial_count} | '
            f'dir={self.exp_dir}')

        # [FIX-STANDBYWARN] Jika telemetri mengalir tapi state bukan RUNNING,
        # terbitkan WARN ber-throttle agar masalah log-kosong terlihat saat run live.
        if not self.experiment_started and not self.experiment_ended:
            has_telemetry = any(self.robot_pose[ns] is not None for ns in ROBOT_NAMESPACES)
            if not has_telemetry:
                has_telemetry = any(self.robot_cmdvel[ns] is not None for ns in ROBOT_NAMESPACES)
            if has_telemetry:
                import time as _time
                now_wall = _time.time()
                if now_wall - self._last_standby_warn_t >= self._standby_warn_period_s:
                    self.get_logger().warn(
                        '[FIX-STANDBYWARN] Telemetri robot mengalir tapi logger TIDAK RUNNING '
                        '— log per-tick KOSONG. Pastikan /experiment_state=RUNNING atau '
                        'tekan [4] START di CLI.')
                    self._last_standby_warn_t = now_wall

        # [FIX-NOPOSE] Kebalikan STANDBYWARN: state RUNNING tapi pose AMCL tidak
        # pernah masuk dari robot manapun. Akibatnya pose_log/velocity_log/
        # crosstrack_log/interrobot_log KOSONG, DWA tetap IDLE, dan robot tak
        # bergerak (run = DNF semua). Teriak ERROR ber-throttle supaya kondisi
        # ini terlihat live, bukan baru ketahuan dari CSV kosong setelah trial.
        if self.experiment_started and not self.experiment_ended:
            no_pose = all(self.robot_pose[ns] is None for ns in ROBOT_NAMESPACES)
            if no_pose:
                import time as _time
                now_wall = _time.time()
                if now_wall - self._last_nopose_warn_t >= self._standby_warn_period_s:
                    self.get_logger().error(
                        '[FIX-NOPOSE] RUNNING tapi TIDAK ada pose AMCL dari robot manapun '
                        '— pose_log/velocity_log KOSONG & robot tetap IDLE. Cek AMCL tiap '
                        'robot + UDP pose (port 9001-9003). RUN INI TIDAK VALID, STOP & perbaiki.')
                    self._last_nopose_warn_t = now_wall

    # ═══════════════════════════════════════════════════════════════════════
    # CLEANUP
    # ═══════════════════════════════════════════════════════════════════════

    def close_files(self):
        if self._trial_count == 0 and not self.experiment_started:
            self._close_csv_files()
            self.get_logger().info(f'[LOGGER] Ditutup tanpa trial: {self.exp_dir}')
            return
        if not self.experiment_ended:
            self.get_logger().warn(
                '[LOGGER] Eksperimen belum selesai saat shutdown — '
                'menulis hasil parsial')
            self._write_goal_results()
            self._write_summary()
        self._close_csv_files()
        self.get_logger().info(f'[LOGGER] Semua file ditutup: {self.exp_dir}')


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logger stopped by user')
    except Exception:
        node.get_logger().error('[LOGGER] Unhandled exception:\n' + traceback.format_exc())
        raise
    finally:
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
