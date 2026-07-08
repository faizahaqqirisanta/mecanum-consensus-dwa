#!/usr/bin/env python3
"""
Modified DWA Node — haqqi_ta
Basis: HeadingAlignedDWATracker (fardli_dwa/simple_dwa_node.py)

MODIFIKASI:
  [MOD-1] Namespace robot via parameter robot_ns
  [MOD-2] Subscriber vmax_consensus (L4) dan vmax_priority (L5)
  [MOD-3] Subscriber priority_stop (L5) dan fault_active
  [MOD-4] Dynamic r_safe dari EKF covariance di check_collision()
  [MOD-5] Cross-track deviation score di calculate_path_following_score()
  [MOD-6] Direction-conditioned motion policy (FRONT/SIDE/REVERSE MODE)
  [MOD-7] Trajectory collision check di 4 titik sepanjang horizon via
          _predict_and_check_collision() — menggantikan endpoint-only check
  [MOD-8] Threshold collision check: point_dist + r_safe (bukan point_dist - r_safe)
  [MOD-9] Collision check mencakup seluruh 360° (filter ±90° dihapus)
  [MOD-10] path_callback langsung ke STATE_TRACKING (bukan STATE_ALIGNING)
  [MOD-12] Local costmap DWA — grid ringan dibangun dari LaserScan, kandidat
           trajectory dievaluasi terhadap costmap; mode dipilih via avoidance_mode
  [MOD-14] Corner-aware speed control — deteksi tikungan di depan dengan
           membandingkan arah path sekarang vs corner_slowdown_radius ke depan;
           reduce vmax_eff ke corner_speed_ratio saat tikungan > threshold;
           adaptive lookahead: gunakan corner_lookahead_distance saat corner aktif
           agar robot tidak memotong tikungan
  [MOD-15] Blocked recovery with backtracking — jika BLOCKED > blocked_timeout,
           robot mundur ke titik aman di path (backtrack_distance ke belakang),
           lalu resume TRACKING setelah obstacle clear selama clear_time
  [MOD-16] Lateral suppression saat heading error besar — batasi max |vy| saat
           robot belum align dengan arah segmen baru; mencegah robot geser lateral
           alih-alih benar-benar belok (khususnya split/convoy setelah tikungan)
  [MOD-17] Holonomic path tracker (vector-field follower) sebagai primary command:
           hitung CTE dari closest segment, buat (vx, vy, omega) langsung dari
           cross-track error + heading error; DWA/costmap hanya sebagai safety
           filter — jika trajectory aman, pakai; jika tidak, masuk BLOCKED.

BAGIAN DASAR:
  - heading_alignment_control(), approach_control(), final_alignment_control()
  - predict_trajectory()
  - find_current_position_on_path(), update_tracking_position()
  - get_next_target_with_heading()
  - publish_command(), publish_local_plan(), stop_robot()
  - closest_point_on_segment(), normalize_angle()

CATATAN state machine:
  - STATE_ALIGNING masih ada di kode untuk heading_alignment_control(),
    tapi determine_state() tidak pernah masuk ke sana secara normal —
    mecanum tidak perlu spin awal. ALIGNING hanya dimasuki sebagai
    fallback dari path_callback yang lama (sudah diganti ke TRACKING).
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import json
import os
import csv
import math
import time
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, String
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

# Urutan prioritas Stop-and-Go L5: robot1 (leader) > robot2 > robot3 (tail).
# Angka lebih besar = prioritas lebih tinggi.
# HARUS konsisten dengan priority_manager_node.ALL_PAIRS:
#   ('robot2','robot1'), ('robot3','robot1'), ('robot3','robot2')
#   → robot2 berhenti untuk robot1, robot3 berhenti untuk robot1 dan robot2.
_PRIORITY_RANK = {'robot1': 3, 'robot2': 2, 'robot3': 1}


class ModifiedDWANode(Node):
    def __init__(self):
        super().__init__('modified_dwa_node')

        # ── [MOD-1] Namespace robot via parameter ──────────────────────────
        # Set saat launch: ros2 run haqqi_ta modified_dwa_node
        #                  --ros-args -p robot_ns:=robot1
        self.declare_parameter('robot_ns', 'robot1')
        # Topic AMCL — '/amcl_pose' jika AMCL tanpa namespace,
        # '/{robot_ns}/amcl_pose' jika AMCL dijalankan dalam namespace.
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.ns = self.get_parameter('robot_ns').value
        self._amcl_topic = self.get_parameter('amcl_pose_topic').value
        self.get_logger().info(f'Modified DWA starting for namespace: /{self.ns}')

        # ── Parameter gerak ────────────────────────────────────────────────
        # Default values = nilai safe yang sama dengan dwa_robot*.yaml
        # Ini hanya fallback; nilai aktual dioverride oleh YAML saat launch
        self.declare_parameter('max_vel_x', 0.20)
        self.declare_parameter('max_vel_y', 0.05)
        self.declare_parameter('max_rot_vel', 0.10)
        self.declare_parameter('min_vel_x', -0.03)   # batas bawah vx (mundur maks)
        # ── [FF-CATCHUP] Feedforward catch-up (robot tertinggal mempercepat) ─
        # Konsensus sudah menghitung v = v_nom + k_consensus*(p_bar - p_i) hingga
        # v_consensus_ceiling (mis. 0.50). TAPI vmax_callback & path-tracker
        # meng-clamp nilai itu ke max_vel_x, sehingga boost catch-up hilang dan
        # sistem hanya bisa MEMPERLAMBAT. Bila parameter ini True, boost
        # feedforward diteruskan di ATAS max_vel_x sampai vmax_catchup_ceiling
        # sehingga robot tertinggal benar-benar bisa MEMPERCEPAT.
        # Default False = perilaku baseline (deselerasi murni) -> hasil lama aman.
        self.declare_parameter('feedforward_catchup_enabled', True)
        self.declare_parameter('vmax_catchup_ceiling', 0.50)
        # [FF-CATCHUP] Plafon kecepatan FASE AKHIR saat catch-up aktif (m/s).
        # Hanya dipakai bila feedforward_catchup_enabled=True; default baseline
        # tetap 0.1 (approach) & 0.03 (align) sehingga perilaku lama tak berubah.
        self.declare_parameter('approach_speed_catchup', 0.35)
        self.declare_parameter('align_speed_catchup', 0.06)
        self.declare_parameter('min_vel_y', -0.05)   # batas bawah vy (lateral negatif)
        self.declare_parameter('lookahead_distance', 0.50)
        self.declare_parameter('goal_tolerance', 0.10)   # m — ambang "boleh berhenti" (error goal min 0.1)
        self.declare_parameter('path_tracking_tolerance', 0.40)
        self.declare_parameter('heading_alignment_tolerance', 0.50)
        self.declare_parameter('min_heading_error_for_sideways', 0.3)
        self.declare_parameter('max_sideways_ratio', 0.3)
        self.declare_parameter('final_orientation_weight', 3.0)
        self.declare_parameter('position_first', True)
        self.declare_parameter('prediction_time', 1.5)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('vx_samples', 7)
        self.declare_parameter('vy_samples', 7)
        self.declare_parameter('w_samples', 5)
        # [ALGO-TRACE] perekaman kipas kandidat DWA + local plan untuk live-plot
        self.declare_parameter('algo_trace_enabled', False)
        self.declare_parameter('algo_trace_dir', '/tmp/algo_trace')
        self.declare_parameter('algo_trace_dwa_period_s', 0.5)
        self.declare_parameter('r_safe_base', 0.23)
        self.declare_parameter('k_sigma', 3.0)
        self.declare_parameter('r_safe_min', 0.18)
        self.declare_parameter('r_safe_max', 0.50)
        self.declare_parameter('max_pose_variance', 0.30)

        # [MOD-11] Reactive LiDAR avoidance parameters
        self.declare_parameter('enable_reactive_avoidance',  False)
        self.declare_parameter('front_warning_dist',         0.80)
        self.declare_parameter('front_stop_dist',            0.45)
        self.declare_parameter('side_clearance_min',         0.35)
        self.declare_parameter('avoid_vx_max',               0.06)
        self.declare_parameter('avoid_vy_speed',             0.08)
        self.declare_parameter('avoid_w_max',                0.12)
        self.declare_parameter('obstacle_clearance_weight',  0.08)
        self.declare_parameter('avoid_lateral_weight',       0.40)
        self.declare_parameter('stuck_timeout',              8.0)
        self.declare_parameter('stuck_min_progress',         0.10)
        self.declare_parameter('enable_stuck_detector',      False)
        self.declare_parameter('dynamic_side_avoid_enabled', True)
        self.declare_parameter('dynamic_side_avoid_speed',   0.12)
        self.declare_parameter('dynamic_side_avoid_vy',      0.18)
        self.declare_parameter('dynamic_side_avoid_duration', 4.0)
        self.declare_parameter('dynamic_side_return_dist',   0.45)
        self.declare_parameter('dynamic_side_min_shift',     0.22)
        self.declare_parameter('dynamic_side_clear_confirm', 4)
        self.declare_parameter('dynamic_side_cooldown',      0.50)
        self.declare_parameter('hard_peer_escape_enabled',   True)
        self.declare_parameter('hard_peer_escape_distance',  0.38)
        self.declare_parameter('hard_peer_escape_speed',     0.09)
        self.declare_parameter('hard_peer_escape_side_clearance', 0.18)

        # [MOD-12] Local costmap DWA parameters
        # avoidance_mode: 'reactive' (sector-based, legacy) | 'costmap' (grid-based)
        self.declare_parameter('avoidance_mode',             'costmap')
        self.declare_parameter('local_costmap_size',         3.0)
        self.declare_parameter('local_costmap_resolution',   0.05)
        self.declare_parameter('obstacle_min_range',         0.08)
        self.declare_parameter('obstacle_max_range',         2.5)
        self.declare_parameter('inflation_radius',           0.15)
        self.declare_parameter('lethal_cost',                255)
        self.declare_parameter('lethal_threshold',           250)
        self.declare_parameter('local_cost_weight',          0.05)
        self.declare_parameter('cost_max_weight',            0.7)
        self.declare_parameter('cost_avg_weight',            0.3)

        # [MOD-14] Corner-aware speed control + adaptive lookahead
        self.declare_parameter('corner_slowdown_enabled',    False)
        self.declare_parameter('corner_angle_threshold_deg', 35.0)
        self.declare_parameter('corner_slowdown_radius',     0.90)
        self.declare_parameter('corner_speed_ratio',         0.30)
        self.declare_parameter('corner_lookahead_distance',  0.25)

        # [MOD-15] Blocked recovery with backtracking
        self.declare_parameter('enable_backtracking_recovery', True)
        self.declare_parameter('blocked_timeout',            3.0)
        self.declare_parameter('backtrack_distance',         0.40)
        self.declare_parameter('clear_time',                 1.5)
        self.declare_parameter('backtracking_speed',         0.05)

        # [MOD-16] Lateral suppression saat heading error besar
        self.declare_parameter('heading_lateral_suppress_enabled', True)
        self.declare_parameter('heading_lateral_threshold_deg',    30.0)
        self.declare_parameter('heading_lateral_max_vy',           0.01)

        # [MOD-17] Holonomic path tracker
        self.declare_parameter('use_holonomic_path_tracker', True)
        self.declare_parameter('k_cte',                      0.6)
        self.declare_parameter('k_heading',                  0.4)
        self.declare_parameter('max_lateral_correction',     0.05)
        self.declare_parameter('max_heading_w',              0.10)
        self.declare_parameter('path_heading_tracking_enabled', False)
        self.declare_parameter('heading_translation_gate_enabled', True)
        self.declare_parameter('heading_translation_gate_deg', 25.0)
        self.declare_parameter('heading_translation_gate_scale', 0.0)
        self.declare_parameter('angular_translation_lock_enabled', True)
        self.declare_parameter('angular_translation_lock_w', 0.08)
        self.declare_parameter('angular_translation_lock_scale', 0.0)
        self.declare_parameter('final_goal_vector_radius', 1.20)
        self.declare_parameter('final_goal_vector_kp', 0.8)
        self.declare_parameter('omega_global_limit',          0.10)
        self.declare_parameter('omega_slew_rate_limit',       0.04)
        self.declare_parameter('motion_mixing_guard_enabled', True)
        self.declare_parameter('mixing_omega_vy_zero_threshold', 0.045)
        self.declare_parameter('mixing_heading_error_deg', 25.0)
        self.declare_parameter('mixing_heading_vx_scale', 0.35)
        self.declare_parameter('mixing_corner_vx_scale', 0.70)
        self.declare_parameter('mixing_corner_omega_limit', 0.06)
        self.declare_parameter('mixing_corner_vy_zero', True)
        self.declare_parameter('localization_guard_enabled',  True)    # [LOC-GUARD] diaktifkan lagi utk crossing
        self.declare_parameter('localization_sigma_threshold', 0.30)
        self.declare_parameter('localization_hard_sigma_threshold', 0.80)
        self.declare_parameter('localization_invalid_hold_s', 0.40)
        self.declare_parameter('localization_recover_s',      0.30)
        self.declare_parameter('localization_guard_min_start_s', 10.0)  # [LOC-GUARD] grace: tidak boleh aktif di 10 detik pertama
        self.declare_parameter('localization_guard_require_valid_once', True)
        self.declare_parameter('localization_consistency_guard_enabled', False)
        self.declare_parameter('localization_consistency_trans_threshold', 0.60)
        self.declare_parameter('localization_consistency_yaw_threshold_deg', 55.0)
        self.declare_parameter('goal_reached_stable_s', 1.0)
        # [DEMO-FINAL-STOP] Paksa berhenti bila sudah di posisi goal sekian detik
        # walau yaw belum sempurna (final-align tak diberi batas waktu -> bisa lama).
        # 0 = nonaktif (perilaku lama). Default 3 s utk demo.
        self.declare_parameter('final_align_timeout_s', 3.0)
        # [DEMO-FINAL-STOP] Jaring pengaman: bila sudah sekian detik di zona goal
        # tapi AMCL invalid (position_reached tak pernah latch), tetap paksa berhenti.
        self.declare_parameter('goal_zone_force_stop_s', 5.0)
        self.declare_parameter('scenario', 'convoy')
        self.declare_parameter('wait_peer_dwa_disable_in_crossing', True)
        self.declare_parameter('debug_telemetry_period_s', 0.2)
        self.declare_parameter('local_plan_publish_period_s', 0.2)

        # [MOD-20] Peer robot sebagai dynamic obstacle + local bypass
        self.declare_parameter('dynamic_robot_obstacle_enabled', True)
        self.declare_parameter('peer_pose_timeout_s',            1.0)
        self.declare_parameter('robot_obstacle_radius',          0.20)
        self.declare_parameter('robot_obstacle_margin',          0.15)
        self.declare_parameter('robot_obstacle_influence_radius', 0.80)
        self.declare_parameter('robot_path_blocking_radius',     0.35)
        self.declare_parameter('bypass_offset',                  0.45)
        self.declare_parameter('bypass_clear_distance',          0.70)
        self.declare_parameter('dynamic_obstacle_weight',        2.0)
        self.declare_parameter('dynamic_obstacle_debug_period_s', 0.2)
        self.declare_parameter('crossing_owner_ignore_peer_obstacle_enabled', True)
        self.declare_parameter('crossing_owner_ignore_hard_dist', 0.35)

        # [MOD-21] Footprint OBB ber-heading + prediksi gerakan peer dari cmd_vel UDP
        # Yahboom Mecanum RDK X5: panjang 0.236 m x lebar 0.181 m (bumper-ke-bumper).
        self.declare_parameter('footprint_length_m',     0.236)
        self.declare_parameter('footprint_width_m',      0.181)
        self.declare_parameter('footprint_margin_m',     0.10)   # clearance keselamatan ekstra (m) — diperlebar utk crossing (roda sempat nyenggol)
        self.declare_parameter('peer_predict_enabled',   True)
        self.declare_parameter('peer_predict_horizon_s', 1.5)    # batas atas ekstrapolasi peer (s)

        # [M1] Convoy peer-blocking threshold: hanya trigger _peer_blocks_path()
        # jika jarak ke peer < nilai ini. Set ke convoy_spacing * 0.8 untuk skenario
        # convoy agar convoy-mate di jarak normal tidak trigger HOLO_BLK.
        # 0.0 = disabled (perilaku lama, semua peer dalam influence_radius diperiksa).
        self.declare_parameter('peer_blocking_max_dist_m',       0.0)
        self.declare_parameter('convoy_same_direction_hard_block_dist', 0.30)
        self.declare_parameter('convoy_same_direction_heading_deg', 70.0)

        # [M5] EKF warmup: jangan aktifkan collision check sampai EKF mendapat
        # cukup update AMCL. Kovarians EKF besar di awal → r_safe besar → robot
        # mendeteksi tetangga di titik start sebagai ancaman sebelum bergerak.
        # 0 = disabled (perilaku lama, collision check aktif sejak awal).
        self.declare_parameter('ekf_warmup_steps',               0)

        self._last_locwarn = 0.0
        self.update_parameters()

        # ── State machine (bagian dasar) ────────────────────────────────
        self.STATE_ALIGNING       = 1
        self.STATE_TRACKING       = 2
        self.STATE_APPROACHING    = 3
        self.STATE_FINAL_ALIGNING = 4
        self.STATE_IDLE           = 5
        self.local_plan_size      = 20

        # ── State variabel (bagian dasar) ───────────────────────────────
        self.robot_pose            = [0.0, 0.0, 0.0]
        self.odom_pose             = None
        self.robot_vel             = [0.0, 0.0, 0.0]
        self.robot_covariance      = [0.0] * 36
        self.laser_data            = None
        self.current_state         = self.STATE_ALIGNING
        self.global_path           = []
        self.path_id               = 0
        self.current_path_index    = 0
        self.progress_along_segment = 0.0
        self.target_point          = None
        self.target_heading        = 0.0
        self.goal_orientation      = 0.0
        self.position_reached      = False
        self.final_goal_pose       = None
        self._goal_inside_since    = None
        self._position_reached_since = None  # [DEMO-FINAL-STOP] kapan posisi goal tercapai (utk timeout final-align)
        self._aligning_since       = None   # timestamp masuk ALIGNING, untuk timeout
        self._align_cooldown_until = 0.0   # blokir re-entry ke ALIGNING setelah timeout

        # ── [MOD-2] State v_max dari consensus ────────────────────────────
        self.vmax_from_consensus = None   # None = pakai max_vel_x default
        # [FF-CATCHUP] plafon kecepatan saat catch-up feedforward aktif
        self.feedforward_catchup_enabled = bool(
            self.get_parameter('feedforward_catchup_enabled').value)
        self.vmax_catchup_ceiling = float(
            self.get_parameter('vmax_catchup_ceiling').value)
        self.approach_speed_catchup = float(
            self.get_parameter('approach_speed_catchup').value)
        self.align_speed_catchup = float(
            self.get_parameter('align_speed_catchup').value)
        self.vmax_from_priority  = None
        self._ctrl_eff_vmax      = None   # computed tiap cycle di control_loop
        # [FIX-YAW] Floor kecepatan saat fase penyelarasan yaw akhir di tempat, agar
        # rotasi menghadap goal tidak digerus vmax=0 dari koordinasi (efek goal_reached).
        self._final_align_vmax_floor = 0.05   # m/s (rotasi tetap memakai max_rot_vel)

        # ── [MOD-6] State fault injection ─────────────────────────────────
        self.fault_active = False

        # ── [MOD-7] State stop command dari priority manager ───────────────
        self.priority_stop = False

        # ── [MOD-9] Hold sampai experiment_state == RUNNING ───────────────
        self.experiment_state = 'STOP'

        # ── [MOD-11] Reactive avoidance state ─────────────────────────────
        self.reactive_mode        = 'TRACKING'  # TRACKING/AVOIDING/BLOCKED/STUCK_ESCAPE
        self._current_sectors     = None
        self._remaining_length    = float('inf')
        self._escape_until        = 0.0
        self._escape_vy_sign      = 1
        self._stuck_last_remaining  = None
        self._stuck_last_check_time = None
        self._dyn_avoid_until        = 0.0
        self._dyn_avoid_cooldown_until = 0.0
        self._dyn_avoid_direction    = 1
        self._dyn_avoid_start_pos    = None
        self._dyn_avoid_clear_count  = 0
        self._dyn_avoid_reason       = ''

        # ── [MOD-12] Local costmap state ──────────────────────────────────
        self._local_costmap = None   # numpy uint8 array, rebuilt each TRACKING cycle
        self._costmap_stats = {      # logged in status_report
            'valid': 0, 'rejected': 0, 'best_cost': 0.0}

        # ── Debug velocity telemetry (untuk status_report) ─────────────────
        self._last_effective_vmax = 0.0   # vmax_eff setelah min(consensus, priority)
        self._last_debug_telemetry_pub = 0.0
        self._last_local_plan_pub = 0.0

        # ── [MOD-14] Corner slowdown + adaptive lookahead state ───────────
        self._last_corner_scale    = 1.0
        self._last_corner_angle    = 0.0
        self._last_lookahead_used  = 0.0   # lookahead aktual yang dipakai siklus ini

        # ── [MOD-15] Backtracking recovery state ──────────────────────────
        self._blocked_since             = None   # monotonic time when BLOCKED started
        self._backtrack_target          = None   # (x, y) target for backtracking
        self._backtrack_clear_since     = None   # monotonic time obstacle became clear
        self._last_no_valid_trajectory  = False  # True bila semua DWA kandidat ditolak

        # ── [MOD-16] Lateral suppression state ────────────────────────────
        self._last_vy_limited = False   # True bila vy dibatasi karena heading error besar

        # ── [MOD-17] Holonomic path tracker state ─────────────────────────
        self._last_cte                = 0.0
        self._last_path_heading_error = 0.0
        self._last_tracking_mode      = 'DWA'   # 'HOLO' | 'HOLO_BLK' | 'DWA'
        self._last_cmd_omega          = 0.0
        self._last_cmd_time           = None
        self._last_omega_raw          = 0.0
        self._last_omega_after_clamp  = 0.0
        self._last_motion_mix_guard   = False
        self._last_motion_mix_reason  = ''
        self._localization_invalid_since = None
        self._localization_valid_since   = None
        self._localization_hold_active   = False
        self._localization_seen_valid    = False
        self._experiment_running_since   = None
        self._amcl_consistency_base      = None
        self._odom_consistency_base      = None
        self._localization_consistency_hold = False
        # [FIX-LOC-A] AMCL update tracking
        self._amcl_pose_updated  = False   # True setiap pose_callback dipanggil
        self._last_amcl_recv_t   = None    # wall time penerimaan AMCL terbaru
        self._localization_hold_reason = ''  # [FIX-LOC-B] sub-reason untuk logging
        self._last_vx            = 0.0     # kecepatan terakhir diperintahkan
        self._last_vy            = 0.0
        self._last_consistency_warn      = 0.0

        # ── [LANE NEG] Lane negotiation lateral offset ─────────────────────
        self.crossing_lane_offset = 0.0

        # ── [MOD-18] Path-order-aware obstacle handling ────────────────────
        # Hanya obstacle DI DEPAN (relatif arah path) yang boleh trigger BLOCKED.
        # Robot di belakang tidak boleh membuat robot depan berhenti.
        self._last_front_blocker = False   # True bila BLOCKED dipicu oleh front obstacle
        self._last_front_dist    = float('inf')   # jarak front sector saat terakhir
        self._last_holo_blk_reason = ''

        # ── [MOD-20] Dynamic robot obstacle state ─────────────────────────
        self.peer_poses = {}  # peer_ns -> {'x','y','theta','stamp','dwa_active'}
        self._conflict_zone_detail = {'stamp': 0.0, 'zones': []}
        self._ekf_step_count = 0   # [M5] counter update AMCL sejak terakhir reset
        self._bypass_active = False
        self._bypass_peer = None
        self._bypass_point = None
        self._last_peer_blocks_path = False
        self._last_dyn_peer = ''
        self._dyn_rejected_count = 0
        self._dyn_min_distance = float('inf')
        self._last_convoy_follow_peer = ''
        self._last_peer_in_front_sector = False
        self._last_hold_reason = ''
        self._last_dynobs_debug_pub = 0.0


        # ═══════════════════════════════════════════════════════════════════
        # SUBSCRIBERS
        # ═══════════════════════════════════════════════════════════════════

        # Pose robot dari AMCL — topic via parameter amcl_pose_topic
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_topic,
            self.pose_callback, 10)

        self.create_subscription(
            Odometry,
            f'/{self.ns}/odom',
            self.odom_callback, 10)

        # Global path dari global_path_node — namespace dinamis
        self.create_subscription(
            Path,
            f'/{self.ns}/plan',
            self.path_callback, 10)

        # LaserScan — BEST_EFFORT agar tidak memblokir pipeline saat paket drop
        _sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            LaserScan,
            f'/{self.ns}/scan',
            self.scan_callback, _sensor_qos)

        # [MOD-2] v_max dari consensus_node
        self.create_subscription(
            Float32,
            f'/{self.ns}/vmax_consensus',
            self.vmax_callback, 10)

        # [MOD-6] Fault injection command
        self.create_subscription(
            Bool,
            f'/{self.ns}/fault_active',
            self.fault_callback, 10)

        # [MOD-8] v_max override dari priority_manager_node
        self.create_subscription(
            Float32,
            f'/{self.ns}/vmax_priority',
            self.vmax_priority_callback, 10)
        
        # [MOD-7] Stop command dari priority_manager
        self.create_subscription(
            Bool,
            f'/{self.ns}/priority_stop',
            self.priority_stop_callback, 10)

        # [MOD-9] Experiment state heartbeat — bergerak hanya saat RUNNING
        self.create_subscription(
            String, '/experiment_state',
            self._experiment_state_cb, 10)

        self.create_subscription(
            String, '/experiment_scenario',
            self._experiment_scenario_cb, 10)

        self.create_subscription(
            String,
            f'/{self.ns}/peer_robot_poses',
            self.peer_poses_callback, 10)

        self.create_subscription(
            String,
            '/conflict_zone_detail',
            self.conflict_zone_detail_callback, 10)

        # [MOD-11] Remaining length — untuk stuck detector
        self.create_subscription(
            Float32,
            f'/{self.ns}/remaining_length',
            self._remaining_length_cb, 10)

        # [LANE NEG] Lateral offset dari priority_manager (head-on negotiation)
        self.create_subscription(
            Float32,
            f'/{self.ns}/crossing_lane_offset',
            self.lane_offset_callback, 10)

        # ═══════════════════════════════════════════════════════════════════
        # PUBLISHERS
        # ═══════════════════════════════════════════════════════════════════

        # namespace dinamis
        self.cmd_pub        = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path,  f'/{self.ns}/local_plan', 10)

        # [MOD-13] Debug telemetry untuk sync_monitor_node
        self._pub_vmax_eff  = self.create_publisher(Float32, f'/{self.ns}/dwa_vmax_eff',  10)
        self._pub_speed_mag = self.create_publisher(Float32, f'/{self.ns}/dwa_speed_mag', 10)
        self._pub_dwa_mode  = self.create_publisher(String,  f'/{self.ns}/dwa_mode',      10)
        self._pub_omega_raw = self.create_publisher(Float32, f'/{self.ns}/omega_raw',      10)
        self._pub_omega_clamped = self.create_publisher(
            Float32, f'/{self.ns}/omega_after_clamp', 10)
        self._pub_omega_limit = self.create_publisher(
            Float32, f'/{self.ns}/omega_global_limit', 10)
        self._pub_loc_hold = self.create_publisher(
            Bool, f'/{self.ns}/localization_hold_active', 10)

        # [MOD-17] Holonomic path tracker telemetry
        self._pub_cte   = self.create_publisher(Float32, f'/{self.ns}/crosstrack_error', 10)
        self._pub_herr  = self.create_publisher(Float32, f'/{self.ns}/heading_error',    10)
        self._pub_tmode = self.create_publisher(String,  f'/{self.ns}/tracking_mode',    10)
        self._pub_dynobs = self.create_publisher(
            String, f'/{self.ns}/dynamic_obstacle_debug', 10)

        # [FIX-6] Heartbeat — udp_sender/experiment_cli deteksi DWA alive
        self._pub_dwa_alive = self.create_publisher(Bool, f'/{self.ns}/dwa_alive', 10)
        # [FIX-ARRIVE-PUB] Status SAMPAI-posisi (dikonsumsi experiment_logger & experiment_master_cli NEW)
        self._pub_pos_reached = self.create_publisher(Bool, f'/{self.ns}/position_reached', 10)

        # [MOD-12] Local costmap RViz visualization (TRANSIENT_LOCAL agar RViz connect)
        _costmap_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self._local_costmap_pub = self.create_publisher(
            OccupancyGrid, f'/{self.ns}/local_costmap', _costmap_qos)
        # Publish sekali saat startup supaya RViz langsung bisa subscribe
        self._empty_costmap_timer = self.create_timer(0.5, self._publish_empty_local_costmap_once)

        # ═══════════════════════════════════════════════════════════════════
        # TIMERS
        # ═══════════════════════════════════════════════════════════════════

        self.create_timer(0.1, self.control_loop)          # 10 Hz
        self.create_timer(1.0, self.status_report)
        self.create_timer(2.0, self.update_parameters)
        self.create_timer(1.0, self._republish_local_costmap)  # 1 Hz — RViz keepalive
        self.create_timer(1.0, self._publish_heartbeat)    # [FIX-6] DWA alive heartbeat

        self.get_logger().info(f'Modified DWA ready — /{self.ns}')

    # ═══════════════════════════════════════════════════════════════════════
    # PARAMETER UPDATE — bagian dasar + tambahan parameter baru
    # ═══════════════════════════════════════════════════════════════════════

    def update_parameters(self):
        """Refresh parameter runtime."""
        self.max_vel_x                   = self.get_parameter('max_vel_x').value
        self.feedforward_catchup_enabled = bool(self.get_parameter('feedforward_catchup_enabled').value)
        self.vmax_catchup_ceiling        = float(self.get_parameter('vmax_catchup_ceiling').value)
        self.approach_speed_catchup      = float(self.get_parameter('approach_speed_catchup').value)
        self.align_speed_catchup         = float(self.get_parameter('align_speed_catchup').value)
        self.max_vel_y                   = self.get_parameter('max_vel_y').value
        self.max_rot_vel                 = self.get_parameter('max_rot_vel').value
        self.min_vel_x                   = self.get_parameter('min_vel_x').value
        self.min_vel_y                   = self.get_parameter('min_vel_y').value
        self.lookahead_distance          = self.get_parameter('lookahead_distance').value
        self.goal_tolerance              = self.get_parameter('goal_tolerance').value
        self.path_tracking_tolerance     = self.get_parameter('path_tracking_tolerance').value
        self.heading_alignment_tolerance = self.get_parameter('heading_alignment_tolerance').value
        self.min_heading_error_for_sideways = self.get_parameter('min_heading_error_for_sideways').value
        self.max_sideways_ratio          = self.get_parameter('max_sideways_ratio').value
        self.final_orientation_weight    = self.get_parameter('final_orientation_weight').value
        self.position_first              = self.get_parameter('position_first').value
        self.prediction_time             = self.get_parameter('prediction_time').value
        self.dt                          = self.get_parameter('dt').value
        self.vx_samples                  = int(self.get_parameter('vx_samples').value)
        self.vy_samples                  = int(self.get_parameter('vy_samples').value)
        self.w_samples                   = int(self.get_parameter('w_samples').value)
        # ── [ALGO-TRACE] state perekaman proses DWA ───────────────────────
        self.algo_trace_enabled = bool(self.get_parameter('algo_trace_enabled').value)
        self.algo_trace_dir = str(self.get_parameter('algo_trace_dir').value)
        self.algo_trace_dwa_period_s = float(self.get_parameter('algo_trace_dwa_period_s').value)
        self._dwa_trace_buf = []
        self._dwa_trace_seq = 0
        self._last_dwa_trace_t = 0.0
        self._local_plan_trace_seq = 0
        if self.algo_trace_enabled:
            try:
                os.makedirs(self.algo_trace_dir, exist_ok=True)
                self.get_logger().info(f'[ALGO-TRACE] DWA aktif → {self.algo_trace_dir}')
            except Exception as e:
                self.get_logger().warn(f'[ALGO-TRACE] gagal buat dir: {e}')
        self.r_safe_base       = self.get_parameter('r_safe_base').value
        self.k_sigma           = self.get_parameter('k_sigma').value
        self.r_safe_min        = self.get_parameter('r_safe_min').value
        self.r_safe_max        = self.get_parameter('r_safe_max').value
        self.max_pose_variance = self.get_parameter('max_pose_variance').value
        # [MOD-11]
        self.enable_reactive_avoidance  = self.get_parameter('enable_reactive_avoidance').value
        self.front_warning_dist         = self.get_parameter('front_warning_dist').value
        self.front_stop_dist            = self.get_parameter('front_stop_dist').value
        self.side_clearance_min         = self.get_parameter('side_clearance_min').value
        self.avoid_vx_max               = self.get_parameter('avoid_vx_max').value
        self.avoid_vy_speed             = self.get_parameter('avoid_vy_speed').value
        self.avoid_w_max                = self.get_parameter('avoid_w_max').value
        self.obstacle_clearance_weight  = self.get_parameter('obstacle_clearance_weight').value
        self.avoid_lateral_weight       = self.get_parameter('avoid_lateral_weight').value
        self.stuck_timeout              = self.get_parameter('stuck_timeout').value
        self.stuck_min_progress         = self.get_parameter('stuck_min_progress').value
        self.enable_stuck_detector      = self.get_parameter('enable_stuck_detector').value
        self.dynamic_side_avoid_enabled = bool(
            self.get_parameter('dynamic_side_avoid_enabled').value)
        self.dynamic_side_avoid_speed = float(
            self.get_parameter('dynamic_side_avoid_speed').value)
        self.dynamic_side_avoid_vy = float(
            self.get_parameter('dynamic_side_avoid_vy').value)
        self.dynamic_side_avoid_duration = float(
            self.get_parameter('dynamic_side_avoid_duration').value)
        self.dynamic_side_return_dist = float(
            self.get_parameter('dynamic_side_return_dist').value)
        self.dynamic_side_min_shift = float(
            self.get_parameter('dynamic_side_min_shift').value)
        self.dynamic_side_clear_confirm = max(1, int(
            self.get_parameter('dynamic_side_clear_confirm').value))
        self.dynamic_side_cooldown = float(
            self.get_parameter('dynamic_side_cooldown').value)
        self.hard_peer_escape_enabled = bool(
            self.get_parameter('hard_peer_escape_enabled').value)
        self.hard_peer_escape_distance = float(
            self.get_parameter('hard_peer_escape_distance').value)
        self.hard_peer_escape_speed = float(
            self.get_parameter('hard_peer_escape_speed').value)
        self.hard_peer_escape_side_clearance = float(
            self.get_parameter('hard_peer_escape_side_clearance').value)
        # [MOD-12]
        self.avoidance_mode           = self.get_parameter('avoidance_mode').value
        self.local_costmap_size       = self.get_parameter('local_costmap_size').value
        self.local_costmap_resolution = self.get_parameter('local_costmap_resolution').value
        self.obstacle_min_range       = self.get_parameter('obstacle_min_range').value
        self.obstacle_max_range       = self.get_parameter('obstacle_max_range').value
        self.inflation_radius         = self.get_parameter('inflation_radius').value
        self.lethal_cost              = int(self.get_parameter('lethal_cost').value)
        self.lethal_threshold         = int(self.get_parameter('lethal_threshold').value)
        self.local_cost_weight        = self.get_parameter('local_cost_weight').value
        self.cost_max_weight          = self.get_parameter('cost_max_weight').value
        self.cost_avg_weight          = self.get_parameter('cost_avg_weight').value
        # [MOD-14]
        self.corner_slowdown_enabled    = self.get_parameter('corner_slowdown_enabled').value
        self.corner_angle_threshold     = math.radians(
            self.get_parameter('corner_angle_threshold_deg').value)
        self.corner_slowdown_radius     = self.get_parameter('corner_slowdown_radius').value
        self.corner_speed_ratio         = self.get_parameter('corner_speed_ratio').value
        self.corner_lookahead_distance  = self.get_parameter('corner_lookahead_distance').value
        # [MOD-15]
        self.enable_backtracking_recovery = self.get_parameter('enable_backtracking_recovery').value
        self.blocked_timeout            = self.get_parameter('blocked_timeout').value
        self.backtrack_distance         = self.get_parameter('backtrack_distance').value
        self.clear_time                 = self.get_parameter('clear_time').value
        self.backtracking_speed         = self.get_parameter('backtracking_speed').value
        # [MOD-16]
        self.heading_lateral_suppress_enabled = self.get_parameter(
            'heading_lateral_suppress_enabled').value
        self.heading_lateral_threshold  = math.radians(
            self.get_parameter('heading_lateral_threshold_deg').value)
        self.heading_lateral_max_vy     = self.get_parameter('heading_lateral_max_vy').value
        # [MOD-17]
        self.use_holonomic_path_tracker = self.get_parameter('use_holonomic_path_tracker').value
        self.k_cte                      = self.get_parameter('k_cte').value
        self.k_heading                  = self.get_parameter('k_heading').value
        self.max_lateral_correction     = self.get_parameter('max_lateral_correction').value
        self.max_heading_w              = self.get_parameter('max_heading_w').value
        self.path_heading_tracking_enabled = self.get_parameter(
            'path_heading_tracking_enabled').value
        self.heading_translation_gate_enabled = self.get_parameter(
            'heading_translation_gate_enabled').value
        self.heading_translation_gate = math.radians(
            self.get_parameter('heading_translation_gate_deg').value)
        self.heading_translation_gate_scale = self.get_parameter(
            'heading_translation_gate_scale').value
        self.angular_translation_lock_enabled = self.get_parameter(
            'angular_translation_lock_enabled').value
        self.angular_translation_lock_w = self.get_parameter(
            'angular_translation_lock_w').value
        self.angular_translation_lock_scale = self.get_parameter(
            'angular_translation_lock_scale').value
        self.final_goal_vector_radius = self.get_parameter(
            'final_goal_vector_radius').value
        self.final_goal_vector_kp = self.get_parameter(
            'final_goal_vector_kp').value
        self.omega_global_limit         = self.get_parameter('omega_global_limit').value
        self.omega_slew_rate_limit      = self.get_parameter('omega_slew_rate_limit').value
        self.motion_mixing_guard_enabled = self.get_parameter(
            'motion_mixing_guard_enabled').value
        self.mixing_omega_vy_zero_threshold = self.get_parameter(
            'mixing_omega_vy_zero_threshold').value
        self.mixing_heading_error = math.radians(
            self.get_parameter('mixing_heading_error_deg').value)
        self.mixing_heading_vx_scale = self.get_parameter(
            'mixing_heading_vx_scale').value
        self.mixing_corner_vx_scale = self.get_parameter(
            'mixing_corner_vx_scale').value
        self.mixing_corner_omega_limit = self.get_parameter(
            'mixing_corner_omega_limit').value
        self.mixing_corner_vy_zero = self.get_parameter(
            'mixing_corner_vy_zero').value
        self.localization_guard_enabled = self.get_parameter(
            'localization_guard_enabled').value
        self.localization_sigma_threshold = self.get_parameter(
            'localization_sigma_threshold').value
        self.localization_hard_sigma_threshold = self.get_parameter(
            'localization_hard_sigma_threshold').value
        self.localization_invalid_hold_s = self.get_parameter(
            'localization_invalid_hold_s').value
        self.localization_recover_s = self.get_parameter(
            'localization_recover_s').value
        self.localization_guard_min_start_s = self.get_parameter(
            'localization_guard_min_start_s').value
        self.localization_guard_require_valid_once = self.get_parameter(
            'localization_guard_require_valid_once').value
        self.localization_consistency_guard_enabled = self.get_parameter(
            'localization_consistency_guard_enabled').value
        self.localization_consistency_trans_threshold = self.get_parameter(
            'localization_consistency_trans_threshold').value
        self.localization_consistency_yaw_threshold = math.radians(
            self.get_parameter('localization_consistency_yaw_threshold_deg').value)
        self.goal_reached_stable_s = self.get_parameter(
            'goal_reached_stable_s').value
        self.final_align_timeout_s = float(self.get_parameter(
            'final_align_timeout_s').value)   # [DEMO-FINAL-STOP]
        self.goal_zone_force_stop_s = float(self.get_parameter(
            'goal_zone_force_stop_s').value)   # [DEMO-FINAL-STOP]
        self.scenario = self.get_parameter('scenario').value
        self.wait_peer_dwa_disable_in_crossing = bool(
            self.get_parameter('wait_peer_dwa_disable_in_crossing').value)
        self.debug_telemetry_period_s = max(0.1, float(
            self.get_parameter('debug_telemetry_period_s').value))
        self.local_plan_publish_period_s = max(0.1, float(
            self.get_parameter('local_plan_publish_period_s').value))
        # [MOD-20]
        self.dynamic_robot_obstacle_enabled = self.get_parameter(
            'dynamic_robot_obstacle_enabled').value
        self.peer_pose_timeout_s = self.get_parameter('peer_pose_timeout_s').value
        self.robot_obstacle_radius = self.get_parameter('robot_obstacle_radius').value
        self.robot_obstacle_margin = self.get_parameter('robot_obstacle_margin').value
        self.robot_obstacle_influence_radius = self.get_parameter(
            'robot_obstacle_influence_radius').value
        self.robot_path_blocking_radius = self.get_parameter(
            'robot_path_blocking_radius').value
        self.bypass_offset = self.get_parameter('bypass_offset').value
        self.bypass_clear_distance = self.get_parameter('bypass_clear_distance').value
        # [MOD-21] footprint OBB + prediksi peer
        self.footprint_length_m = float(self.get_parameter('footprint_length_m').value)
        self.footprint_width_m  = float(self.get_parameter('footprint_width_m').value)
        self.footprint_margin_m = float(self.get_parameter('footprint_margin_m').value)
        self.peer_predict_enabled = bool(self.get_parameter('peer_predict_enabled').value)
        self.peer_predict_horizon_s = float(self.get_parameter('peer_predict_horizon_s').value)
        self.dynamic_obstacle_weight = self.get_parameter(
            'dynamic_obstacle_weight').value
        self.dynamic_obstacle_debug_period_s = max(0.1, float(
            self.get_parameter('dynamic_obstacle_debug_period_s').value))
        self.crossing_owner_ignore_peer_obstacle_enabled = bool(
            self.get_parameter('crossing_owner_ignore_peer_obstacle_enabled').value)
        self.crossing_owner_ignore_hard_dist = max(0.25, float(
            self.get_parameter('crossing_owner_ignore_hard_dist').value))
        # [M1] Convoy peer-blocking distance threshold
        self.peer_blocking_max_dist_m = self.get_parameter(
            'peer_blocking_max_dist_m').value
        self.convoy_same_direction_hard_block_dist = float(
            self.get_parameter('convoy_same_direction_hard_block_dist').value)
        self.convoy_same_direction_heading = math.radians(float(
            self.get_parameter('convoy_same_direction_heading_deg').value))
        # [M5] EKF warmup step threshold
        self.ekf_warmup_steps = int(self.get_parameter('ekf_warmup_steps').value)

    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def pose_callback(self, msg):
        """ [FIX-LOC-A] catat waktu update AMCL."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_pose       = [x, y, theta]
        self.robot_covariance = list(msg.pose.covariance)
        # [FIX-LOC-A] Flag & timestamp untuk consistency guard sliding-window
        self._amcl_pose_updated = True
        self._last_amcl_recv_t  = time.time()
        self._ekf_step_count   += 1   # [M5] setiap pose AMCL masuk = satu EKF step

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = [p.x, p.y, theta]

    def path_callback(self, msg):
        """bagian dasar"""
        new_path = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            q = pose.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            new_path.append([x, y, theta])

        if len(new_path) < 2:
            return

        new_goal = new_path[-1]
        goal_changed = (
            self.final_goal_pose is None or
            math.hypot(new_goal[0] - self.final_goal_pose[0],
                       new_goal[1] - self.final_goal_pose[1]) > 0.1
        )

        self.get_logger().info(f'New path: {len(new_path)} points')
        self.global_path  = new_path
        self.path_id     += 1

        self.final_goal_pose  = new_goal
        self.goal_orientation = new_goal[2]
        self.position_reached = False
        self._goal_inside_since = None

        if goal_changed:
            # Goal baru — reset penuh ke awal path
            self.current_path_index     = 0
            self.progress_along_segment = 0.0
            # Langsung TRACKING: mecanum tidak perlu align dulu, determine_state()
            # akan memilih TRACKING atau APPROACHING sesuai jarak ke goal.
            self.current_state          = self.STATE_TRACKING
            self._aligning_since        = None
            self.get_logger().info('Goal baru — reset path tracking')
        # Kalau goal sama (path di-republish ulang), pertahankan progress

        self.find_current_position_on_path()

    def scan_callback(self, msg):
        """bagian dasar"""
        self.laser_data = msg
        self._last_hold_reason = ''

    # ── Callbacks baru ─────────────────────────────────────────���────────

    def vmax_callback(self, msg):
        """[MOD-2] Terima v_max dari consensus_node (Layer 4).

        [FF-CATCHUP] Bila feedforward catch-up aktif, nilai v_max dari konsensus
        (yang sudah berisi boost v_nom + k*(p_bar - p_i)) diteruskan hingga
        vmax_catchup_ceiling -- BUKAN dipotong ke max_vel_x -- agar robot
        tertinggal benar-benar bisa mempercepat. Default: clamp ke max_vel_x.
        """
        v = float(msg.data)
        cap = self.vmax_catchup_ceiling if self.feedforward_catchup_enabled else self.max_vel_x
        self.vmax_from_consensus = max(0.0, min(v, cap))

    def vmax_priority_callback(self, msg):
        """[MOD-8] Terima v_max override dari priority_manager_node (Layer 5)"""
        v = float(msg.data)
        self.vmax_from_priority = max(0.0, min(v, self.max_vel_x))

    def _abs_vmax_cap(self):
        """[FF-CATCHUP] Plafon kecepatan absolut: max_vel_x pada baseline, atau
        vmax_catchup_ceiling saat feedforward catch-up aktif."""
        if getattr(self, 'feedforward_catchup_enabled', False):
            return max(self.max_vel_x,
                       float(getattr(self, 'vmax_catchup_ceiling', self.max_vel_x)))
        return self.max_vel_x

    def _coordination_v_limit(self):
        abs_cap = self._abs_vmax_cap()
        return min(
            self.vmax_from_consensus
            if self.vmax_from_consensus is not None else abs_cap,
            self.vmax_from_priority
            if self.vmax_from_priority is not None else abs_cap,
            abs_cap,
        )

    def fault_callback(self, msg):
        """[MOD-6] Terima status fault dari fault_injector_node"""
        prev = self.fault_active
        self.fault_active = bool(msg.data)
        if self.fault_active != prev:
            state = 'AKTIF' if self.fault_active else 'SELESAI'
            self.get_logger().warn(f'[FAULT] Fault injection {state} pada /{self.ns}')

    def priority_stop_callback(self, msg):
        """[MOD-7] Terima perintah stop dari priority_manager (Layer 5)"""
        prev = self.priority_stop
        self.priority_stop = bool(msg.data)
        if self.priority_stop != prev:
            state = 'STOP' if self.priority_stop else 'RESUME'
            self.get_logger().info(f'[PRIORITY] {state} command dari priority_manager')
            # [MOD-19] Saat resume: kalau DWA sudah position_reached tapi posisi
            # belum benar-benar di goal, reset ke APPROACHING agar mengejar goal
            # dengan approach_control (100ms) bukan final_alignment (30ms).
            if not self.priority_stop and self.position_reached and self.final_goal_pose:
                try:
                    rx, ry, _ = self.robot_pose
                    gx, gy, _ = self.final_goal_pose
                    pos_err = math.hypot(gx - rx, gy - ry)
                    if pos_err > self.goal_tolerance:
                        self.position_reached = False
                        self.current_state    = self.STATE_APPROACHING
                        self.get_logger().info(
                            f'[MOD-19] priority resume: pos_err={pos_err:.3f}m '
                            f'> tol={self.goal_tolerance:.3f}m — reset ke APPROACHING')
                except Exception:
                    pass

    def _experiment_state_cb(self, msg):
        """[MOD-9] Terima heartbeat /experiment_state — bergerak hanya saat RUNNING."""
        prev = self.experiment_state
        self.experiment_state = msg.data
        if self.experiment_state != prev:
            self.get_logger().info(
                f'[STATE] {prev} → {self.experiment_state}')
            if self.experiment_state == 'RUNNING':
                self._experiment_running_since = time.time()
                self._localization_invalid_since = None
                self._localization_valid_since = None
                self._localization_hold_active = False
                self._localization_seen_valid = False
                self._amcl_consistency_base = None
                self._odom_consistency_base = None
                self._localization_consistency_hold = False
                self._amcl_pose_updated = False        # [FIX-LOC-A]
                self._localization_hold_reason = ''    # [FIX-LOC-B]
                self._goal_inside_since = None
                self._ekf_step_count = 0               # [M5] reset warmup counter saat mulai
        if msg.data in ('STOP', 'READY'):
            self._stuck_last_remaining  = None
            self._stuck_last_check_time = None
            self._escape_until          = 0.0
            self._dyn_avoid_until       = 0.0
            self._dyn_avoid_start_pos   = None
            self._dyn_avoid_clear_count = 0
            self._dyn_avoid_reason      = ''
            self._experiment_running_since = None
            self._localization_invalid_since = None
            self._localization_valid_since = None
            self._localization_hold_active = False
            self._localization_seen_valid = False
            self._amcl_consistency_base = None
            self._odom_consistency_base = None
            self._localization_consistency_hold = False
            self._amcl_pose_updated = False            # [FIX-LOC-A]
            self._localization_hold_reason = ''        # [FIX-LOC-B]
            self._goal_inside_since = None

    def _experiment_scenario_cb(self, msg):
        """Terima scenario aktif dari experiment_master_cli via UDP bridge."""
        scenario = str(msg.data).strip()
        if not scenario or scenario == self.scenario:
            return
        prev = self.scenario
        self.scenario = scenario
        self.set_parameters([Parameter('scenario', Parameter.Type.STRING, scenario)])
        self._dyn_avoid_until = 0.0
        self._dyn_avoid_start_pos = None
        self._dyn_avoid_clear_count = 0
        self._dyn_avoid_reason = ''
        self.get_logger().info(f'[SCENARIO] {prev} → {self.scenario}')

    def _remaining_length_cb(self, msg):
        """[MOD-11] Terima sisa panjang path — untuk stuck detector."""
        self._remaining_length = float(msg.data)

    def lane_offset_callback(self, msg):
        """[LANE NEG] Terima lateral offset dari priority_manager (head-on negotiation)."""
        self.crossing_lane_offset = float(msg.data)

    def peer_poses_callback(self, msg):
        """[MOD-20] Terima pose robot lain dari PC master via UDP receiver.
        [FIX-4] Hormati flag 'fresh'/'age_s' dari PC master berdasarkan umur AMCL sebenarnya.
        """
        try:
            peers = json.loads(msg.data)
        except Exception:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        for peer in peers:
            name = str(peer.get('robot', ''))
            if not name or name == self.ns:
                continue
            age_s    = float(peer.get('age_s', 0.0))
            is_fresh = bool(peer.get('fresh', True))
            if not is_fresh:
                self.get_logger().warn(
                    f'[DWA][{self.ns}] Peer {name} STALE dari PC '
                    f'(amcl_age={age_s:.2f}s) — diabaikan',
                    throttle_duration_sec=3.0)
                continue
            self.peer_poses[name] = {
                'x': float(peer.get('x', 0.0)),
                'y': float(peer.get('y', 0.0)),
                'theta': float(peer.get('theta', 0.0)),
                # [MOD-21] kecepatan badan peer utk prediksi gerakan + uji OBB
                'vx': float(peer.get('vx', 0.0)),
                'vy': float(peer.get('vy', 0.0)),
                'w':  float(peer.get('w', 0.0)),
                'stamp': now,
                'amcl_age_s': age_s,
                'dwa_active': bool(peer.get('dwa_active', False)),  # [M3]
            }

    def conflict_zone_detail_callback(self, msg):
        """Terima owner/yield state dari priority_manager untuk crossing."""
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self._conflict_zone_detail = {
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'zones': payload.get('zones', []),
        }

    # ═══════════════════════════════════════════════════════════════════════
    # [MOD-11] REACTIVE LIDAR AVOIDANCE METHODS
    # ═══════════════════════════════════════════════════════════════════════

    def analyze_lidar_sectors(self):
        """Analisis scan menjadi 5 sektor arah. Return dict min-distance per sektor."""
        inf = float('inf')
        empty = {'front': inf, 'front_left': inf, 'front_right': inf,
                 'left': inf, 'right': inf}
        if not self.laser_data:
            return empty

        ranges    = self.laser_data.ranges
        a_min     = self.laser_data.angle_min
        a_inc     = self.laser_data.angle_increment
        r_min     = self.laser_data.range_min
        r_max     = self.laser_data.range_max

        # Batas sektor dalam radian (robot frame)
        SECTORS = {
            'front':       (-0.436,  0.436),   # ±25°
            'front_left':  ( 0.436,  1.309),   # 25°–75°
            'front_right': (-1.309, -0.436),   # -75°–-25°
            'left':        ( 1.309,  2.094),   # 75°–120°
            'right':       (-2.094, -1.309),   # -120°–-75°
        }
        result = {k: inf for k in SECTORS}

        for i, r in enumerate(ranges):
            if not (r_min < r < r_max):
                continue
            angle = self.normalize_angle(a_min + i * a_inc)
            for name, (lo, hi) in SECTORS.items():
                if lo <= angle <= hi:
                    if r < result[name]:
                        result[name] = r

        return result

    def get_reactive_mode(self, sectors):
        """Tentukan mode reaktif dari sektor depan."""
        if not self.enable_reactive_avoidance:
            return 'TRACKING'
        front = sectors.get('front', float('inf'))
        if front < self.front_stop_dist:
            return 'BLOCKED'
        if front < self.front_warning_dist:
            return 'AVOIDING'
        return 'TRACKING'

    def _choose_free_side(self, sectors):
        """Return +1 (kiri/vy+) atau -1 (kanan/vy-) sesuai sisi yang lebih kosong."""
        fl = sectors.get('front_left',  float('inf'))
        fr = sectors.get('front_right', float('inf'))
        return 1 if fl >= fr else -1

    def _point_clearance(self, x, y):
        """
        Clearance (m) antara titik dunia (x,y) dan obstacle LiDAR terdekat
        di arah titik tersebut dari robot. Nilai positif = ada ruang, negatif = sudah lewat.
        """
        if not self.laser_data:
            return float('inf')
        robot_x, robot_y, robot_theta = self.robot_pose
        dx, dy     = x - robot_x, y - robot_y
        point_dist = math.hypot(dx, dy)
        if point_dist < 0.01:
            return float('inf')
        cos_t = math.cos(-robot_theta)
        sin_t = math.sin(-robot_theta)
        lx    = dx * cos_t - dy * sin_t
        ly    = dx * sin_t + dy * cos_t
        angle = math.atan2(ly, lx)
        beam  = int((angle - self.laser_data.angle_min) / self.laser_data.angle_increment)
        min_r = float('inf')
        for bi in range(beam - 2, beam + 3):
            if 0 <= bi < len(self.laser_data.ranges):
                r = self.laser_data.ranges[bi]
                if self.laser_data.range_min < r < self.laser_data.range_max:
                    if r < min_r:
                        min_r = r
        return float('inf') if min_r == float('inf') else (min_r - point_dist)

    def safe_fallback_cmd(self, sectors):
        """Gerak lateral minimal saat semua trajektori DWA ditolak (BLOCKED)."""
        vy_speed = self._lateral_recovery_speed()
        l_space = min(sectors.get('left', float('inf')),
                      sectors.get('front_left', float('inf')))
        r_space = min(sectors.get('right', float('inf')),
                      sectors.get('front_right', float('inf')))
        if l_space > self.side_clearance_min:
            self._last_tracking_mode = 'LAT_ESCAPE'
            return 0.0,  vy_speed, 0.0
        if r_space > self.side_clearance_min:
            self._last_tracking_mode = 'LAT_ESCAPE'
            return 0.0, -vy_speed, 0.0
        return 0.0, 0.0, 0.0   # benar-benar terkunci — berhenti

    def _lateral_recovery_speed(self):
        return min(self.max_vel_y, max(self.avoid_vy_speed,
                                       min(self.max_vel_y, self.dynamic_side_avoid_vy)))

    def _forward_space_clear(self, clearance=None, width_rad=math.radians(35)):
        if clearance is None:
            clearance = self.dynamic_side_return_dist
        if not self.laser_data:
            return False
        angle_min = self.laser_data.angle_min
        angle_inc = self.laser_data.angle_increment
        for i, d in enumerate(self.laser_data.ranges):
            if not (self.laser_data.range_min < d < self.laser_data.range_max):
                continue
            if abs(angle_min + i * angle_inc) <= width_rad and d < clearance:
                return False
        return True

    def _side_space(self, sectors, direction):
        if direction > 0:
            return min(sectors.get('left', float('inf')),
                       sectors.get('front_left', float('inf')))
        return min(sectors.get('right', float('inf')),
                   sectors.get('front_right', float('inf')))

    def _dynamic_side_available(self, sectors, direction):
        return self._side_space(sectors, direction) > self.side_clearance_min

    def _hard_peer_escape_cmd(self, sectors=None, max_dist=None):
        if (not self.hard_peer_escape_enabled
                or self.scenario != 'crossing'
                or self.robot_pose is None):
            return None

        peers = self._fresh_peer_poses()
        if not peers:
            return None

        rx, ry, rth = self.robot_pose
        limit = self.hard_peer_escape_distance if max_dist is None else max_dist
        closest_ns = ''
        closest_peer = None
        closest_dist = float('inf')
        for peer_ns, peer in peers.items():
            dist = math.hypot(peer['x'] - rx, peer['y'] - ry)
            if dist < closest_dist:
                closest_ns = peer_ns
                closest_peer = peer
                closest_dist = dist

        if closest_peer is None or closest_dist > limit:
            return None

        if sectors is None:
            sectors = self._current_sectors or self.analyze_lidar_sectors()

        dx = closest_peer['x'] - rx
        dy = closest_peer['y'] - ry
        peer_lat = -math.sin(rth) * dx + math.cos(rth) * dy
        if abs(peer_lat) > 0.03:
            direction = -1 if peer_lat > 0.0 else 1
        else:
            direction = self._choose_free_side(sectors)

        other = -direction
        min_clear = max(0.08, self.hard_peer_escape_side_clearance)
        side_space = self._side_space(sectors, direction)
        other_space = self._side_space(sectors, other)
        if side_space <= min_clear and other_space > side_space:
            direction = other
            side_space = other_space
        if side_space <= min_clear:
            return None

        vy = direction * min(self.max_vel_y, self.hard_peer_escape_speed)
        self._last_tracking_mode = 'PEER_ESCAPE'
        self._last_holo_blk_reason = f'PEER_ESCAPE_{closest_ns}'
        self.get_logger().warn(
            f'[DWA][{self.ns}] PEER_ESCAPE from {closest_ns} '
            f'd={closest_dist:.3f}m vy={vy:.2f}',
            throttle_duration_sec=0.5)
        return 0.0, vy, 0.0

    def _start_dynamic_side_avoid(self, reason, sectors):
        if not self.dynamic_side_avoid_enabled:
            return False
        if (self.scenario == 'crossing'
                and self._crossing_zone_robot_cmd(self.ns) in ('HOLD',)):
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self._dyn_avoid_cooldown_until:
            return False
        direction = self._choose_free_side(sectors)
        if not self._dynamic_side_available(sectors, direction):
            other = -direction
            if not self._dynamic_side_available(sectors, other):
                return False
            direction = other
        self.reactive_mode = 'DYN_AVOID'
        self._dyn_avoid_until = now + self.dynamic_side_avoid_duration
        self._dyn_avoid_direction = direction
        self._dyn_avoid_start_pos = (self.robot_pose[0], self.robot_pose[1])
        self._dyn_avoid_clear_count = 0
        self._dyn_avoid_reason = reason
        self.get_logger().warn(
            f'[DWA][{self.ns}] DYN_AVOID start reason={reason} '
            f'dir={"left" if direction > 0 else "right"}',
            throttle_duration_sec=0.5)
        return True

    def _dynamic_side_avoid_control(self, effective_vmax=None):
        now = self.get_clock().now().nanoseconds / 1e9
        sectors = self._current_sectors or self.analyze_lidar_sectors()
        if self.scenario == 'crossing':
            my_cmd = self._crossing_zone_robot_cmd(self.ns)
            if my_cmd in ('HOLD',):
                self.reactive_mode = 'BLOCKED'
                self._dyn_avoid_cooldown_until = now + self.dynamic_side_cooldown
                self._dyn_avoid_start_pos = None
                self._dyn_avoid_clear_count = 0
                self._last_tracking_mode = 'WAIT_CROSSING_YIELD'
                return 0.0, 0.0, 0.0
            yield_side_step = my_cmd in ('YIELD', 'SLOW')
        else:
            yield_side_step = False
        rx, ry, _ = self.robot_pose
        moved = 0.0
        if self._dyn_avoid_start_pos is not None:
            moved = math.hypot(rx - self._dyn_avoid_start_pos[0],
                               ry - self._dyn_avoid_start_pos[1])

        front_clear = self._forward_space_clear()
        if front_clear and moved >= self.dynamic_side_min_shift:
            self._dyn_avoid_clear_count += 1
        else:
            self._dyn_avoid_clear_count = 0

        if self._dyn_avoid_clear_count >= self.dynamic_side_clear_confirm:
            self.reactive_mode = 'TRACKING'
            self._dyn_avoid_cooldown_until = now + self.dynamic_side_cooldown
            self._dyn_avoid_start_pos = None
            self._dyn_avoid_clear_count = 0
            self.find_current_position_on_path()
            self.get_logger().info(
                f'[DWA][{self.ns}] DYN_AVOID done moved={moved:.2f}m')
            return None

        if now >= self._dyn_avoid_until:
            self.reactive_mode = 'BLOCKED'
            self._dyn_avoid_cooldown_until = now + self.dynamic_side_cooldown
            self._dyn_avoid_start_pos = None
            self._dyn_avoid_clear_count = 0
            self.get_logger().warn(
                f'[DWA][{self.ns}] DYN_AVOID timeout, fallback BLOCKED')
            return 0.0, 0.0, 0.0

        direction = self._choose_free_side(sectors)
        if direction != self._dyn_avoid_direction and self._dynamic_side_available(sectors, direction):
            self._dyn_avoid_direction = direction

        vx_limit = self._coordination_v_limit() if effective_vmax is None else effective_vmax
        vx = 0.0 if yield_side_step else min(max(0.0, vx_limit), self.dynamic_side_avoid_speed)
        vy = self._dyn_avoid_direction * min(self.max_vel_y, self.dynamic_side_avoid_vy)
        w = max(-self.avoid_w_max, min(self.avoid_w_max,
                0.3 * self.k_heading * self._heading_error(self.target_heading)))

        _, _, _, collides = self._predict_and_check_collision(vx, vy, w)
        if collides:
            _, _, _, side_collides = self._predict_and_check_collision(0.0, vy, 0.0)
            if side_collides:
                return self.safe_fallback_cmd(sectors)
            vx, w = 0.0, 0.0

        self._last_tracking_mode = 'DYN_AVOID'
        self._last_holo_blk_reason = self._dyn_avoid_reason
        self.get_logger().info(
            f'[DWA][{self.ns}] DYN_AVOID cmd vx={vx:.2f} vy={vy:.2f} '
            f'front_clear={front_clear} moved={moved:.2f} '
            f'confirm={self._dyn_avoid_clear_count}/{self.dynamic_side_clear_confirm}',
            throttle_duration_sec=0.5)
        return vx, vy, w

    def update_stuck_detector(self):
        """
        Cek apakah robot stuck. Return True jika sedang dalam escape mode.
        Escape: 2 detik lateral ke sisi lebih kosong, lalu kembali normal.
        """
        if not self.enable_stuck_detector:
            return False

        now = self.get_clock().now().nanoseconds / 1e9

        if now < self._escape_until:
            return True

        remaining = self._remaining_length
        if self._stuck_last_check_time is None:
            self._stuck_last_remaining  = remaining
            self._stuck_last_check_time = now
            return False

        elapsed = now - self._stuck_last_check_time
        if elapsed < self.stuck_timeout:
            return False

        progress = self._stuck_last_remaining - remaining
        self._stuck_last_remaining  = remaining
        self._stuck_last_check_time = now

        if progress < self.stuck_min_progress and remaining > self.goal_tolerance * 2:
            sectors = self._current_sectors or self.analyze_lidar_sectors()
            self._escape_vy_sign = self._choose_free_side(sectors)
            self._escape_until   = now + 2.0
            self.get_logger().warn(
                f'[DWA] STUCK — progress={progress:.3f}m in {elapsed:.1f}s '
                f'→ ESCAPE vy_sign={self._escape_vy_sign:+d}')
            return True
        return False

    def _backtracking_allowed(self):
        # Pada crossing robot mecanum lebih efektif geser lateral/diagonal.
        # Backtracking membuat robot mundur pelan, lalu sering berhenti lagi
        # sebelum cukup membuka ruang.
        return bool(self.enable_backtracking_recovery) and self.scenario != 'crossing'

    # ═══════════════════════════════════════════════════════════════════════
    # [MOD-12] LOCAL COSTMAP METHODS
    # ═══════════════════════════════════════════════════════════════════════

    def build_local_costmap(self):
        """
        [MOD-12] Build lightweight costmap from latest LaserScan.
        Robot at grid center. Called once per TRACKING cycle when avoidance_mode=='costmap'.
        """
        if not self.laser_data:
            self._local_costmap = None
            return

        n    = int(self.local_costmap_size / self.local_costmap_resolution)
        grid = np.zeros((n, n), dtype=np.uint8)

        ranges  = self.laser_data.ranges
        a_min   = self.laser_data.angle_min
        a_inc   = self.laser_data.angle_increment
        r_min_l = self.laser_data.range_min
        r_max_l = self.laser_data.range_max

        r_lo_eff = max(self.obstacle_min_range, r_min_l)
        r_hi_eff = min(self.obstacle_max_range, r_max_l)

        for i, r in enumerate(ranges):
            if not (r_lo_eff < r < r_hi_eff):
                continue
            angle = a_min + i * a_inc
            lx    = r * math.cos(angle)   # forward in robot frame
            ly    = r * math.sin(angle)   # left in robot frame
            row, col = self.local_to_costmap_cell(lx, ly)
            if 0 <= row < n and 0 <= col < n:
                grid[row, col] = self.lethal_cost

        # Inflation — spread lethal cells outward with decaying cost
        infl_cells = max(1, int(self.inflation_radius / self.local_costmap_resolution))
        obs_rows, obs_cols = np.where(grid >= self.lethal_cost)
        for r_obs, c_obs in zip(obs_rows.tolist(), obs_cols.tolist()):
            r_lo = max(0, r_obs - infl_cells)
            r_hi = min(n, r_obs + infl_cells + 1)
            c_lo = max(0, c_obs - infl_cells)
            c_hi = min(n, c_obs + infl_cells + 1)
            rs    = np.arange(r_lo, r_hi)
            cs    = np.arange(c_lo, c_hi)
            rr, cc = np.meshgrid(rs, cs, indexing='ij')
            dists  = np.hypot(rr - r_obs, cc - c_obs)
            mask   = dists <= infl_cells
            costs  = (self.lethal_cost * (1.0 - dists / (infl_cells + 1))).astype(np.uint8)
            np.maximum(grid[r_lo:r_hi, c_lo:c_hi],
                       np.where(mask, costs, 0),
                       out=grid[r_lo:r_hi, c_lo:c_hi])

        self._local_costmap = grid
        self._publish_local_costmap(grid)

    def _publish_empty_local_costmap_once(self):
        n     = int(self.local_costmap_size / self.local_costmap_resolution)
        empty = np.zeros((n, n), dtype=np.uint8)
        self._publish_local_costmap(empty)
        self._empty_costmap_timer.cancel()

    def _republish_local_costmap(self):
        grid = self._local_costmap
        if grid is None:
            n    = int(self.local_costmap_size / self.local_costmap_resolution)
            grid = np.zeros((n, n), dtype=np.uint8)
        self._publish_local_costmap(grid)

    def _publish_local_costmap(self, grid: np.ndarray):
        n   = grid.shape[0]
        res = self.local_costmap_resolution
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.ns}/base_link'
        msg.info.resolution = res
        msg.info.width      = n
        msg.info.height     = n
        msg.info.origin.position.x = -self.local_costmap_size / 2.0
        msg.info.origin.position.y = -self.local_costmap_size / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # uint8 [0..255] → int8 [-1..100]; lethal=100, free=0
        flat = grid.flatten().tolist()
        msg.data = [100 if v >= self.lethal_threshold else int(v * 100 // self.lethal_cost) for v in flat]
        self._local_costmap_pub.publish(msg)

    def local_to_costmap_cell(self, x_local, y_local):
        """
        [MOD-12] Convert robot-local (x=forward, y=left) coords to grid (row, col).
        Robot at grid center. Row increases downward (decreasing y).
        """
        half = self.local_costmap_size / 2.0
        col  = int((x_local + half) / self.local_costmap_resolution)
        row  = int((half - y_local) / self.local_costmap_resolution)
        return row, col

    def trajectory_costmap_cost(self, trajectory):
        """
        [MOD-12] Evaluate trajectory points against local costmap.
        trajectory: list of [x_world, y_world, theta]
        Returns C_traj = cost_max_weight*C_max + cost_avg_weight*C_avg
        """
        if self._local_costmap is None:
            return 0.0

        n   = self._local_costmap.shape[0]
        rx, ry, rth = self.robot_pose
        cos_t = math.cos(-rth)
        sin_t = math.sin(-rth)

        costs = []
        for px, py, _ in trajectory:
            dx   = px - rx
            dy   = py - ry
            lx   = dx * cos_t - dy * sin_t
            ly   = dx * sin_t + dy * cos_t
            row, col = self.local_to_costmap_cell(lx, ly)
            if 0 <= row < n and 0 <= col < n:
                costs.append(float(self._local_costmap[row, col]))
            # out-of-grid points treated as free

        if not costs:
            return 0.0

        c_max = max(costs)
        c_avg = sum(costs) / len(costs)
        return self.cost_max_weight * c_max + self.cost_avg_weight * c_avg

    # ═══════════════════════════════════════════════════════════════════════
    # [MOD-20] PEER ROBOT DYNAMIC OBSTACLE
    # ═══════════════════════════════════════════════════════════════════════

    def _fresh_peer_poses(self):
        if not self.dynamic_robot_obstacle_enabled:
            return {}
        now = self.get_clock().now().nanoseconds / 1e9
        fresh = {}
        for ns, p in self.peer_poses.items():
            age = now - p.get('stamp', 0.0)
            if age <= self.peer_pose_timeout_s:
                fresh[ns] = p
            else:
                self.get_logger().warn(
                    f'[DWA][{self.ns}] Peer {ns} pose STALE '
                    f'(age={age:.2f}s > {self.peer_pose_timeout_s:.1f}s) — diabaikan',
                    throttle_duration_sec=3.0)
        return fresh

    def _publish_heartbeat(self):
        """[FIX-6] Heartbeat 1Hz — udp_sender dan experiment_cli deteksi DWA alive."""
        msg = Bool()
        msg.data = True
        self._pub_dwa_alive.publish(msg)
        # [FIX-ARRIVE-PUB] Status SAMPAI-posisi tiap heartbeat (logger latch transisi pertama).
        self._pub_pos_reached.publish(Bool(data=bool(self.position_reached)))

    def _set_holo_blocked(self, reason):
        self._last_no_valid_trajectory = True
        self._last_tracking_mode = 'HOLO_BLK'
        self._last_holo_blk_reason = reason
        self.reactive_mode = 'BLOCKED'

    def _convoy_same_direction_peer(self, peer, dist_robot=None):
        if self.scenario != 'convoy':
            return False
        if dist_robot is None:
            rx, ry, _ = self.robot_pose
            dist_robot = math.hypot(peer['x'] - rx, peer['y'] - ry)
        hard_dist = max(0.0, self.convoy_same_direction_hard_block_dist)
        if dist_robot <= hard_dist:
            return False
        peer_theta = peer.get('theta', self.robot_pose[2])
        heading_diff = abs(self.normalize_angle(peer_theta - self.robot_pose[2]))
        if heading_diff > self.convoy_same_direction_heading:
            return False
        return True

    def _crossing_zone_robot_cmd(self, robot_ns):
        if self.scenario != 'crossing':
            return ''
        now = self.get_clock().now().nanoseconds / 1e9
        age = now - self._conflict_zone_detail.get('stamp', 0.0)
        if age > 1.0:
            return ''
        for zone in self._conflict_zone_detail.get('zones', []):
            if zone.get('state') not in ('APPROACH', 'OCCUPIED'):
                continue
            info = zone.get('robots', {}).get(robot_ns, {})
            cmd = str(info.get('cmd', ''))
            if cmd:
                return cmd
        return ''

    def _crossing_owner_can_ignore_peer(self, peer_ns, dist_robot):
        if (not self.crossing_owner_ignore_peer_obstacle_enabled
                or self.scenario != 'crossing'):
            return False
        hard_dist = max(self.crossing_owner_ignore_hard_dist,
                        self.robot_obstacle_radius + 0.05)
        if dist_robot <= hard_dist:
            return False
        my_cmd = self._crossing_zone_robot_cmd(self.ns)
        peer_cmd = self._crossing_zone_robot_cmd(peer_ns)
        if my_cmd == 'GO' and peer_cmd in ('SLOW', 'HOLD'):
            return True
        if my_cmd in ('SLOW', 'HOLD') and peer_cmd == 'GO':
            return True
        return False

    def _peer_in_front_sector(self, peer, front_dist):
        if front_dist == float('inf') or front_dist <= 0.0:
            return False
        rx, ry, rth = self.robot_pose
        dx = peer['x'] - rx
        dy = peer['y'] - ry
        peer_dist = math.hypot(dx, dy)
        if peer_dist <= 1e-3:
            return True
        peer_bearing = abs(self.normalize_angle(math.atan2(dy, dx) - rth))
        front_half_angle = math.radians(22.5)
        return (peer_bearing <= front_half_angle
                and abs(peer_dist - front_dist) <= max(0.18, self.robot_obstacle_radius))

    def _predict_peer_pose(self, peer, t):
        """[MOD-21] Ekstrapolasi pose peer t detik ke depan (kecepatan konstan,
        kerangka dunia). cmd_vel peer (vx,vy) dalam kerangka badan -> rotasi pakai theta.
        Jika prediksi mati / kecepatan tak tersedia, kembalikan pose saat ini (statis)."""
        x  = peer.get('x', 0.0)
        y  = peer.get('y', 0.0)
        th = peer.get('theta', 0.0)
        if not self.peer_predict_enabled:
            return x, y, th
        t = max(0.0, min(t, self.peer_predict_horizon_s))
        vx = peer.get('vx', 0.0)
        vy = peer.get('vy', 0.0)
        w  = peer.get('w', 0.0)
        vx_w = vx * math.cos(th) - vy * math.sin(th)
        vy_w = vx * math.sin(th) + vy * math.cos(th)
        return x + vx_w * t, y + vy_w * t, th + w * t

    def _obb_overlap(self, ax, ay, ath, bx, by, bth, clearance=0.0):
        """[MOD-21] Uji tumpang-tindih dua kotak ber-orientasi via Separating Axis
        Theorem (2D). Kedua robot homogen: half-extent = (L/2, W/2). Mengembalikan
        True bila TIDAK ada sumbu pemisah dengan gap > clearance (yakni footprint
        saling menyentuh/menembus). Ini yang membuat 'mepet tapi aman' lolos dan
        'kelihatan aman tapi nabrak' (sisi panjang berhadapan) tertangkap."""
        hl = 0.5 * self.footprint_length_m
        hw = 0.5 * self.footprint_width_m
        axu = (math.cos(ath), math.sin(ath))
        ayu = (-math.sin(ath), math.cos(ath))
        bxu = (math.cos(bth), math.sin(bth))
        byu = (-math.sin(bth), math.cos(bth))
        dx = bx - ax
        dy = by - ay
        for L in (axu, ayu, bxu, byu):
            proj_d = abs(dx * L[0] + dy * L[1])
            rA = hl * abs(axu[0]*L[0] + axu[1]*L[1]) + hw * abs(ayu[0]*L[0] + ayu[1]*L[1])
            rB = hl * abs(bxu[0]*L[0] + bxu[1]*L[1]) + hw * abs(byu[0]*L[0] + byu[1]*L[1])
            if proj_d > rA + rB + clearance:
                return False   # sumbu pemisah ditemukan -> tidak tumpang tindih
        return True

    def _dynamic_obstacle_cost(self, trajectory):
        """
        Return (reject, penalty, min_dist, peer_ns).
        [MOD-21] Reject memakai uji footprint OBB ber-heading terhadap posisi peer
        yang DIPREDIKSI (kecepatan konstan dari cmd_vel UDP) pada waktu yang SAMA di
        sepanjang horizon -- bukan vs posisi diam & bukan lingkaran. Penalti graded
        saat dekat tapi footprint belum tumpang tindih.
        """
        peers = self._fresh_peer_poses()
        if not peers:
            return False, 0.0, float('inf'), ''
        min_dist = float('inf')
        min_peer = ''
        penalty = 0.0
        rx, ry, _ = self.robot_pose
        n_pts = max(1, len(trajectory))
        T = max(1e-3, self.prediction_time)
        for j, (px, py, pth) in enumerate(trajectory):
            t_j = (j + 1) / n_pts * T   # waktu titik lintasan ini (detik ke depan)
            for peer_ns, peer in peers.items():
                ppx, ppy, ppth = self._predict_peer_pose(peer, t_j)
                d = math.hypot(px - ppx, py - ppy)
                if d < min_dist:
                    min_dist = d
                    min_peer = peer_ns
                # uji footprint OBB ber-heading pada posisi peer terprediksi
                if self._obb_overlap(px, py, pth, ppx, ppy, ppth,
                                     clearance=self.footprint_margin_m):
                    dist_robot = math.hypot(peer['x'] - rx, peer['y'] - ry)
                    if self._crossing_owner_can_ignore_peer(peer_ns, dist_robot):
                        continue
                    if self._convoy_same_direction_peer(peer, dist_robot):
                        self._last_convoy_follow_peer = peer_ns
                        penalty += self.dynamic_obstacle_weight / max(d, 0.05)
                        continue
                    return True, penalty, min_dist, min_peer
                if d < self.robot_obstacle_influence_radius:
                    dist_robot = math.hypot(peer['x'] - rx, peer['y'] - ry)
                    if self._crossing_owner_can_ignore_peer(peer_ns, dist_robot):
                        continue
                    penalty += self.dynamic_obstacle_weight / max(d, 0.05)
        return False, penalty, min_dist, min_peer

    def _path_direction_at(self, idx):
        if not self.global_path or len(self.global_path) < 2:
            th = self.robot_pose[2]
            return math.cos(th), math.sin(th)
        i = max(0, min(idx, len(self.global_path) - 2))
        x1, y1, _ = self.global_path[i]
        x2, y2, _ = self.global_path[i + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < 1e-4:
            th = self.robot_pose[2]
            return math.cos(th), math.sin(th)
        return (x2 - x1) / seg_len, (y2 - y1) / seg_len

    def _peer_blocks_path(self):
        """Deteksi robot lain di depan dan dekat segmen path aktif."""
        peers = self._fresh_peer_poses()
        if not peers or not self.global_path or len(self.global_path) < 2:
            return None
        rx, ry, _ = self.robot_pose
        best = None
        best_d = float('inf')
        end = min(len(self.global_path) - 1, self.current_path_index + 8)
        for peer_ns, peer in peers.items():
            dist_robot = math.hypot(peer['x'] - rx, peer['y'] - ry)
            if dist_robot > self.robot_obstacle_influence_radius:
                continue
            if self._crossing_owner_can_ignore_peer(peer_ns, dist_robot):
                continue
            # [M1] Convoy threshold: jika peer masih di jarak aman (>= threshold),
            # jangan anggap sebagai blocker. Mencegah HOLO_BLK permanen di convoy
            # karena convoy-mate selalu ada di koridor depan pada jarak normal.
            if (self.peer_blocking_max_dist_m > 0.0
                    and dist_robot >= self.peer_blocking_max_dist_m):
                continue
            for i in range(self.current_path_index, end):
                x1, y1, _ = self.global_path[i]
                x2, y2, _ = self.global_path[i + 1]
                # [FIX-1] Capture closest_pt — sebelumnya 'closest' tidak pernah didefinisikan
                closest_pt, d_seg, _ = self.closest_point_on_segment(
                    peer['x'], peer['y'], x1, y1, x2, y2)
                if d_seg > self.robot_path_blocking_radius:
                    continue
                tx, ty = self._path_direction_at(i)
                dot = (peer['x'] - rx) * tx + (peer['y'] - ry) * ty
                ahead = dot > 0.05
                # [FIX-8] Logging relasi per pair — debug convoy
                self.get_logger().info(
                    f'[DWA][{self.ns}↔{peer_ns}] '
                    f'dist={dist_robot:.2f}m d_seg={d_seg:.3f}m '
                    f'dot={dot:.3f} ahead={ahead} seg={i}',
                    throttle_duration_sec=2.0)
                if ahead and self._convoy_same_direction_peer(peer, dist_robot):
                    self._last_convoy_follow_peer = peer_ns
                    self.get_logger().info(
                        f'[DWA][{self.ns}↔{peer_ns}] convoy same-direction '
                        f'follow only dist={dist_robot:.2f}m',
                        throttle_duration_sec=2.0)
                    continue
                if ahead and dist_robot < best_d:
                    best_d = dist_robot
                    best = {
                        'robot': peer_ns,
                        'x': peer['x'],
                        'y': peer['y'],
                        'distance': dist_robot,
                        'path_index': i,
                        'closest': closest_pt,   # [FIX-1]
                    }
        if best is not None:
            self.get_logger().warn(
                f'[DWA][{self.ns}] PEER BLOCKS PATH: {best["robot"]} '
                f'dist={best["distance"]:.2f}m seg={best["path_index"]}')
        return best

    def _choose_bypass_point(self, blocker):
        tx, ty = self._path_direction_at(blocker.get('path_index', self.current_path_index))
        normals = [(-ty, tx), (ty, -tx)]
        if self.ns == 'robot3':
            normals.reverse()
        elif self.ns == 'robot2':
            left_clear = self._point_clearance(
                blocker['x'] + self.bypass_offset * normals[0][0],
                blocker['y'] + self.bypass_offset * normals[0][1])
            right_clear = self._point_clearance(
                blocker['x'] + self.bypass_offset * normals[1][0],
                blocker['y'] + self.bypass_offset * normals[1][1])
            if right_clear > left_clear:
                normals.reverse()
        nx, ny = normals[0]
        return [blocker['x'] + self.bypass_offset * nx,
                blocker['y'] + self.bypass_offset * ny]

    def _update_dynamic_bypass(self):
        self._last_peer_blocks_path = False
        self._last_dyn_peer = ''
        self._last_convoy_follow_peer = ''
        if (not self.dynamic_robot_obstacle_enabled
                or self.priority_stop
                or (self.vmax_from_priority is not None and self.vmax_from_priority <= 0.001)):
            self._bypass_active = False
            self._bypass_peer = None
            self._bypass_point = None
            return
        blocker = self._peer_blocks_path()
        # [FIX-2] Jangan bypass robot prioritas LEBIH TINGGI (leader di convoy).
        # Bypass hanya untuk merge/head-on, bukan convoy following.
        if blocker is not None:
            my_p   = _PRIORITY_RANK.get(self.ns, 0)
            peer_p = _PRIORITY_RANK.get(blocker['robot'], 0)
            if peer_p > my_p:
                self.get_logger().info(
                    f'[DWA][{self.ns}] Peer {blocker["robot"]} adalah LEADER '
                    f'(prio {peer_p}>{my_p}) — bypass DI-SKIP, L5 yang handle',
                    throttle_duration_sec=2.0)
                self._bypass_active = False
                self._bypass_peer   = None
                self._bypass_point  = None
                return
        if blocker is None:
            if self._bypass_active:
                rx, ry, _ = self.robot_pose
                if self._bypass_point is None or math.hypot(
                        self._bypass_point[0] - rx,
                        self._bypass_point[1] - ry) < self.bypass_clear_distance:
                    self._bypass_active = False
                    self._bypass_peer = None
                    self._bypass_point = None
            return
        self._last_peer_blocks_path = True
        self._last_dyn_peer = blocker['robot']
        self._bypass_peer = blocker['robot']
        self._bypass_point = self._choose_bypass_point(blocker)
        self._bypass_active = True

    def _publish_dynamic_obstacle_debug(self):
        if not self.dynamic_robot_obstacle_enabled:
            return
        wall_now = time.time()
        if wall_now - self._last_dynobs_debug_pub < self.dynamic_obstacle_debug_period_s:
            return
        self._last_dynobs_debug_pub = wall_now
        rx, ry, _ = self.robot_pose
        sectors = self._current_sectors or self.analyze_lidar_sectors()
        front_dist = sectors.get('front', float('inf'))
        front_blocked = front_dist < self.front_stop_dist
        peers = self._fresh_peer_poses()
        if not peers and not self._bypass_active:
            return
        peer_front = {
            peer_ns: self._peer_in_front_sector(peer, front_dist)
            for peer_ns, peer in peers.items()
        }
        self._last_peer_in_front_sector = any(peer_front.values())
        rows = []
        my_crossing_cmd = self._crossing_zone_robot_cmd(self.ns)
        peer_crossing_cmd = {
            peer_ns: self._crossing_zone_robot_cmd(peer_ns)
            for peer_ns in peers
        }
        for peer_ns, peer in peers.items():
            rows.append({
                'robot': self.ns,
                'peer_robot': peer_ns,
                'crossing_cmd': my_crossing_cmd,
                'peer_crossing_cmd': peer_crossing_cmd.get(peer_ns, ''),
                'peer_x': peer['x'],
                'peer_y': peer['y'],
                'distance_to_peer': math.hypot(peer['x'] - rx, peer['y'] - ry),
                'blocks_path': self._last_peer_blocks_path and self._last_dyn_peer == peer_ns,
                'bypass_active': self._bypass_active and self._bypass_peer == peer_ns,
                'bypass_x': self._bypass_point[0] if self._bypass_point else '',
                'bypass_y': self._bypass_point[1] if self._bypass_point else '',
                'rejected_candidate_count_dyn': self._dyn_rejected_count,
                'min_dynamic_obstacle_distance': (
                    self._dyn_min_distance if self._dyn_min_distance < float('inf') else -1.0),
                'front_dist': front_dist if front_dist < float('inf') else -1.0,
                'front_blocked': front_blocked,
                'peer_blocks_path': self._last_peer_blocks_path,
                'peer_in_front_sector': peer_front.get(peer_ns, False),
                'holo_blk_reason': self._last_holo_blk_reason,
                'convoy_follow_peer': self._last_convoy_follow_peer,
            })
        msg = String()
        msg.data = json.dumps({'t': wall_now, 'rows': rows})
        self._pub_dynobs.publish(msg)

    def _higher_prio_peer_dwa_active(self) -> bool:
        """[M3] True jika ada peer prioritas lebih tinggi yang sedang DWA-active.
        Dipakai untuk mencegah robot prioritas rendah masuk DWA bersamaan dengan
        robot prioritas tinggi — keputusan avoidance berbasis pose UDP stale
        bisa menyebabkan osilasi atau kedua robot saling mendekati."""
        if (self.wait_peer_dwa_disable_in_crossing
                and self.scenario == 'crossing'):
            return False

        my_p = _PRIORITY_RANK.get(self.ns, 0)
        now  = self.get_clock().now().nanoseconds / 1e9
        for peer_ns, peer in self.peer_poses.items():
            if now - peer.get('stamp', 0.0) > self.peer_pose_timeout_s:
                continue
            if (_PRIORITY_RANK.get(peer_ns, 0) > my_p
                    and peer.get('dwa_active', False)):
                return True
        return False

    # ════════════════════���══════════════════════════════════════════════════
    # CHECK COLLISION — [MOD-4] dynamic r_safe dari EKF covariance
    # ═══════════════════════════════════════════════════════════════════════

    def compute_dynamic_r_safe(self):
        sx2 = max(0.0, self.robot_covariance[0])
        sy2 = max(0.0, self.robot_covariance[7])
        r   = self.r_safe_base + self.k_sigma * math.sqrt(sx2 + sy2)
        return max(self.r_safe_min, min(self.r_safe_max, r))

    def check_collision(self, x, y):
        """bagian dasar"""
        if not self.laser_data:
            return False
        # [M5] EKF warmup: tunda collision check sampai cukup step AMCL agar
        # r_safe yang masih besar di awal tidak salah blokir robot tetangga.
        if self.ekf_warmup_steps > 0 and self._ekf_step_count < self.ekf_warmup_steps:
            return False

        robot_x, robot_y, robot_theta = self.robot_pose

        dx      = x - robot_x
        dy      = y - robot_y
        cos_t   = math.cos(-robot_theta)
        sin_t   = math.sin(-robot_theta)
        point_x = dx * cos_t - dy * sin_t
        point_y = dx * sin_t + dy * cos_t

        point_dist  = math.hypot(point_x, point_y)
        point_angle = math.atan2(point_y, point_x)

        # ±90° filter dihapus: robot mendukung REVERSE_MODE dan diagonal movement,
        # lidar 360° — obstacle di belakang harus tetap dicek.
        beam_idx = int((point_angle - self.laser_data.angle_min) /
                       self.laser_data.angle_increment)

        r_safe = self.compute_dynamic_r_safe()
        for bi in [beam_idx - 1, beam_idx, beam_idx + 1]:
            if 0 <= bi < len(self.laser_data.ranges):
                laser_dist = self.laser_data.ranges[bi]
                # Collision jika obstacle lebih dekat dari (point_dist + r_safe):
                # mencakup obstacle antara robot↔kandidat DAN obstacle dalam r_safe
                # di sisi luar kandidat. Threshold lama (point_dist - r_safe) terlalu
                # longgar — obstacle di dalam safety radius tidak terdeteksi.
                if (self.laser_data.range_min < laser_dist < self.laser_data.range_max
                        and laser_dist < point_dist + r_safe):
                    return True

        return False

    def _predict_and_check_collision(self, vx, vy, w):
        """
        Predict trajectory sekaligus check collision di 4 titik sepanjang horizon.
        Return: (final_x, final_y, final_theta, collides: bool)

        Penggabungan predict_pose + check_collision dalam satu pass — lebih efisien
        dari dua iterasi terpisah, dan mencakup titik-titik ANTARA start↔endpoint
        sehingga lintasan diagonal/mundur yang melewati obstacle di tengah terdeteksi.
        """
        x, y, theta = self.robot_pose
        steps    = int(self.prediction_time / self.dt)
        if steps == 0:
            return x, y, theta, False
        interval = max(1, steps // 4)   # cek di 25%, 50%, 75%, 100% horizon
        for step in range(1, steps + 1):
            theta += w * self.dt
            x     += (vx * math.cos(theta) - vy * math.sin(theta)) * self.dt
            y     += (vx * math.sin(theta) + vy * math.cos(theta)) * self.dt
            if step % interval == 0 and self.check_collision(x, y):
                return x, y, theta, True   # early exit — kandidat ini di-skip
        return x, y, theta, False

    # ═══════════════════════════════════════════════════════════════════════
    # TRACKING CONTROL — [MOD-2] v_max consensus
    # ═══════════════════════════════════════════════════════════════════════

    def tracking_control(self):
        """
        DWA trajectory sampling.

        Perubahan utama:
          - vx_max di-clamp oleh vmax_from_consensus (Layer 4) [MOD-2]
          - Scoring function tambah path_deviation_score [MOD-5]
          - check_collision() memakai threshold 0.1 m
        """
        robot_x, robot_y, robot_theta = self.robot_pose
        target_x, target_y = self.target_point
        self._dyn_rejected_count = 0
        self._dyn_min_distance = float('inf')
        self._last_holo_blk_reason = ''

        # [LANE NEG] Terapkan lateral offset tegak lurus heading robot
        if abs(self.crossing_lane_offset) > 0.01:
            perp_x = -math.sin(robot_theta)
            perp_y =  math.cos(robot_theta)
            target_x += self.crossing_lane_offset * perp_x
            target_y += self.crossing_lane_offset * perp_y

        current_vx, current_vy, current_w = self.robot_vel

        # [MOD-2] Gunakan v_max dari consensus jika tersedia
        effective_vmax = self._coordination_v_limit()
        # [MOD-14] Corner-aware speed reduction
        corner_scale = self._compute_corner_scale()
        effective_vmax *= corner_scale
        self._last_corner_scale = corner_scale
        self._last_effective_vmax = effective_vmax

        if self.reactive_mode == 'DYN_AVOID':
            dyn_cmd = self._dynamic_side_avoid_control(effective_vmax)
            if dyn_cmd is not None:
                return dyn_cmd

        # ── [MOD-17] Holonomic path tracker (primary command source) ──────
        if self.use_holonomic_path_tracker:
            result = self._holonomic_path_track(effective_vmax)
            if result is not None:
                vx, vy, w = result
                # Safety layer 1: point-based collision check
                _, _, _, collides = self._predict_and_check_collision(vx, vy, w)
                if collides:
                    vx2, vy2, w2 = vx * 0.5, vy * 0.5, w * 0.5
                    _, _, _, still = self._predict_and_check_collision(vx2, vy2, w2)
                    if still:
                        # [MOD-18] Only BLOCKED if front obstacle confirmed
                        if self._path_front_blocked():
                            reason = 'HOLO_BLK_PRED_COLLISION_FRONT'
                            sectors = self._current_sectors or self.analyze_lidar_sectors()
                            if self._start_dynamic_side_avoid(reason, sectors):
                                dyn_cmd = self._dynamic_side_avoid_control(effective_vmax)
                                if dyn_cmd is not None:
                                    return dyn_cmd
                            self._set_holo_blocked(reason)
                            self._costmap_stats = {'valid': 0, 'rejected': 1, 'best_cost': 0.0}
                            return 0.0, 0.0, 0.0
                        # Obstacle is behind/side — slow down but don't BLOCK
                        vx, vy, w = vx2, vy2, w2
                    else:
                        vx, vy, w = vx2, vy2, w2
                # Safety layer 2: costmap check
                if self.avoidance_mode == 'costmap' and self._local_costmap is not None:
                    traj  = self.predict_trajectory(vx, vy, w)
                    reject_dyn, penalty_dyn, min_dyn, peer_dyn = self._dynamic_obstacle_cost(traj)
                    if min_dyn < self._dyn_min_distance:
                        self._dyn_min_distance = min_dyn
                        self._last_dyn_peer = peer_dyn
                    if reject_dyn:
                        self._dyn_rejected_count += 1
                        front_blocked = self._path_front_blocked()
                        if front_blocked or self._last_peer_blocks_path:
                            reason = ('HOLO_BLK_DYN_REJECT_FRONT' if front_blocked
                                      else 'HOLO_BLK_DYN_REJECT_PEER')
                            sectors = self._current_sectors or self.analyze_lidar_sectors()
                            if reason == 'HOLO_BLK_DYN_REJECT_PEER':
                                escape_cmd = self._hard_peer_escape_cmd(sectors)
                                if escape_cmd is not None:
                                    return escape_cmd
                            if self._start_dynamic_side_avoid(reason, sectors):
                                dyn_cmd = self._dynamic_side_avoid_control(effective_vmax)
                                if dyn_cmd is not None:
                                    return dyn_cmd
                            self._set_holo_blocked(reason)
                            return 0.0, 0.0, 0.0
                        vx, vy, w = vx * 0.5, vy * 0.5, w * 0.5
                        traj = self.predict_trajectory(vx, vy, w)
                    c_map = self.trajectory_costmap_cost(traj)
                    self._costmap_stats = {'valid': 1, 'rejected': 0, 'best_cost': c_map}
                    if c_map >= self.lethal_threshold:
                        # [MOD-18] Only BLOCKED if front obstacle confirmed
                        if self._path_front_blocked():
                            reason = 'HOLO_BLK_COSTMAP_LETHAL_FRONT'
                            sectors = self._current_sectors or self.analyze_lidar_sectors()
                            if self._start_dynamic_side_avoid(reason, sectors):
                                dyn_cmd = self._dynamic_side_avoid_control(effective_vmax)
                                if dyn_cmd is not None:
                                    return dyn_cmd
                            self._set_holo_blocked(reason)
                            return 0.0, 0.0, 0.0
                        # High cost but not a front obstacle (rear/side robot) → slow down
                        vx = vx * 0.5
                        vy = vy * 0.5
                    if penalty_dyn > 0.0:
                        vx = vx * max(0.25, 1.0 - min(0.5, penalty_dyn * 0.05))
                        vy = vy * max(0.25, 1.0 - min(0.5, penalty_dyn * 0.05))
                else:
                    traj  = self.predict_trajectory(vx, vy, w)
                    reject_dyn, penalty_dyn, min_dyn, peer_dyn = self._dynamic_obstacle_cost(traj)
                    if min_dyn < self._dyn_min_distance:
                        self._dyn_min_distance = min_dyn
                        self._last_dyn_peer = peer_dyn
                    if reject_dyn:
                        self._dyn_rejected_count += 1
                        front_blocked = self._path_front_blocked()
                        if front_blocked or self._last_peer_blocks_path:
                            reason = ('HOLO_BLK_DYN_REJECT_FRONT' if front_blocked
                                      else 'HOLO_BLK_DYN_REJECT_PEER')
                            sectors = self._current_sectors or self.analyze_lidar_sectors()
                            if reason == 'HOLO_BLK_DYN_REJECT_PEER':
                                escape_cmd = self._hard_peer_escape_cmd(sectors)
                                if escape_cmd is not None:
                                    return escape_cmd
                            if self._start_dynamic_side_avoid(reason, sectors):
                                dyn_cmd = self._dynamic_side_avoid_control(effective_vmax)
                                if dyn_cmd is not None:
                                    return dyn_cmd
                            self._set_holo_blocked(reason)
                            return 0.0, 0.0, 0.0
                        vx, vy, w = vx * 0.5, vy * 0.5, w * 0.5
                    if penalty_dyn > 0.0:
                        vx = vx * max(0.25, 1.0 - min(0.5, penalty_dyn * 0.05))
                        vy = vy * max(0.25, 1.0 - min(0.5, penalty_dyn * 0.05))
                    self._costmap_stats = {'valid': 1, 'rejected': 0, 'best_cost': 0.0}
                self._last_no_valid_trajectory = False
                self._last_tracking_mode = 'HOLO'
                return vx, vy, w

        self._last_tracking_mode = 'DWA'
        # ── DWA trajectory sampling (fallback / holonomic disabled) ───────
        # Hitung heading error ke target dulu (untuk mode classification)
        desired_dir   = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = self.normalize_angle(desired_dir - robot_theta)
        abs_err       = abs(heading_error)

        # Dynamic window — batas bawah dari min_vel_x/y (bukan -effective_vmax)
        # agar robot tidak mundur lebih dari parameter membolehkan
        vx_min = max(self.min_vel_x,   current_vx - 0.1)
        vx_max = min(effective_vmax,   current_vx + 0.1)
        vy_min = max(self.min_vel_y,   current_vy - 0.05)
        vy_max = min(self.max_vel_y,   current_vy + 0.05)
        w_min  = max(-self.max_rot_vel, current_w - 0.2)
        w_max  = min(self.max_rot_vel,  current_w + 0.2)

        # Direction-conditioned motion policy — hindari spin besar untuk mecanum
        if abs_err < 0.5:
            # FRONT_MODE: target di depan → maju dominan, minimal spin/lateral
            vx_min = max(vx_min, self.min_vel_x)
            vy_min = max(vy_min, -self.max_vel_y * 0.5)
            vy_max = min(vy_max,  self.max_vel_y * 0.5)
            w_min  = max(w_min,  -self.max_rot_vel * 0.5)
            w_max  = min(w_max,   self.max_rot_vel * 0.5)
        elif abs_err < 2.0:
            # SIDE_MODE: target samping → diagonal ok, batasi spin
            w_lim = self.max_rot_vel * 0.7
            w_min = max(w_min, -w_lim)
            w_max = min(w_max,  w_lim)
        else:
            # REVERSE_MODE: target di belakang → mundur + sedikit rotasi, blokir maju
            vx_max = min(vx_max, 0.03)
            w_lim  = self.max_rot_vel * 0.8
            w_min  = max(w_min, -w_lim)
            w_max  = min(w_max,  w_lim)
            vy_min = max(vy_min, -self.max_vel_y * 0.5)
            vy_max = min(vy_max,  self.max_vel_y * 0.5)

        # [MOD-11] Reactive mode: batasi vx dan w saat AVOIDING/BLOCKED
        mode      = self.reactive_mode
        sectors   = self._current_sectors or {}
        free_side = self._choose_free_side(sectors) if sectors else 1
        react_active = self.enable_reactive_avoidance and mode in ('AVOIDING', 'BLOCKED')
        if react_active:
            vx_max = min(vx_max, self.avoid_vx_max)
            w_min  = max(w_min,  -self.avoid_w_max)
            w_max  = min(w_max,   self.avoid_w_max)

        # [MOD-16] Lateral suppression: batasi |vy| saat heading error besar
        # Mencegah robot geser lateral alih-alih benar-benar belok setelah tikungan.
        # Tidak aktif saat reactive avoidance perlu lateral untuk menghindari obstacle.
        _vy_limited = False
        if (self.heading_lateral_suppress_enabled and not react_active
                and abs(heading_error) > self.heading_lateral_threshold):
            vy_min = max(vy_min, -self.heading_lateral_max_vy)
            vy_max = min(vy_max,  self.heading_lateral_max_vy)
            _vy_limited = True
        self._last_vy_limited = _vy_limited

        vx_samples = np.linspace(vx_min, vx_max, self.vx_samples)
        vy_samples = np.linspace(vy_min, vy_max, self.vy_samples)
        w_samples  = np.linspace(w_min,  w_max,  self.w_samples)

        # [ALGO-TRACE] siklus ini direkam? (subsample agar file tak meledak)
        _trace_due = (self.algo_trace_enabled
                      and (time.time() - self._last_dwa_trace_t)
                          >= self.algo_trace_dwa_period_s)
        if _trace_due:
            self._dwa_trace_buf = []

        best_score              = -float('inf')
        best_vx, best_vy, best_w = 0.0, 0.0, 0.0
        use_costmap = (self.avoidance_mode == 'costmap'
                       and self._local_costmap is not None)
        _valid_count    = 0
        _rejected_cost  = 0
        _best_cost      = 0.0

        for vx in vx_samples:
            for vy in vy_samples:
                # Filter vy kecil saat heading OK — kecuali sedang menghindari obstacle
                if (abs(heading_error) < self.min_heading_error_for_sideways
                        and not react_active):
                    if abs(vy) > 0.05:
                        continue

                for w in w_samples:
                    if (abs(vx) + abs(vy)) < 0.05 and abs(w) > 0.5:
                        continue

                    # [MOD-2] Holonomic magnitude constraint:
                    # untuk mecanum, membatasi vx saja tidak cukup —
                    # sqrt(vx²+vy²) juga harus <= effective_vmax
                    if math.hypot(vx, vy) > effective_vmax + 1e-6:
                        continue

                    pred_x, pred_y, pred_theta, collides = \
                        self._predict_and_check_collision(vx, vy, w)
                    if collides:
                        if _trace_due:
                            self._dwa_trace_buf.append(
                                (vx, vy, w, 'COLLISION', None,
                                 pred_x, pred_y, pred_theta, None))
                        continue

                    # [MOD-12] Costmap cost — reject lethal trajectories before scoring
                    c_traj = 0.0
                    traj_for_dyn = None
                    if use_costmap:
                        traj_for_dyn = self.predict_trajectory(vx, vy, w)
                        c_traj = self.trajectory_costmap_cost(traj_for_dyn)
                        if c_traj >= self.lethal_threshold:
                            if _trace_due:
                                self._dwa_trace_buf.append(
                                    (vx, vy, w, 'LETHAL', c_traj,
                                     pred_x, pred_y, pred_theta, None))
                            _rejected_cost += 1
                            continue

                    if self.dynamic_robot_obstacle_enabled:
                        if traj_for_dyn is None:
                            traj_for_dyn = self.predict_trajectory(vx, vy, w)
                        reject_dyn, penalty_dyn, min_dyn, peer_dyn = \
                            self._dynamic_obstacle_cost(traj_for_dyn)
                        if min_dyn < self._dyn_min_distance:
                            self._dyn_min_distance = min_dyn
                            self._last_dyn_peer = peer_dyn
                        if reject_dyn:
                            if _trace_due:
                                self._dwa_trace_buf.append(
                                    (vx, vy, w, 'DYN_REJECT', c_traj,
                                     pred_x, pred_y, pred_theta, None))
                            self._dyn_rejected_count += 1
                            continue
                    else:
                        penalty_dyn = 0.0

                    _valid_count += 1
                    score = self.calculate_path_following_score(
                        pred_x, pred_y, pred_theta,
                        target_x, target_y,
                        vx, vy, w,
                        robot_x, robot_y, robot_theta,
                        heading_error)

                    # [MOD-11] Clearance penalty — prefer trajectories jauh dari obstacle
                    if self.enable_reactive_avoidance and self.obstacle_clearance_weight > 0:
                        clearance = self._point_clearance(pred_x, pred_y)
                        if clearance != float('inf'):
                            score -= self.obstacle_clearance_weight / (
                                max(clearance, 0.02) + 0.05)

                    # [MOD-11] Lateral bias — saat AVOIDING/BLOCKED, reward vy ke sisi kosong
                    if react_active:
                        score += self.avoid_lateral_weight * free_side * vy

                    # [MOD-12] Costmap penalty — penalize trajectories near obstacles
                    if use_costmap and c_traj > 0.0:
                        score -= self.local_cost_weight * c_traj / 255.0

                    if penalty_dyn > 0.0:
                        score -= penalty_dyn

                    if _trace_due:
                        self._dwa_trace_buf.append(
                            (vx, vy, w, 'VALID', c_traj,
                             pred_x, pred_y, pred_theta, score))

                    if score > best_score:
                        best_score            = score
                        best_vx, best_vy, best_w = vx, vy, w
                        _best_cost            = c_traj

        # [MOD-12] Store stats for status_report
        self._costmap_stats = {
            'valid':    _valid_count,
            'rejected': _rejected_cost + self._dyn_rejected_count,
            'best_cost': _best_cost,
        }

        # [MOD-15] Semua kandidat ditolak → sinyal BLOCKED agar backtracking bisa aktif
        # [MOD-18] Tapi hanya BLOCKED jika ada obstacle di front sector.
        # Jika front bersih, block berasal dari obstacle samping/belakang (robot yang
        # mengikuti di belakang) → jangan BLOCK robot depan.
        if _valid_count == 0:
            self._last_no_valid_trajectory = True
            if self._path_front_blocked():
                self.reactive_mode = 'BLOCKED'
            # else: front bersih → jangan BLOCKED, biarkan control_loop handle
            if _trace_due:
                try:
                    self._flush_dwa_trace(0.0, 0.0, 0.0,
                                          robot_x, robot_y, robot_theta,
                                          target_x, target_y, effective_vmax)
                except Exception as e:
                    self.get_logger().warn(f'[ALGO-TRACE] flush DWA gagal: {e}')
                self._last_dwa_trace_t = time.time()
            return 0.0, 0.0, 0.0
        self._last_no_valid_trajectory = False

        # Post-selection hard clamp — safety net agar magnitude tidak melebihi
        # effective_vmax meskipun floating-point sampling tidak tepat
        speed_mag = math.hypot(best_vx, best_vy)
        if speed_mag > effective_vmax + 1e-6 and speed_mag > 1e-4:
            scale    = effective_vmax / speed_mag
            best_vx *= scale
            best_vy *= scale

        if _trace_due:
            try:
                self._flush_dwa_trace(best_vx, best_vy, best_w,
                                      robot_x, robot_y, robot_theta,
                                      target_x, target_y, effective_vmax)
            except Exception as e:
                self.get_logger().warn(f'[ALGO-TRACE] flush DWA gagal: {e}')
            self._last_dwa_trace_t = time.time()

        return best_vx, best_vy, best_w

    # ═══════════════════════════════════════════════════════════════════════
    # SCORING FUNCTION — [MOD-5] path deviation score
    # ═══════════════════════════════════════════════════════════════════════

    def calculate_path_following_score(self, pred_x, pred_y, pred_theta,
                                       target_x, target_y,
                                       vx, vy, w,
                                       robot_x, robot_y, robot_theta,
                                       heading_error):
        """Hitung score trajectory dengan tambahan path deviation.

        Tambahan utama:
          - S_path: cross-track distance dari predicted pose ke global path
            → Trajectory yang "kabur" dari path dihukum, robot ditarik kembali
              ke rute setelah menghindar obstacle (Yang et al. 2023)
        """
        # ── Score dasar ────────────────────────────────────────────
        dist_to_target = math.hypot(target_x - pred_x, target_y - pred_y)
        goal_score     = -3.0 * dist_to_target

        trans_speed        = math.hypot(vx, vy)
        translation_score  = 1.2 * trans_speed / max(self.max_vel_x, self.max_vel_y, 0.1)

        target_heading_error = abs(self.normalize_angle(self.target_heading - pred_theta))
        heading_score        = -0.25 * target_heading_error

        if len(self.global_path) > 0:
            final_x, final_y, _ = self.global_path[-1]
            current_dist  = math.hypot(final_x - robot_x, final_y - robot_y)
            pred_dist     = math.hypot(final_x - pred_x,  final_y - pred_y)
            progress_score = 1.5 * max(0, current_dist - pred_dist)
        else:
            progress_score = 0.0

        sideways_penalty = 0.5 * abs(vy) / max(self.max_vel_y, 0.1)

        vel_angle       = math.atan2(vy, vx) if abs(vx) > 0.01 or abs(vy) > 0.01 else 0.0
        target_angle_rel = self.normalize_angle(heading_error - vel_angle)
        direction_score  = -0.5 * abs(target_angle_rel)

        smooth_penalty = (0.1 * abs(vx - self.robot_vel[0]) +
                          0.2 * abs(vy - self.robot_vel[1]) +
                          0.1 * abs(w  - self.robot_vel[2]))

        # [MOD-5] Cross-track deviation from global path — pull robot back after avoidance
        if len(self.global_path) > 1:
            min_cross_track = float('inf')
            search_start = max(0, self.current_path_index - 1)
            search_end   = min(len(self.global_path) - 1, self.current_path_index + 6)
            for i in range(search_start, search_end):
                x1, y1, _ = self.global_path[i]
                x2, y2, _ = self.global_path[i + 1]
                _, cross_dist, _ = self.closest_point_on_segment(
                    pred_x, pred_y, x1, y1, x2, y2)
                if cross_dist < min_cross_track:
                    min_cross_track = cross_dist
            path_dev_score = -2.0 * min_cross_track
        else:
            path_dev_score = 0.0

        rotation_penalty = 0.8 * abs(w)
        total_score = (goal_score + translation_score + heading_score +
                       progress_score + direction_score + path_dev_score -
                       sideways_penalty - smooth_penalty - rotation_penalty)

        return total_score

    # ═══════════════════════════════════════════════════════════════════════
    # CONTROL LOOP — [MOD-6] fault + [MOD-7] priority stop
    # ═══════════════════════════════════════════════════════════════════════

    def control_loop(self):
        """[FIX-3] Wrapper try/except agar exception tidak bunuh node."""
        try:
            self._control_loop_impl()
        except Exception as exc:
            import traceback
            self.get_logger().error(
                f'[DWA][{self.ns}] EXCEPTION di control_loop: {exc!r} — '
                f'robot dihentikan sementara, node tetap hidup',
                throttle_duration_sec=2.0)
            self.get_logger().error(traceback.format_exc(), throttle_duration_sec=5.0)
            try:
                self.stop_robot()
            except Exception:
                pass

    def _control_loop_impl(self):
        """Jalankan kontrol setelah cek fault_active dan priority_stop."""
        # [MOD-6] Fault active → robot diam, cmd_vel diblokir
        if self.fault_active:
            self._last_hold_reason = 'fault'
            self.stop_robot()
            return

        # [MOD-7] Priority stop dari Layer 5 → robot diam sementara
        if self.priority_stop:
            self._last_hold_reason = 'priority_stop'
            escape_cmd = None
            if self.experiment_state == 'RUNNING':
                escape_cmd = self._hard_peer_escape_cmd()
            if escape_cmd is not None:
                vx, vy, w = escape_cmd
                self._last_hold_reason = 'priority_escape'
                self._ctrl_eff_vmax = max(
                    self.hard_peer_escape_speed, self._coordination_v_limit())
                self.publish_command(
                    vx, vy, w,
                    ignore_priority_stop=True,
                    override_v_limit=self.hard_peer_escape_speed)
                self.publish_local_plan(self.predict_trajectory(vx, vy, w))
                self._publish_debug_telemetry()
                self._publish_dynamic_obstacle_debug()
                return
            self.stop_robot()
            return

        # [MOD-18] vmax_effective == 0 → stop (zone/priority memberi vmax nol)
        # Ini menangkap kasus APPROACH/FINAL yang masih bergerak walaupun master kirim vmax=0
        _eff = self._coordination_v_limit()
        if _eff <= 0.001:
            # [FIX-YAW] Jangan early-stop bila robot SUDAH di area goal dan hanya perlu
            # menyelaraskan yaw akhir (menghadap goal). Saat goal_reached, koordinasi
            # mengirim vmax=0; tanpa pengecualian ini loop berhenti SEBELUM sempat men-set
            # position_reached & menjalankan FINAL_ALIGNING, sehingga robot membeku dengan
            # yaw belum menghadap goal. Rotasi di tempat aman & tak memakai budget tiba.
            _near_goal = False
            if self.final_goal_pose is not None:
                _gx, _gy, _ = self.final_goal_pose
                _rx, _ry, _ = self.robot_pose
                _near_goal = math.hypot(_gx - _rx, _gy - _ry) < self.goal_tolerance * 1.5
            if _near_goal or self.position_reached or self.current_state == self.STATE_FINAL_ALIGNING:
                self._ctrl_eff_vmax = max(_eff, self._final_align_vmax_floor)
            else:
                self._last_hold_reason = 'vmax_zero'
                self.stop_robot()
                return
        else:
            self._ctrl_eff_vmax = _eff

        # ── Sisa alur dasar ─────────────────────────────────────────
        if not self.laser_data:
            self._last_hold_reason = 'no_scan'
            # Jangan diam total saat scan belum masuk. analyze_lidar_sectors()
            # dan build_local_costmap() sudah punya fallback kosong, jadi robot
            # masih bisa tracking path sambil telemetry menandai HOLD_no_scan.

        if not self.global_path or len(self.global_path) < 2:
            self._last_hold_reason = 'no_path'
            self.stop_robot()
            return

        # Localization guard — jika AMCL mulai diverge, jangan lanjut muter.
        if self._update_localization_guard():
            reason = getattr(self, '_localization_hold_reason', '') or 'COV_INVALID'
            self._last_tracking_mode = f'LOCALIZATION_HOLD/{reason}'
            self.stop_robot()
            self._publish_debug_telemetry()
            self._publish_dynamic_obstacle_debug()
            return
        if self._update_localization_consistency_guard():
            reason = getattr(self, '_localization_hold_reason', '') or 'AMCL_ODOM_INCONSISTENT'
            self._last_tracking_mode = f'LOCALIZATION_HOLD/{reason}'
            self.stop_robot()
            self._publish_debug_telemetry()
            self._publish_dynamic_obstacle_debug()
            return

        self.update_tracking_position()

        # [MOD-14] Adaptive lookahead: gunakan jarak lebih pendek saat corner aktif
        # (corner_scale dari siklus sebelumnya — sedikit lag tapi cukup untuk anticipate)
        _la_normal = self.lookahead_distance
        if self.corner_slowdown_enabled and self._last_corner_scale < 0.99:
            self.lookahead_distance = self.corner_lookahead_distance
        self.target_point, self.target_heading = self.get_next_target_with_heading()
        self._last_lookahead_used = self.lookahead_distance
        self.lookahead_distance   = _la_normal   # restore setelah dipakai

        if not self.target_point:
            self._last_hold_reason = 'no_target'
            self.stop_robot()
            return

        self._update_dynamic_bypass()
        if self._bypass_active and self._bypass_point is not None:
            self.target_point = list(self._bypass_point)
            robot_x, robot_y, _ = self.robot_pose
            self.target_heading = math.atan2(
                self.target_point[1] - robot_y,
                self.target_point[0] - robot_x)

        robot_x, robot_y, robot_theta = self.robot_pose
        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

            if dist_to_goal < self.goal_tolerance and not self.position_reached:
                now = time.time()
                valid, _, _ = self._localization_sigma_valid()
                if self._goal_inside_since is None:
                    self._goal_inside_since = now
                elif (valid and not self._localization_consistency_hold
                      and now - self._goal_inside_since >= self.goal_reached_stable_s):
                    self.position_reached = True
                    self.get_logger().info('Position goal reached! Starting final alignment...')
                    self.current_state  = self.STATE_FINAL_ALIGNING
                    self.target_heading = self.goal_orientation
                elif (self.goal_zone_force_stop_s > 0.0
                      and now - self._goal_inside_since >= self.goal_zone_force_stop_s):
                    # [DEMO-FINAL-STOP] Jaring pengaman: sudah lama di zona goal tapi
                    # localization invalid / consistency-hold -> tetap latch supaya
                    # robot tidak berputar selamanya menunggu AMCL valid.
                    self.position_reached = True
                    self.get_logger().warn(
                        f'Position goal reached (force-latch tanpa loc valid): '
                        f'{now - self._goal_inside_since:.1f}s di zona goal '
                        f'>= {self.goal_zone_force_stop_s:.1f}s -> final align.')
                    self.current_state  = self.STATE_FINAL_ALIGNING
                    self.target_heading = self.goal_orientation
            elif dist_to_goal >= self.goal_tolerance:
                self._goal_inside_since = None

        if not self.position_reached:
            self._position_reached_since = None   # [DEMO-FINAL-STOP] reset timer saat belum/lepas dari goal
            self.determine_state()
        else:
            now = time.time()
            if self._position_reached_since is None:
                self._position_reached_since = now
            final_heading_error = abs(
                self.normalize_angle(self.goal_orientation - robot_theta))
            if final_heading_error <= self.heading_alignment_tolerance:
                self.get_logger().info('GOAL REACHED! (position + orientation)')
                self.stop_robot()
                self.current_state = self.STATE_IDLE
                return
            # [DEMO-FINAL-STOP] Paksa berhenti: posisi sudah di goal cukup lama,
            # jangan tunggu yaw sempurna (biar demo tidak menggantung di APPR final).
            held = now - self._position_reached_since
            if self.final_align_timeout_s > 0.0 and held >= self.final_align_timeout_s:
                self.get_logger().info(
                    f'GOAL REACHED (forced)! Posisi di goal {held:.1f}s '
                    f'>= {self.final_align_timeout_s:.1f}s, yaw_err={math.degrees(final_heading_error):.1f}deg '
                    f'-> berhenti tanpa tunggu align.')
                self.stop_robot()
                self.current_state = self.STATE_IDLE
                return

        # Timeout ALIGNING — paksa ke TRACKING jika tidak konvergen dalam 3 detik
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.current_state == self.STATE_ALIGNING:
            if self._aligning_since is None:
                self._aligning_since = now_sec
            elif now_sec - self._aligning_since > 3.0:
                self.get_logger().warn(
                    f'[{self.ns}] ALIGNING timeout — paksa TRACKING (cooldown 5s)')
                self.current_state         = self.STATE_TRACKING
                self._aligning_since       = None
                self._align_cooldown_until = now_sec + 5.0  # blokir re-entry 5s
        else:
            self._aligning_since = None

        if self.current_state == self.STATE_ALIGNING:
            vx, vy, w = self.heading_alignment_control()
        elif self.current_state == self.STATE_TRACKING:
            # [MOD-11] Analyze sectors once per cycle — shared with tracking_control
            sectors     = self.analyze_lidar_sectors()
            self._current_sectors = sectors
            new_reactive = self.get_reactive_mode(sectors)

            # [MOD-12] Rebuild costmap each cycle when in costmap mode
            if self.avoidance_mode == 'costmap':
                self.build_local_costmap()

            escaping = self.update_stuck_detector()
            if escaping:
                self.reactive_mode          = 'STUCK_ESCAPE'
                self._blocked_since         = None
                self._backtrack_clear_since = None
                vx, vy, w = 0.0, self._escape_vy_sign * self._lateral_recovery_speed(), 0.0

            elif self.reactive_mode == 'DYN_AVOID':
                dyn_cmd = self._dynamic_side_avoid_control()
                if dyn_cmd is None:
                    self.reactive_mode = 'TRACKING'
                    vx, vy, w = self.tracking_control()
                else:
                    vx, vy, w = dyn_cmd

            elif self.reactive_mode == 'BACKTRACKING':
                if not self._backtracking_allowed():
                    self.reactive_mode          = 'BLOCKED'
                    self._backtrack_target      = None
                    self._backtrack_clear_since = None
                    vx, vy, w = self.safe_fallback_cmd(sectors)
                # [MOD-15] Currently backtracking — check if path ahead is clear
                elif new_reactive != 'BLOCKED':
                    if self._backtrack_clear_since is None:
                        self._backtrack_clear_since = now_sec
                    if now_sec - self._backtrack_clear_since >= self.clear_time:
                        # Obstacle clear long enough — resume normal tracking
                        self.reactive_mode          = 'TRACKING'
                        self._backtrack_target      = None
                        self._blocked_since         = None
                        self._backtrack_clear_since = None
                        vx, vy, w = self.tracking_control()
                    else:
                        vx, vy, w = self._backtracking_control()
                else:
                    self._backtrack_clear_since = None
                    vx, vy, w = self._backtracking_control()

            elif self._backtracking_allowed() and new_reactive == 'BLOCKED':
                # [MOD-15] Obstacle blocking — start/extend blocked timer
                self.reactive_mode = 'BLOCKED'
                if self._blocked_since is None:
                    self._blocked_since = now_sec
                if now_sec - self._blocked_since >= self.blocked_timeout:
                    # Timeout — trigger backtracking
                    self._backtrack_target      = self._get_backtrack_target()
                    self.reactive_mode          = 'BACKTRACKING'
                    self._backtrack_clear_since = None
                    vx, vy, w = self._backtracking_control()
                else:
                    vx, vy, w = self.tracking_control()
                    if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(w) < 0.01:
                        vx, vy, w = self.safe_fallback_cmd(sectors)

            else:
                # Normal tracking — set mode dulu, lalu jalankan DWA
                self.reactive_mode = new_reactive
                vx, vy, w = self.tracking_control()

                # [MOD-15] tracking_control() mungkin mengubah reactive_mode menjadi
                # BLOCKED dari dalam (zero valid trajectory — costmap/collision).
                # Accumulate _blocked_since di sini agar backtracking bisa aktif
                # meskipun enable_reactive_avoidance=false.
                if self.reactive_mode == 'BLOCKED':
                    if self._backtracking_allowed():
                        if self._blocked_since is None:
                            self._blocked_since = now_sec
                        if now_sec - self._blocked_since >= self.blocked_timeout:
                            self._backtrack_target      = self._get_backtrack_target()
                            self.reactive_mode          = 'BACKTRACKING'
                            self._backtrack_clear_since = None
                            vx, vy, w = self._backtracking_control()
                        else:
                            if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(w) < 0.01:
                                vx, vy, w = self.safe_fallback_cmd(sectors)
                    else:
                        if abs(vx) < 0.01 and abs(vy) < 0.01 and abs(w) < 0.01:
                            vx, vy, w = self.safe_fallback_cmd(sectors)
                else:
                    self._blocked_since = None
            # [M3] DWA simultaneous prevention: berlaku untuk SEMUA path di
            # STATE_TRACKING (normal, BLOCKED recovery, BACKTRACKING resume).
            # Jika robot masuk DWA/HOLO_BLK dan ada peer prio-lebih-tinggi yang
            # juga DWA-aktif → stop. Pose UDP bisa stale → keputusan avoidance
            # dua robot sekaligus bisa osilasi atau saling mendekati.
            if (self._last_tracking_mode in ('HOLO_BLK', 'DWA', 'BLOCKED', 'DYN_AVOID')
                    and self._higher_prio_peer_dwa_active()):
                vx, vy, w = 0.0, 0.0, 0.0
                self._last_tracking_mode = 'WAIT_PEER_DWA'
                self._last_hold_reason   = 'wait_peer_dwa'
        elif self.current_state == self.STATE_APPROACHING:
            vx, vy, w = self.approach_control()
        elif self.current_state == self.STATE_FINAL_ALIGNING:
            vx, vy, w = self.final_alignment_control()
        else:
            self.stop_robot()
            return

        if self.experiment_state != 'RUNNING':
            self._last_hold_reason = f'state_{self.experiment_state}'
            self.stop_robot()
            self._publish_debug_telemetry()
            return
        if self._last_hold_reason != 'no_scan':
            self._last_hold_reason = ''
        if self.current_state == self.STATE_FINAL_ALIGNING:
            # [FIX-YAW] Izinkan rotasi penyelarasan yaw di tempat walau koordinasi
            # mengirim vmax=0; override floor agar omega tidak digerus gate v_limit<=0.001.
            self.publish_command(
                vx, vy, w, override_v_limit=self._final_align_vmax_floor)
        else:
            self.publish_command(vx, vy, w)
        self.publish_local_plan(self.predict_trajectory(vx, vy, w))
        self._publish_debug_telemetry()
        self._publish_dynamic_obstacle_debug()

    # ═══════════════════════════════════════════════════════════════════════
    # CONTROL BEHAVIORS — bagian dasar
    # ═══════════════════════════════════════════════════════════════════════

    def determine_state(self):
        """bagian dasar"""
        robot_x, robot_y, _ = self.robot_pose
        target_x, target_y = self.target_point
        dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)

        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        else:
            dist_to_goal = dist_to_target

        if dist_to_goal < self.goal_tolerance * 1.5:
            self.current_state = self.STATE_APPROACHING
        else:
            # Mecanum holonomic: tidak perlu heading align sebelum tracking.
            # DWA menangani heading koreksi gradual via vx+vy+w sekaligus.
            # STATE_ALIGNING hanya menyebabkan spin besar → AMCL drift.
            self.current_state = self.STATE_TRACKING

    def heading_alignment_control(self):
        """ clamp translasi ke _ctrl_eff_vmax [MOD-18]"""
        robot_theta   = self.robot_pose[2]
        heading_error = self.normalize_angle(self.target_heading - robot_theta)
        kp = 0.8
        w  = max(-self.max_rot_vel, min(self.max_rot_vel, kp * heading_error))
        if abs(heading_error) < 0.3:
            w *= 0.7
        # Translasi kecil saat aligning membantu AMCL scan matching — clamp ke vmax efektif
        # [FF-CATCHUP] Saat catch-up aktif, plafon translasi align dinaikkan ke
        # align_speed_catchup (default 0.06) agar robot tertinggal sedikit lebih gesit.
        align_cap = (self.align_speed_catchup
                     if getattr(self, 'feedforward_catchup_enabled', False)
                     else 0.03)
        v_trans = min(align_cap, self._ctrl_eff_vmax if self._ctrl_eff_vmax is not None else align_cap)
        return v_trans, 0.0, w

    def approach_control(self):
        """bagian dasar"""
        robot_x, robot_y, robot_theta = self.robot_pose
        target_x, target_y = self.target_point
        dx   = target_x - robot_x
        dy   = target_y - robot_y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            return 0.0, 0.0, 0.0

        cos_t    = math.cos(robot_theta)
        sin_t    = math.sin(robot_theta)
        vx_local = dx * cos_t + dy * sin_t
        vy_local = -dx * sin_t + dy * cos_t

        kp = 0.5
        vx = kp * vx_local
        vy = kp * vy_local
        # [MOD-18] Clamp approach speed ke vmax_effective agar zone/priority dihormati
        # [FF-CATCHUP] Saat catch-up aktif, plafon approach dinaikkan ke
        # approach_speed_catchup agar robot tertinggal tidak merayap di fase akhir.
        approach_cap = (self.approach_speed_catchup
                        if getattr(self, 'feedforward_catchup_enabled', False)
                        else 0.1)
        max_approach_speed = min(approach_cap,
                                 self._ctrl_eff_vmax if self._ctrl_eff_vmax is not None else approach_cap)
        speed = math.hypot(vx, vy)
        if speed > max_approach_speed:
            vx *= max_approach_speed / speed
            vy *= max_approach_speed / speed

        return vx, vy, 0.0

    def final_alignment_control(self):
        """bagian dasar"""
        robot_x, robot_y, robot_theta = self.robot_pose

        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            pos_error = math.hypot(goal_x - robot_x, goal_y - robot_y)
            if pos_error > self.goal_tolerance * 1.1:
                self.get_logger().warn('Drifted from goal, correcting position...')
                self.position_reached = False
                self.current_state    = self.STATE_APPROACHING
                return self.approach_control()

        heading_error = self.normalize_angle(self.goal_orientation - robot_theta)
        if abs(heading_error) < self.heading_alignment_tolerance:
            return 0.0, 0.0, 0.0

        kp = 2.0
        w  = max(-self.max_rot_vel * 0.8,
                 min(self.max_rot_vel * 0.8, kp * heading_error))
        if abs(heading_error) < 0.5:
            w *= abs(heading_error) / 0.5

        vx, vy = 0.0, 0.0
        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            pos_error = math.hypot(goal_x - robot_x, goal_y - robot_y)
            if pos_error > self.goal_tolerance:
                dx    = goal_x - robot_x
                dy    = goal_y - robot_y
                cos_t = math.cos(robot_theta)
                sin_t = math.sin(robot_theta)
                # [MOD-18] Clamp koreksi posisi ke vmax_effective
                corr_limit = min(0.03, self._ctrl_eff_vmax if self._ctrl_eff_vmax is not None else 0.03)
                vx    = corr_limit * (dx * cos_t + dy * sin_t)
                vy    = corr_limit * (-dx * sin_t + dy * cos_t)
                corr_speed = math.hypot(vx, vy)
                if corr_speed > corr_limit:
                    vx *= corr_limit / corr_speed
                    vy *= corr_limit / corr_speed

        return vx, vy, w

    # ═══════════════════════════════════════════════════════════════════════
    # PATH TRACKING — bagian dasar
    # ═══════════════════════════════════════════════════════════════════════

    def find_current_position_on_path(self):
        """bagian dasar"""
        if not self.global_path or len(self.global_path) < 2:
            return

        robot_x, robot_y, _ = self.robot_pose
        min_dist = float('inf')
        best_idx = 0
        best_t   = 0.0

        for i in range(len(self.global_path) - 1):
            x1, y1, _ = self.global_path[i]
            x2, y2, _ = self.global_path[i + 1]
            _, dist, t = self.closest_point_on_segment(
                robot_x, robot_y, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_t   = t

        if best_idx >= len(self.global_path) - 1:
            best_idx = max(0, len(self.global_path) - 2)
            best_t   = 0.0

        self.current_path_index     = best_idx
        self.progress_along_segment = best_t

        if best_t < 0.1:
            self.progress_along_segment = 0.0
        elif best_t > 0.9 and self.current_path_index < len(self.global_path) - 2:
            self.current_path_index    += 1
            self.progress_along_segment = 0.0

    def update_tracking_position(self):
        """bagian dasar"""
        if not self.global_path or len(self.global_path) < 2:
            return

        robot_x, robot_y, _ = self.robot_pose
        search_start = max(0, self.current_path_index - 1)
        search_end   = min(len(self.global_path) - 1,
                           self.current_path_index + 3)

        min_dist = float('inf')
        best_idx = self.current_path_index
        best_t   = self.progress_along_segment

        for i in range(search_start, search_end):
            if i >= len(self.global_path) - 1:
                break
            x1, y1, _ = self.global_path[i]
            x2, y2, _ = self.global_path[i + 1]
            _, dist, t = self.closest_point_on_segment(
                robot_x, robot_y, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_t   = t

        if best_idx >= len(self.global_path) - 1:
            best_idx = max(0, len(self.global_path) - 2)
            best_t   = 0.0

        if min_dist < self.path_tracking_tolerance:
            self.current_path_index     = best_idx
            self.progress_along_segment = best_t
            if best_t > 0.9 and self.current_path_index < len(self.global_path) - 2:
                self.current_path_index    += 1
                self.progress_along_segment = 0.0

    def get_next_target_with_heading(self):
        """bagian dasar"""
        if not self.global_path or len(self.global_path) < 2:
            return None, 0.0

        if (self.position_reached and
                self.current_state == self.STATE_FINAL_ALIGNING and
                self.final_goal_pose):
            robot_x, robot_y, _ = self.robot_pose
            return [robot_x, robot_y], self.goal_orientation

        start_idx      = self.current_path_index
        start_progress = self.progress_along_segment
        lookahead_rem  = self.lookahead_distance
        current_idx    = start_idx

        if start_progress > 0 and current_idx < len(self.global_path) - 1:
            x1, y1, _ = self.global_path[current_idx]
            x2, y2, _ = self.global_path[current_idx + 1]
            seg_len    = math.hypot(x2 - x1, y2 - y1)

            if seg_len == 0:
                current_idx    += 1
                start_progress  = 0.0
                start_x, start_y = (self.global_path[current_idx][:2]
                                    if current_idx < len(self.global_path)
                                    else self.global_path[-1][:2])
            else:
                start_x = x1 + start_progress * (x2 - x1)
                start_y = y1 + start_progress * (y2 - y1)
                rem_on_seg = (1 - start_progress) * seg_len

                if rem_on_seg >= lookahead_rem:
                    ratio    = lookahead_rem / seg_len
                    target_x = start_x + ratio * (x2 - x1)
                    target_y = start_y + ratio * (y2 - y1)
                    return [target_x, target_y], self._path_heading_at_segment(current_idx, ratio)
                else:
                    lookahead_rem -= rem_on_seg
                    current_idx   += 1
                    if current_idx >= len(self.global_path):
                        tx, ty, _ = self.global_path[-1]
                        if len(self.global_path) > 1:
                            return [tx, ty], self.global_path[-1][2]
                        return [tx, ty], self.goal_orientation
                    start_x, start_y, _ = self.global_path[current_idx]
        else:
            if current_idx < len(self.global_path):
                start_x, start_y, _ = self.global_path[current_idx]
            else:
                tx, ty, _ = self.global_path[-1]
                if len(self.global_path) > 1:
                    px, py, _ = self.global_path[-2]
                    return [tx, ty], math.atan2(ty - py, tx - px)
                return [tx, ty], self.goal_orientation

        while current_idx < len(self.global_path) - 1 and lookahead_rem > 0:
            x1, y1, _ = self.global_path[current_idx]
            x2, y2, _ = self.global_path[current_idx + 1]
            seg_len    = math.hypot(x2 - x1, y2 - y1)
            if seg_len == 0:
                current_idx += 1
                continue
            if seg_len >= lookahead_rem:
                ratio    = lookahead_rem / seg_len
                target_x = x1 + ratio * (x2 - x1)
                target_y = y1 + ratio * (y2 - y1)
                return [target_x, target_y], self._path_heading_at_segment(current_idx, ratio)
            else:
                lookahead_rem -= seg_len
                current_idx   += 1

        tx, ty, _ = self.global_path[-1]
        if len(self.global_path) > 1:
            return [tx, ty], self.global_path[-1][2]
        return [tx, ty], self.goal_orientation

    # ═══════════════════════════════════════════════════════════════════════
    # PREDICTION — bagian dasar
    # ═══════════════════════════════════════════════════════════════════════

    def predict_trajectory(self, vx, vy, w):
        """bagian dasar"""
        trajectory = []
        x, y, theta = self.robot_pose
        steps    = int(self.prediction_time / self.dt)
        viz_step = max(1, steps // self.local_plan_size)

        for step in range(steps):
            theta    += w * self.dt
            dx_world  = vx * math.cos(theta) - vy * math.sin(theta)
            dy_world  = vx * math.sin(theta) + vy * math.cos(theta)
            x        += dx_world * self.dt
            y        += dy_world * self.dt
            if step % viz_step == 0:
                trajectory.append([x, y, theta])

        if steps > 0 and (steps - 1) % viz_step != 0:
            trajectory.append([x, y, theta])
        return trajectory

    # ═══════════════════════════════════════════════════════════════════════
    # UTILITIES — bagian dasar
    # ═══════════════════════════════════════════════════════════════════════

    def closest_point_on_segment(self, px, py, x1, y1, x2, y2):
        """bagian dasar"""
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return [x1, y1], math.hypot(px - x1, py - y1), 0.0
        t         = ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)
        t         = max(0.0, min(1.0, t))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return [closest_x, closest_y], math.hypot(px - closest_x, py - closest_y), t

    def normalize_angle(self, angle):
        """bagian dasar"""
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _interp_angle(self, a, b, t):
        return self.normalize_angle(a + self.normalize_angle(b - a) * max(0.0, min(1.0, t)))

    def _path_heading_at_segment(self, idx, t):
        if not self.global_path:
            return self.goal_orientation
        idx = max(0, min(idx, len(self.global_path) - 1))
        if idx >= len(self.global_path) - 1:
            return self.global_path[idx][2]
        return self._interp_angle(self.global_path[idx][2], self.global_path[idx + 1][2], t)

    def _localization_sigma_valid(self):
        """True jika covariance AMCL masih kecil dan finite."""
        try:
            sx = math.sqrt(max(0.0, float(self.robot_covariance[0])))
            sy = math.sqrt(max(0.0, float(self.robot_covariance[7])))
        except (TypeError, ValueError, IndexError):
            return False, float('inf'), float('inf')
        if not (math.isfinite(sx) and math.isfinite(sy)):
            return False, sx, sy
        limit = max(0.01, float(self.localization_sigma_threshold))
        return (sx <= limit and sy <= limit), sx, sy

    def _update_localization_guard(self):
        """Latch LOCALIZATION_HOLD jika AMCL sigma invalid atau pose stale.
        [FIX-LOC-B] Set self._localization_hold_reason agar sub-reason bisa di-log.
        """
        if not self.localization_guard_enabled or self.experiment_state != 'RUNNING':
            self._localization_invalid_since = None
            self._localization_valid_since = None
            self._localization_hold_active = False
            return False

        now = time.time()

        # [LOC-GUARD] Hard grace window: guard TIDAK BOLEH aktif (jalur apa pun:
        # POSE_STALE / COV_INVALID hard maupun soft) selama N detik pertama sejak
        # trial RUNNING. Mencegah false-hold saat AMCL belum konvergen di awal.
        if self._experiment_running_since is not None:
            runtime0 = now - self._experiment_running_since
            if runtime0 < self.localization_guard_min_start_s:
                self._localization_invalid_since = None
                self._localization_valid_since = None
                self._localization_hold_active = False
                self._localization_hold_reason = ''
                return False

        # [FIX-LOC-B] Cek AMCL staleness — jika AMCL tidak update > 3s saat robot bergerak
        amcl_age = now - self._last_amcl_recv_t if self._last_amcl_recv_t else 0.0
        robot_speed = math.hypot(
            getattr(self, '_last_vx', 0.0),
            getattr(self, '_last_vy', 0.0))
        if amcl_age > 3.0 and robot_speed > 0.02:
            self._localization_hold_active = True
            self._localization_hold_reason = f'POSE_STALE(age={amcl_age:.1f}s,speed={robot_speed:.3f})'
            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - self._last_locwarn > 2.0:
                self.get_logger().warn(
                    f'[{self.ns}] LOCALIZATION_HOLD — POSE_STALE: '
                    f'AMCL tidak update {amcl_age:.1f}s, robot masih bergerak '
                    f'speed={robot_speed:.3f}m/s')
                self._last_locwarn = now_s
            return True

        valid, sx, sy = self._localization_sigma_valid()
        if valid:
            self._localization_seen_valid = True
            self._localization_invalid_since = None
            if self._localization_valid_since is None:
                self._localization_valid_since = now
            if (self._localization_hold_active
                    and now - self._localization_valid_since >= self.localization_recover_s):
                self._localization_hold_active = False
                self._localization_hold_reason = ''
            return self._localization_hold_active

        hard_limit = max(float(self.localization_hard_sigma_threshold),
                         float(self.localization_sigma_threshold))
        hard_invalid = (sx >= hard_limit or sy >= hard_limit)
        if not hard_invalid:
            if self._experiment_running_since is not None:
                runtime = now - self._experiment_running_since
                if runtime < self.localization_guard_min_start_s:
                    self._localization_invalid_since = None
                    return self._localization_hold_active
            if (self.localization_guard_require_valid_once
                    and not self._localization_seen_valid):
                self._localization_invalid_since = None
                return self._localization_hold_active

        self._localization_valid_since = None
        if self._localization_invalid_since is None:
            self._localization_invalid_since = now
        if now - self._localization_invalid_since >= self.localization_invalid_hold_s:
            self._localization_hold_active = True
            # [FIX-LOC-B] Sub-reason COV_INVALID
            self._localization_hold_reason = (
                f'COV_INVALID(sx={sx:.3f},sy={sy:.3f},'
                f'limit={self.localization_sigma_threshold:.3f},'
                f'hard={hard_limit:.3f})')
            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - self._last_locwarn > 2.0:
                self.get_logger().warn(
                    f'[{self.ns}] LOCALIZATION_HOLD — COV_INVALID: '
                    f'σx={sx:.3f} σy={sy:.3f} '
                    f'> {self.localization_sigma_threshold:.3f} '
                    f'(hard={hard_limit:.3f})')
                self._last_locwarn = now_s
        return self._localization_hold_active

    def _update_localization_consistency_guard(self):
        """Hold jika AMCL tidak konsisten dengan odom dalam interval terakhir.

        [FIX-LOC-C] Desain ulang dari KUMULATIF sejak START menjadi SLIDING WINDOW
        per update AMCL. Root cause bug lama:

        - Guard lama membandingkan Δ_AMCL_total vs Δ_odom_total dari t=RUNNING.
        - Mecanum odom terakumulasi lebih cepat dari AMCL update rate (~0.7Hz).
        - Setelah 15-18s, trans_err = 0.65m > threshold 0.60m → false positive.
        - Padahal AMCL dan odom sebetulnya konsisten — odom hanya drift wajar.

        Fix: reset baseline setiap kali AMCL menerima update baru. Guard hanya
        mengecek konsistensi dalam interval pendek (max ~1.5s), bukan akumulasi.

        Threshold per-interval: 0.40m (bukan 0.60m kumulatif), cukup ketat untuk
        deteksi pose jump nyata dalam satu AMCL interval tanpa false positive drift.
        """
        if (not self.localization_consistency_guard_enabled
                or self.experiment_state != 'RUNNING'):
            self._amcl_consistency_base = None
            self._odom_consistency_base = None
            self._localization_consistency_hold = False
            return False

        if self.odom_pose is None or self.robot_pose is None:
            return self._localization_consistency_hold

        # [FIX-LOC-C] Reset baseline setiap AMCL update — KUNCI sliding window
        if getattr(self, '_amcl_pose_updated', False):
            self._amcl_pose_updated = False
            self._amcl_consistency_base = list(self.robot_pose)
            self._odom_consistency_base = list(self.odom_pose)
            # Jika hold aktif dan AMCL baru saja update dengan data valid,
            # lepaskan hold — bukti AMCL masih hidup dan koreksi sudah masuk.
            if self._localization_consistency_hold:
                self.get_logger().info(
                    f'[{self.ns}] Consistency guard RESET — AMCL update baru diterima, '
                    f'hold dilepas')
                self._localization_consistency_hold = False
                self._localization_hold_reason = ''
            return False

        if self._amcl_consistency_base is None or self._odom_consistency_base is None:
            # Inisialisasi awal (sebelum AMCL pertama update setelah RUNNING)
            self._amcl_consistency_base = list(self.robot_pose)
            self._odom_consistency_base = list(self.odom_pose)
            return False

        ax0, ay0, ath0 = self._amcl_consistency_base
        ox0, oy0, oth0 = self._odom_consistency_base
        ax, ay, ath    = self.robot_pose
        ox, oy, oth    = self.odom_pose

        amcl_dx = ax - ax0;  amcl_dy = ay - ay0
        odom_dx = ox - ox0;  odom_dy = oy - oy0
        trans_err = math.hypot(amcl_dx - odom_dx, amcl_dy - odom_dy)

        amcl_dyaw = self.normalize_angle(ath - ath0)
        odom_dyaw = self.normalize_angle(oth - oth0)
        yaw_err   = abs(self.normalize_angle(amcl_dyaw - odom_dyaw))

        # [FIX-LOC-C] Per-interval threshold: 0.40m (bukan 0.60m kumulatif).
        # Odom drift dalam 1 AMCL interval (~1.5s) seharusnya < 0.05m pada Mecanum.
        # 0.40m memberi margin 8x dari expected drift — hanya pose jump nyata yg trigger.
        per_interval_trans_thresh = 0.40
        per_interval_yaw_thresh   = self.localization_consistency_yaw_threshold

        if (trans_err >= per_interval_trans_thresh
                or yaw_err >= per_interval_yaw_thresh):
            self._localization_consistency_hold = True
            # [FIX-LOC-B] Sub-reason
            self._localization_hold_reason = (
                f'AMCL_ODOM_INCONSISTENT(dxy={trans_err:.3f}m'
                f'>{per_interval_trans_thresh:.2f}m,'
                f'dyaw={math.degrees(yaw_err):.1f}deg'
                f'>{math.degrees(per_interval_yaw_thresh):.1f}deg)'
            )
            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - self._last_consistency_warn > 2.0:
                self.get_logger().warn(
                    f'[{self.ns}] LOCALIZATION_HOLD — AMCL_ODOM_INCONSISTENT: '
                    f'dxy_err={trans_err:.3f}m (limit={per_interval_trans_thresh:.2f}m), '
                    f'dyaw_err={math.degrees(yaw_err):.1f}deg '
                    f'(limit={math.degrees(per_interval_yaw_thresh):.1f}deg) '
                    f'| amcl_d=({amcl_dx:.3f},{amcl_dy:.3f}) '
                    f'odom_d=({odom_dx:.3f},{odom_dy:.3f})')
                self._last_consistency_warn = now_s
        else:
            # Konsisten — tidak perlu reset hold di sini, biarkan AMCL update yang reset
            pass

        return self._localization_consistency_hold

    def _apply_omega_limits(self, w, force_zero=False):
        """Final omega gate: hard clamp + per-cycle slew limiter."""
        raw = float(w) if math.isfinite(float(w)) else 0.0
        self._last_omega_raw = raw

        if force_zero:
            self._last_cmd_omega = 0.0
            self._last_cmd_time = time.time()
            self._last_omega_after_clamp = 0.0
            return 0.0

        limit = abs(float(self.omega_global_limit))
        if not math.isfinite(limit) or limit <= 0.0:
            limit = abs(float(self.max_rot_vel))
        w_clamped = max(-limit, min(limit, raw))

        slew = abs(float(self.omega_slew_rate_limit))
        if math.isfinite(slew) and slew > 0.0:
            lo = self._last_cmd_omega - slew
            hi = self._last_cmd_omega + slew
            w_clamped = max(lo, min(hi, w_clamped))

        self._last_cmd_omega = w_clamped
        self._last_cmd_time = time.time()
        self._last_omega_after_clamp = w_clamped
        return w_clamped

    def _current_command_heading_error(self):
        """Heading error yang relevan untuk gate output akhir."""
        errors = [abs(float(getattr(self, '_last_path_heading_error', 0.0)))]
        if self.target_heading is not None and self.robot_pose:
            robot_theta = self.robot_pose[2]
            errors.append(abs(self.normalize_angle(self.target_heading - robot_theta)))
        return max(errors)

    def _apply_motion_mixing_guard(self, vx, vy, w, v_limit):
        """Batasi kombinasi vx+vy+omega yang berat untuk AMCL/scan matching."""
        self._last_motion_mix_guard = False
        self._last_motion_mix_reason = ''
        if not self.motion_mixing_guard_enabled:
            return vx, vy, w

        reasons = []
        abs_w = abs(w)
        heading_err = self._current_command_heading_error()
        corner_active = (
            self.corner_slowdown_enabled
            and getattr(self, '_last_corner_scale', 1.0) < 0.99)

        if abs_w >= self.mixing_omega_vy_zero_threshold and abs(vy) > 1e-4:
            vy = 0.0
            reasons.append('omega_vy0')

        if heading_err >= self.mixing_heading_error:
            scale = max(0.0, min(1.0, float(self.mixing_heading_vx_scale)))
            vx *= scale
            if abs(vy) > 1e-4:
                vy = 0.0
            reasons.append('heading_gate')

        if corner_active:
            scale = max(0.0, min(1.0, float(self.mixing_corner_vx_scale)))
            vx *= scale
            if self.mixing_corner_vy_zero and abs(vy) > 1e-4:
                vy = 0.0
            omega_limit = abs(float(self.mixing_corner_omega_limit))
            if math.isfinite(omega_limit) and omega_limit > 0.0:
                w = max(-omega_limit, min(omega_limit, w))
                self._last_cmd_omega = w
                self._last_omega_after_clamp = w
            reasons.append('corner_gate')

        speed = math.hypot(vx, vy)
        if speed > v_limit and speed > 1e-6:
            rescale = v_limit / speed
            vx *= rescale
            vy *= rescale

        if reasons:
            self._last_motion_mix_guard = True
            self._last_motion_mix_reason = '+'.join(reasons)
        return vx, vy, w

    def publish_command(self, vx, vy, w, force_zero=False,
                        ignore_priority_stop=False, override_v_limit=None):
        """Publish command dengan final speed/omega safety gate."""
        priority_hold = self.priority_stop and not ignore_priority_stop
        if (priority_hold or self._localization_hold_active
                or self._localization_consistency_hold or force_zero):
            vx, vy, w = 0.0, 0.0, 0.0
        # [FIX-LOC-B] Catat kecepatan terakhir untuk AMCL staleness check
        self._last_vx = float(vx)
        self._last_vy = float(vy)
        v_limit = min(
            self._coordination_v_limit(),
            self._ctrl_eff_vmax if self._ctrl_eff_vmax is not None else self._abs_vmax_cap(),
        )
        if override_v_limit is not None:
            mechanical_limit = max(self.max_vel_x, self.max_vel_y)
            v_limit = max(v_limit, min(float(override_v_limit), mechanical_limit))
        if v_limit <= 0.001:
            vx, vy, w = 0.0, 0.0, 0.0
        else:
            speed = math.hypot(vx, vy)
            if speed > v_limit and speed > 1e-6:
                scale = v_limit / speed
                vx *= scale
                vy *= scale
        w = self._apply_omega_limits(w, force_zero=(
            force_zero or priority_hold or self._localization_hold_active
            or self._localization_consistency_hold
            or v_limit <= 0.001))
        vx, vy, w = self._apply_motion_mixing_guard(vx, vy, w, v_limit)
        if self.angular_translation_lock_enabled and abs(w) >= self.angular_translation_lock_w:
            scale = max(0.0, min(1.0, float(self.angular_translation_lock_scale)))
            vx *= scale
            vy *= scale
        twist             = Twist()
        twist.linear.x    = float(vx)
        twist.linear.y    = float(vy)
        twist.angular.z   = float(w)
        self.cmd_pub.publish(twist)
        self.robot_vel    = [vx, vy, w]

    def publish_local_plan(self, trajectory):
        """bagian dasar"""
        wall_now = time.time()
        if wall_now - self._last_local_plan_pub < self.local_plan_publish_period_s:
            return
        self._last_local_plan_pub = wall_now

        path_msg             = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in trajectory:
            x, y, theta = point
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x    = float(x)
            ps.pose.position.y    = float(y)
            ps.pose.position.z    = 0.0
            ps.pose.orientation.z = math.sin(theta / 2.0)
            ps.pose.orientation.w = math.cos(theta / 2.0)
            path_msg.poses.append(ps)

        self.local_plan_pub.publish(path_msg)

        # [ALGO-TRACE] catat local plan ke disk robot (local_plan tidak ikut
        # di-bridge ke logger PC, jadi direkam langsung di sisi robot).
        if self.algo_trace_enabled:
            try:
                self._write_local_plan_trace(trajectory)
            except Exception as e:
                self.get_logger().warn(f'[ALGO-TRACE] tulis local_plan gagal: {e}')

    def _flush_dwa_trace(self, best_vx, best_vy, best_w,
                         rx, ry, rth, tx, ty, evmax):
        """[ALGO-TRACE] Tulis semua kandidat DWA satu siklus ke CSV.

        Tiap baris = satu kandidat (vx, vy, w) yang dievaluasi DWA, lengkap
        status (COLLISION/LETHAL/DYN_REJECT/VALID), skor, dan titik akhir
        prediksi. is_best=1 = kandidat terpilih.
        """
        self._dwa_trace_seq += 1
        t_ros = self.get_clock().now().nanoseconds / 1e9
        fname = os.path.join(self.algo_trace_dir, f'dwa_candidates_{self.ns}.csv')
        new_file = not os.path.exists(fname)
        with open(fname, 'a', newline='') as f:
            wr = csv.writer(f)
            if new_file:
                wr.writerow(['t_ros', 'cycle_seq', 'cand_id', 'vx', 'vy', 'w',
                             'status', 'c_traj', 'score',
                             'end_x', 'end_y', 'end_theta', 'is_best',
                             'robot_x', 'robot_y', 'robot_theta',
                             'target_x', 'target_y', 'effective_vmax'])
            for i, rec in enumerate(self._dwa_trace_buf):
                vx, vy, wv, status, c_traj, ex, ey, eth, score = rec
                is_best = 1 if (status == 'VALID' and vx == best_vx
                                and vy == best_vy and wv == best_w) else 0
                wr.writerow([f'{t_ros:.3f}', self._dwa_trace_seq, i,
                             f'{vx:.4f}', f'{vy:.4f}', f'{wv:.4f}', status,
                             '' if c_traj is None else f'{c_traj:.2f}',
                             '' if score is None else f'{score:.4f}',
                             f'{ex:.4f}', f'{ey:.4f}', f'{eth:.4f}', is_best,
                             f'{rx:.4f}', f'{ry:.4f}', f'{rth:.4f}',
                             f'{tx:.4f}', f'{ty:.4f}', f'{evmax:.4f}'])
        self._dwa_trace_buf = []

    def _write_local_plan_trace(self, trajectory):
        """[ALGO-TRACE] Local plan DWA per snapshot → CSV robot-side."""
        self._local_plan_trace_seq += 1
        seq = self._local_plan_trace_seq
        t_ros = self.get_clock().now().nanoseconds / 1e9
        fname = os.path.join(self.algo_trace_dir, f'local_plan_{self.ns}.csv')
        new_file = not os.path.exists(fname)
        with open(fname, 'a', newline='') as f:
            wr = csv.writer(f)
            if new_file:
                wr.writerow(['t_ros', 'plan_seq', 'point_index',
                             'point_count', 'x', 'y', 'theta'])
            pts = list(trajectory)
            for i, pt in enumerate(pts):
                x, y, theta = pt
                wr.writerow([f'{t_ros:.3f}', seq, i, len(pts),
                             f'{x:.4f}', f'{y:.4f}', f'{theta:.4f}'])

    def stop_robot(self):
        """bagian dasar"""
        self.publish_command(0.0, 0.0, 0.0, force_zero=True)
        self.robot_vel = [0.0, 0.0, 0.0]
        empty_path             = Path()
        empty_path.header.stamp    = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        self.local_plan_pub.publish(empty_path)

    # ═══════════════════════════════════════════════════════════════════════
    # [MOD-14] CORNER SLOWDOWN
    # ═══════════════════════════════════════════════════════════════════════

    def _compute_corner_scale(self):
        """Hitung faktor pengurangan kecepatan berdasarkan sudut tikungan di depan."""
        if not self.corner_slowdown_enabled:
            return 1.0
        if not self.global_path or len(self.global_path) < 2:
            return 1.0

        idx = self.current_path_index
        if idx + 1 >= len(self.global_path):
            return 1.0

        # Heading saat ini (segmen path yang sedang dilalui)
        x0, y0, _ = self.global_path[idx]
        x1, y1, _ = self.global_path[idx + 1]
        if math.hypot(x1 - x0, y1 - y0) < 1e-4:
            return 1.0
        h_now = math.atan2(y1 - y0, x1 - x0)

        # Heading setelah maju corner_slowdown_radius ke depan
        ahead = self.corner_slowdown_radius
        h_ahead = h_now
        for i in range(idx, len(self.global_path) - 1):
            x0, y0, _ = self.global_path[i]
            x1, y1, _ = self.global_path[i + 1]
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-4:
                continue
            h_ahead = math.atan2(y1 - y0, x1 - x0)
            if ahead <= seg_len:
                break
            ahead -= seg_len

        angle_change = abs(self.normalize_angle(h_ahead - h_now))
        self._last_corner_angle = angle_change

        if angle_change > self.corner_angle_threshold:
            return self.corner_speed_ratio
        return 1.0

    # ═══════════════════════════════════════════════════════════════════════
    # [MOD-15] BACKTRACKING RECOVERY
    # ═══════════════════════════════════════════════════════════════════════

    def _get_backtrack_target(self):
        """Cari titik backtrack_distance meter ke belakang pada global path."""
        if not self.global_path:
            return None
        idx = max(0, self.current_path_index)
        remaining = self.backtrack_distance

        for i in range(idx, 0, -1):
            x1, y1, _ = self.global_path[i]
            x0, y0, _ = self.global_path[i - 1]
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len >= remaining:
                t  = remaining / seg_len
                tx = x1 - t * (x1 - x0)
                ty = y1 - t * (y1 - y0)
                return (tx, ty)
            remaining -= seg_len

        # Jika habis, kembalikan titik awal path
        return (self.global_path[0][0], self.global_path[0][1])

    def _backtracking_control(self):
        """Gerakkan robot menuju backtrack_target dengan kecepatan backtracking_speed."""
        if self._backtrack_target is None:
            return 0.0, 0.0, 0.0

        robot_x, robot_y, robot_theta = self.robot_pose
        bx, by = self._backtrack_target
        dx   = bx - robot_x
        dy   = by - robot_y
        dist = math.hypot(dx, dy)

        if dist < 0.08:
            return 0.0, 0.0, 0.0

        cos_t    = math.cos(robot_theta)
        sin_t    = math.sin(robot_theta)
        vx_local = dx * cos_t + dy * sin_t
        vy_local = -dx * sin_t + dy * cos_t

        kp    = 0.8
        vx    = kp * vx_local
        vy    = kp * vy_local
        speed = math.hypot(vx, vy)
        if speed > self.backtracking_speed:
            vx *= self.backtracking_speed / speed
            vy *= self.backtracking_speed / speed

        vx = max(self.min_vel_x, min(self.max_vel_x, vx))
        vy = max(self.min_vel_y, min(self.max_vel_y, vy))
        return vx, vy, 0.0

    # ═══════════════════════════════════════════════════════════════════════
    # [MOD-18] PATH-ORDER-AWARE BLOCKING
    # ═══════════════════════════════════════════════════════════════════════

    def _path_front_blocked(self):
        """
        [MOD-18] Return True hanya jika ada obstacle NYATA di front sector.

        Prinsip path-order-aware: hanya obstacle di depan (dalam arah path)
        yang boleh memicu BLOCKED. Obstacle di samping / belakang (misalnya
        robot lain yang mengikuti di belakang) tidak boleh membuat robot depan
        berhenti — hanya robot belakang yang harus mengalah.

        Menggunakan front LiDAR sector sebagai proxy: jika sector depan bersih
        (> front_stop_dist), maka BLOCKED dari costmap/DWA bukan dari obstacle
        di depan path, dan tidak perlu memicu stop penuh.
        """
        sectors    = self._current_sectors or self.analyze_lidar_sectors()
        front_dist = sectors.get('front', float('inf'))
        self._last_front_dist    = front_dist
        self._last_front_blocker = front_dist < self.front_stop_dist
        return self._last_front_blocker

    # ═══════════════════════════════════════════════════════════════════════
    # [MOD-17] HOLONOMIC PATH TRACKER
    # ═══════════════════════════════════════════════════════════════════════

    def _holonomic_path_track(self, effective_vmax):
        """
        [MOD-17] Vector-field holonomic path tracker.

        Computes (vx, vy, w) in robot frame from:
          v_forward  — along path tangent, scaled by cos(heading_error)
          v_cte      — perpendicular correction toward path center
          omega      — heading alignment to path tangent

        Returns (vx, vy, w) robot-frame velocity, or None if path invalid.
        """
        if not self.global_path or len(self.global_path) < 2:
            return None

        robot_x, robot_y, robot_theta = self.robot_pose

        # Find closest segment around current path index
        search_start = max(0, self.current_path_index - 1)
        search_end   = min(len(self.global_path) - 1, self.current_path_index + 5)
        min_dist  = float('inf')
        best_idx  = self.current_path_index
        best_t    = 0.0

        for i in range(search_start, search_end):
            x1, y1, _ = self.global_path[i]
            x2, y2, _ = self.global_path[i + 1]
            _, dist, t = self.closest_point_on_segment(robot_x, robot_y, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_t = t

        x1, y1, _ = self.global_path[best_idx]
        x2, y2, _ = self.global_path[best_idx + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < 1e-4:
            return None

        # Unit tangent and signed CTE (positive = robot LEFT of path)
        tx = (x2 - x1) / seg_len
        ty = (y2 - y1) / seg_len
        cte_raw = tx * (robot_y - y1) - ty * (robot_x - x1)
        # [LANE NEG] Geser CTE target: robot kejar centerline + crossing_lane_offset
        # offset > 0 = kiri path, offset < 0 = kanan path (traffic convention)
        cte = cte_raw - self.crossing_lane_offset
        self._last_cte = cte

        # Heading error. Untuk mecanum fisik, translasi boleh tetap mengikuti
        # tangent path, tetapi yaw perlu diarahkan ke heading lookahead seperti
        # tracker Ilham agar robot benar-benar ikut "menghadap" belokan.
        path_heading = math.atan2(ty, tx)
        control_heading = (
            self.target_heading
            if self.target_heading is not None
            else path_heading)
        h_err = self.normalize_angle(control_heading - robot_theta)
        self._last_path_heading_error = h_err

        is_final_segment = best_idx >= len(self.global_path) - 2
        if self.final_goal_pose:
            goal_x, goal_y, goal_th = self.final_goal_pose
            dist_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        else:
            goal_x, goal_y, goal_th = self.global_path[-1]
            dist_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)

        # Pada segmen terakhir, jangan terus dorong "maju sepanjang path"
        # setelah robot dekat/terlanjur melewati goal. Arahkan langsung ke
        # titik goal agar robot bisa recover dari overshoot dan offset lateral.
        # [FIX-GOALAPPROACH] Tambah near_final_goal: kapan pun robot ≤ radius dari
        # goal akhir, langsung arahkan ke goal — tidak bergantung pada is_final_segment
        # (yang bisa beku jika current_path_index macet di segmen non-final).
        near_final_goal = (dist_goal <= self.final_goal_vector_radius)
        if (is_final_segment and (dist_goal <= self.final_goal_vector_radius or best_t > 0.92)) \
                or near_final_goal:
            dx = goal_x - robot_x
            dy = goal_y - robot_y
            vx_world = self.final_goal_vector_kp * dx
            vy_world = self.final_goal_vector_kp * dy
            speed = math.hypot(vx_world, vy_world)
            if speed > effective_vmax and speed > 1e-6:
                vx_world *= effective_vmax / speed
                vy_world *= effective_vmax / speed
            if dist_goal > self.goal_tolerance * 1.5:
                w = 0.0
            else:
                w = max(-self.max_heading_w,
                        min(self.max_heading_w,
                            self.k_heading * self.normalize_angle(goal_th - robot_theta)))
            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            vx_robot =  vx_world * cos_t + vy_world * sin_t
            vy_robot = -vx_world * sin_t + vy_world * cos_t
            vx_robot = max(self.min_vel_x, min(self._abs_vmax_cap(), vx_robot))
            vy_robot = max(self.min_vel_y, min(self.max_vel_y, vy_robot))
            return vx_robot, vy_robot, w

        # CTE correction in world frame:
        #   To correct positive CTE (robot LEFT), move RIGHT → direction (ty, -tx)
        #   v_cte_world = k_cte * cte * (ty, -tx)
        v_cte_wx = self.k_cte * cte * ty
        v_cte_wy = -self.k_cte * cte * tx
        v_cte_mag = math.hypot(v_cte_wx, v_cte_wy)
        # Klem v_cte_mag agar selalu ada sisa vmax untuk gerak maju.
        # Tanpa ini, saat effective_vmax rendah (mis. consensus floor 0.03 m/s)
        # dan max_lateral_correction=0.05, v_cte_mag bisa melebihi effective_vmax
        # → available = 0 → robot hanya bergerak lateral tanpa kemajuan.
        cte_cap = min(self.max_lateral_correction, effective_vmax * 0.7)
        if v_cte_mag > cte_cap:
            s = cte_cap / max(v_cte_mag, 1e-9)
            v_cte_wx *= s
            v_cte_wy *= s
            v_cte_mag = cte_cap

        # Forward speed: mecanum dapat mengikuti tangent path tanpa harus yaw
        # sejajar dengan path. Jika path heading tracking dimatikan, jangan
        # kurangi translasi hanya karena heading error 90 derajat.
        available   = math.sqrt(max(0.0, effective_vmax ** 2 - v_cte_mag ** 2))
        v_forward = available

        # Total world-frame velocity
        vx_world = v_forward * tx + v_cte_wx
        vy_world = v_forward * ty + v_cte_wy

        # Saat perubahan heading besar, jangan campur translasi dengan rotasi.
        # Pada robot fisik, kombinasi ini membuat AMCL mudah lepas di tikungan.
        if (self.path_heading_tracking_enabled
                and self.heading_translation_gate_enabled
                and abs(h_err) > self.heading_translation_gate):
            scale = max(0.0, min(1.0, float(self.heading_translation_gate_scale)))
            vx_world *= scale
            vy_world *= scale

        # Convert to robot frame
        cos_t    = math.cos(robot_theta)
        sin_t    = math.sin(robot_theta)
        vx_robot =  vx_world * cos_t + vy_world * sin_t
        vy_robot = -vx_world * sin_t + vy_world * cos_t

        # Heading correction
        w = max(-self.max_heading_w, min(self.max_heading_w, self.k_heading * h_err))
        if abs(h_err) < 0.30:
            w *= 0.7

        # Clamp to robot mechanical limits
        vx_robot = max(self.min_vel_x, min(self._abs_vmax_cap(), vx_robot))
        vy_robot = max(self.min_vel_y, min(self.max_vel_y, vy_robot))
        return vx_robot, vy_robot, w

    def _publish_debug_telemetry(self):
        """[MOD-13/17] Publish vmax_eff, speed_mag, mode + holonomic telemetry."""
        wall_now = time.time()
        if wall_now - self._last_debug_telemetry_pub < self.debug_telemetry_period_s:
            return
        self._last_debug_telemetry_pub = wall_now

        vx, vy = self.robot_vel[0], self.robot_vel[1]
        mag = math.hypot(vx, vy)

        state_mode_map = {
            self.STATE_IDLE:           'IDLE',
            self.STATE_ALIGNING:       'ALIGN',
            self.STATE_APPROACHING:    'APPR',
            self.STATE_FINAL_ALIGNING: 'FINAL',
        }
        if self.current_state == self.STATE_TRACKING:
            reactive     = getattr(self, 'reactive_mode', 'NORMAL')
            tmode        = getattr(self, '_last_tracking_mode', 'DWA')
            corner_active = (self.corner_slowdown_enabled
                             and getattr(self, '_last_corner_scale', 1.0) < 0.99)
            if tmode == 'HOLO':
                mode_str = 'CORN_H' if corner_active else 'HOLO'
            elif tmode == 'DYN_AVOID':
                mode_str = 'DYNAV'
            elif tmode == 'LAT_ESCAPE':
                mode_str = 'LAT'
            elif tmode == 'PEER_ESCAPE':
                mode_str = 'PEER_ESC'
            elif tmode == 'HOLO_BLK':
                mode_str = 'HOLO_B'
            else:
                mode_str = {
                    'NORMAL':       'CORN' if corner_active else 'TRACK',
                    'TRACKING':     'CORN' if corner_active else 'TRACK',
                    'AVOIDING':     'AVOID',
                    'DYN_AVOID':    'DYNAV',
                    'LAT_ESCAPE':   'LAT',
                    'PEER_ESCAPE':  'PEER_ESC',
                    'BLOCKED':      'BLOCK',
                    'STUCK_ESCAPE': 'STUCK',
                    'BACKTRACKING': 'BKTRK',
                }.get(reactive, 'CORN' if corner_active else 'TRACK')
        else:
            mode_str = state_mode_map.get(self.current_state, '?')
        if self._localization_hold_active or self._localization_consistency_hold:
            mode_str = 'LOCALIZATION_HOLD'
        elif self._last_hold_reason == 'priority_escape':
            mode_str = 'PRIO_ESC'
        elif self._last_hold_reason:
            mode_str = ('DEG_no_scan' if self._last_hold_reason == 'no_scan'
                        else f'HOLD_{self._last_hold_reason}')

        vmax_report = min(
            float(self._last_effective_vmax),
            float(self._coordination_v_limit()))
        self._last_effective_vmax = vmax_report
        self._pub_vmax_eff.publish(Float32(data=vmax_report))
        self._pub_speed_mag.publish(Float32(data=float(mag)))
        self._pub_dwa_mode.publish(String(data=mode_str))
        self._pub_omega_raw.publish(Float32(data=float(self._last_omega_raw)))
        self._pub_omega_clamped.publish(Float32(data=float(self._last_omega_after_clamp)))
        self._pub_omega_limit.publish(Float32(data=float(self.omega_global_limit)))
        self._pub_loc_hold.publish(Bool(
            data=bool(self._localization_hold_active
                      or self._localization_consistency_hold)))

        # [MOD-17] Holonomic telemetry
        self._pub_cte.publish(Float32(data=float(self._last_cte)))
        self._pub_herr.publish(Float32(data=float(self._last_path_heading_error)))
        self._pub_tmode.publish(String(data=getattr(self, '_last_tracking_mode', 'DWA')))

    def status_report(self):
        """ info vmax_consensus"""
        if not self.global_path:
            self.get_logger().info('Status: No path')
            return

        robot_x, robot_y, robot_theta = self.robot_pose

        if self.final_goal_pose:
            gx, gy, _ = self.final_goal_pose
            dist_goal = math.hypot(gx - robot_x, gy - robot_y)
        else:
            dist_goal = 0.0

        heading_err = abs(self.normalize_angle(self.target_heading - robot_theta))
        state_names = {
            self.STATE_ALIGNING:       'ALIGN',
            self.STATE_TRACKING:       'TRACK',
            self.STATE_APPROACHING:    'APPROACH',
            self.STATE_FINAL_ALIGNING: 'FINAL_ALIGN',
            self.STATE_IDLE:           'IDLE',
        }
        vmax_cons_str = (f'{self.vmax_from_consensus:.3f}'
                         if self.vmax_from_consensus is not None else 'none')
        vmax_pri_str  = (f'{self.vmax_from_priority:.3f}'
                         if self.vmax_from_priority  is not None else 'none')

        sectors = self._current_sectors or {}

        def _fmt_dist(v):
            return f'{v:.2f}' if v != float('inf') else 'inf'

        sel_vx, sel_vy = self.robot_vel[0], self.robot_vel[1]
        speed_mag = math.hypot(sel_vx, sel_vy)

        front_dist = sectors.get('front', float('inf'))

        corner_active = self._last_corner_scale < 0.99
        tmode_str = getattr(self, '_last_tracking_mode', 'DWA')
        parts = [
            f'ns={self.ns}',
            f'state={state_names.get(self.current_state, "?")}',
            f'tmode={tmode_str}',
            f'avd={self.avoidance_mode}',
            f'mode={self.reactive_mode}',
            f'front={_fmt_dist(front_dist)}',
            f'front_blk={"T" if self._last_front_blocker else "F"}',
            f'dyn_min={_fmt_dist(self._dyn_min_distance)}',
            f'dyn_rej={self._dyn_rejected_count}',
            f'peer_blk={"T" if self._last_peer_blocks_path else "F"}',
            f'peer_front={"T" if self._last_peer_in_front_sector else "F"}',
            f'holo_reason={self._last_holo_blk_reason or "-"}',
            f'goal_dist={dist_goal:.2f}m',
            f'heading_err={math.degrees(heading_err):.1f}deg',
            f'cte={self._last_cte:+.3f}m',
            f'path_herr={math.degrees(self._last_path_heading_error):.1f}deg',
            f'vmax_cons={vmax_cons_str}',
            f'vmax_pri={vmax_pri_str}',
            f'vmax_eff={self._last_effective_vmax:.3f}',
            f'corner={"ON" if corner_active else "off"}'
            f'({math.degrees(self._last_corner_angle):.0f}deg*{self._last_corner_scale:.2f})',
            f'la={self._last_lookahead_used:.2f}',
            f'vy_lim={"Y" if self._last_vy_limited else "n"}',
            f'mix_guard={self._last_motion_mix_reason if self._last_motion_mix_guard else "off"}',
            f'sel_vx={sel_vx:.3f}',
            f'sel_vy={sel_vy:.3f}',
            f'mag={speed_mag:.3f}',
            f'fault={self.fault_active}',
            f'pstop={self.priority_stop}',
        ]
        cs = self._costmap_stats
        parts.append(
            f'[costmap n={cs["valid"]} rej={cs["rejected"]} '
            f'best={cs["best_cost"]:.0f}'
            + (' NO_TRAJ]' if self._last_no_valid_trajectory else ']'))
        self.get_logger().info('[DWA] ' + ' | '.join(parts))


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ModifiedDWANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
