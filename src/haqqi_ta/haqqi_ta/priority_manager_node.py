#!/usr/bin/env python3
"""
Priority Manager Node — haqqi_ta (versi UDP)
Layer 5: Stop-and-Go Hierarchical Conflict Resolution

PERUBAHAN dari versi sebelumnya:
  - Subscriber /{ns}/amcl_pose → UDP receiver (terima pose dari udp_sender_node)
  - Publisher /{ns}/priority_stop → dua jalur:
      1. UDP langsung ke robot via _send_priority_udp()
      2. ROS topic lokal /{ns}/priority_stop (untuk experiment_logger di PC)

Logika Stop-and-Go TIDAK BERUBAH sama sekali.

Conflict zone memakai ETA prediktif. Jika telemetry DWA fresh, ETA dihitung
dari kecepatan efektif aktual robot agar owner/gate tidak terlalu optimistis
saat robot melambat karena belokan, obstacle, atau tracking path.

Predictive conflict layer menambah right-of-way sebelum robot terlalu dekat:
pairwise trajectory diprediksi beberapa detik ke depan, lalu robot low-priority
diperlambat/ditahan jika d_min_pred masuk warning/emergency. DWA tetap hanya
menerima vmax_priority dan priority_stop.

Untuk skenario crossing, right-of-way dibuat yield-first: robot prioritas rendah
melambat di zona konflik dan hanya full-stop pada hard-collision brake.
"""

import rclpy
from rclpy.node import Node
import math
import os
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from itertools import combinations

import socket
import json
import threading


ROBOT_NAMESPACES = ['robot1', 'robot2', 'robot3']

# ─────────────────────────────────────────────────────────────────────────────
# MANUAL CONFLICT ZONE DEFINITIONS — satu zona per titik konflik per skenario.
# Koordinat disesuaikan dengan scenarios.yaml. Untuk eksperimen fisik TA,
# zona manual ini dapat dipakai sebagai mode utama karena lebih deterministik
# daripada auto-zone ketika path/AMCL/UDP belum stabil.
# ─────────────────────────────────────────────────────────────────────────────
CONFLICT_ZONES_BY_SCENARIO = {
    # detect_radius : mulai hitung ETA, owner dipilih, non-owner SLOW
    # zone_radius   : batas masuk zona inti → owner dianggap OCCUPIED
    # hold_radius   : non-owner berhenti di sini (tidak boleh masuk lebih dekat)
    # clear_radius  : owner dianggap keluar zona jika dist > nilai ini
    # gap_s         : jeda setelah owner clear sebelum zona kembali IDLE
    'crossing': [
        {
            'name'          : 'junction',
            'x'             : 2.94,
            'y'             : 2.75,
            'detect_radius' : 1.60,
            'zone_radius'   : 0.65,
            'hold_radius'   : 0.85,
            'clear_radius'  : 0.95,
            'gap_s'         : 2.0,
            'source'        : 'manual',
        },
    ],
    'merge': [
        {
            # Zona entry sebelum robot masuk formasi merge di sekitar goal RViz.
            # Ditempatkan sedikit sebelum anchor agar urutan selesai sebelum
            # robot saling menutup ruang formasi.
            'name'          : 'merge_entry',
            'x'             : 3.01,
            'y'             : 2.55,
            'detect_radius' : 1.80,
            'zone_radius'   : 0.85,
            'hold_radius'   : 0.95,
            'clear_radius'  : 1.05,
            'gap_s'         : 2.0,
            'source'        : 'manual',
        },
    ],
    'convoy' : [],
    'split'  : [],
}


class ConflictZone:
    """State machine per zona konflik dengan gap-time coordination."""
    IDLE        = 'IDLE'
    APPROACH    = 'APPROACH'
    OCCUPIED    = 'OCCUPIED'
    CLEARING    = 'CLEARING'
    OWNER_STUCK = 'OWNER_STUCK'   # owner timeout TAPI masih di dalam zona

    def __init__(self, cfg: dict, owner_timeout_s: float = 10.0):
        self.name          = cfg['name']
        self.x             = cfg['x']
        self.y             = cfg['y']
        self.detect_radius = cfg.get('detect_radius', 1.60)
        self.zone_radius   = cfg.get('zone_radius',   0.65)  # batas masuk zona inti
        self.hold_radius   = cfg.get('hold_radius',   0.85)  # batas stop non-owner
        self.clear_radius  = cfg.get('clear_radius',  0.95)  # batas owner dianggap keluar
        self.gap_s         = cfg.get('gap_s',         2.0)   # jeda setelah owner clear
        self.owner_timeout = owner_timeout_s
        self.source        = cfg.get('source', 'manual')
        self.robot_pair    = cfg.get('robot_pair', '')
        self.last_seen     = time.time()

        self.state            = self.IDLE
        self.owner            = None    # ns pemegang hak masuk zona
        self.owner_entry_time = None    # wall clock saat owner jadi owner
        self.owner_d0         = None    # jarak ke zona saat owner dipilih (untuk stall detection)
        self.owner_entered    = False   # owner sudah masuk zone_radius (bukan hanya approach)
        self.last_clear_time  = None    # wall clock saat owner clear → mulai gap timer
        self.owner_stuck_since = None   # wall clock saat owner masuk OWNER_STUCK


# Lane negotiation constants
HEADON_THRESHOLD_DEG = 150.0   # heading diff > ini → head-on
LANE_OFFSET_INIT     = 0.20    # offset lateral awal (m)
LANE_OFFSET_MAX      = 0.25    # offset lateral maksimum (m)
D_CLEAR_LANE         = 1.80    # mulai lane negotiation lebih awal sebelum yield/slow
CLEAR_CYCLES_NEEDED  = 10      # siklus berturut-turut aman sebelum clear

# ───────────────────────────────────────────────────────────────────────────
# Urutan prioritas (right-of-way) — SCENARIO-BASED.
# Diurutkan dari prioritas TERTINGGI → TERENDAH; robot dengan prioritas lebih
# rendah berhenti/yield untuk yang lebih tinggi. Pair tuple: (low, high) → low
# berhenti untuk high.
#   convoy  : R1 leader > R2 > R3 tail (leader tak pernah distop follower)
#   crossing: R3 > R2 > R1 (R3 prioritas tertinggi, lihat scenarios.yaml)
#   merge   : R1 > R2 > R3 (default formation role)
#   split   : R1 > R2 > R3 (robot beda lane; priority jarang aktif)
# Override global via ROS param 'priority_order' (csv, mis. "robot3,robot2,robot1").
DEFAULT_PRIORITY_ORDER = ['robot1', 'robot2', 'robot3']

PRIORITY_ORDER_BY_SCENARIO = {
    'convoy':   ['robot1', 'robot2', 'robot3'],
    'crossing': ['robot3', 'robot2', 'robot1'],
    'merge':    ['robot1', 'robot2', 'robot3'],
    'split':    ['robot1', 'robot2', 'robot3'],
}


def priority_order_for(scenario, override=None):
    """Kembalikan urutan prioritas (tertinggi→terendah) untuk skenario."""
    if override:
        return list(override)
    return list(PRIORITY_ORDER_BY_SCENARIO.get(scenario, DEFAULT_PRIORITY_ORDER))


def build_priority_pairs(order):
    """Bangun daftar pasangan (low_prio, high_prio) dari urutan prioritas.
    order[0]=tertinggi. Robot yang lebih belakang di list = low (yang yield)."""
    pairs = []
    for hi in range(len(order)):
        for lo in range(hi + 1, len(order)):
            pairs.append((order[lo], order[hi]))   # (low, high)
    return pairs


# ───────────────────────────────────────────────────────────────────────────
# Radius keselamatan per-skenario (override d_emergency / d_warning / d_clear).
# Kosongkan / hilangkan key → skenario itu pakai nilai global default.
# Isi HANYA skenario yang butuh ambang berbeda. Syarat: d_emergency < d_warning
# < d_clear. Bisa juga di-override saat runtime lewat ROS param biasa.
# Contoh (di-comment): crossing pakai zona lebih lebar karena lintasan menyilang.
SAFETY_RADII_BY_SCENARIO = {
    # 'crossing': {'d_emergency': 1.30, 'd_warning': 1.85, 'd_clear': 2.05},
    'convoy':   {'d_emergency': 0.80, 'd_warning': 1.20, 'd_clear': 1.45},   # [CONVOY-SPACING] formasi beriringan, zona lebih rapat (>d_hard_collision 0.65)
}

# Port PC Master listen pose dari tiap robot (sama dengan consensus_node)
RECV_PORT_MAP = {
    'robot1': 9031,
    'robot2': 9032,
    'robot3': 9033,
}

# Port tiap robot listen priority_stop dari PC Master
SEND_PORT_MAP = {
    'robot1': 9011,
    'robot2': 9012,
    'robot3': 9013,
}


class PairState:
    CLEAR     = 'CLEAR'
    WARNING   = 'WARNING'
    EMERGENCY = 'EMERGENCY'
    OVERRIDE  = 'OVERRIDE'

    def __init__(self, low_prio_ns, high_prio_ns):
        self.low_ns            = low_prio_ns
        self.high_ns           = high_prio_ns
        self.state             = self.CLEAR
        self.stop_start_time   = None
        self.override_active   = False
        self.override_start_time = None


class PriorityManagerNode(Node):
    def __init__(self):
        super().__init__('priority_manager_node')

        # ── Parameter (identik versi lama) ────────────────────────────────
        self.declare_parameter('d_emergency',       1.15)
        self.declare_parameter('d_warning',         1.70)
        self.declare_parameter('d_clear',           1.90)
        self.declare_parameter('d_hard_collision',  0.65)  # last-resort: stop kedua robot
        self.declare_parameter('t_max_stop',        4.0)
        self.declare_parameter('t_override',        3.0)
        self.declare_parameter('v_warning_ratio',   0.20)
        self.declare_parameter('check_rate',       10.0)
        self.declare_parameter('v_nominal',         0.15)
        # [FIX-PRIOCAP] Plafon vmax baseline saat TIDAK ada konflik. Sebelumnya baseline
        # dikunci ke v_nominal (0.30) sehingga perintah catch-up consensus (>0.30) selalu
        # dipotong oleh min(consensus, vmax_priority, max_vel_x). Set = max_vel_x DWA (0.50)
        # agar consensus yang menentukan kecepatan jelajah/catch-up. Throttle konflik/
        # warning/emergency tetap berlaku karena memakai min() di bawah baseline ini.
        self.declare_parameter('vmax_priority_ceiling', 0.50)
        self.declare_parameter('startup_grace',     3.0)
        self.declare_parameter('stale_timeout',     1.5)   # detik — pose lebih tua dari ini diabaikan

        # ── Conflict zone parameters ──────────────────────────────────────
        self.declare_parameter('scenario',               'convoy')  # nama skenario aktif
        self.declare_parameter('v_conflict_slow',        0.06)      # m/s — kecepatan non-owner
        self.declare_parameter('owner_timeout_s',        7.0)       # s — batas waktu owner zona [FIX-DEADLOCK] 10->7 putus deadlock merge lebih cepat
        self.declare_parameter('approach_stall_timeout_s', 8.0)    # s — robot stuck di APPROACH
        self.declare_parameter('approach_progress_min_m',  0.15)   # m — min kemajuan dalam stall window
        self.declare_parameter('owner_cooldown_s',          15.0)  # s — cooldown setelah timeout/stall
        self.declare_parameter('owner_stuck_release_s',      8.0)  # s — tambahan tahan saat OWNER_STUCK sebelum paksa release (anti-deadlock)
        self.declare_parameter('priority_order', '')  # csv override urutan prioritas (kosong = scenario-based, mis. "robot3,robot2,robot1")
        self.declare_parameter('final_goal_proximity',      0.25)  # m — robot dianggap sudah final
        # [PSTOP-OFF] Gerbang priority-stop. True=normal. False=matikan SEMUA stop & throttle
        # dari priority pair + conflict-zone + predictive (anti-backlash → spread minimum, mis. merge).
        # Bisa di-override per-skenario lewat flag 'priority_stop_enabled' di scenarios.yaml.
        self.declare_parameter('priority_stop_enabled',        True)
        # Rem darurat hard-collision (d_hard_collision). Tetap aktif walau priority_stop_enabled=False,
        # kecuali sengaja dimatikan (TIDAK disarankan untuk robot fisik).
        self.declare_parameter('hard_collision_brake_enabled', True)
        self.declare_parameter('lane_negotiation_enabled', True)    # False → skip head-on offset
        self.declare_parameter('auto_conflict_zone_enabled', True)
        self.declare_parameter('path_horizon_m', 3.0)
        self.declare_parameter('path_sample_step_m', 0.10)
        self.declare_parameter('conflict_path_distance', 0.45)
        self.declare_parameter('conflict_cluster_radius', 0.50)
        self.declare_parameter('auto_zone_radius', 0.75)
        self.declare_parameter('auto_detect_radius', 1.80)
        self.declare_parameter('auto_hold_radius', 0.90)
        self.declare_parameter('auto_clear_radius', 1.05)
        self.declare_parameter('auto_gap_s', 2.0)
        self.declare_parameter('min_conflict_angle_deg', 45.0)
        self.declare_parameter('ignore_near_goal_radius', 0.0)
        self.declare_parameter('auto_zone_ttl_s', 2.0)
        self.declare_parameter('feasibility_aware_eta_enabled', True)
        self.declare_parameter('eta_v_eff_floor', 0.03)
        self.declare_parameter('eta_v_eff_timeout_s', 1.0)
        self.declare_parameter('predictive_conflict_enabled', True)
        self.declare_parameter('prediction_horizon_s', 3.0)
        self.declare_parameter('prediction_dt_s', 0.2)
        self.declare_parameter('prediction_warning_dist', 0.0)
        self.declare_parameter('prediction_emergency_dist', 0.0)
        self.declare_parameter('prediction_slow_vmax', 0.0)
        self.declare_parameter('prediction_stop_ttc_s', 1.2)
        self.declare_parameter('dynamic_zone_priority_enabled', True)
        self.declare_parameter('dynamic_waiting_weight', 1.0)
        # [M5] Priority order berbasis ETA (vs tabel statis)
        self.declare_parameter('priority_mode', 'eta')            # 'eta' atau 'static'
        self.declare_parameter('agent_failure_detection_enabled', True)
        self.declare_parameter('priority_recompute_period_s', 1.0)
        self.declare_parameter('priority_switch_hysteresis_s', 2.0)
        self.declare_parameter('agent_stall_window_s', 4.0)
        self.declare_parameter('agent_stall_eps_m', 0.05)
        self.declare_parameter('crossing_slow_only_enabled', True)
        self.declare_parameter('path_debug_period_s', 1.0)
        self.declare_parameter('prediction_debug_period_s', 0.5)

        # IP tiap robot untuk kirim priority_stop
        # [MOD-IPENV] IP robot bisa diubah dari satu tempat lewat env var (opsional)
        self.declare_parameter('robot1_ip', os.environ.get('ROBOT1_IP', '192.168.0.91'))
        self.declare_parameter('robot2_ip', os.environ.get('ROBOT2_IP', '192.168.0.88'))
        self.declare_parameter('robot3_ip', os.environ.get('ROBOT3_IP', '192.168.0.82'))

        self.d_emergency      = self.get_parameter('d_emergency').value
        self.d_warning        = self.get_parameter('d_warning').value
        self.d_clear          = self.get_parameter('d_clear').value
        self.d_hard_collision = self.get_parameter('d_hard_collision').value
        # Simpan nilai dasar global; override per-skenario diterapkan di _apply_safety_radii
        self._d_emergency_base = self.d_emergency
        self._d_warning_base   = self.d_warning
        self._d_clear_base     = self.d_clear
        self.t_max_stop       = self.get_parameter('t_max_stop').value
        self.t_override       = self.get_parameter('t_override').value
        self.v_warning_ratio  = self.get_parameter('v_warning_ratio').value
        self.check_rate       = self.get_parameter('check_rate').value
        self.v_nominal        = self.get_parameter('v_nominal').value
        self.vmax_priority_ceiling = self.get_parameter('vmax_priority_ceiling').value  # [FIX-PRIOCAP]
        self.startup_grace    = self.get_parameter('startup_grace').value
        self.stale_timeout    = self.get_parameter('stale_timeout').value
        self.scenario                 = self.get_parameter('scenario').value
        self.priority_stop_enabled        = bool(self.get_parameter('priority_stop_enabled').value)
        self.hard_collision_brake_enabled = bool(self.get_parameter('hard_collision_brake_enabled').value)
        # Override flag pstop dari scenarios.yaml (mis. merge → priority_stop_enabled: false)
        self._apply_scenario_pstop_flags()
        self.v_conflict_slow          = self.get_parameter('v_conflict_slow').value
        self.owner_timeout_s          = self.get_parameter('owner_timeout_s').value
        self.approach_stall_timeout_s = self.get_parameter('approach_stall_timeout_s').value
        self.approach_progress_min_m  = self.get_parameter('approach_progress_min_m').value
        self.owner_cooldown_s         = self.get_parameter('owner_cooldown_s').value
        self.owner_stuck_release_s    = self.get_parameter('owner_stuck_release_s').value
        _po_raw = str(self.get_parameter('priority_order').value or '').strip()
        self.priority_order_override = (
            [r.strip() for r in _po_raw.split(',') if r.strip()] if _po_raw else None)
        self.final_goal_proximity     = self.get_parameter('final_goal_proximity').value
        self.lane_negotiation_enabled = self.get_parameter('lane_negotiation_enabled').value
        self.auto_conflict_zone_enabled = self.get_parameter('auto_conflict_zone_enabled').value
        self.path_horizon_m = self.get_parameter('path_horizon_m').value
        self.path_sample_step_m = self.get_parameter('path_sample_step_m').value
        self.conflict_path_distance = self.get_parameter('conflict_path_distance').value
        self.conflict_cluster_radius = self.get_parameter('conflict_cluster_radius').value
        self.auto_zone_radius = self.get_parameter('auto_zone_radius').value
        self.auto_detect_radius = self.get_parameter('auto_detect_radius').value
        self.auto_hold_radius = self.get_parameter('auto_hold_radius').value
        self.auto_clear_radius = self.get_parameter('auto_clear_radius').value
        self.auto_gap_s = self.get_parameter('auto_gap_s').value
        self.min_conflict_angle = math.radians(
            self.get_parameter('min_conflict_angle_deg').value)
        ignore_goal = self.get_parameter('ignore_near_goal_radius').value
        self.ignore_near_goal_radius = (
            ignore_goal if ignore_goal > 0.0 else self.final_goal_proximity)
        self.auto_zone_ttl_s = self.get_parameter('auto_zone_ttl_s').value
        self.feasibility_aware_eta_enabled = bool(
            self.get_parameter('feasibility_aware_eta_enabled').value)
        self.eta_v_eff_floor = max(0.01, float(
            self.get_parameter('eta_v_eff_floor').value))
        self.eta_v_eff_timeout_s = max(0.1, float(
            self.get_parameter('eta_v_eff_timeout_s').value))
        self.predictive_conflict_enabled = bool(
            self.get_parameter('predictive_conflict_enabled').value)
        self.prediction_horizon_s = max(0.2, float(
            self.get_parameter('prediction_horizon_s').value))
        self.prediction_dt_s = max(0.05, float(
            self.get_parameter('prediction_dt_s').value))
        pred_warn = float(self.get_parameter('prediction_warning_dist').value)
        pred_emg = float(self.get_parameter('prediction_emergency_dist').value)
        pred_slow = float(self.get_parameter('prediction_slow_vmax').value)
        self._pred_warn_auto = not (pred_warn > 0.0)  # auto-derive dari d_warning
        self._pred_emg_auto  = not (pred_emg > 0.0)   # auto-derive dari d_emergency
        self.prediction_warning_dist = pred_warn if pred_warn > 0.0 else self.d_warning
        self.prediction_emergency_dist = pred_emg if pred_emg > 0.0 else self.d_emergency
        self.prediction_slow_vmax = pred_slow if pred_slow > 0.0 else self.v_conflict_slow
        self.prediction_stop_ttc_s = max(0.0, float(
            self.get_parameter('prediction_stop_ttc_s').value))
        self.dynamic_zone_priority_enabled = bool(
            self.get_parameter('dynamic_zone_priority_enabled').value)
        self.dynamic_waiting_weight = max(0.0, float(
            self.get_parameter('dynamic_waiting_weight').value))
        # [M5] Parameter priority ETA-based
        self.priority_mode = (str(self.get_parameter('priority_mode').value)
                              .strip().lower() or 'eta')
        self.agent_failure_detection_enabled = bool(
            self.get_parameter('agent_failure_detection_enabled').value)
        self.priority_recompute_period_s = max(0.2, float(
            self.get_parameter('priority_recompute_period_s').value))
        self.priority_switch_hysteresis_s = max(0.0, float(
            self.get_parameter('priority_switch_hysteresis_s').value))
        self.agent_stall_window_s = max(0.5, float(
            self.get_parameter('agent_stall_window_s').value))
        self.agent_stall_eps_m = max(0.0, float(
            self.get_parameter('agent_stall_eps_m').value))
        self.crossing_slow_only_enabled = bool(
            self.get_parameter('crossing_slow_only_enabled').value)
        self.path_debug_period_s = max(0.1, float(
            self.get_parameter('path_debug_period_s').value))
        self.prediction_debug_period_s = max(0.1, float(
            self.get_parameter('prediction_debug_period_s').value))
        self._owner_cooldown          = {}  # ns → wall_time_until (jangan pilih sebagai owner)
        self._final_goals             = self._load_final_goals()  # {ns: (gx, gy)}

        self.robot_ip = {
            'robot1': self.get_parameter('robot1_ip').value,
            'robot2': self.get_parameter('robot2_ip').value,
            'robot3': self.get_parameter('robot3_ip').value,
        }

        # Terapkan radius keselamatan sesuai skenario aktif (overlay + validasi)
        self._apply_safety_radii(self.scenario)

        # ── State (identik versi lama) ────────────────────────────────────
        self._state_lock      = threading.RLock()
        self.robot_poses      = {ns: None  for ns in ROBOT_NAMESPACES}
        self.last_pose_update = {ns: 0.0   for ns in ROBOT_NAMESPACES}  # wall clock
        self.robot_goal_reached = {ns: False for ns in ROBOT_NAMESPACES}
        self.robot_paths      = {ns: []    for ns in ROBOT_NAMESPACES}
        self.robot_path_source = {ns: ''   for ns in ROBOT_NAMESPACES}
        self.robot_path_update = {ns: 0.0  for ns in ROBOT_NAMESPACES}
        self.robot_dwa_vmax_eff = {ns: None for ns in ROBOT_NAMESPACES}
        self.robot_dwa_speed_mag = {ns: None for ns in ROBOT_NAMESPACES}
        self.robot_dwa_metric_update = {ns: 0.0 for ns in ROBOT_NAMESPACES}
        self._zone_wait_since = {ns: None for ns in ROBOT_NAMESPACES}
        self._last_prediction_debug = {}
        self._predictive_stop_reason = {ns: '' for ns in ROBOT_NAMESPACES}
        # Priority order scenario-based (lihat PRIORITY_ORDER_BY_SCENARIO)
        self.priority_order = priority_order_for(
            self.scenario, self.priority_order_override)
        self.all_pairs = build_priority_pairs(self.priority_order)
        self._hard_stop_since = {pair: None for pair in self.all_pairs}
        # [M5] State recompute priority ETA-based
        self._last_prio_recompute_t = 0.0
        self._pending_prio_order    = None
        self._pending_prio_since    = 0.0
        self._prio_motion_ref       = {}
        self._prio_motion_t         = {}
        self._latest_conflict_zone_detail_payload = {'zones': [], 't': time.time()}
        self._last_path_debug_pub = 0.0
        self._last_prediction_debug_pub = 0.0
        self.pair_states    = {
            (low, high): PairState(low, high)
            for low, high in self.all_pairs
        }
        self.robot_stop    = {ns: False for ns in ROBOT_NAMESPACES}
        self.robot_vmax    = {ns: self.v_nominal for ns in ROBOT_NAMESPACES}
        self.stop_events   = []
        self._active_stops = {ns: None for ns in ROBOT_NAMESPACES}
        self.experiment_active  = False
        self._experiment_start_time = None  # waktu experiment_state jadi RUNNING

        # ── Conflict zone state ───────────────────────────────────────────
        self.manual_conflict_zones: list[ConflictZone] = self._load_conflict_zones(self.scenario)
        self.auto_conflict_zones: list[ConflictZone] = []
        self.conflict_zones: list[ConflictZone] = list(self.manual_conflict_zones)

        # ── [LANE NEG] Lane negotiation state ────────────────────────────────
        self.lane_offset        = {ns: 0.0   for ns in ROBOT_NAMESPACES}
        self.clear_cycles       = {ns: 0     for ns in ROBOT_NAMESPACES}
        self.negotiation_active = {ns: False  for ns in ROBOT_NAMESPACES}

        # ── Publisher priority_stop sebagai ROS topic (untuk experiment_logger) ──
        self.priority_stop_pub = {
            ns: self.create_publisher(Bool, f'/{ns}/priority_stop', 10)
            for ns in ROBOT_NAMESPACES
        }
        self.vmax_priority_pub = {
            ns: self.create_publisher(Float32, f'/{ns}/vmax_priority', 10)
            for ns in ROBOT_NAMESPACES
        }

        # ── Publisher conflict zone state (event: perubahan state) ──────────
        self._conflict_state_pub = self.create_publisher(
            String, '/conflict_zone_state', 10)

        # ── Publisher conflict zone detail (per-tick: d, eta, cmd tiap robot) ─
        self._conflict_detail_pub = self.create_publisher(
            String, '/conflict_zone_detail', 10)

        self._path_debug_pub = self.create_publisher(
            String, '/path_debug', 10)
        self._prediction_debug_pub = self.create_publisher(
            String, '/predictive_conflict_debug', 10)

        # ── UDP: Kirim priority_stop ke tiap robot ────────────────────────
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── UDP: Terima pose dari tiap robot ──────────────────────────────
        # Port 9031-9033 berbeda dari consensus_node (9001-9003) — tidak ada sharing.
        for ns in ROBOT_NAMESPACES:
            port = RECV_PORT_MAP[ns]
            t = threading.Thread(
                target=self._udp_pose_listener,
                args=(ns, port),
                daemon=True)
            t.start()

        # ── Subscriber /experiment_state heartbeat ────────────────────────
        self.create_subscription(
            String, '/experiment_state',
            self._experiment_state_cb, 10)
        self.create_subscription(
            String, '/experiment_scenario',
            self._experiment_scenario_cb, 10)

        # ── Subscriber /start_signal Bool ────────────────────────────────
        signal_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Bool, '/start_signal',
            self.start_signal_callback, signal_qos)

        for ns in ROBOT_NAMESPACES:
            self.create_subscription(
                Path,
                f'/{ns}/plan',
                lambda msg, n=ns: self._path_cb(msg, n), 10)

        # ── Timer (identik versi lama) ────────────────────────────────────
        period = 1.0 / self.check_rate
        self.create_timer(period, self.priority_loop)
        self.create_timer(1.0, self.status_report)

        self.get_logger().info(
            f'Priority Manager (UDP) ready | '
            f'd_hard={self.d_hard_collision}m | d_emg={self.d_emergency}m | '
            f'd_warn={self.d_warning}m | d_clr={self.d_clear}m | '
            f'v_warn_ratio={self.v_warning_ratio} | T_max={self.t_max_stop}s | '
            f'predictive={self.predictive_conflict_enabled}')

    # ═══════════════════════════════════════════════════════════════════════
    # LANE NEGOTIATION — head-on detection & lateral offset
    # ═══════════════════════════════════════════════════════════════════════

    def _get_path_heading(self, ns):
        """Ambil heading arah lintasan/gerak; fallback ke pose yaw."""
        pose = self.robot_poses.get(ns)
        if pose is None:
            return 0.0

        forward = self._forward_path_points(ns)
        if len(forward) >= 2:
            rx, ry = pose[0], pose[1]
            for x, y in forward[1:]:
                dx = x - rx
                dy = y - ry
                if math.hypot(dx, dy) >= 0.15:
                    return math.atan2(dy, dx)
            x0, y0 = forward[0]
            x1, y1 = forward[-1]
            dx = x1 - x0
            dy = y1 - y0
            if math.hypot(dx, dy) > 1e-3:
                return math.atan2(dy, dx)

        return pose[2] if len(pose) >= 3 else 0.0

    def _heading_diff_deg(self, ns_a, ns_b):
        ha = self._get_path_heading(ns_a)
        hb = self._get_path_heading(ns_b)
        return abs(math.degrees(
            math.atan2(math.sin(ha - hb), math.cos(ha - hb))))

    def _ignore_parallel_priority_pair(self, low_ns, high_ns, dist):
        """Convoy/split: robot di lane paralel tidak perlu di-stop satu sama lain.
        Priority stop hanya berlaku saat heading berbeda (ada risiko tabrakan nyata).
        Hard-collision safety brake tetap aktif sebagai last-resort.
        """
        if self.scenario not in ('convoy', 'split'):
            return False
        if dist <= self.d_hard_collision:
            return False
        return self._heading_diff_deg(low_ns, high_ns) < 35.0

    def _heading_offset(self, heading):
        """
        Tentukan offset berdasarkan arah gerak robot.
        crossing_lane_offset < 0 = geser kanan path (traffic convention: keep right).
        Robot vertikal tidak perlu geser (offset = 0.0).
        """
        if abs(math.cos(heading)) > 0.7:
            return -LANE_OFFSET_INIT   # horizontal → geser kanan path
        return 0.0                     # vertikal → tidak geser

    def _negotiate_lane(self, ns_a, ns_b):
        """Deteksi head-on dan terapkan offset berdasarkan arah gerak masing-masing robot."""
        ha = self._get_path_heading(ns_a)
        hb = self._get_path_heading(ns_b)
        diff_deg = abs(math.degrees(
            math.atan2(math.sin(ha - hb), math.cos(ha - hb))))

        if diff_deg < HEADON_THRESHOLD_DEG:
            return

        # Head-on terdeteksi — offset ditentukan per-robot dari heading masing-masing
        target_a = self._heading_offset(ha)
        target_b = self._heading_offset(hb)

        if not self.negotiation_active[ns_a]:
            self.negotiation_active[ns_a] = True
            self.negotiation_active[ns_b] = True
            self.lane_offset[ns_a] = target_a
            self.lane_offset[ns_b] = target_b
            self.clear_cycles[ns_a] = 0
            self.clear_cycles[ns_b] = 0
            self.get_logger().info(
                f'[LANE NEG] Head-on {ns_a}↔{ns_b} (Δhdg={diff_deg:.1f}°) '
                f'→ {ns_a}={target_a:.2f}m {ns_b}={target_b:.2f}m')
        else:
            # Geser menuju batas maksimum per heading
            max_a = -LANE_OFFSET_MAX if abs(math.cos(ha)) > 0.7 else 0.0
            max_b = -LANE_OFFSET_MAX if abs(math.cos(hb)) > 0.7 else 0.0
            if self.lane_offset[ns_a] > max_a:
                self.lane_offset[ns_a] = max(self.lane_offset[ns_a] - 0.01, max_a)
            if self.lane_offset[ns_b] > max_b:
                self.lane_offset[ns_b] = max(self.lane_offset[ns_b] - 0.01, max_b)
        self.clear_cycles[ns_a] = 0
        self.clear_cycles[ns_b] = 0

    def _clear_negotiation(self, ns_a, ns_b):
        """Naikkan clear_cycles; reset offset jika cukup siklus aman."""
        for ns in (ns_a, ns_b):
            self.clear_cycles[ns] += 1
            if self.clear_cycles[ns] >= CLEAR_CYCLES_NEEDED:
                if self.negotiation_active[ns]:
                    self.negotiation_active[ns] = False
                    self.lane_offset[ns]  = 0.0
                    self.clear_cycles[ns] = 0
                    self.get_logger().info(
                        f'[LANE NEG] {ns} clear — offset reset ke 0')

    # ═════════════════════════════════════════════════════════���═════════════
    # CONFLICT ZONE COORDINATION
    # ═══════════════════════════════════════════════════════════════════════

    def _load_conflict_zones(self, scenario: str) -> list:
        cfgs = CONFLICT_ZONES_BY_SCENARIO.get(scenario, [])
        return [ConflictZone(cfg, self.owner_timeout_s) for cfg in cfgs]

    def _path_cb(self, msg: Path, ns: str):
        with self._state_lock:
            self.robot_paths[ns] = [
                (float(p.pose.position.x), float(p.pose.position.y))
                for p in msg.poses
            ]
            if len(self.robot_paths[ns]) >= 2:
                self.robot_path_source[ns] = 'topic'
                self.robot_path_update[ns] = time.time()

    def _publish_path_debug(self):
        wall_now = time.time()
        if wall_now - self._last_path_debug_pub < self.path_debug_period_s:
            return
        self._last_path_debug_pub = wall_now
        paths = {}
        for ns in ROBOT_NAMESPACES:
            path = self.robot_paths.get(ns, [])
            paths[ns] = {
                'point_count': len(path),
                'source': self.robot_path_source.get(ns, ''),
                'age_s': (
                    wall_now - self.robot_path_update[ns]
                    if self.robot_path_update.get(ns, 0.0) > 0.0 else -1.0),
                'has_forward_path': len(self._forward_path_points(ns)) >= 2,
            }
        msg = String()
        msg.data = json.dumps({
            't': wall_now,
            'auto_conflict_zone_enabled': self.auto_conflict_zone_enabled,
            'auto_zone_count': len(self.auto_conflict_zones),
            'active_zone_count': len(self.conflict_zones),
            'paths': paths,
        })
        self._path_debug_pub.publish(msg)

    def _nearest_path_index(self, ns: str):
        pose = self.robot_poses.get(ns)
        path = self.robot_paths.get(ns, [])
        if pose is None or len(path) < 2:
            return None
        best_i = 0
        best_d = float('inf')
        for i, (x, y) in enumerate(path):
            d = math.hypot(pose[0] - x, pose[1] - y)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def _forward_path_points(self, ns: str):
        path = self.robot_paths.get(ns, [])
        start = self._nearest_path_index(ns)
        if start is None:
            return []
        points = []
        accum = 0.0
        last = path[start]
        points.append((last[0], last[1], start))
        step_gate = 0.0
        for i in range(start + 1, len(path)):
            p = path[i]
            seg = math.hypot(p[0] - last[0], p[1] - last[1])
            accum += seg
            step_gate += seg
            last = p
            if step_gate >= self.path_sample_step_m:
                points.append((p[0], p[1], i))
                step_gate = 0.0
            if accum >= self.path_horizon_m:
                break
        return points

    def _path_dir(self, ns: str, idx: int):
        path = self.robot_paths.get(ns, [])
        if len(path) < 2:
            pose = self.robot_poses.get(ns) or [0.0, 0.0, 0.0]
            return math.cos(pose[2]), math.sin(pose[2])
        i = max(0, min(idx, len(path) - 2))
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        d = math.hypot(x2 - x1, y2 - y1)
        if d < 1e-4:
            pose = self.robot_poses.get(ns) or [0.0, 0.0, 0.0]
            return math.cos(pose[2]), math.sin(pose[2])
        return (x2 - x1) / d, (y2 - y1) / d

    def _near_final_goal(self, x: float, y: float) -> bool:
        for gx, gy in self._final_goals.values():
            if math.hypot(x - gx, y - gy) < self.ignore_near_goal_radius:
                return True
        return False

    def _make_auto_zone(self, name, pair, center):
        cfg = {
            'name': name,
            'x': center[0],
            'y': center[1],
            'detect_radius': self.auto_detect_radius,
            'zone_radius': self.auto_zone_radius,
            'hold_radius': self.auto_hold_radius,
            'clear_radius': self.auto_clear_radius,
            'gap_s': self.auto_gap_s,
            'source': 'auto',
            'robot_pair': pair,
        }
        return ConflictZone(cfg, self.owner_timeout_s)

    def _sync_auto_conflict_zones(self):
        if not self.auto_conflict_zone_enabled:
            self.conflict_zones = list(self.manual_conflict_zones)
            return

        candidates = []
        forward = {ns: self._forward_path_points(ns) for ns in ROBOT_NAMESPACES}
        for ns_a, ns_b in combinations(ROBOT_NAMESPACES, 2):
            pts_a = forward.get(ns_a, [])
            pts_b = forward.get(ns_b, [])
            if not pts_a or not pts_b:
                continue
            for ax, ay, ai in pts_a:
                for bx, by, bi in pts_b:
                    path_dist = math.hypot(ax - bx, ay - by)
                    merge_like = (
                        self.scenario == 'merge'
                        and path_dist <= max(self.conflict_path_distance,
                                             self.auto_zone_radius)
                    )
                    if path_dist > self.conflict_path_distance and not merge_like:
                        continue
                    mx = 0.5 * (ax + bx)
                    my = 0.5 * (ay + by)
                    if self._near_final_goal(mx, my):
                        continue
                    da = self._path_dir(ns_a, ai)
                    db = self._path_dir(ns_b, bi)
                    dot = max(-1.0, min(1.0, da[0] * db[0] + da[1] * db[1]))
                    angle = math.acos(dot)
                    if angle < self.min_conflict_angle and not merge_like:
                        continue
                    candidates.append((mx, my, f'{ns_a}-{ns_b}'))

        clusters = []
        for x, y, pair in candidates:
            placed = False
            for cluster in clusters:
                cx = sum(p[0] for p in cluster['points']) / len(cluster['points'])
                cy = sum(p[1] for p in cluster['points']) / len(cluster['points'])
                if math.hypot(x - cx, y - cy) <= self.conflict_cluster_radius:
                    cluster['points'].append((x, y))
                    cluster['pairs'].add(pair)
                    placed = True
                    break
            if not placed:
                clusters.append({'points': [(x, y)], 'pairs': {pair}})

        now = time.time()
        detected = []
        for idx, cluster in enumerate(clusters):
            cx = sum(p[0] for p in cluster['points']) / len(cluster['points'])
            cy = sum(p[1] for p in cluster['points']) / len(cluster['points'])
            pair = '+'.join(sorted(cluster['pairs']))
            name = f'auto_{idx}_{pair}'
            existing = None
            for cz in self.auto_conflict_zones:
                if cz.robot_pair == pair and math.hypot(cz.x - cx, cz.y - cy) <= self.conflict_cluster_radius:
                    existing = cz
                    break
            if existing is None:
                existing = self._make_auto_zone(name, pair, (cx, cy))
            else:
                existing.x = cx
                existing.y = cy
                existing.name = name
            existing.last_seen = now
            detected.append(existing)

        keep = []
        for cz in self.auto_conflict_zones:
            still_detected = any(cz is dz for dz in detected)
            hold_state = cz.state in (ConflictZone.OCCUPIED, ConflictZone.CLEARING,
                                      ConflictZone.OWNER_STUCK)
            if still_detected or hold_state or (now - cz.last_seen) <= self.auto_zone_ttl_s:
                keep.append(cz)
        for cz in detected:
            if cz not in keep:
                keep.append(cz)
        self.auto_conflict_zones = keep
        self.conflict_zones = self.auto_conflict_zones if self.auto_conflict_zones else list(self.manual_conflict_zones)

    def _apply_scenario_pstop_flags(self):
        """Override priority_stop_enabled / hard_collision_brake_enabled dari flag
        per-skenario di scenarios.yaml bila tersedia. Skenario tanpa flag memakai
        nilai parameter (default True), jadi convoy/crossing/split tidak berubah."""
        try:
            pkg_dir   = get_package_share_directory('haqqi_ta')
            yaml_path = os.path.join(pkg_dir, 'param', 'scenarios.yaml')
            with open(yaml_path) as f:
                data = yaml.safe_load(f) or {}
            scenario_data = data.get('scenarios', {}).get(self.scenario, {}) or {}
        except Exception as e:
            self.get_logger().warn(f'[PSTOP-OFF] gagal baca scenarios.yaml: {e}')
            return

        def _as_bool(v, default):
            if v is None:
                return default
            return str(v).strip().lower() in ('1', 'true', 'yes', 'on')

        if 'priority_stop_enabled' in scenario_data:
            self.priority_stop_enabled = _as_bool(
                scenario_data.get('priority_stop_enabled'), self.priority_stop_enabled)
        if 'hard_collision_brake_enabled' in scenario_data:
            self.hard_collision_brake_enabled = _as_bool(
                scenario_data.get('hard_collision_brake_enabled'),
                self.hard_collision_brake_enabled)

        if not self.priority_stop_enabled:
            self.get_logger().warn(
                f"[PSTOP-OFF] scenario '{self.scenario}': priority-stop DINONAKTIFKAN "
                f"(anti-backlash, spread minimum) | rem hard-collision="
                f"{'ON' if self.hard_collision_brake_enabled else 'OFF'}")

    def _load_final_goals(self) -> dict:
        """Baca posisi goal akhir tiap robot dari scenarios.yaml/formation override."""
        try:
            pkg_dir   = get_package_share_directory('haqqi_ta')
            yaml_path = os.path.join(pkg_dir, 'param', 'scenarios.yaml')
            with open(yaml_path) as f:
                data = yaml.safe_load(f)
            scenario_data = data.get('scenarios', {}).get(self.scenario, {})
            goals = {}
            for ns in ROBOT_NAMESPACES:
                wps = scenario_data.get(ns, {}).get('waypoints', [])
                if wps:
                    last = wps[-1]
                    goals[ns] = (float(last[0]), float(last[1]))

            layout = str(scenario_data.get(
                'final_formation_layout',
                scenario_data.get('formation_layout', 'line'))).strip().lower()
            formation_at_goal = str(
                scenario_data.get('formation_at_goal', False)).strip().lower()
            gp = scenario_data.get('gathering_point')
            if (layout in ('circle', 'circular', 'radial', 'around_gathering_point')
                    and formation_at_goal in ('1', 'true', 'yes', 'on')
                    and isinstance(gp, dict)
                    and 'x' in gp and 'y' in gp):
                ordered = [ns for ns in ROBOT_NAMESPACES if ns in goals]
                if len(ordered) >= 2:
                    radius = float(scenario_data.get(
                        'formation_radius',
                        scenario_data.get('formation_spacing', 0.30)))
                    start_angle = math.radians(float(
                        scenario_data.get('formation_start_angle_deg', 90.0)))
                    direction = str(scenario_data.get(
                        'formation_direction', 'ccw')).strip().lower()
                    step_sign = -1.0 if direction in ('cw', 'clockwise') else 1.0
                    step = step_sign * (2.0 * math.pi / len(ordered))
                    anchor_x = float(gp['x'])
                    anchor_y = float(gp['y'])
                    for idx, ns in enumerate(ordered):
                        raw_angle = scenario_data.get(f'formation_angle_{ns}')
                        angle = (
                            math.radians(float(raw_angle))
                            if raw_angle is not None else start_angle + idx * step
                        )
                        raw_radius = scenario_data.get(f'formation_radius_{ns}')
                        r = float(raw_radius) if raw_radius is not None else radius
                        goals[ns] = (
                            anchor_x + r * math.cos(angle),
                            anchor_y + r * math.sin(angle),
                        )

            for ns in ROBOT_NAMESPACES:
                if ns in goals:
                    self.get_logger().info(
                        f'[PRIORITY] final goal {ns}: '
                        f'({goals[ns][0]:.3f}, {goals[ns][1]:.3f})')
            return goals
        except Exception as e:
            self.get_logger().warn(f'[PRIORITY] Gagal load final goals: {e}')
            return {}

    def _apply_safety_radii(self, scenario):
        """Set d_emergency/d_warning/d_clear dari base global + override per-skenario.

        Override diambil dari SAFETY_RADII_BY_SCENARIO; jika skenario tidak ada di
        tabel, pakai nilai dasar global. Dipanggil saat init & saat ganti skenario.
        """
        override = SAFETY_RADII_BY_SCENARIO.get(scenario, {}) or {}
        self.d_emergency = float(override.get('d_emergency', self._d_emergency_base))
        self.d_warning   = float(override.get('d_warning',   self._d_warning_base))
        self.d_clear     = float(override.get('d_clear',     self._d_clear_base))
        if not (self.d_emergency < self.d_warning < self.d_clear):
            self.get_logger().error(
                f'[SAFETY RADII] urutan tidak valid utk "{scenario}": '
                f'd_emergency({self.d_emergency}) < d_warning({self.d_warning}) '
                f'< d_clear({self.d_clear}) harus terpenuhi!')
        # Jika ambang prediksi tadinya auto-derive, ikut menyesuaikan
        if getattr(self, '_pred_warn_auto', False):
            self.prediction_warning_dist = self.d_warning
        if getattr(self, '_pred_emg_auto', False):
            self.prediction_emergency_dist = self.d_emergency
        if override:
            self.get_logger().info(
                f'[SAFETY RADII] "{scenario}": d_emg={self.d_emergency:.2f} '
                f'd_warn={self.d_warning:.2f} d_clr={self.d_clear:.2f}')

    def _is_zone_candidate(self, ns: str) -> bool:
        """True jika robot masih aktif dan perlu ikut giliran conflict zone."""
        if self.robot_goal_reached.get(ns, False):
            return False
        if (time.time() - self.last_pose_update.get(ns, 0.0)) > self.stale_timeout:
            return False
        pose = self.robot_poses[ns]
        if pose is not None and ns in self._final_goals:
            gx, gy = self._final_goals[ns]
            if math.hypot(pose[0] - gx, pose[1] - gy) < self.final_goal_proximity:
                return False
        return True

    def _dist_to_zone(self, ns: str, cz: 'ConflictZone') -> float:
        pose = self.robot_poses[ns]
        if pose is None:
            return float('inf')
        return math.hypot(pose[0] - cz.x, pose[1] - cz.y)

    def _eta_speed_for_zone(self, ns: str) -> tuple[float, str]:
        if not self.feasibility_aware_eta_enabled:
            return max(self.v_nominal, self.eta_v_eff_floor), 'v_nominal'

        age = time.time() - self.robot_dwa_metric_update.get(ns, 0.0)
        if age > self.eta_v_eff_timeout_s:
            return max(self.v_nominal, self.eta_v_eff_floor), 'stale_nominal'

        candidates = []
        speed = self.robot_dwa_speed_mag.get(ns)
        vmax_eff = self.robot_dwa_vmax_eff.get(ns)
        if speed is not None and math.isfinite(float(speed)):
            candidates.append(('dwa_speed', abs(float(speed))))
        if vmax_eff is not None and math.isfinite(float(vmax_eff)):
            candidates.append(('dwa_vmax_eff', max(0.0, float(vmax_eff))))
        if not candidates:
            return max(self.v_nominal, self.eta_v_eff_floor), 'v_nominal'

        source, value = max(candidates, key=lambda item: item[1])
        return max(self.eta_v_eff_floor, value), source

    def _motion_vector(self, ns: str) -> tuple[float, float, float, str]:
        """Velocity estimate in map frame for short-horizon prediction."""
        pose = self.robot_poses.get(ns)
        if pose is None:
            return 0.0, 0.0, 0.0, 'no_pose'

        speed, source = self._eta_speed_for_zone(ns)
        if self.robot_goal_reached.get(ns, False):
            speed = 0.0
            source = 'goal_reached'
        elif self.robot_stop.get(ns, False):
            # Robot yang sedang ditahan tetap dianggap punya niat jalan pelan
            # agar prediksi tidak hilang total ketika ia menunggu giliran.
            speed = min(speed, self.prediction_slow_vmax)
            source = f'stopped_{source}'

        idx = self._nearest_path_index(ns)
        if idx is not None:
            dx, dy = self._path_dir(ns, idx)
            return speed * dx, speed * dy, speed, f'path_{source}'

        yaw = pose[2]
        return speed * math.cos(yaw), speed * math.sin(yaw), speed, f'yaw_{source}'

    @staticmethod
    def _dot_norm(ax, ay, bx, by) -> float:
        na = math.hypot(ax, ay)
        nb = math.hypot(bx, by)
        if na < 1e-6 or nb < 1e-6:
            return 0.0
        return max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))

    def _predict_pair_conflict(self, ns_a: str, ns_b: str) -> dict:
        pose_a = self.robot_poses.get(ns_a)
        pose_b = self.robot_poses.get(ns_b)
        if pose_a is None or pose_b is None:
            return {'valid': False}

        vax, vay, va, src_a = self._motion_vector(ns_a)
        vbx, vby, vb, src_b = self._motion_vector(ns_b)
        rel_dot = self._dot_norm(vax, vay, vbx, vby)

        min_d = float('inf')
        min_t = 0.0
        steps = max(1, int(self.prediction_horizon_s / self.prediction_dt_s))
        for step in range(steps + 1):
            tau = step * self.prediction_dt_s
            ax = pose_a[0] + vax * tau
            ay = pose_a[1] + vay * tau
            bx = pose_b[0] + vbx * tau
            by = pose_b[1] + vby * tau
            d = math.hypot(ax - bx, ay - by)
            if d < min_d:
                min_d = d
                min_t = tau

        conflict_type = 'crossing'
        if rel_dot < -0.70:
            conflict_type = 'head_on'
        elif rel_dot > 0.70:
            conflict_type = 'rear_end'

        level = 'clear'
        if min_d <= self.prediction_emergency_dist:
            level = 'emergency'
        elif min_d <= self.prediction_warning_dist:
            level = 'warning'

        return {
            'valid': True,
            'pair': f'{ns_a}-{ns_b}',
            'd_min_pred': min_d,
            't_min_pred': min_t,
            'level': level,
            'type': conflict_type,
            'rel_dot': rel_dot,
            'speed_a': va,
            'speed_b': vb,
            'source_a': src_a,
            'source_b': src_b,
        }

    def _priority_agent_failed(self, ns: str) -> bool:
        """[M5] Robot dianggap GAGAL (untuk demosi prioritas) bila telemetry basi
        (heartbeat) atau diperintah bergerak tapi pose diam (motion-stall)."""
        if not self.agent_failure_detection_enabled:
            return False
        now = time.time()
        # (1) Heartbeat: pose basi / comms-loss
        if (now - self.last_pose_update.get(ns, 0.0)) > self.stale_timeout:
            return True
        pose = self.robot_poses.get(ns)
        if pose is None:
            return True
        # (2) Motion-stall: HANYA saat robot seharusnya bergerak
        running = self._experiment_start_time is not None
        expected_to_move = (running
                            and not self.robot_goal_reached.get(ns, False)
                            and not self.robot_stop.get(ns, False))
        if not expected_to_move:
            self._prio_motion_ref[ns] = (pose[0], pose[1])
            self._prio_motion_t[ns] = now
            return False
        ref = self._prio_motion_ref.get(ns)
        if ref is None:
            self._prio_motion_ref[ns] = (pose[0], pose[1])
            self._prio_motion_t[ns] = now
            return False
        if math.hypot(pose[0] - ref[0], pose[1] - ref[1]) >= self.agent_stall_eps_m:
            self._prio_motion_ref[ns] = (pose[0], pose[1])
            self._prio_motion_t[ns] = now
            return False
        return (now - self._prio_motion_t.get(ns, now)) > self.agent_stall_window_s

    def _compute_eta_priority_order(self):
        """[M5] Urutan prioritas (tertinggi->terendah) dari ETA ke titik konflik.
        Robot dengan ETA terkecil (tiba lebih dulu) = prioritas tertinggi; yang
        lain yield. Agen GAGAL -> ETA tak hingga -> otomatis prioritas terendah.
        Return None bila data tak cukup (-> fallback tabel statis)."""
        if self.priority_mode != 'eta':
            return None
        zones = self.conflict_zones
        if not zones:
            return None
        etas = {}
        for ns in ROBOT_NAMESPACES:
            if (self._priority_agent_failed(ns)
                    or self.robot_goal_reached.get(ns, False)):
                etas[ns] = float('inf')
                continue
            pose = self.robot_poses.get(ns)
            if pose is None:
                etas[ns] = float('inf')
                continue
            dmin = min(self._dist_to_zone(ns, cz) for cz in zones)
            v_eta = self._eta_speed_for_zone(ns)[0]
            etas[ns] = dmin / max(v_eta, self.eta_v_eff_floor)
        order = sorted(
            ROBOT_NAMESPACES,
            key=lambda ns: (etas[ns], ROBOT_NAMESPACES.index(ns)))
        return order

    def _maybe_update_priority_order(self, wall_now: float):
        """[M5] Recompute & commit urutan prioritas ETA dengan debounce + hysteresis
        agar tidak thrashing. Bangun ulang all_pairs/pair_states HANYA saat urutan
        benar-benar berubah dan bertahan melewati hysteresis."""
        if self.priority_mode != 'eta':
            return
        if (wall_now - self._last_prio_recompute_t) < self.priority_recompute_period_s:
            return
        self._last_prio_recompute_t = wall_now
        new_order = self._compute_eta_priority_order()
        if not new_order or new_order == self.priority_order:
            self._pending_prio_order = None
            return
        if new_order != self._pending_prio_order:
            self._pending_prio_order = new_order
            self._pending_prio_since = wall_now
            return
        if (wall_now - self._pending_prio_since) < self.priority_switch_hysteresis_s:
            return
        old = self.priority_order
        self.priority_order = list(new_order)
        self.all_pairs = build_priority_pairs(self.priority_order)
        self.pair_states = {
            (low, high): PairState(low, high) for low, high in self.all_pairs}
        self._hard_stop_since = {pair: None for pair in self.all_pairs}
        self._pending_prio_order = None
        self.get_logger().info(
            f'[M5][PRIORITY-ETA] order update: {old} -> {self.priority_order}')

    def _select_dynamic_zone_owner(self, candidates, robot_d, wall_now):
        eligible = [ns for ns in candidates
                    if wall_now >= self._owner_cooldown.get(ns, 0.0)]
        if not eligible:
            return None
        if not self.dynamic_zone_priority_enabled:
            etas = {ns: (float('inf') if self._priority_agent_failed(ns)
                         else robot_d[ns] / self._eta_speed_for_zone(ns)[0])
                    for ns in eligible}
            return min(etas, key=etas.get)

        scores = {}
        for ns in eligible:
            # [M5] Agen gagal tak boleh jadi owner zona (auto-demote)
            if self._priority_agent_failed(ns):
                scores[ns] = float('-inf')
                continue
            eta = robot_d[ns] / self._eta_speed_for_zone(ns)[0]
            waiting_since = self._zone_wait_since.get(ns)
            waiting = max(0.0, wall_now - waiting_since) if waiting_since else 0.0
            scores[ns] = self.dynamic_waiting_weight * waiting - eta
        return max(scores, key=scores.get)

    def _zone_affects_robot(self, cz: 'ConflictZone', ns: str) -> bool:
        if cz.source != 'auto' or not cz.robot_pair:
            return True
        robots = set()
        for pair in cz.robot_pair.split('+'):
            robots.update(part for part in pair.split('-') if part)
        return ns in robots

    def _release_conflict_owner(self, cz: 'ConflictZone', wall_now: float,
                                cooldown: bool = False) -> str | None:
        old_owner = cz.owner
        if cooldown and old_owner is not None:
            self._owner_cooldown[old_owner] = wall_now + self.owner_cooldown_s
        cz.state            = ConflictZone.CLEARING
        cz.last_clear_time  = wall_now
        cz.owner            = None
        cz.owner_entered    = False
        cz.owner_entry_time = None
        cz.owner_d0         = None
        cz.owner_stuck_since = None
        return old_owner

    def _owner_near_final_goal(self, ns: str) -> bool:
        pose = self.robot_poses.get(ns)
        goal = self._final_goals.get(ns)
        if pose is None or goal is None:
            return False
        return math.hypot(pose[0] - goal[0], pose[1] - goal[1]) < self.final_goal_proximity

    def _crossing_yield_only(self) -> bool:
        return self.crossing_slow_only_enabled and self.scenario == 'crossing'

    def _apply_predictive_pair_conflicts(self, stop_votes: dict, vmax_votes: dict):
        if not self.predictive_conflict_enabled:
            return

        wall_now = time.time()
        details = []
        self._predictive_stop_reason = {ns: '' for ns in ROBOT_NAMESPACES}

        for low_ns, high_ns in self.all_pairs:
            if not (self._is_zone_candidate(low_ns)
                    and self._is_zone_candidate(high_ns)):
                continue
            pose_low = self.robot_poses.get(low_ns)
            pose_high = self.robot_poses.get(high_ns)
            if pose_low is None or pose_high is None:
                continue
            current_dist = math.hypot(pose_low[0] - pose_high[0],
                                      pose_low[1] - pose_high[1])
            if self._ignore_parallel_priority_pair(low_ns, high_ns, current_dist):
                continue

            pred = self._predict_pair_conflict(low_ns, high_ns)
            if not pred.get('valid'):
                continue

            level = pred['level']
            cmd_low = 'NORMAL'
            if level == 'emergency':
                if (pred['t_min_pred'] <= self.prediction_stop_ttc_s
                        and not self._crossing_yield_only()):
                    stop_votes[low_ns] = True
                    cmd_low = 'YIELD'
                    self._predictive_stop_reason[low_ns] = (
                        f'predicted {pred["type"]} with {high_ns} '
                        f'd={pred["d_min_pred"]:.2f}m t={pred["t_min_pred"]:.1f}s')
                else:
                    vmax_votes[low_ns] = min(vmax_votes[low_ns],
                                             self.prediction_slow_vmax)
                    cmd_low = 'SLOW'
            elif level == 'warning':
                vmax_votes[low_ns] = min(vmax_votes[low_ns],
                                         self.prediction_slow_vmax)
                cmd_low = 'SLOW'

            pred['current_dist'] = current_dist
            pred['cmd_low'] = cmd_low
            pred['low'] = low_ns
            pred['high'] = high_ns
            details.append(pred)

        if details:
            payload = {
                't': wall_now,
                'horizon_s': self.prediction_horizon_s,
                'dt_s': self.prediction_dt_s,
                'warning_dist': self.prediction_warning_dist,
                'emergency_dist': self.prediction_emergency_dist,
                'pairs': [
                    {
                        'pair': d['pair'],
                        'low': d['low'],
                        'high': d['high'],
                        'type': d['type'],
                        'level': d['level'],
                        'cmd_low': d['cmd_low'],
                        'current_dist': round(d['current_dist'], 3),
                        'd_min_pred': round(d['d_min_pred'], 3),
                        't_min_pred': round(d['t_min_pred'], 2),
                        'rel_dot': round(d['rel_dot'], 3),
                        'speed_low': round(d['speed_a'], 3),
                        'speed_high': round(d['speed_b'], 3),
                        'source_low': d['source_a'],
                        'source_high': d['source_b'],
                    }
                    for d in details
                ],
            }
            self._last_prediction_debug = payload
            if wall_now - self._last_prediction_debug_pub >= self.prediction_debug_period_s:
                msg = String()
                msg.data = json.dumps(payload)
                self._prediction_debug_pub.publish(msg)
                self._last_prediction_debug_pub = wall_now

    def _update_conflict_zones(self, stop_votes: dict, vmax_votes: dict):
        """
        Zone/gap coordination:
          IDLE     → tidak ada robot di detect_radius
          APPROACH → owner dipilih berdasarkan ETA terkecil; non-owner SLOW/HOLD
          OCCUPIED → owner masuk zone_radius; non-owner tetap HOLD
          CLEARING → owner sudah lewat clear_radius; tunggu gap_s detik; non-owner SLOW

        Owner tidak pernah diganti selama APPROACH/OCCUPIED kecuali dia keluar
        detect_radius lebih dulu (sebelum masuk zone_radius).
        Clear hanya terjadi jika owner sudah benar-benar masuk (owner_entered=True).
        """
        if not self.conflict_zones:
            return

        wall_now    = time.time()
        zone_events = []   # event perubahan state → /conflict_zone_state
        zone_detail = []   # data per-tick lengkap → /conflict_zone_detail

        # Kalau semua robot sudah selesai/final, reset zona ke IDLE dan hentikan logika
        if not any(self._is_zone_candidate(ns) for ns in ROBOT_NAMESPACES):
            for cz in self.conflict_zones:
                if cz.state != ConflictZone.IDLE:
                    self.get_logger().info(
                        f'[CONFLICT] {cz.name}: semua robot sudah final — reset ke IDLE')
                    cz.state = ConflictZone.IDLE; cz.owner = None
                    cz.owner_entered = False; cz.owner_entry_time = None
                    cz.owner_d0 = None; cz.last_clear_time = None
                    cz.owner_stuck_since = None
            return

        for cz in self.conflict_zones:
            prev_state = cz.state
            prev_owner = cz.owner

            # Hitung jarak & ETA semua robot ke zona
            robot_d = {}
            for ns in ROBOT_NAMESPACES:
                pose = self.robot_poses[ns]
                stale = (wall_now - self.last_pose_update[ns]) > self.stale_timeout
                if pose is None or stale:
                    robot_d[ns] = float('inf')
                else:
                    robot_d[ns] = math.hypot(pose[0] - cz.x, pose[1] - cz.y)

            # Hanya robot aktif (bukan goal_reached / near-final / stale) yang bisa jadi kandidat
            approaching = [
                ns for ns in ROBOT_NAMESPACES
                if (self._zone_affects_robot(cz, ns)
                    and robot_d[ns] <= cz.detect_radius
                    and self._is_zone_candidate(ns))
            ]

            # ── Helper: pilih owner dari kandidat (filter cooldown) ──────
            def _select_owner(candidates):
                return self._select_dynamic_zone_owner(candidates, robot_d, wall_now)

            # Owner yang sudah sampai final tidak boleh menahan zona. Pada merge,
            # final goal bisa masih berada di dalam clear_radius, jadi clear-by-
            # distance saja tidak cukup.
            if cz.owner is not None and (
                    self.robot_goal_reached.get(cz.owner, False)
                    or self._owner_near_final_goal(cz.owner)):
                old_owner = self._release_conflict_owner(
                    cz, wall_now, cooldown=False)
                self.get_logger().info(
                    f'[CONFLICT] {cz.name}: owner={old_owner} sudah final '
                    f'— release zone ke CLEARING')

            # ── Owner timeout: JANGAN buru-buru beri akses non-owner ─────
            # Jika owner sudah benar-benar KELUAR zona (d > clear_radius),
            # aman release ke CLEARING. Tapi jika owner timeout PADAHAL masih
            # di dalam bottleneck (atau pose-nya stale sehingga kita tidak tahu
            # dia sudah keluar), tandai OWNER_STUCK dan TETAP tahan non-owner —
            # memberi akses sekarang berisiko tabrakan di zona inti.
            elif (cz.owner is not None
                    and cz.owner_entry_time is not None
                    and (wall_now - cz.owner_entry_time) > cz.owner_timeout):
                owner_d_timeout = robot_d.get(cz.owner, float('inf'))
                owner_left_zone = (owner_d_timeout < float('inf')
                                   and owner_d_timeout > cz.clear_radius)
                if owner_left_zone:
                    old_owner = self._release_conflict_owner(
                        cz, wall_now, cooldown=True)
                    self.get_logger().warn(
                        f'[CONFLICT] {cz.name}: owner={old_owner} timeout & '
                        f'sudah keluar zona (d={owner_d_timeout:.3f}m > '
                        f'clear={cz.clear_radius:.3f}m) — release ke CLEARING '
                        f'(cooldown {self.owner_cooldown_s:.0f}s)')
                elif cz.state != ConflictZone.OWNER_STUCK:
                    # Owner timeout tapi MASIH di zona (atau pose stale) →
                    # OWNER_STUCK. Non-owner tetap HOLD; owner tetap diberi GO
                    # agar bisa keluar sendiri saat fault/halangannya hilang.
                    cz.state = ConflictZone.OWNER_STUCK
                    cz.owner_stuck_since = wall_now
                    self.get_logger().error(
                        f'[CONFLICT] {cz.name}: owner={cz.owner} OWNER_STUCK '
                        f'(timeout, d={owner_d_timeout:.3f}m masih di zona) — '
                        f'non-owner tetap DITAHAN, tunggu owner keluar '
                        f'(maks {self.owner_stuck_release_s:.0f}s)')
                elif (cz.owner_stuck_since is not None
                        and (wall_now - cz.owner_stuck_since)
                             > self.owner_stuck_release_s):
                    # Sudah terlalu lama stuck → kemungkinan agen benar-benar
                    # mati (bukan fault sementara). Paksa release agar misi
                    # tidak deadlock permanen (dengan cooldown).
                    old_owner = self._release_conflict_owner(
                        cz, wall_now, cooldown=True)
                    self.get_logger().error(
                        f'[CONFLICT] {cz.name}: owner={old_owner} OWNER_STUCK '
                        f'> {self.owner_stuck_release_s:.0f}s — paksa release ke '
                        f'CLEARING (anti-deadlock, cooldown '
                        f'{self.owner_cooldown_s:.0f}s)')

            # ── Approach stall: owner tidak maju menuju zona → paksa CLEARING
            elif (cz.state == ConflictZone.APPROACH
                    and cz.owner is not None
                    and cz.owner_entry_time is not None
                    and not cz.owner_entered
                    and (wall_now - cz.owner_entry_time) > self.approach_stall_timeout_s):
                d_now  = robot_d.get(cz.owner, float('inf'))
                d_init = cz.owner_d0 if cz.owner_d0 is not None else d_now
                if (d_init - d_now) < self.approach_progress_min_m:
                    old_owner = self._release_conflict_owner(
                        cz, wall_now, cooldown=True)
                    self.get_logger().warn(
                        f'[CONFLICT] {cz.name}: owner={old_owner} stall '
                        f'd0={d_init:.2f}m d_now={d_now:.2f}m (<{self.approach_progress_min_m}m progress) '
                        f'— paksa CLEARING (cooldown {self.owner_cooldown_s:.0f}s)')

            # ── State machine ─────────────────────────────────────────────
            if cz.state == ConflictZone.IDLE:
                if approaching:
                    new_owner = _select_owner(approaching)
                    if new_owner is not None:
                        cz.owner            = new_owner
                        cz.owner_entry_time = wall_now
                        cz.owner_d0         = robot_d[new_owner]
                        cz.owner_entered    = False
                        cz.state = ConflictZone.APPROACH
                        eta = robot_d[new_owner] / self._eta_speed_for_zone(new_owner)[0]
                        self.get_logger().info(
                            f'[CONFLICT] {cz.name} IDLE→APPROACH | '
                            f'owner={cz.owner} ETA={eta:.2f}s d0={cz.owner_d0:.2f}m | '
                            f'candidates={approaching}')

            elif cz.state == ConflictZone.APPROACH:
                if not approaching:
                    cz.state            = ConflictZone.IDLE
                    cz.owner            = None
                    cz.owner_entered    = False
                    cz.owner_entry_time = None
                    cz.owner_d0         = None
                elif cz.owner not in approaching:
                    # Owner menghilang sebelum masuk → pilih owner baru (filter cooldown)
                    new_owner = _select_owner(approaching)
                    if new_owner is not None:
                        cz.owner            = new_owner
                        cz.owner_entry_time = wall_now
                        cz.owner_d0         = robot_d[new_owner]
                        cz.owner_entered    = False
                        self.get_logger().info(
                            f'[CONFLICT] {cz.name} owner berganti → {cz.owner} '
                            f'd0={cz.owner_d0:.2f}m')
                    else:
                        # Semua kandidat sedang cooldown → kembali IDLE sementara
                        cz.state            = ConflictZone.IDLE
                        cz.owner            = None
                        cz.owner_entered    = False
                        cz.owner_entry_time = None
                        cz.owner_d0         = None
                else:
                    # Cek apakah owner sudah masuk zone_radius
                    if robot_d[cz.owner] <= cz.zone_radius:
                        cz.owner_entered = True
                        cz.state = ConflictZone.OCCUPIED
                        self.get_logger().info(
                            f'[CONFLICT] {cz.name} APPROACH→OCCUPIED | '
                            f'owner={cz.owner} dist={robot_d[cz.owner]:.3f}m')

            elif cz.state == ConflictZone.OCCUPIED:
                owner_d = robot_d.get(cz.owner, float('inf'))
                # Cek terus owner_entered (mungkin baru masuk di iterasi ini)
                if not cz.owner_entered and owner_d <= cz.zone_radius:
                    cz.owner_entered = True
                # Clear normal terjadi jika owner SUDAH masuk lalu keluar
                # clear_radius. Owner yang sampai final atau timeout sudah
                # ditangani lebih awal agar zona tidak terkunci permanen.
                if cz.owner_entered and owner_d > cz.clear_radius:
                    cz.state            = ConflictZone.CLEARING
                    cz.last_clear_time  = wall_now
                    old_owner           = cz.owner
                    cz.owner            = None
                    cz.owner_entered    = False
                    cz.owner_entry_time = None
                    cz.owner_d0         = None
                    self.get_logger().info(
                        f'[CONFLICT] {cz.name} OCCUPIED→CLEARING | '
                        f'prev_owner={old_owner} dist={owner_d:.3f}m '
                        f'gap={cz.gap_s:.1f}s')

            elif cz.state == ConflictZone.CLEARING:
                # Tunggu gap_s detik setelah owner clear — lalu IDLE
                if (cz.last_clear_time is not None
                        and (wall_now - cz.last_clear_time) >= cz.gap_s):
                    cz.state           = ConflictZone.IDLE
                    cz.last_clear_time = None
                    self.get_logger().info(
                        f'[CONFLICT] {cz.name} CLEARING→IDLE (gap={cz.gap_s:.1f}s elapsed)')

            if cz.state != prev_state or cz.owner != prev_owner:
                zone_events.append({
                    'zone' : cz.name,
                    'state': cz.state,
                    'owner': cz.owner or '',
                    'source': cz.source,
                    'robot_pair': cz.robot_pair,
                })

            # ── Terapkan constraint + bangun cmd_map untuk logging ────────
            gap_remain = -1.0
            if cz.state == ConflictZone.CLEARING and cz.last_clear_time is not None:
                gap_remain = round(max(0.0, cz.gap_s - (wall_now - cz.last_clear_time)), 2)

            cmd_map = {}
            for ns in ROBOT_NAMESPACES:
                d   = robot_d[ns]
                eta_v, eta_source = self._eta_speed_for_zone(ns)
                eta = d / eta_v if d < float('inf') else -1.0
                cmd = 'NORMAL'

                if (self._zone_affects_robot(cz, ns)
                        and d <= cz.detect_radius
                        and not self.robot_goal_reached.get(ns, False)):
                    if cz.state in (ConflictZone.APPROACH, ConflictZone.OCCUPIED,
                                    ConflictZone.OWNER_STUCK):
                        if ns == cz.owner:
                            cmd = 'GO'
                            # Conflict-zone ownership is the right-of-way source
                            # for crossing/merge. Static pair priority may have
                            # voted STOP earlier; clear it for the owner here.
                            # The hard-collision brake below still wins.
                            stop_votes[ns] = False
                            # Owner punya hak jalan penuh: reset throttle vmax yang
                            # mungkin sudah dipasang pair-priority/predictive sebelumnya
                            # agar owner tidak ikut melambat. (Hard-collision brake di
                            # bawah tetap dapat menahan bila benar-benar kritis.)
                            vmax_votes[ns] = self.vmax_priority_ceiling  # [FIX-PRIOCAP] owner hak jalan penuh (bukan dikunci v_nominal)
                            self._zone_wait_since[ns] = None
                        elif d <= cz.hold_radius:
                            cmd = 'HOLD'
                            stop_votes[ns] = True
                            if self._zone_wait_since[ns] is None:
                                self._zone_wait_since[ns] = wall_now
                        else:
                            cmd = 'SLOW'
                            vmax_votes[ns] = min(vmax_votes[ns], self.v_conflict_slow)
                            if self._zone_wait_since[ns] is None:
                                self._zone_wait_since[ns] = wall_now
                    elif cz.state == ConflictZone.CLEARING:
                        # Gap masih berjalan: pertahankan slow agar robot berikutnya
                        # tidak langsung tancap gas
                        cmd = 'SLOW'
                        vmax_votes[ns] = min(vmax_votes[ns], self.v_conflict_slow)
                        if self._zone_wait_since[ns] is None:
                            self._zone_wait_since[ns] = wall_now
                elif cmd == 'NORMAL':
                    self._zone_wait_since[ns] = None

                cmd_map[ns] = {
                    'd'  : round(d, 3) if d < float('inf') else -1.0,
                    'eta': round(eta, 2),
                    'eta_v': round(eta_v, 3),
                    'eta_source': eta_source,
                    'waiting_s': round(
                        max(0.0, wall_now - self._zone_wait_since[ns])
                        if self._zone_wait_since[ns] else 0.0, 2),
                    'cmd': cmd,
                }

            zone_detail.append({
                'zone'        : cz.name,
                'source'      : cz.source,
                'robot_pair'  : cz.robot_pair,
                'center_x'    : round(cz.x, 3),
                'center_y'    : round(cz.y, 3),
                'radius'      : round(cz.zone_radius, 3),
                'detect_radius': round(cz.detect_radius, 3),
                'hold_radius' : round(cz.hold_radius, 3),
                'clear_radius': round(cz.clear_radius, 3),
                'state'       : cz.state,
                'owner'       : cz.owner or '',
                'gap_remain_s': gap_remain,
                'robots'      : cmd_map,
            })

        self._latest_conflict_zone_detail_payload = {
            'zones': zone_detail,
            't': wall_now,
        }

        # Publish event (hanya saat ada perubahan state)
        if zone_events:
            msg      = String()
            msg.data = json.dumps({'zones': zone_events, 't': wall_now})
            self._conflict_state_pub.publish(msg)

        # Publish detail per-tick (selalu, untuk conflict_detail_log.csv)
        if zone_detail:
            msg      = String()
            msg.data = json.dumps({'zones': zone_detail, 't': wall_now})
            self._conflict_detail_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # UDP RECEIVER — pose dari tiap robot
    # ═══════════════════════════════════════════════════════════════════════

    def _experiment_scenario_cb(self, msg):
        scenario = str(msg.data).strip()
        if not scenario or scenario == self.scenario:
            return
        with self._state_lock:
            old = self.scenario
            self.scenario = scenario
            # Rebuild priority order & pasangan sesuai skenario baru
            self.priority_order = priority_order_for(
                self.scenario, self.priority_order_override)
            self.all_pairs = build_priority_pairs(self.priority_order)
            self.pair_states = {
                (low, high): PairState(low, high) for low, high in self.all_pairs}
            # Radius keselamatan ikut menyesuaikan skenario baru
            self._apply_safety_radii(self.scenario)
            self.manual_conflict_zones = self._load_conflict_zones(self.scenario)
            self.auto_conflict_zones = []
            self.conflict_zones = list(self.manual_conflict_zones)
            self._final_goals = self._load_final_goals()
            self._owner_cooldown = {}
            self._zone_wait_since = {ns: None for ns in ROBOT_NAMESPACES}
            self._hard_stop_since = {pair: None for pair in self.all_pairs}
            self._latest_conflict_zone_detail_payload = {'zones': [], 't': time.time()}
        self.get_logger().info(
            f'[PRIORITY] scenario update: {old} → {self.scenario} | '
            f'manual_zones={len(self.manual_conflict_zones)}')

    def start_signal_callback(self, msg):
        with self._state_lock:
            self.experiment_active = bool(msg.data)
            active = self.experiment_active
        state = 'AKTIF' if active else 'STANDBY'
        self.get_logger().info(f'[PRIORITY] {state}')
        if not active:
            # Reset lane offset agar trial berikutnya tidak punya sisa bias
            with self._state_lock:
                for ns in ROBOT_NAMESPACES:
                    self.lane_offset[ns]        = 0.0
                    self.negotiation_active[ns] = False
                    self.clear_cycles[ns]       = 0
                    self._send_priority_udp(ns, False, self.v_nominal, 0.0)

    def _experiment_state_cb(self, msg):
        """Aktifkan priority manager hanya saat state RUNNING."""
        with self._state_lock:
            was_active = self.experiment_active
            self.experiment_active = (msg.data == 'RUNNING')
            active = self.experiment_active
            if not was_active and self.experiment_active:
                self._experiment_start_time = time.time()
                self.get_logger().info(
                    f'Priority Manager AKTIF — grace period {self.startup_grace:.1f}s')
            if msg.data in ('STOP', 'READY'):
                self._experiment_start_time = None
                self.robot_goal_reached = {ns: False for ns in ROBOT_NAMESPACES}
                self.robot_stop         = {ns: False for ns in ROBOT_NAMESPACES}
                self._active_stops      = {ns: None  for ns in ROBOT_NAMESPACES}
                for ps in self.pair_states.values():
                    ps.state              = PairState.CLEAR
                    ps.stop_start_time    = None
                    ps.override_active    = False
                    ps.override_start_time = None
                self._latest_conflict_zone_detail_payload = {'zones': [], 't': time.time()}
                for ns in ROBOT_NAMESPACES:
                    self.lane_offset[ns]        = 0.0
                    self.negotiation_active[ns] = False
                    self.clear_cycles[ns]       = 0
                    self._send_priority_udp(ns, False, self.v_nominal, 0.0)
                    stop_msg = Bool()
                    stop_msg.data = False
                    self.priority_stop_pub[ns].publish(stop_msg)
                # Reset conflict zone state machines + cooldown
                self.manual_conflict_zones = self._load_conflict_zones(self.scenario)
                self.auto_conflict_zones = []
                self.conflict_zones  = list(self.manual_conflict_zones)
                self.robot_path_source = {ns: '' for ns in ROBOT_NAMESPACES}
                self.robot_path_update = {ns: 0.0 for ns in ROBOT_NAMESPACES}
                self._owner_cooldown = {}
                self._zone_wait_since = {ns: None for ns in ROBOT_NAMESPACES}
                self._predictive_stop_reason = {ns: '' for ns in ROBOT_NAMESPACES}
                self._hard_stop_since = {pair: None for pair in self.all_pairs}
                self.get_logger().info(
                    f'[PRIORITY RESET] state {msg.data} — '
                    f'all pair states, stop flags, goal flags, lane offsets, '
                    f'dan conflict zones cleared.')
        label = 'AKTIF' if active else 'STANDBY'
        self.get_logger().info(f'Priority Manager {label} (state={msg.data})')

    def _process_udp_pose_packet(self, ns, packet):
        with self._state_lock:
            pose = packet.get('pose')
            if pose:
                self.robot_poses[ns] = [
                    pose['x'],
                    pose['y'],
                    pose.get('yaw', 0.0)
                ]
                self.last_pose_update[ns] = time.time()
            path_points = packet.get('path_points')
            if path_points:
                clean_path = []
                for pt in path_points:
                    if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                        clean_path.append((float(pt[0]), float(pt[1])))
                if len(clean_path) >= 2:
                    self.robot_paths[ns] = clean_path
                    self.robot_path_source[ns] = 'udp'
                    self.robot_path_update[ns] = time.time()
            new_goal_state = bool(packet.get('goal_reached', False))
            if new_goal_state != self.robot_goal_reached[ns]:
                self.get_logger().info(
                    f'[PRIORITY] {ns} goal_reached: '
                    f'{self.robot_goal_reached[ns]} → {new_goal_state}')
            self.robot_goal_reached[ns] = new_goal_state
            metric_now = time.time()
            dwa_vmax_eff = packet.get('dwa_vmax_eff')
            if dwa_vmax_eff is not None:
                self.robot_dwa_vmax_eff[ns] = max(0.0, float(dwa_vmax_eff))
                self.robot_dwa_metric_update[ns] = metric_now
            dwa_speed_mag = packet.get('dwa_speed_mag')
            if dwa_speed_mag is not None:
                self.robot_dwa_speed_mag[ns] = max(0.0, float(dwa_speed_mag))
                self.robot_dwa_metric_update[ns] = metric_now

    def _udp_pose_listener(self, ns, port):
        """
        Terima pose dari udp_sender_node di robot ns.
        Port berbeda dari consensus_node (9031-9033 vs 9001-9003) —
        tidak ada port sharing. SO_REUSEPORT diset sebagai safety untuk
        restart cepat agar OS tidak hold port di TIME_WAIT.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass  # SO_REUSEPORT tidak tersedia di semua OS

        sock.bind(('0.0.0.0', port))
        sock.settimeout(1.0)

        while rclpy.ok():
            try:
                # [FIX-UDPBUF] Samakan dgn udp_receiver di robot (128KB). 16KB lama
                # berisiko memotong datagram besar (path 300 titik + debug) -> json
                # gagal -> paket pose/telemetri dibuang diam-diam. Perbesar buffer.
                data, _ = sock.recvfrom(131072)
                packet  = json.loads(data.decode('utf-8'))
                self._process_udp_pose_packet(ns, packet)

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'[{ns}] pose UDP error: {e}')

        sock.close()

    # ═══════════════════════════════════════════════════════════════════════
    # UDP SENDER — kirim priority_stop ke robot
    # ═══════════════════════════════════════════════════════════════════════

    def _send_priority_udp(self, ns, priority_stop, vmax_priority, lane_offset=0.0):
        """Kirim priority_stop, vmax_priority, dan lane_offset ke udp_receiver_node di robot."""
        packet = {
            'priority_stop' : priority_stop,
            'vmax_priority' : vmax_priority,
            'lane_offset'   : lane_offset,
            'conflict_zone_detail': self._latest_conflict_zone_detail_payload,
        }
        try:
            self.send_sock.sendto(
                json.dumps(packet).encode('utf-8'),
                (self.robot_ip[ns], SEND_PORT_MAP[ns])
            )
        except Exception as e:
            self.get_logger().warn(f'[{ns}] UDP send priority failed: {e}')

    # ═══════════════════════════════════════════════════════════════════════
    # PRIORITY LOOP — 10 Hz (IDENTIK versi lama)
    # ═══════════════════════════════════════════════════════════════════════

    def priority_loop(self):
        with self._state_lock:
            self._priority_loop_locked()

    def _priority_loop_locked(self):
        if not self.experiment_active:
            return
        # Startup grace period: beri waktu semua robot bergerak dari posisi awal
        if self._experiment_start_time is not None:
            elapsed = time.time() - self._experiment_start_time
            if elapsed < self.startup_grace:
                return
        now        = self.get_clock().now().nanoseconds / 1e9
        wall_now   = time.time()
        # [M5] Perbarui urutan prioritas berbasis ETA (debounced) sebelum arbitrase
        self._maybe_update_priority_order(wall_now)
        stop_votes = {ns: False          for ns in ROBOT_NAMESPACES}
        # [FIX-PRIOCAP] Baseline non-konflik = plafon (0.50), bukan v_nominal (0.30).
        # Throttle konflik/warning/emergency di bawah tetap menurunkan via min();
        # ini hanya melepas BATAS ATAS agar perintah catch-up consensus bisa dieksekusi.
        vmax_votes = {ns: self.vmax_priority_ceiling for ns in ROBOT_NAMESPACES}
        self._sync_auto_conflict_zones()
        self._publish_path_debug()
        self._apply_predictive_pair_conflicts(stop_votes, vmax_votes)

        for (low_ns, high_ns), ps in self.pair_states.items():
            pose_low  = self.robot_poses[low_ns]
            pose_high = self.robot_poses[high_ns]

            # Abaikan pasangan dengan pose None atau data basi (> stale_timeout)
            if (pose_low is None or pose_high is None
                    or (wall_now - self.last_pose_update[low_ns])  > self.stale_timeout
                    or (wall_now - self.last_pose_update[high_ns]) > self.stale_timeout):
                continue

            dist = math.hypot(
                pose_low[0]  - pose_high[0],
                pose_low[1]  - pose_high[1])

            prev_state = ps.state

            # Jika high_ns sudah goal_reached, jangan full-stop low_ns di
            # dekatnya — cukup slow-down agar DWA bisa navigate via obstacle.
            high_at_goal = self.robot_goal_reached.get(high_ns, False)

            if ps.override_active:
                elapsed_override = now - ps.override_start_time
                if elapsed_override >= self.t_override or dist >= self.d_clear:
                    ps.override_active     = False
                    ps.override_start_time = None
                    ps.state               = self._state_from_dist(dist)
                    ps.stop_start_time     = None
                    self.get_logger().info(
                        f'[PRIORITY] Override selesai: {low_ns}↔{high_ns}')
                else:
                    ps.state = PairState.OVERRIDE
                    vmax_votes[high_ns] = min(
                        vmax_votes[high_ns],
                        self.v_nominal * self.v_warning_ratio)
            else:
                # [M1b] Convoy/split paralel: abaikan priority stop jika heading sama.
                # Robots di lane paralel convoy tidak perlu saling menghentikan.
                # Hard-collision brake tetap aktif di bawah sebagai last-resort.
                if self._ignore_parallel_priority_pair(low_ns, high_ns, dist):
                    if ps.state != PairState.CLEAR:
                        ps.state = PairState.CLEAR
                        ps.stop_start_time = None
                        ps.override_active = False
                elif dist <= self.d_emergency:
                    if high_at_goal:
                        # high_ns sudah di goal: turunkan ke WARNING saja
                        ps.state           = PairState.WARNING
                        ps.stop_start_time = None
                        vmax_votes[low_ns] = min(
                            vmax_votes[low_ns],
                            self.v_nominal * self.v_warning_ratio)
                    elif self._crossing_yield_only():
                        ps.state           = PairState.WARNING
                        ps.stop_start_time = None
                        vmax_votes[low_ns] = min(
                            vmax_votes[low_ns], self.v_conflict_slow)
                    else:
                        ps.state = PairState.EMERGENCY
                        if ps.stop_start_time is None:
                            ps.stop_start_time = now
                        time_stopped = now - ps.stop_start_time
                        if time_stopped >= self.t_max_stop:
                            ps.override_active     = True
                            ps.override_start_time = now
                            ps.stop_start_time     = None
                            self.get_logger().warn(
                                f'[PRIORITY OVERRIDE] {low_ns} stuck '
                                f'{time_stopped:.1f}s >= T_max={self.t_max_stop}s')
                        else:
                            stop_votes[low_ns] = True

                elif dist <= self.d_warning:
                    ps.state           = PairState.WARNING
                    ps.stop_start_time = None
                    ratio  = ((dist - self.d_emergency) /
                              (self.d_warning - self.d_emergency))
                    ratio  = max(0.0, min(1.0, ratio))
                    v_warn = self.v_nominal * (
                        self.v_warning_ratio +
                        (1.0 - self.v_warning_ratio) * ratio)
                    vmax_votes[low_ns] = min(vmax_votes[low_ns], v_warn)

                else:
                    if dist >= self.d_clear:
                        ps.state           = PairState.CLEAR
                        ps.stop_start_time = None

            if ps.state != prev_state:
                self.get_logger().info(
                    f'[PRIORITY] {low_ns}↔{high_ns}: '
                    f'{prev_state} → {ps.state} | dist={dist:.3f}m')

        # ── [LANE NEG] Head-on detection & lateral offset ────────────────────
        # Matikan dengan lane_negotiation_enabled:=false untuk tes zone/gap murni
        if self.lane_negotiation_enabled:
            wall_now_for_lane = time.time()
            for ns_a, ns_b in combinations(ROBOT_NAMESPACES, 2):
                pose_a = self.robot_poses[ns_a]
                pose_b = self.robot_poses[ns_b]
                if (pose_a is None or pose_b is None
                        or (wall_now_for_lane - self.last_pose_update[ns_a]) > self.stale_timeout
                        or (wall_now_for_lane - self.last_pose_update[ns_b]) > self.stale_timeout):
                    continue
                dist_ab = math.hypot(pose_a[0] - pose_b[0], pose_a[1] - pose_b[1])
                if dist_ab <= D_CLEAR_LANE:
                    self._negotiate_lane(ns_a, ns_b)
                else:
                    self._clear_negotiation(ns_a, ns_b)
        else:
            for ns in ROBOT_NAMESPACES:
                self.lane_offset[ns] = 0.0
        # ── Conflict zone coordination (prediktif, ETA-based) ────────────────
        self._update_conflict_zones(stop_votes, vmax_votes)

        # [PSTOP-OFF] Bila priority-stop dinonaktifkan (mis. merge): hapus SEMUA stop &
        # throttle dari pair/zone/predictive agar tidak ada backlash stop-go → spread minimum.
        # Rem hard-collision di bawah TETAP aktif kecuali hard_collision_brake_enabled=False.
        if not self.priority_stop_enabled:
            stop_votes = {ns: False                          for ns in ROBOT_NAMESPACES}
            vmax_votes = {ns: self.vmax_priority_ceiling     for ns in ROBOT_NAMESPACES}

        # ── Hard collision safety brake: stop KEDUANYA jika sangat kritis ──
        # Ini last-resort — aktif di luar logika priority normal.
        # Berlaku meski high_priority robot atau bahkan keduanya goal_reached=False.
        _hard_pairs = self.all_pairs if self.hard_collision_brake_enabled else []
        for (low_ns, high_ns) in _hard_pairs:
            pose_low  = self.robot_poses[low_ns]
            pose_high = self.robot_poses[high_ns]
            if pose_low is None or pose_high is None:
                continue
            # Formasi akhir (mis. lingkaran R=0.30 → jarak antar-robot ~0.52m) sengaja
            # lebih rapat dari d_hard_collision. Jangan picu rem darurat saat KEDUA
            # robot sudah sampai goal/formasi — itu spacing yang memang diinginkan.
            # Saat transit (belum goal_reached) rem darurat tetap aktif penuh.
            if (self.robot_goal_reached.get(low_ns, False)
                    and self.robot_goal_reached.get(high_ns, False)):
                self._hard_stop_since[(low_ns, high_ns)] = None
                continue
            dist_hard = math.hypot(
                pose_low[0] - pose_high[0],
                pose_low[1] - pose_high[1])
            if dist_hard <= self.d_hard_collision:
                key = (low_ns, high_ns)
                if self._hard_stop_since.get(key) is None:
                    self._hard_stop_since[key] = now
                hard_elapsed = now - self._hard_stop_since[key]
                stop_votes[low_ns]  = True
                if hard_elapsed >= self.t_max_stop:
                    vmax_votes[high_ns] = min(
                        vmax_votes[high_ns],
                        self.v_nominal * self.v_warning_ratio)
                    self.get_logger().warn(
                        f'[HARD COLLISION RELEASE] {low_ns}↔{high_ns} '
                        f'dist={dist_hard:.3f}m masih <= d_hard setelah '
                        f'{hard_elapsed:.1f}s — high-priority dilepas slow',
                        throttle_duration_sec=2.0)
                else:
                    if not (stop_votes[low_ns] and stop_votes[high_ns]):
                        self.get_logger().warn(
                            f'[HARD COLLISION] {low_ns}↔{high_ns} dist={dist_hard:.3f}m '
                            f'<= d_hard={self.d_hard_collision}m — stop keduanya!')
                    stop_votes[high_ns] = True
            else:
                self._hard_stop_since[(low_ns, high_ns)] = None

        # Agregasi: kirim via UDP ke robot + publish ROS topic untuk logger
        for ns in ROBOT_NAMESPACES:
            self._process_stop(ns, stop_votes[ns], now)
            self._send_priority_udp(ns, stop_votes[ns], vmax_votes[ns],
                                    self.lane_offset[ns])
            msg = Bool(); msg.data = stop_votes[ns]
            self.priority_stop_pub[ns].publish(msg)
            vmax_msg = Float32(); vmax_msg.data = float(vmax_votes[ns])
            self.vmax_priority_pub[ns].publish(vmax_msg)

    def _process_stop(self, ns, should_stop, now):
        """Catat event stop/resume untuk evaluasi — identik versi lama."""
        if should_stop and not self.robot_stop[ns]:
            self.robot_stop[ns]    = True
            self._active_stops[ns] = {
                'robot'   : ns,
                'start_t' : now,
                'reason'  : self._get_stop_reason(ns),
                'override': False,
            }
            self.get_logger().info(
                f'[STOP] {ns} | {self._active_stops[ns]["reason"]}')

        elif not should_stop and self.robot_stop[ns]:
            self.robot_stop[ns] = False
            if self._active_stops[ns] is not None:
                event = self._active_stops[ns]
                event['end_t']      = now
                event['duration_s'] = now - event['start_t']
                self.stop_events.append(event)
                self._active_stops[ns] = None
                self.get_logger().info(
                    f'[RESUME] {ns} | dur={event["duration_s"]:.2f}s')

    def _get_stop_reason(self, ns):
        reasons = [
            f'dekat {high_ns}'
            for (low_ns, high_ns), ps in self.pair_states.items()
            if low_ns == ns and ps.state == PairState.EMERGENCY
        ]
        pred_reason = self._predictive_stop_reason.get(ns, '')
        if pred_reason:
            reasons.append(pred_reason)
        return ', '.join(reasons) if reasons else 'unknown'

    def _state_from_dist(self, dist):
        if dist <= self.d_emergency: return PairState.EMERGENCY
        if dist <= self.d_warning:   return PairState.WARNING
        return PairState.CLEAR

    # ═══════════════════════════════════════════════════════════════════════
    # STATISTIK & STATUS — identik versi lama
    # ═══════════════════════════════════════════════════════════════════════

    def get_stop_stats(self):
        if not self.stop_events:
            return {'count': 0, 'mean_duration_s': 0.0, 'total_stop_time_s': 0.0}
        durations = [e['duration_s'] for e in self.stop_events]
        return {
            'count'            : len(durations),
            'mean_duration_s'  : sum(durations) / len(durations),
            'max_duration_s'   : max(durations),
            'total_stop_time_s': sum(durations),
            'override_count'   : sum(1 for e in self.stop_events
                                     if e.get('override')),
            'log'              : self.stop_events,
        }

    def get_min_inter_robot_distance(self):
        min_dist = float('inf')
        min_pair = None
        for (low_ns, high_ns) in self.all_pairs:
            p1 = self.robot_poses[low_ns]
            p2 = self.robot_poses[high_ns]
            if p1 is None or p2 is None:
                continue
            d = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
            if d < min_dist:
                min_dist = d
                min_pair = (low_ns, high_ns)
        return min_dist, min_pair

    def status_report(self):
        min_dist, min_pair = self.get_min_inter_robot_distance()
        pair_summary = []
        for (low_ns, high_ns), ps in self.pair_states.items():
            p1 = self.robot_poses[low_ns]
            p2 = self.robot_poses[high_ns]
            if p1 is None or p2 is None:
                pair_summary.append(f'{low_ns}↔{high_ns}=?')
                continue
            d = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
            pair_summary.append(
                f'{low_ns}↔{high_ns}={d:.2f}m[{ps.state[:3]}]')

        stop_summary = [
            f'{ns}={"STOP" if self.robot_stop[ns] else "GO"}'
            for ns in ROBOT_NAMESPACES
        ]
        self.get_logger().info(
            f'[PRIORITY] {" | ".join(pair_summary)} || '
            f'{" ".join(stop_summary)} || '
            f'min={min_dist:.3f}m'
            + (f' ({min_pair[0]}↔{min_pair[1]})' if min_pair else ''))

    def destroy_node(self):
        self.send_sock.close()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PriorityManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Priority Manager stopped')
        stats = node.get_stop_stats()
        node.get_logger().info(
            f'Stop stats: count={stats["count"]} | '
            f'mean={stats["mean_duration_s"]:.2f}s | '
            f'override={stats["override_count"]}x')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
