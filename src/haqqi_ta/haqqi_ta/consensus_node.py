#!/usr/bin/env python3
"""
Consensus Node — haqqi_ta (versi UDP)
Layer 4: Average Consensus Progress Synchronization

PERUBAHAN dari versi sebelumnya:
  - Subscribe ROS topic diganti → UDP receiver (terima dari udp_sender_node tiap robot)
  - Publish ROS topic diganti  → UDP sender (kirim vmax ke udp_receiver_node tiap robot)
  - Logika consensus TIDAK BERUBAH sama sekali
  - Telemetry republisher: data robot diterima UDP lalu dipublish ulang sebagai ROS topic
    agar experiment_logger (PC domain) bisa subscribe

Progress p_i dihitung dari mission-level remaining (tidak reset saat ganti waypoint):
  p_i = 1 - mission_remaining / mission_total
  mission_remaining = sisa segmen aktif + panjang semua segmen yang belum dikerjakan
  Fallback ke per-segmen (1 - remaining/path_length) jika mission metrics belum tersedia.

Port UDP:
  Terima dari robot:
    robot1 → port 9001
    robot2 → port 9002
    robot3 → port 9003
  Kirim ke robot:
    robot1 ← port 9011
    robot2 ← port 9012
    robot3 ← port 9013

Matematika (tidak berubah):
  p_i[k+1] = p_i[k] + ε · Σ_{j∈N_i} (p_j[k] - p_i[k])
  v_max_i  = clip(v_nominal - K_c · (p_i - p̄), 0, v_nominal)
  Syarat konvergensi: ε < 0.5 (fully-connected 3 robot)

Feasibility-aware ETA:
  Opsional. Default dimatikan karena v_eff DWA dipengaruhi oleh output
  consensus itu sendiri; jika dipakai sebagai ETA utama, v_consensus bisa
  berosilasi floor/ceiling sebelum robot benar-benar bergerak.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32, Int32, String
import math
import socket
import json
import threading
import time
import os
import yaml
from ament_index_python.packages import get_package_share_directory


ROBOT_NAMESPACES = ['robot1', 'robot2', 'robot3']

# Port PC Master listen (terima dari tiap robot)
RECV_PORT_MAP = {
    'robot1': 9001,
    'robot2': 9002,
    'robot3': 9003,
}

# Port tiap robot listen (PC Master kirim vmax ke sini)
SEND_PORT_MAP = {
    'robot1': 9011,
    'robot2': 9012,
    'robot3': 9013,
}


class ConsensusNode(Node):
    def __init__(self):
        super().__init__('consensus_node')

        # ── Parameter ─────────────────────────────────────────────────────
        self.declare_parameter('v_nominal',              0.30)
        self.declare_parameter('vnom_pathlen_scaling',  True)   # [VNOM-PATHLEN] baseline v per-robot ~ panjang lintasan (terpanjang=v_nominal)
        self.declare_parameter('epsilon',                0.03)   # step size consensus (ε < 1/3 untuk 3 robot fully-connected)
        self.declare_parameter('epsilon_deadband',       0.03)   # deadband: |e_i| < epsilon_deadband → v_nominal
        self.declare_parameter('k_consensus',            0.70)
        self.declare_parameter('consensus_rate',        20.0)
        self.declare_parameter('convergence_threshold',  0.02)
        self.declare_parameter('min_path_length',        0.1)
        self.declare_parameter('goal_tolerance',         0.10)   # m — validasi p=1.0 (error goal min 0.1)
        self.declare_parameter('v_consensus_floor',      0.015)
        self.declare_parameter('v_consensus_ceiling',    0.50)   # ruang catch-up untuk robot tertinggal
        # ── [SEG] Segmented progress / koreksi middle-way ─────────────────
        # Dipakai oleh mode 'consensus_seg' (orde-1) & 'consensus_so_seg' (orde-2).
        self.declare_parameter('n_progress_segments',    4)      # K milestone progress (4 = quartile)
        self.declare_parameter('seg_barrier_weight',     2.0)    # bobot soft-barrier overshoot milestone
        self.declare_parameter('epsilon_deadband_seg',   0.008)  # deadband lebih ketat utk mode *_seg (spread->0)
        self.declare_parameter('so_gamma',               0.8)    # bobot pencocokan laju p_dot (orde-2)
        self.declare_parameter('so_dv_max',              0.20)   # otoritas offset kecepatan orde-2 (m/s)
        # ── [DIST] Konsensus TERDISTRIBUSI atas graph komunikasi ──────────
        # Mode 'consensus_dist': tiap robot memakai rata-rata progress TETANGGA
        # (bukan rata-rata global), sisi (edge) berbasis kedekatan -> topologi
        # BERGANTI, plus kopling resiprokal antisimetris di junction
        # (anti-tabrak yang menjaga spread). Membuat graph benar-benar multi-agen.
        self.declare_parameter('dist_comm_radius_m',     6.0)    # r_comm: jarak max sisi komunikasi (graph konsensus)
        self.declare_parameter('dist_junction_radius_m', 1.20)   # r_junc: jarak aktifnya kopling resiprokal
        self.declare_parameter('dist_reciprocal_gain',   0.18)   # amplitudo max ±delta resiprokal (m/s)
        self.declare_parameter('dist_deadband',          0.008)  # deadband progress mode dist (spread->0)
        self.declare_parameter('active_robots',
                               ['robot1', 'robot2', 'robot3'])
        # [M4] L4 sync switch: false → kirim v_nominal ke semua robot tanpa throttling.
        # Sub-variasi convoy async (percobaan 3): set l4_sync_enabled:=false di launch.
        # true  = consensus aktif (default).
        # false = semua robot dapat v_nominal, tidak ada sinkronisasi progress.
        self.declare_parameter('l4_sync_enabled',        True)

        # ── Arrival offset consensus parameter ────────────────────────────
        self.declare_parameter('scenario', 'convoy')
        # Default = 'time_consensus': pengendali utama adalah KONSENSUS pada
        # sisa-waktu tiba (remaining / v_est). Offset tetap relatif: positif
        # berarti robot ini ditargetkan tiba lebih lambat.
        self.declare_parameter('arrival_mode', 'time_consensus')
        # Default = 'consensus_ftso' (v9): finite-time second-order consensus.
        self.declare_parameter('coordination_mode', 'consensus_ftso')
        self.declare_parameter('target_arrival_robot1', 0.0)
        self.declare_parameter('target_arrival_robot2', 0.0)
        self.declare_parameter('target_arrival_robot3', 0.0)
        # arrival_offset_robot*: offset waktu tiba relatif (detik).
        #   Positif = robot ini "dijadwal lebih lambat" → consensus throttle lebih awal.
        #   0.0 = sinkronisasi normal tanpa bias (default untuk semua skenario kecuali variasi convoy).
        self.declare_parameter('arrival_offset_robot1',  0.0)
        self.declare_parameter('arrival_offset_robot2',  0.0)
        self.declare_parameter('arrival_offset_robot3',  0.0)
        # [VNOM-OFFSET] Perkiraan waktu tempuh baseline (detik) untuk konversi
        #   offset(detik) -> reduksi v_nominal saat panjang lintasan belum diketahui.
        #   Bila panjang lintasan tersedia, T0 = L_max / v_nominal dipakai sebagai gantinya.
        self.declare_parameter('arrival_offset_ref_time_s', 75.0)
        # [VNOM-OFFSET] Gain agresivitas stagger pada baseline: v_i = base_i/(1 + gain*off_i/T0).
        #   1.0 = stagger "fisik" (tiba tepat off_i detik lebih lambat). >1.0 memperlebar
        #   selisih kecepatan cruise antar-robot agar feedforward menjaga urutan walau
        #   consensus/DWA berisik (mis. robot tengah tetap unggul dari laggard).
        self.declare_parameter('arrival_offset_vnom_gain', 1.5)
        self.declare_parameter('k_arrival_consensus',    0.06)
        self.declare_parameter('eta_v_ref',              0.0)
        self.declare_parameter('feasibility_aware_eta_enabled', False)
        self.declare_parameter('eta_v_eff_floor',        0.03)
        self.declare_parameter('eta_v_eff_timeout_s',    1.0)
        # [FIX-TIMECONS] Konsensus sisa-waktu (time-to-go) — feedback, bukan ETA open-loop
        self.declare_parameter('k_time',         0.02)   # gain (m/s) per detik selisih sisa-waktu
        self.declare_parameter('epsilon_time_s', 1.0)    # deadband selisih sisa-waktu (detik)
        # [FIX-ETAFILT] Low-pass (EMA) v_hat sebelum dipakai di ETA = remaining/v_hat.
        #   alpha kecil -> lebih halus (lebih banyak filtering); 1.0 = tanpa filter (perilaku lama).
        #   Tujuan: redam hipersensitivitas 1/v_hat yang memicu limit-cycle bang-bang.
        self.declare_parameter('eta_v_filter_alpha', 0.20)   # [FIX-SMOOTH] 0.30->0.20 (selaras changelog Fix B)
        # [FIX-VRATE] Slew-rate limit perubahan vmax consensus (m/s per detik).
        #   Mencegah lompatan floor<->ceiling dalam satu tick. <=0 -> nonaktif (perilaku lama).
        self.declare_parameter('v_consensus_max_rate', 0.50)   # [FIX-SMOOTH] 0.80->0.50 (selaras changelog Fix B)
        self.declare_parameter('v_consensus_catchup_rate', 1.50)  # [FIX-CATCHUP] slew lebih cepat saat ada peer sudah sampai goal
        # [FIX-VOUTLPF] EMA low-pass pada OUTPUT v_consensus (cap akhir) untuk
        #   meredam surging limit-cycle lambat. alpha kecil -> lebih halus;
        #   1.0 = tanpa filter (perilaku lama).
        self.declare_parameter('v_consensus_output_filter_alpha', 0.10)   # [FIX-SMOOTH] 0.15->0.10 (selaras changelog Fix B)
        # [FIX-DSDT] Pakai laju kemajuan lintasan ds/dt (-d(remaining)/dt) sebagai
        #   v_hat ETA, bukan |v|=sqrt(vx^2+vy^2). Lebih konsisten kinematik (abaikan
        #   gerak menyamping holonomic). False -> perilaku lama (dwa_speed_mag).
        self.declare_parameter('progress_speed_enabled', True)
        # Lompatan |d(remaining)| > nilai ini (m) dianggap replan/AMCL-jump -> diabaikan.
        self.declare_parameter('progress_speed_max_jump_m', 0.5)
        # [FIX-ROTETA] Tambah waktu rotasi terminal |heading_err|/omega_max ke ETA
        #   (kinematik theta). Hanya aktif saat remaining <= eta_rot_activation_m.
        self.declare_parameter('eta_rot_term_enabled', False)   # [FINALROT-OFF] rotasi-akhir mati -> jangan tambah ETA rotasi terminal
        self.declare_parameter('eta_rot_omega_max', 0.20)      # rad/s = laju efektif final-align (max_rot_vel*0.8)
        self.declare_parameter('eta_rot_activation_m', 0.30)   # m: aktif hanya dekat terminal
        self.declare_parameter('eta_rot_time_cap_s', 8.0)      # batas atas term (anti blow-up)

        # [M4] Deteksi kegagalan agen untuk exclude dari konsensus q_bar
        self.declare_parameter('agent_failure_detection_enabled', True)
        self.declare_parameter('agent_alive_timeout_s', 1.5)   # heartbeat: telemetry basi -> gagal
        self.declare_parameter('agent_stall_window_s',  4.0)   # motion-stall: lama diam -> gagal
        self.declare_parameter('agent_stall_eps_m',     0.05)  # ambang perpindahan dianggap bergerak
        # [FIX-FAILLATCH] Latch agen GAGAL agar lompatan pose AMCL tak mereset deteksi
        self.declare_parameter('agent_failure_latch_enabled', True)
        self.declare_parameter('agent_failure_confirm_s',     2.0)
        # [FIX-DETECT-FP] Ambang progress untuk MENGUNCI status "sudah tiba" pada
        # detektor: begitu robot pernah mencapai progress ini (atau goal_reached /
        # mission_remaining <= goal_tolerance), robot TIDAK pernah lagi dianggap
        # "harus bergerak", sehingga tidak salah ditandai gagal saat diam di goal.
        self.declare_parameter('agent_arrived_progress_latch', 0.97)

        # IP tiap robot — PC Master kirim vmax ke sini
        # [MOD-IPENV] IP robot bisa diubah dari satu tempat lewat env var (opsional)
        self.declare_parameter('robot1_ip', os.environ.get('ROBOT1_IP', '192.168.0.91'))
        self.declare_parameter('robot2_ip', os.environ.get('ROBOT2_IP', '192.168.0.88'))
        self.declare_parameter('robot3_ip', os.environ.get('ROBOT3_IP', '192.168.0.82'))

        self.v_nominal             = self.get_parameter('v_nominal').value
        self.vnom_pathlen_scaling  = bool(self.get_parameter('vnom_pathlen_scaling').value)
        self.epsilon               = self.get_parameter('epsilon').value
        self.epsilon_deadband      = self.get_parameter('epsilon_deadband').value
        self.k_consensus           = self.get_parameter('k_consensus').value
        self.consensus_rate        = self.get_parameter('consensus_rate').value
        self.convergence_threshold = self.get_parameter('convergence_threshold').value
        self.min_path_length       = self.get_parameter('min_path_length').value
        self.goal_tolerance        = self.get_parameter('goal_tolerance').value
        self.v_consensus_floor     = self.get_parameter('v_consensus_floor').value
        self.v_consensus_ceiling   = self.get_parameter('v_consensus_ceiling').value
        self.l4_sync_enabled       = self.get_parameter('l4_sync_enabled').value
        self.n_progress_segments   = int(self.get_parameter('n_progress_segments').value)
        self.seg_barrier_weight    = float(self.get_parameter('seg_barrier_weight').value)
        self.epsilon_deadband_seg  = float(self.get_parameter('epsilon_deadband_seg').value)
        self.dist_comm_radius_m     = float(self.get_parameter('dist_comm_radius_m').value)
        self.dist_junction_radius_m = float(self.get_parameter('dist_junction_radius_m').value)
        self.dist_reciprocal_gain   = float(self.get_parameter('dist_reciprocal_gain').value)
        self.dist_deadband          = float(self.get_parameter('dist_deadband').value)
        self._latest_comm_graph     = None
        self.so_gamma              = float(self.get_parameter('so_gamma').value)
        self.so_dv_max             = float(self.get_parameter('so_dv_max').value)
        self.scenario              = str(self.get_parameter('scenario').value)
        self.arrival_mode          = str(self.get_parameter('arrival_mode').value).strip() or 'time_consensus'
        coord_param                = str(self.get_parameter('coordination_mode').value).strip()
        self.coordination_mode     = coord_param or self.arrival_mode
        if self.coordination_mode == 'arrival_offset':
            self.coordination_mode = 'arrival_offset_consensus'
        if self.coordination_mode == 'time_offset':
            self.coordination_mode = 'time_offset_consensus'
        if self.coordination_mode == 'scheduler':
            self.get_logger().warn(
                '[CONSENSUS] arrival_mode=scheduler tidak dipakai sebagai kontrol utama. '
                'Fallback ke mode time_consensus agar eksperimen tetap consensus-based.')
            self.coordination_mode = 'time_consensus'
        self.target_arrival        = {
            'robot1': self._as_float(self.get_parameter('target_arrival_robot1').value),
            'robot2': self._as_float(self.get_parameter('target_arrival_robot2').value),
            'robot3': self._as_float(self.get_parameter('target_arrival_robot3').value),
        }
        self.arrival_offset        = {
            'robot1': self._as_float(self.get_parameter('arrival_offset_robot1').value),
            'robot2': self._as_float(self.get_parameter('arrival_offset_robot2').value),
            'robot3': self._as_float(self.get_parameter('arrival_offset_robot3').value),
        }
        self._arrival_offset_explicit = any(
            abs(v) > 1e-6 for v in self.arrival_offset.values())
        # [VNOM-OFFSET] T0 fallback untuk konversi offset detik -> reduksi baseline v.
        self.arrival_offset_ref_time_s = max(1e-3, self._as_float(
            self.get_parameter('arrival_offset_ref_time_s').value, 75.0))
        # [VNOM-OFFSET] gain agresivitas stagger baseline (>=0)
        self.arrival_offset_vnom_gain = max(0.0, self._as_float(
            self.get_parameter('arrival_offset_vnom_gain').value, 1.5))
        self.k_arrival_consensus   = self._as_float(
            self.get_parameter('k_arrival_consensus').value, 0.06)
        eta_ref_param              = self._as_float(
            self.get_parameter('eta_v_ref').value, 0.0)
        self.eta_v_ref             = eta_ref_param if eta_ref_param > 0.0 else self.v_nominal
        self.feasibility_aware_eta_enabled = bool(
            self.get_parameter('feasibility_aware_eta_enabled').value)
        self.eta_v_eff_floor       = max(0.01, self._as_float(
            self.get_parameter('eta_v_eff_floor').value, 0.03))
        self.eta_v_eff_timeout_s   = max(0.1, self._as_float(
            self.get_parameter('eta_v_eff_timeout_s').value, 1.0))
        # [FIX-TIMECONS] gain & deadband konsensus sisa-waktu
        self.k_time                = self._as_float(
            self.get_parameter('k_time').value, 0.02)
        self.epsilon_time_s        = max(0.0, self._as_float(
            self.get_parameter('epsilon_time_s').value, 1.0))
        # [FIX-ETAFILT] EMA alpha untuk v_hat (clamp 0..1; 1.0 = tanpa filter)
        self.eta_v_filter_alpha    = min(1.0, max(0.0, self._as_float(
            self.get_parameter('eta_v_filter_alpha').value, 0.30)))
        # [FIX-VRATE] slew-rate vmax consensus (m/s per s); <=0 menonaktifkan
        self.v_consensus_max_rate  = self._as_float(
            self.get_parameter('v_consensus_max_rate').value, 0.80)
        self.v_consensus_catchup_rate = self._as_float(
            self.get_parameter('v_consensus_catchup_rate').value, 1.50)
        # [FIX-VOUTLPF] alpha EMA output v_consensus (0..1); 1.0 = tanpa filter
        self.v_consensus_output_filter_alpha = min(1.0, max(0.0, self._as_float(
            self.get_parameter('v_consensus_output_filter_alpha').value, 0.15)))
        # [FIX-DSDT] ds/dt sebagai v_hat ETA
        self.progress_speed_enabled = bool(
            self.get_parameter('progress_speed_enabled').value)
        self.progress_speed_max_jump_m = max(0.0, self._as_float(
            self.get_parameter('progress_speed_max_jump_m').value, 0.5))
        # [FIX-ROTETA] term rotasi terminal pada ETA
        self.eta_rot_term_enabled = bool(
            self.get_parameter('eta_rot_term_enabled').value)
        self.eta_rot_omega_max = max(1e-3, self._as_float(
            self.get_parameter('eta_rot_omega_max').value, 0.20))
        self.eta_rot_activation_m = max(0.0, self._as_float(
            self.get_parameter('eta_rot_activation_m').value, 0.30))
        self.eta_rot_time_cap_s = max(0.0, self._as_float(
            self.get_parameter('eta_rot_time_cap_s').value, 8.0))
        # [M4] Parameter detektor kegagalan agen
        self.agent_failure_detection_enabled = bool(
            self.get_parameter('agent_failure_detection_enabled').value)
        self.agent_alive_timeout_s = max(0.2, self._as_float(
            self.get_parameter('agent_alive_timeout_s').value, 1.5))
        self.agent_stall_window_s  = max(0.5, self._as_float(
            self.get_parameter('agent_stall_window_s').value, 4.0))
        self.agent_stall_eps_m     = max(0.0, self._as_float(
            self.get_parameter('agent_stall_eps_m').value, 0.05))
        self.agent_failure_latch_enabled = bool(
            self.get_parameter('agent_failure_latch_enabled').value)
        self.agent_failure_confirm_s = max(0.0, self._as_float(
            self.get_parameter('agent_failure_confirm_s').value, 2.0))
        self.agent_arrived_progress_latch = min(1.0, max(0.5, self._as_float(
            self.get_parameter('agent_arrived_progress_latch').value, 0.97)))

        if (self._uses_scenario_arrival_schedule()
                and all(abs(v) < 1e-6 for v in self.arrival_offset.values())):
            self._load_arrival_schedule_as_offsets()

        active = self.get_parameter('active_robots').value
        global ROBOT_NAMESPACES
        ROBOT_NAMESPACES = list(active)

        self.robot_ip = {
            'robot1': self.get_parameter('robot1_ip').value,
            'robot2': self.get_parameter('robot2_ip').value,
            'robot3': self.get_parameter('robot3_ip').value,
        }

        if self.epsilon >= 0.33:
            self.get_logger().warn(
                f'epsilon={self.epsilon} >= 0.33 (1/N untuk N=3 fully-connected)! '
                f'Consensus mungkin tidak konvergen atau konvergen sangat lambat.')

        # ── State per robot ───────────────────────────────────────────────
        self.p                 = {ns: 0.0   for ns in ROBOT_NAMESPACES}
        self.path_length       = {ns: 0.0   for ns in ROBOT_NAMESPACES}
        self.remaining         = {ns: 0.0   for ns in ROBOT_NAMESPACES}
        self.goal_reached      = {ns: False for ns in ROBOT_NAMESPACES}
        self._goal_reached_lock = threading.Lock()
        self._state_lock       = threading.RLock()
        self.data_valid        = {ns: False for ns in ROBOT_NAMESPACES}
        self.last_update       = {ns: 0.0   for ns in ROBOT_NAMESPACES}
        self.pose_valid        = {ns: False for ns in ROBOT_NAMESPACES}
        self.robot_pose        = {ns: None  for ns in ROBOT_NAMESPACES}
        # [FIX-ROTETA] heading_error terakhir per robot (rad) utk ETA rotasi terminal
        self.robot_heading_err = {ns: None  for ns in ROBOT_NAMESPACES}
        # [M2] priority_stop dari tiap robot — robot yang berstatus STOP dikecualikan
        # dari rata-rata p_bar/q_bar agar robot yang jalan tidak di-throttle.
        self.robot_stopped     = {ns: False for ns in ROBOT_NAMESPACES}
        # [M4] State detektor kegagalan agen
        self.agent_failed     = {ns: False for ns in ROBOT_NAMESPACES}
        # [FIX-DETECT-FP] Latch "sudah tiba" khusus detektor — sekali True, robot
        # tidak lagi dianggap harus bergerak (di-reset tiap trial baru).
        self._arrived_latched = {ns: False for ns in ROBOT_NAMESPACES}
        # [FIX-FAILLATCH] Latch kegagalan + timer konfirmasi (reset tiap trial)
        self._failed_latched  = {ns: False for ns in ROBOT_NAMESPACES}
        self._failed_since    = {ns: None for ns in ROBOT_NAMESPACES}
        self._motion_ref_pose = {}   # ns -> (x, y) terakhir saat terlihat bergerak
        self._motion_ref_t    = {}   # ns -> waktu referensi motion-stall
        # [M3] dwa_active dari tiap robot — disertakan dalam peer_poses ke robot lain.
        self.robot_dwa_active  = {ns: False for ns in ROBOT_NAMESPACES}
        # [MOD-21] cmd_vel terakhir tiap robot (vx,vy,w kerangka badan) — diteruskan
        # dalam peer_poses agar DWA peer bisa memprediksi gerakan & cek footprint OBB.
        self.robot_cmd_vel     = {ns: None for ns in ROBOT_NAMESPACES}
        # Feasibility-aware ETA inputs dari DWA/udp_sender_node.
        self.robot_dwa_vmax_eff = {ns: None for ns in ROBOT_NAMESPACES}
        self.robot_dwa_speed_mag = {ns: None for ns in ROBOT_NAMESPACES}
        # [FIX-ETAFILT] v_hat ter-filter (EMA) per robot untuk estimasi ETA
        self.eta_v_filt          = {ns: None for ns in ROBOT_NAMESPACES}
        # [FIX-VRATE] vmax consensus tick sebelumnya per robot (untuk slew-rate)
        self.last_vmax_cmd       = {ns: None for ns in ROBOT_NAMESPACES}
        # [FIX-VOUTLPF] nilai output v_consensus ter-EMA per robot
        self.vmax_out_filt       = {ns: None for ns in ROBOT_NAMESPACES}
        # [FIX-DSDT] riwayat remaining utk hitung ds/dt per robot
        self._prev_remaining     = {ns: None for ns in ROBOT_NAMESPACES}
        self._prev_remaining_t   = {ns: None for ns in ROBOT_NAMESPACES}
        self.robot_dwa_metric_update = {ns: 0.0 for ns in ROBOT_NAMESPACES}

        # Mission-level metrics (dari global_path_node via UDP)
        self.mission_remaining = {ns: None  for ns in ROBOT_NAMESPACES}
        self.mission_total     = {ns: None  for ns in ROBOT_NAMESPACES}

        # Progress normalization — p0 diambil saat experiment RUNNING dimulai
        # p_norm = (p_raw - p0) / max(1 - p0, 0.01), clamped [0, 1]
        self.p0                = {ns: None  for ns in ROBOT_NAMESPACES}
        self.experiment_running = False

        self._experiment_start_wall = None          # wall clock saat RUNNING
        self.arrival_debug          = {
            'coordination_mode': self.coordination_mode,
            'q_bar': None,
            'robots': {
                ns: {
                    'ETA': None, 'A': None, 'offset': self.arrival_offset.get(ns, 0.0),
                    'target_arrival': self.target_arrival.get(ns, 0.0),
                    'time_left': None,
                    'q': None, 'e': None, 'v_consensus': self.v_nominal,
                    'eta_v': None, 'eta_source': 'none',
                    'included': False,
                }
                for ns in ROBOT_NAMESPACES
            },
        }

        # Metrik evaluasi
        self.convergence_start_time = None
        self.was_converged          = True
        self.convergence_log        = []
        # [FIX-OBS] Snapshot konvergensi otoritatif untuk di-log (single source of
        # truth). _check_convergence mengisi ini setiap siklus; logger memakainya
        # agar max_deviation/converged konsisten & mengecualikan agen gagal.
        self._last_max_deviation = None
        self._last_is_converged  = bool(self.was_converged)

        # ── UDP: Kirim vmax ke tiap robot ─────────────────────────────────
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── UDP: Terima data dari tiap robot ──────────────────────────────
        # Satu socket per robot, masing-masing di thread terpisah
        self.recv_threads = []
        for ns in ROBOT_NAMESPACES:
            port = RECV_PORT_MAP[ns]
            t = threading.Thread(
                target=self._udp_listener,
                args=(ns, port),
                daemon=True)
            t.start()
            self.recv_threads.append(t)
            self.get_logger().info(
                f'UDP listener untuk {ns} di port {port}')

        # ── Publisher progress untuk rqt_plot (opsional, tetap ada) ───────
        self.progress_pub = {
            ns: self.create_publisher(Float32, f'/{ns}/consensus_progress', 10)
            for ns in ROBOT_NAMESPACES
        }

        # ── Telemetry republisher — data robot tersedia sebagai ROS topic di PC ──
        self.telemetry_pub = {
            ns: {
                'amcl_pose'        : self.create_publisher(PoseWithCovarianceStamped, f'/{ns}/amcl_pose',                10),
                'path_length'      : self.create_publisher(Float32,                   f'/{ns}/path_length',              10),
                'remaining_length' : self.create_publisher(Float32,                   f'/{ns}/remaining_length',         10),
                'goal_reached'     : self.create_publisher(Bool,                      f'/{ns}/goal_reached',             10),
                'position_reached' : self.create_publisher(Bool,                      f'/{ns}/position_reached',         10),  # [FIX-POSREACH]
                'fault_active'     : self.create_publisher(Bool,                      f'/{ns}/fault_active',             10),
                'fault_log'        : self.create_publisher(String,                    f'/{ns}/fault_log',                10),
                'waypoint_index'   : self.create_publisher(Int32,                     f'/{ns}/waypoint_index',           10),
                'vmax_consensus'   : self.create_publisher(Float32,                   f'/{ns}/vmax_consensus',           10),
                'mission_remaining': self.create_publisher(Float32,                   f'/{ns}/mission_remaining_length', 10),
                'mission_total'    : self.create_publisher(Float32,                   f'/{ns}/mission_total_length',     10),
                'cmd_vel'          : self.create_publisher(Twist,                     f'/{ns}/cmd_vel',                  10),
                'dwa_mode'         : self.create_publisher(String,                    f'/{ns}/dwa_mode',                 10),
                'dwa_vmax_eff'     : self.create_publisher(Float32,                   f'/{ns}/dwa_vmax_eff',             10),
                'omega_raw'        : self.create_publisher(Float32,                   f'/{ns}/omega_raw',                10),
                'omega_clamped'    : self.create_publisher(Float32,                   f'/{ns}/omega_after_clamp',        10),
                'omega_limit'      : self.create_publisher(Float32,                   f'/{ns}/omega_global_limit',       10),
                'loc_hold'         : self.create_publisher(Bool,                      f'/{ns}/localization_hold_active', 10),
                'plan'             : self.create_publisher(Path,                      f'/{ns}/plan',                     10),
                'local_plan'       : self.create_publisher(Path,                      f'/{ns}/local_plan',               10),  # [MOD-LOCALPLAN]
                'dynobs_debug'     : self.create_publisher(String,                    f'/{ns}/dynamic_obstacle_debug',   10),
                'tracking_mode'    : self.create_publisher(String,                    f'/{ns}/tracking_mode',            10),
                'heading_error'    : self.create_publisher(Float32,                   f'/{ns}/heading_error',            10),
                'dwa_speed_mag'    : self.create_publisher(Float32,                   f'/{ns}/dwa_speed_mag',            10),
                'vmax_prio_robot'  : self.create_publisher(Float32,                   f'/{ns}/vmax_priority_robot',      10),
                'pstop_robot'      : self.create_publisher(Bool,                      f'/{ns}/priority_stop_robot',      10),
                'lane_off_robot'   : self.create_publisher(Float32,                   f'/{ns}/lane_offset_robot',        10),
            }
            for ns in ROBOT_NAMESPACES
        }
        self._coordination_debug_pub = self.create_publisher(
            String, '/coordination_debug', 10)

        # ── Subscriber experiment state — untuk normalisasi progress ─────
        self.create_subscription(
            String, '/experiment_state', self._experiment_state_cb, 10)
        self.create_subscription(
            String, '/experiment_scenario', self._experiment_scenario_cb, 10)

        # ── Timer consensus — 10 Hz ────────────────────────────────────────
        period = 1.0 / self.consensus_rate
        self.create_timer(period, self.consensus_loop)
        self.create_timer(0.5, self.status_report)

        self.get_logger().info(
            f'Consensus Node (UDP) ready | robots={ROBOT_NAMESPACES} | '
            f'mode={self.coordination_mode} | '
            f'ε={self.epsilon} | deadband={self.epsilon_deadband} | '
            f'Kc={self.k_consensus} | Kt={self.k_time} | '
            f'v_nom={self.v_nominal} | '
            f'v=[{self.v_consensus_floor}, {self.v_consensus_ceiling}] m/s | '
            f'{self.consensus_rate}Hz | offsets={self.arrival_offset} | '
            f'targets={self.target_arrival} | '
            f'feasible_eta={self.feasibility_aware_eta_enabled}')

    @staticmethod
    def _as_float(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _uses_scenario_arrival_schedule(self):
        """Mode yang sengaja memakai arrival_schedule YAML sebagai offset."""
        return self.coordination_mode in (
            'arrival_offset_consensus',
            'time_offset_consensus',
            'time_consensus',
        )

    def _load_arrival_schedule_as_offsets(self):
        """Gunakan arrival_schedule sebagai offset relatif, bukan scheduler absolut."""
        try:
            pkg_dir = get_package_share_directory('haqqi_ta')
            yaml_path = os.path.join(pkg_dir, 'param', 'scenarios.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            schedule = (data.get('scenarios', {})
                           .get(self.scenario, {})
                           .get('arrival_schedule', {}))
            targets = {}
            for ns in ROBOT_NAMESPACES:
                val = schedule.get(ns, 0.0)
                if val and float(val) > 0.0:
                    targets[ns] = float(val)
            if len(targets) < 2:
                return
            t0 = min(targets.values())
            for ns, target in targets.items():
                self.arrival_offset[ns] = max(0.0, target - t0)
            self.get_logger().info(
                f'[CONSENSUS] arrival_schedule dipakai sebagai relative offsets '
                f'scenario={self.scenario}: {self.arrival_offset}')
        except Exception as e:
            self.get_logger().warn(
                f'[CONSENSUS] Gagal baca arrival_schedule untuk offset: {e}')

    def _experiment_scenario_cb(self, msg):
        scenario = str(msg.data).strip()
        if not scenario or scenario == self.scenario:
            return
        with self._state_lock:
            old = self.scenario
            self.scenario = scenario
            if self._uses_scenario_arrival_schedule() and not self._arrival_offset_explicit:
                for ns in ROBOT_NAMESPACES:
                    self.arrival_offset[ns] = 0.0
                self._load_arrival_schedule_as_offsets()
                self._reset_arrival_debug()
        self.get_logger().info(
            f'[CONSENSUS] scenario update: {old} → {self.scenario} '
            f'offsets={self.arrival_offset}')

    # ═══════════════════════════════════════════════════════════════════════
    # EXPERIMENT STATE — normalisasi progress
    # ═══════════════════════════════════════════════════════════════════════

    def _experiment_state_cb(self, msg: String):
        with self._state_lock:
            state = msg.data.strip().upper()
            if state == 'RUNNING' and not self.experiment_running:
                self.experiment_running     = True
                self._experiment_start_wall = time.time()
                # Capture p0 dari actual progress saat ini — semua robot
                for ns in ROBOT_NAMESPACES:
                    p_raw = self._compute_actual_p(ns)
                    self.p0[ns] = p_raw
                    self.eta_v_filt[ns] = None
                    self.last_vmax_cmd[ns] = None
                    self.vmax_out_filt[ns] = None
                    self._prev_remaining[ns] = None
                    self._prev_remaining_t[ns] = None
                    self.robot_heading_err[ns] = None
                    self._arrived_latched[ns] = False
                    self._failed_latched[ns] = False
                    self._failed_since[ns] = None
                self.get_logger().info(
                    f'[CONSENSUS] Experiment RUNNING | mode={self.coordination_mode} | '
                    f'p0=' + ', '.join(f'{ns}={self.p0[ns]:.4f}' for ns in ROBOT_NAMESPACES)
                    + f' | offsets={self.arrival_offset}')
            elif state in ('STOP', 'READY'):
                self.experiment_running     = False
                self._experiment_start_wall = None
                for ns in ROBOT_NAMESPACES:
                    self.p0[ns] = None
                    self.eta_v_filt[ns] = None
                    self.last_vmax_cmd[ns] = None
                    self.vmax_out_filt[ns] = None
                    self._prev_remaining[ns] = None
                    self._prev_remaining_t[ns] = None
                    self.robot_heading_err[ns] = None
                    self._arrived_latched[ns] = False
                    self._failed_latched[ns] = False
                    self._failed_since[ns] = None
                self._reset_arrival_debug()

    def _compute_actual_p(self, ns) -> float:
        """Hitung raw progress robot ns (belum dinormalisasi).

        p=1.0 hanya dikunci jika goal_reached DAN mission_remaining kecil.
        Ini mencegah false-positive goal_reached (misal: remaining=0 saat
        path baru dimuat, tapi robot belum benar-benar di final goal).
        """
        if self.goal_reached[ns]:
            mr = self.mission_remaining.get(ns)
            # Validasi: mission_remaining harus ada dan kecil (< goal_tolerance).
            # mr is None → data belum masuk, jangan klaim p=1, fallback ke progress biasa.
            if mr is not None and mr <= self.goal_tolerance:
                return 1.0
            # goal_reached tapi remaining belum terkonfirmasi — fallback ke mission progress
        if (self.mission_remaining.get(ns) is not None
                and self.mission_total.get(ns) is not None
                and self.mission_total[ns] > self.min_path_length):
            return max(0.0, min(1.0,
                1.0 - self.mission_remaining[ns] / self.mission_total[ns]))
        if self.path_length[ns] > self.min_path_length:
            return max(0.0, min(1.0,
                1.0 - self.remaining[ns] / self.path_length[ns]))
        return 0.0

    def _normalize_p(self, ns, p_raw: float) -> float:
        """Normalisasi progress relatif terhadap p0 saat start."""
        p0 = self.p0.get(ns)
        if p0 is None or p0 >= 1.0:
            return p_raw
        return max(0.0, min(1.0, (p_raw - p0) / max(1.0 - p0, 0.01)))


    def _reset_arrival_debug(self):
        self.arrival_debug = {
            'coordination_mode': self.coordination_mode,
            'q_bar': None,
            'robots': {
                ns: {
                    'ETA': None, 'A': None,
                    'offset': self.arrival_offset.get(ns, 0.0),
                    'target_arrival': self.target_arrival.get(ns, 0.0),
                    'time_left': None,
                    'q': None, 'e': None,
                    'v_consensus': self.v_nominal,
                    'eta_v': None,
                    'eta_source': 'none',
                    'included': False,
                }
                for ns in ROBOT_NAMESPACES
            },
        }

    def _agent_alive(self, ns: str) -> bool:
        """[M4] True jika agen dianggap sehat. Heartbeat + motion-stall.
        Tidak memakai /fault_active (ground-truth injector) agar deteksi realistis
        dan latency-nya bisa diukur terhadap ground-truth."""
        if not self.agent_failure_detection_enabled:
            return True
        now = time.time()
        alive = True

        # (1) Heartbeat: telemetry berhenti / comms-loss
        if (now - self.last_update.get(ns, 0.0)) > self.agent_alive_timeout_s:
            alive = False
        else:
            # (2) Motion-stall: HANYA saat robot seharusnya bergerak
            running = self._experiment_start_wall is not None
            # [FIX-DETECT-FP] Kunci status "sudah tiba" agar robot yang diam DI GOAL
            # tidak salah ditandai gagal. Sebelumnya hanya bergantung pada flag
            # goal_reached yang bisa TER-RESET (mirror bidirectional saat robot
            # re-publish False / AMCL lompat dekat goal), sehingga robot yang sudah
            # tiba & berhenti memicu motion-stall -> false-positive (kasus 20 Jun:
            # robot2 di-flag 107-173s padahal tiba 103s). Latch dari goal_reached
            # ATAU progress >= ambang ATAU mission_remaining <= goal_tolerance.
            if not self._arrived_latched.get(ns, False):
                mr = self.mission_remaining.get(ns)
                if (self.goal_reached.get(ns, False)
                        or self.p.get(ns, 0.0) >= self.agent_arrived_progress_latch
                        or (mr is not None and math.isfinite(float(mr))
                            and float(mr) <= self.goal_tolerance)):
                    self._arrived_latched[ns] = True
            arrived = self._arrived_latched.get(ns, False)
            expected_to_move = (running
                                and not arrived
                                and not self.robot_stopped.get(ns, False))
            pose = self.robot_pose.get(ns)
            if not expected_to_move or pose is None:
                self._motion_ref_pose[ns] = (pose['x'], pose['y']) if pose else None
                self._motion_ref_t[ns] = now
            else:
                ref = self._motion_ref_pose.get(ns)
                if ref is None or ref[0] is None:
                    self._motion_ref_pose[ns] = (pose['x'], pose['y'])
                    self._motion_ref_t[ns] = now
                else:
                    dist = math.hypot(pose['x'] - ref[0], pose['y'] - ref[1])
                    if dist >= self.agent_stall_eps_m:
                        self._motion_ref_pose[ns] = (pose['x'], pose['y'])
                        self._motion_ref_t[ns] = now
                    elif (now - self._motion_ref_t.get(ns, now)) > self.agent_stall_window_s:
                        alive = False

        # [FIX-FAILLATCH] Latch status GAGAL setelah dikonfirmasi bertahan
        # agent_failure_confirm_s. Tanpa ini, satu lompatan pose AMCL (re-localize)
        # mereset timer motion-stall -> agen yang sudah gagal permanen ter-"RECOVER"
        # sesaat -> q basi-nya masuk lagi ke q_bar -> max_deviation melonjak &
        # konsensus pecah (kasus split: robot2 di-un-flag t=66.8-70.8s, dev->0.496).
        raw_failed = (not alive)
        if self.agent_failure_latch_enabled:
            if raw_failed:
                if self._failed_since.get(ns) is None:
                    self._failed_since[ns] = now
                if (now - self._failed_since[ns]) >= self.agent_failure_confirm_s:
                    self._failed_latched[ns] = True
            elif not self._failed_latched.get(ns, False):
                self._failed_since[ns] = None
            if self._failed_latched.get(ns, False):
                raw_failed = True
                alive = False

        prev = self.agent_failed.get(ns, False)
        self.agent_failed[ns] = raw_failed
        if raw_failed and (not prev):
            self.get_logger().warn(f'[M4][FAILURE] {ns} terdeteksi GAGAL — exclude dari q_bar')
        elif (not raw_failed) and prev:
            self.get_logger().info(f'[M4][RECOVER] {ns} kembali sehat')
        return alive

    def _mission_remaining_valid(self, ns: str) -> bool:
        L = self.mission_remaining.get(ns)
        return L is not None and math.isfinite(float(L)) and float(L) >= 0.0

    def _sync_remaining_for(self, ns: str) -> tuple[float | None, str]:
        """Sisa jarak untuk sinkronisasi L4: mission-level jika tersedia."""
        if self._mission_remaining_valid(ns):
            return max(0.0, float(self.mission_remaining[ns])), 'mission_remaining'
        path_len = float(self.path_length.get(ns, 0.0))
        if path_len > self.min_path_length:
            rem = max(0.0, float(self.remaining.get(ns, 0.0)))
            return rem, 'segment_remaining'
        return None, 'invalid_remaining'

    def _progress_speed_for(self, ns, remaining):
        # [FIX-DSDT] Laju kemajuan lintasan ds/dt = -d(remaining)/dt.
        # Konsisten kinematik utk robot holonomic: gerak menyamping (vy) TIDAK
        # dihitung sbg kemajuan. Return None jika belum bisa / ada lompatan
        # remaining besar (replan / AMCL-jump) -> caller fallback ke dwa_speed.
        sample_t = self.last_update.get(ns, 0.0) or time.time()
        prev_r = self._prev_remaining.get(ns)
        prev_t = self._prev_remaining_t.get(ns)
        if prev_t is not None and sample_t <= prev_t + 1e-6:
            return None
        self._prev_remaining[ns] = remaining
        self._prev_remaining_t[ns] = sample_t
        if prev_r is None or prev_t is None:
            return None
        dt = sample_t - prev_t
        if dt <= 1e-3 or dt > 2.0:
            return None
        ds = float(prev_r) - float(remaining)   # positif jika maju
        if abs(ds) > self.progress_speed_max_jump_m:
            return None                          # replan/AMCL-jump -> abaikan
        v = ds / dt
        if v < 0.0:
            v = 0.0
        return min(v, self.v_consensus_ceiling)

    def _terminal_rot_time(self, ns, remaining):
        # [FIX-ROTETA] Estimasi waktu rotasi terminal = |heading_err| / omega_max.
        # Hanya aktif saat robot dekat terminal (remaining kecil); di tengah
        # perjalanan heading_error = error path-following, bukan alignment akhir,
        # jadi diabaikan agar tidak mengganggu ETA perjalanan.
        if not self.eta_rot_term_enabled:
            return 0.0
        if remaining is None or remaining > self.eta_rot_activation_m:
            return 0.0
        herr = self.robot_heading_err.get(ns)
        if herr is None or not math.isfinite(float(herr)):
            return 0.0
        t_rot = abs(float(herr)) / max(1e-3, self.eta_rot_omega_max)
        return min(t_rot, self.eta_rot_time_cap_s)

    def _eta_speed_for(self, ns: str, remaining=None) -> tuple[float, str]:
        """Kecepatan ETA per robot; fallback stabil jika telemetry belum fresh."""
        fallback = max(float(self.eta_v_ref), self.eta_v_eff_floor)
        if not self.feasibility_aware_eta_enabled:
            return fallback, 'eta_v_ref'

        age = time.time() - self.robot_dwa_metric_update.get(ns, 0.0)
        if age > self.eta_v_eff_timeout_s:
            return fallback, 'stale_ref'

        # [FIX-DSDT] Utamakan laju kemajuan lintasan ds/dt sebagai v_hat (kinematik
        # holonomic: gerak menyamping tidak dihitung sebagai kemajuan).
        source = None
        value = None
        if self.progress_speed_enabled and remaining is not None:
            ds_dt = self._progress_speed_for(ns, remaining)
            if ds_dt is not None:
                source, value = 'ds_dt', ds_dt

        if value is None:
            # Fallback: magnitudo kecepatan translasi terencana DWA.
            speed = self.robot_dwa_speed_mag.get(ns)
            vmax_eff = self.robot_dwa_vmax_eff.get(ns)
            candidates = []
            if speed is not None and math.isfinite(float(speed)):
                candidates.append(('dwa_speed', abs(float(speed))))
            if vmax_eff is not None and math.isfinite(float(vmax_eff)):
                candidates.append(('dwa_vmax_eff', max(0.0, float(vmax_eff))))
            if not candidates:
                return fallback, 'eta_v_ref'
            source, value = max(candidates, key=lambda item: item[1])

        # [FIX-ETAFILT] Low-pass v_hat agar ETA=remaining/v_hat tidak hipersensitif
        # terhadap noise kecepatan sesaat (sumber utama limit-cycle bang-bang).
        a = self.eta_v_filter_alpha
        prev = self.eta_v_filt.get(ns)
        if a < 1.0 and prev is not None and math.isfinite(float(prev)):
            value = a * value + (1.0 - a) * float(prev)
        self.eta_v_filt[ns] = value
        return max(self.eta_v_eff_floor, value), source

    def _rate_limit_vmax(self, ns, v_target, rate=None):
        # [FIX-VRATE] Batasi laju perubahan vmax consensus untuk memutus
        # limit-cycle bang-bang (floor<->ceiling dalam 1 tick).
        # max_rate<=0 -> nonaktif (kembali ke perilaku lama).
        rate = self.v_consensus_max_rate if rate is None else rate
        prev = self.last_vmax_cmd.get(ns)
        if prev is None:
            prev = self.v_nominal
        if rate is None or rate <= 0.0:
            self.last_vmax_cmd[ns] = v_target
            return v_target
        max_delta = rate / max(1e-6, float(self.consensus_rate))
        delta = v_target - prev
        if delta > max_delta:
            v_out = prev + max_delta
        elif delta < -max_delta:
            v_out = prev - max_delta
        else:
            v_out = v_target
        self.last_vmax_cmd[ns] = v_out
        return v_out

    def _smooth_output_vmax(self, ns, v_target):
        # [FIX-VOUTLPF] EMA pada OUTPUT v_consensus untuk meredam surging
        # (limit-cycle lambat amplitudo besar). alpha kecil -> lebih halus.
        a = self.v_consensus_output_filter_alpha
        prev = self.vmax_out_filt.get(ns)
        if a < 1.0 and (prev is None or not math.isfinite(float(prev))):
            prev = self.v_nominal
        if a < 1.0 and prev is not None and math.isfinite(float(prev)):
            v_out = a * float(v_target) + (1.0 - a) * float(prev)
        else:
            v_out = v_target
        self.vmax_out_filt[ns] = v_out
        return v_out

    def _compute_arrival_offset_vmax(self, active_robots):
        """
        Relative arrival-offset consensus:
          ETA_i = L_remain_i / eta_v_ref
          q_i   = (t_elapsed + ETA_i) - offset_i
          v_i   = v_nominal + k * (q_i - q_bar)
        """
        self._reset_arrival_debug()
        elapsed = 0.0
        if self._experiment_start_wall is not None:
            elapsed = max(0.0, time.time() - self._experiment_start_wall)

        fresh = []
        q_values = {}
        debug = self.arrival_debug['robots']

        for ns in active_robots:
            if self.goal_reached.get(ns, False):
                continue
            # [M2] Exclude robot yang sedang STOP dari perhitungan q_bar
            # agar robot yang jalan tidak di-throttle oleh ETA robot yang berhenti.
            if self.robot_stopped.get(ns, False):
                continue
            # [M4] Exclude agen yang terdeteksi GAGAL dari q_bar
            if not self._agent_alive(ns):
                continue
            if not self._mission_remaining_valid(ns):
                continue
            L = float(self.mission_remaining[ns])
            eta_v, eta_source = self._eta_speed_for(ns, remaining=L)
            ETA = L / eta_v
            ETA += self._terminal_rot_time(ns, L)   # [FIX-ROTETA] waktu rotasi terminal
            A = elapsed + ETA
            offset = self.arrival_offset.get(ns, 0.0)
            q = A - offset
            fresh.append(ns)
            q_values[ns] = q
            debug[ns].update({
                'ETA': ETA,
                'A': A,
                'offset': offset,
                'q': q,
                'eta_v': eta_v,
                'eta_source': eta_source,
                'included': True,
            })

        if len(fresh) < 2:
            self.get_logger().warn(
                f'[L4] arrival_offset_consensus: hanya {len(fresh)} robot '
                f'valid untuk q_bar ({fresh}) — fallback v_nominal',
                throttle_duration_sec=3.0)
            return {}

        q_bar = sum(q_values.values()) / len(q_values)
        self.arrival_debug['q_bar'] = q_bar
        result = {}
        for ns in fresh:
            e_i = q_values[ns] - q_bar
            v_i = self.v_nominal + self.k_arrival_consensus * e_i
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, v_i))
            debug[ns]['e'] = e_i
            debug[ns]['v_consensus'] = v_i
            result[ns] = v_i
        return result

    def _v_nominal_for(self, ns):
        """[VNOM-PATHLEN] Baseline v per-robot agar laju progress (v/L) seragam.

        p_i = jarak/L_i -> pdot_i = v_i/L_i. Dengan v_nominal global yang sama,
        robot lintasan panjang punya pdot lebih kecil -> tertinggal -> spread.
        Solusi: robot TERPANJANG pakai v_nominal penuh, yang lebih pendek
        DIPERLAMBAT proporsional: v_nominal_i = v_nominal * L_i / L_max, sehingga
        pdot_i = v_nominal/L_max sama untuk semua di titik error-nol. Consensus
        tetap bekerja di atas baseline ini untuk mengoreksi gangguan. L dari
        mission_total (fallback path_length); data belum lengkap -> v_nominal.
        """
        if not self.vnom_pathlen_scaling:
            return self.v_nominal
        lengths = {}
        for r in ROBOT_NAMESPACES:
            L = self.mission_total.get(r)
            if L is None or L <= self.min_path_length:
                L = self.path_length.get(r, 0.0)
            if L > self.min_path_length:
                lengths[r] = L
        L_i = lengths.get(ns)
        if not lengths or L_i is None:
            return self.v_nominal
        L_max = max(lengths.values())
        if L_max <= self.min_path_length:
            return self.v_nominal
        return self.v_nominal * (L_i / L_max)

    def _v_nominal_offset_for(self, ns):
        """[VNOM-OFFSET] Baseline v per-robot yang SUDAH memasukkan arrival_offset.

        Tanpa offset identik dengan _v_nominal_for (tiba serempak). Dengan
        offset_i > 0 robot ditargetkan tiba offset_i detik lebih lambat:
            T0   = L_max / v_nominal   (atau arrival_offset_ref_time_s bila panjang
                                        lintasan belum diketahui)
            v_i  = base_i / (1 + off_i / T0)
        dengan base_i = _v_nominal_for(ns) dan off_i = offset RELATIF (dikurangi
        offset minimum, jadi robot acuan tetap kecepatan penuh). Reduksi ini
        membuat pdot_i = 1 / (T0 + off_i) -> stagger kedatangan muncul dari
        baseline (feedforward), bukan hanya dari koreksi consensus.
        """
        base = self._v_nominal_for(ns)
        offs = [max(0.0, self.arrival_offset.get(r, 0.0)) for r in ROBOT_NAMESPACES]
        off_i = max(0.0, self.arrival_offset.get(ns, 0.0)) - (min(offs) if offs else 0.0)
        if off_i <= 1e-6:
            return base
        T0 = None
        lengths = {}
        for r in ROBOT_NAMESPACES:
            L = self.mission_total.get(r)
            if L is None or L <= self.min_path_length:
                L = self.path_length.get(r, 0.0)
            if L > self.min_path_length:
                lengths[r] = L
        if lengths and self.v_nominal > 1e-6:
            T0 = max(lengths.values()) / self.v_nominal
        if T0 is None or T0 <= 1e-6:
            T0 = self.arrival_offset_ref_time_s
        return base / (1.0 + self.arrival_offset_vnom_gain * off_i / T0)

    def _compute_progress_consensus_vmax(self, active_robots, actual_p, p_bar):
        """Progress consensus lama, tapi memakai actual mission progress."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        for ns in active_robots:
            if (self.goal_reached.get(ns, False)
                    or self.robot_stopped.get(ns, False)
                    or not self._agent_alive(ns)):
                continue
            e_i = actual_p.get(ns, 0.0) - p_bar
            v_nom = self._v_nominal_for(ns)
            if abs(e_i) < self.epsilon_deadband:
                v_i = v_nom
            else:
                v_i = v_nom - self.k_consensus * e_i
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, v_i))
            debug[ns].update({
                'e': e_i,
                'v_consensus': v_i,
                'eta_source': 'progress_consensus',
                'included': True,
            })
            result[ns] = v_i
        return result

    def _interrobot_dist(self, a, b):
        """[DIST] Jarak Euclidean antar dua robot dari pose UDP terakhir (None bila tak ada)."""
        pa = self.robot_pose.get(a)
        pb = self.robot_pose.get(b)
        if not pa or not pb:
            return None
        return math.hypot(pa['x'] - pb['x'], pa['y'] - pb['y'])

    def _build_comm_graph(self, members):
        """[DIST] Bangun graph komunikasi berbasis kedekatan + sub-layer junction.
        - comm_adj[i] = tetangga j dengan jarak <= dist_comm_radius_m. Pose basi/None
          -> tidak ada sisi; inilah yang membuat topologi BERGANTI (switching).
        - junc_edges  = pasangan (i,j) dengan jarak <= dist_junction_radius_m, tempat
          kopling resiprokal anti-tabrak aktif.
        Mengembalikan (comm_adj, junc_edges, dists)."""
        r_comm = float(getattr(self, 'dist_comm_radius_m', 6.0))
        r_junc = float(getattr(self, 'dist_junction_radius_m', 1.2))
        comm_adj = {ns: [] for ns in members}
        junc_edges = []
        dists = {}
        for ii in range(len(members)):
            for jj in range(ii + 1, len(members)):
                a, b = members[ii], members[jj]
                d = self._interrobot_dist(a, b)
                if d is None:
                    continue
                dists[(a, b)] = d
                if d <= r_comm:
                    comm_adj[a].append(b)
                    comm_adj[b].append(a)
                if d <= r_junc:
                    junc_edges.append((a, b))
        return comm_adj, junc_edges, dists

    def _compute_distributed_consensus_vmax(self, active_robots, actual_p):
        """[DIST] Konsensus progress TERDISTRIBUSI + kopling resiprokal junction.

        Tiap robot i:
          p_bar_i = rata-rata progress atas {i} ∪ tetangga-komunikasi(i)
          e_i     = p_i - p_bar_i
          v_i     = v_nom - Kc*e_i          (deadband dist_deadband)   [konsensus lokal]
                    + delta_recip_i         (antisimetris di junction)  [anti-tabrak]
        delta_recip antisimetris (a:+δ, b:−δ) sehingga rata-rata progress kolektif
        TERJAGA -> spread tidak dikorbankan. Robot dengan progress lebih tinggi
        dipercepat untuk membersihkan junction, yang di belakang mengalah halus.
        δ berskala mulus dengan kedekatan (kontinu, tanpa stop-go). Graph di-log
        tiap tick untuk analisis topologi berganti (Laplacian/konektivitas)."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        members = [ns for ns in active_robots if not self._so_skip(ns)]
        if len(members) < 1:
            self._latest_comm_graph = {'comm': {}, 'junction': [], 't': time.time()}
            return result
        db = float(getattr(self, 'dist_deadband', self.epsilon_deadband))
        kr = float(getattr(self, 'dist_reciprocal_gain', 0.18))
        r_junc = float(getattr(self, 'dist_junction_radius_m', 1.2))

        comm_adj, junc_edges, dists = self._build_comm_graph(members)

        # p_bar LOKAL per robot (rata-rata tetangga + diri sendiri) = inti distributif
        p_bar_local = {}
        for ns in members:
            grp = [ns] + comm_adj.get(ns, [])
            p_bar_local[ns] = sum(actual_p.get(m, 0.0) for m in grp) / len(grp)

        # Kopling resiprokal antisimetris di sisi junction (zero-sum per pasangan)
        recip = {ns: 0.0 for ns in members}
        for (a, b) in junc_edges:
            d = dists.get((a, b))
            if d is None:
                continue
            prox = max(0.0, min(1.0, (r_junc - d) / max(1e-6, r_junc)))  # 0 di r_junc, 1 saat berimpit
            mag = kr * prox
            if actual_p.get(a, 0.0) >= actual_p.get(b, 0.0):
                recip[a] += mag; recip[b] -= mag
            else:
                recip[b] += mag; recip[a] -= mag

        for ns in members:
            e_i = actual_p.get(ns, 0.0) - p_bar_local[ns]
            if abs(e_i) < db:
                v_i = self.v_nominal
            else:
                v_i = self.v_nominal - self.k_consensus * e_i
            v_i = v_i + recip[ns]
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, v_i))
            debug[ns].update({
                'e': e_i, 'v_consensus': v_i,
                'eta_source': 'consensus_dist', 'included': True,
                'p_bar_local': p_bar_local[ns],
                'n_neighbors': len(comm_adj.get(ns, [])),
                'recip': recip[ns],
            })
            result[ns] = v_i

        # Snapshot graph utk logging/analisis (topologi berganti)
        self._latest_comm_graph = {
            't': time.time(),
            'comm': {ns: list(comm_adj.get(ns, [])) for ns in members},
            'junction': [list(e) for e in junc_edges],
            'dists': {f'{a}-{b}': round(dv, 3) for (a, b), dv in dists.items()},
        }
        return result

    def _compute_progress_consensus_offset_vmax(self, active_robots, actual_p, p_bar):
        """[OFFSET] Progress consensus dengan offset GANDA (baseline + consensus).

        v_i = v_nominal_offset_i - k*e_i. Offset masuk di DUA tempat:
          (1) baseline v_nominal_offset_i (feedforward) -> robot ber-offset besar
              cruise lebih lambat sehingga stagger muncul alami;
          (2) target progress consensus tiap robot DIGESER sesuai arrival_offset:
            shift_i  = (offset_i - mean_offset) * pdot_bar   # detik -> fraksi progress
            e_i      = actual_p_i - (p_bar - shift_i)
        Robot dengan offset lebih besar sengaja dijaga LEBIH LAMBAT (progress
        lebih rendah) sehingga tiba ~offset_i detik setelah robot acuan (offset 0).
        Konversi detik->progress memakai laju progress rata-rata pdot_bar yang
        sudah dihitung tiap tick. Geseran zero-sum (dikurangi mean_offset) supaya
        tidak menggeser kecepatan kolektif, hanya stagger relatif antar-robot.
        """
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        members = [ns for ns in active_robots
                   if not (self.goal_reached.get(ns, False)
                           or self.robot_stopped.get(ns, False)
                           or not self._agent_alive(ns))]
        # [OFFSET-FIX] Acuan offset dihitung atas SEMUA robot (set TETAP), bukan hanya
        #   members. Saat satu robot fault/stop keluar dari members, acuan TIDAK boleh
        #   bergeser: kalau bergeser, robot tengah (mis. R2) ikut di-throttle saat
        #   laggard sempat drop, sehingga gagal unggul dari laggard. Set tetap menjaga
        #   stagger relatif konsisten sepanjang run (tetap zero-sum di set penuh).
        _all_offsets = [self.arrival_offset.get(r, 0.0) for r in ROBOT_NAMESPACES]
        mean_off = (sum(_all_offsets) / len(_all_offsets)) if _all_offsets else 0.0
        pdot = max(0.0, getattr(self, '_pdot_bar', 0.0))  # fraksi progress / detik (jangan negatif)
        for ns in members:
            offset = self.arrival_offset.get(ns, 0.0)
            shift = (offset - mean_off) * pdot
            target = p_bar - shift
            e_i = actual_p.get(ns, 0.0) - target
            # [VNOM-OFFSET] baseline v sudah memasukkan offset (feedforward stagger);
            #   consensus tetap bekerja di atasnya dengan target progress yang digeser.
            v_nom = self._v_nominal_offset_for(ns)
            if abs(e_i) < self.epsilon_deadband:
                v_i = v_nom
            else:
                v_i = v_nom - self.k_consensus * e_i
            v_i = max(self.v_consensus_floor,
                      min(self.v_consensus_ceiling, v_i))
            debug[ns].update({
                'e': e_i,
                'offset': offset,
                'v_consensus': v_i,
                'eta_source': 'consensus_offset_vnom',
                'included': True,
            })
            result[ns] = v_i
        return result

    def _so_skip(self, ns):
        return (self.goal_reached.get(ns, False)
                or self.robot_stopped.get(ns, False)
                or not self._agent_alive(ns))

    @staticmethod
    def _sgnpow(x, a):
        if x > 0.0:
            return (x ** a)
        if x < 0.0:
            return -((-x) ** a)
        return 0.0

    def _compute_second_order_progress_vmax(self, active_robots, actual_p, p_bar):
        """[V7-SO] Konsensus ORDE-2 pada progress: samakan posisi (p) DAN laju (p_dot).
        State offset-kecepatan u_i diintegrasikan -> perintah halus, anti-jerk.
        du = -(kp*e_p + gamma*e_v) dt ; v = v_nominal + u ; u disaturasi +-dv_max."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        if not hasattr(self, '_so_u'):
            self._so_u = {}
        _kp = self.k_consensus      # gain posisi (pakai gain progress yg sudah ada)
        _gamma = 0.8                # bobot pencocokan laju
        _dv_max = 0.20              # otoritas offset kecepatan (m/s)
        _dt = 1.0 / max(1e-6, float(self.consensus_rate))
        pdot = getattr(self, '_pdot', {})
        pdot_bar = getattr(self, '_pdot_bar', 0.0)
        for ns in active_robots:
            if self._so_skip(ns):
                self._so_u[ns] = 0.0
                continue
            e_p = actual_p.get(ns, 0.0) - p_bar
            e_v = pdot.get(ns, 0.0) - pdot_bar
            u_prev = self._so_u.get(ns, 0.0)
            u_new = max(-_dv_max, min(_dv_max, u_prev - (_kp * e_p + _gamma * e_v) * _dt))
            self._so_u[ns] = u_new
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, self.v_nominal + u_new))
            debug[ns].update({'e': e_p, 'pdot': pdot.get(ns, 0.0), 'edot': e_v,
                              'u_cmd': u_new, 'v_consensus': v_i,
                              'eta_source': 'consensus_so', 'included': True})
            result[ns] = v_i
        return result

    def _segment_overshoot(self, ns, actual_p, members):
        """[SEG] Soft mid-way barrier untuk koreksi di tengah jalan.
        Bagi progress [0,1] jadi K=n_progress_segments segmen. 'ceil' = milestone
        berikutnya tepat di atas progress robot PALING LAMBAT. 'over' = seberapa
        jauh robot ini sudah melewati ceil itu (0 bila belum). 'over' dipakai untuk
        perlambatan halus proporsional sehingga pemimpin 'menunggu' laggard di tiap
        milestone, lalu rilis begitu laggard maju (ceil naik). Re-sync berulang di
        K titik -> akumulasi error tidak menumpuk -> spread akhir mendekati 0.
        Halus & kontinu (tanpa stop-go, tanpa pangkat pecahan)."""
        K = max(1, int(getattr(self, 'n_progress_segments', 4)))
        seg = 1.0 / K
        if members:
            p_min = min(actual_p.get(m, 0.0) for m in members)
        else:
            p_min = 0.0
        ceil = min(1.0, (math.floor(p_min / seg + 1e-9) + 1) * seg)
        over = max(0.0, actual_p.get(ns, 0.0) - ceil)
        return over, ceil

    def _compute_progress_consensus_seg_vmax(self, active_robots, actual_p, p_bar):
        """[SEG-O1] Orde-1 progress consensus + koreksi middle-way (segmented).
        v = v_nom - Kc*(e_i + w_seg*over_i). Suku over_i (dari _segment_overshoot)
        menahan pemimpin di tiap milestone -> sinkronisasi ulang di tengah jalan.
        Deadband memakai epsilon_deadband_seg (lebih ketat) agar sisa spread -> 0."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        members = [ns for ns in active_robots if not self._so_skip(ns)]
        db = float(getattr(self, 'epsilon_deadband_seg', self.epsilon_deadband))
        w = float(getattr(self, 'seg_barrier_weight', 2.0))
        for ns in members:
            e_i = actual_p.get(ns, 0.0) - p_bar
            over, ceil = self._segment_overshoot(ns, actual_p, members)
            e_eff = e_i + w * over
            v_nom = self._v_nominal_for(ns)
            if abs(e_eff) < db:
                v_i = v_nom
            else:
                v_i = v_nom - self.k_consensus * e_eff
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, v_i))
            debug[ns].update({
                'e': e_i, 'v_consensus': v_i,
                'eta_source': 'progress_consensus_seg', 'included': True,
                'seg_ceil': ceil, 'seg_over': over,
            })
            result[ns] = v_i
        return result

    def _compute_second_order_progress_seg_vmax(self, active_robots, actual_p, p_bar):
        """[SEG-O2] Orde-2 (cocokkan posisi p DAN laju p_dot) + koreksi middle-way.
        du = -(kp*(e_p + w_seg*over) + gamma*e_v) dt ; v = v_nom + u (disaturasi +-dv_max).
        Menggabungkan kehalusan/anti-jerk orde-2 dengan re-sync milestone -> spread -> 0."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        if not hasattr(self, '_so_seg_u'):
            self._so_seg_u = {}
        _kp = self.k_consensus
        _gamma = float(getattr(self, 'so_gamma', 0.8))
        _dv_max = float(getattr(self, 'so_dv_max', 0.20))
        _dt = 1.0 / max(1e-6, float(self.consensus_rate))
        w = float(getattr(self, 'seg_barrier_weight', 2.0))
        pdot = getattr(self, '_pdot', {})
        pdot_bar = getattr(self, '_pdot_bar', 0.0)
        members = [ns for ns in active_robots if not self._so_skip(ns)]
        for ns in active_robots:
            if self._so_skip(ns):
                self._so_seg_u[ns] = 0.0
        for ns in members:
            e_p = actual_p.get(ns, 0.0) - p_bar
            e_v = pdot.get(ns, 0.0) - pdot_bar
            over, ceil = self._segment_overshoot(ns, actual_p, members)
            e_p_eff = e_p + w * over
            u_prev = self._so_seg_u.get(ns, 0.0)
            u_new = max(-_dv_max, min(_dv_max, u_prev - (_kp * e_p_eff + _gamma * e_v) * _dt))
            self._so_seg_u[ns] = u_new
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, self.v_nominal + u_new))
            debug[ns].update({
                'e': e_p, 'pdot': pdot.get(ns, 0.0), 'edot': e_v,
                'u_cmd': u_new, 'v_consensus': v_i,
                'eta_source': 'consensus_so_seg', 'included': True,
                'seg_ceil': ceil, 'seg_over': over,
            })
            result[ns] = v_i
        return result

    def _compute_finite_time_progress_vmax(self, active_robots, actual_p, p_bar):
        """[V8-FT] Konsensus FINITE-TIME pada progress: v = v_nom - k*|e|^a*sign(e), 0<a<1.
        Pangkat<1 menjaga 'gigitan' saat e kecil -> deviasi dipaksa 0 dlm waktu terhingga."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        _a = 0.5
        _k = self.k_consensus
        pdot = getattr(self, '_pdot', {})
        pdot_bar = getattr(self, '_pdot_bar', 0.0)
        for ns in active_robots:
            if self._so_skip(ns):
                continue
            e_i = actual_p.get(ns, 0.0) - p_bar
            if abs(e_i) < self.epsilon_deadband:
                v_i = self.v_nominal
            else:
                v_i = self.v_nominal - _k * self._sgnpow(e_i, _a)
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, v_i))
            debug[ns].update({'e': e_i, 'pdot': pdot.get(ns, 0.0),
                              'edot': pdot.get(ns, 0.0) - pdot_bar, 'v_consensus': v_i,
                              'eta_source': 'consensus_ft', 'included': True})
            result[ns] = v_i
        return result

    def _compute_ft_second_order_progress_vmax(self, active_robots, actual_p, p_bar):
        """[V9-FTSO] Gabungan ORDE-2 + FINITE-TIME pada progress.
        du = -(kp*phi_a(e_p) + gamma*phi_a(e_v)) dt ; v = v_nom + u. Halus + konvergen tepat-waktu."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        if not hasattr(self, '_ftso_u'):
            self._ftso_u = {}
        _kp = self.k_consensus
        _gamma = 0.8
        _a = 0.5
        _dv_max = 0.20
        _dt = 1.0 / max(1e-6, float(self.consensus_rate))
        pdot = getattr(self, '_pdot', {})
        pdot_bar = getattr(self, '_pdot_bar', 0.0)
        for ns in active_robots:
            if self._so_skip(ns):
                self._ftso_u[ns] = 0.0
                continue
            e_p = actual_p.get(ns, 0.0) - p_bar
            e_v = pdot.get(ns, 0.0) - pdot_bar
            u_prev = self._ftso_u.get(ns, 0.0)
            du = -(_kp * self._sgnpow(e_p, _a) + _gamma * self._sgnpow(e_v, _a)) * _dt
            u_new = max(-_dv_max, min(_dv_max, u_prev + du))
            self._ftso_u[ns] = u_new
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, self.v_nominal + u_new))
            debug[ns].update({'e': e_p, 'pdot': pdot.get(ns, 0.0), 'edot': e_v,
                              'u_cmd': u_new, 'v_consensus': v_i,
                              'eta_source': 'consensus_ftso', 'included': True})
            result[ns] = v_i
        return result

    def _compute_fixed_time_progress_vmax(self, active_robots, actual_p, p_bar):
        """[V10-FXT] Konsensus FIXED-TIME pada progress: v = v_nom - k*(|e|^a*sgn + |e|^b*sgn), a<1<b.
        Batas-atas waktu konvergensi TETAP, independen kondisi awal."""
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        _a = 0.5
        _b = 1.6
        _k = self.k_consensus
        pdot = getattr(self, '_pdot', {})
        pdot_bar = getattr(self, '_pdot_bar', 0.0)
        for ns in active_robots:
            if self._so_skip(ns):
                continue
            e_i = actual_p.get(ns, 0.0) - p_bar
            if abs(e_i) < self.epsilon_deadband:
                v_i = self.v_nominal
            else:
                v_i = self.v_nominal - _k * (self._sgnpow(e_i, _a) + self._sgnpow(e_i, _b))
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, v_i))
            debug[ns].update({'e': e_i, 'pdot': pdot.get(ns, 0.0),
                              'edot': pdot.get(ns, 0.0) - pdot_bar, 'v_consensus': v_i,
                              'eta_source': 'consensus_fxt', 'included': True})
            result[ns] = v_i
        return result

    def _compute_remaining_time_consensus_vmax(self, active_robots):
        """Konsensus sisa-waktu tiba, dengan offset relatif opsional.

        t_remain_i = remaining_i / v_est_i.
        q_i = t_remain_i - offset_i.
        e_t_i = q_i - q_bar. e_t>0 (telat) -> percepat; e_t<0 -> perlambat.
        Offset positif berarti robot ditargetkan tiba lebih lambat.
        """
        self._reset_arrival_debug()
        result = {}
        debug = self.arrival_debug['robots']
        q_values = {}
        time_info = {}
        for ns in active_robots:
            if (self.goal_reached.get(ns, False)
                    or self.robot_stopped.get(ns, False)
                    or not self._agent_alive(ns)):
                continue
            rem, rem_src = self._sync_remaining_for(ns)
            if rem is None:
                continue
            v_hat, src = self._eta_speed_for(ns, remaining=rem)
            t_i = rem / max(v_hat, self.eta_v_eff_floor)
            t_i += self._terminal_rot_time(ns, rem)   # [FIX-ROTETA] waktu rotasi terminal
            offset = self.arrival_offset.get(ns, 0.0)
            q_i = t_i - offset
            q_values[ns] = q_i
            time_info[ns] = (t_i, v_hat, src, rem, rem_src, offset)
        if not q_values:
            return result
        # [V2-LAGGARD] referensi = ETA robot paling lambat (sehat) via soft-max (LSE).
        # Set included sudah mengecualikan robot stop/gagal -> anti-deadlock.
        _tau = 1.5   # detik; kecil -> mendekati max keras
        _qs = list(q_values.values())
        _m = max(_qs)
        q_bar = _m + _tau * math.log(sum(math.exp((x - _m) / _tau) for x in _qs) / len(_qs))
        self.arrival_debug['q_bar'] = q_bar
        for ns, (t_i, v_hat, src, rem, rem_src, offset) in time_info.items():
            q_i = q_values[ns]
            e_t = q_i - q_bar
            # [V6-COMBO] normalisasi gain (V1) + PI anti-windup (V3); dipakai dgn referensi laggard (V2)
            if not hasattr(self, '_pi_I'):
                self._pi_I = {}
            _e_scale = 6.0; _dv_max = 0.22; _kI = 0.02; _lam = 0.98; _Imax = 0.75
            _dt = 1.0 / max(1e-6, float(self.consensus_rate))
            _I_prev = self._pi_I.get(ns, 0.0)
            if abs(e_t) < self.epsilon_time_s:
                _I_new = _lam * _I_prev
                v_unsat = self.v_nominal + _kI * _I_new
            else:
                _ehat = max(-1.0, min(1.0, e_t / _e_scale))
                _I_new = max(-_Imax, min(_Imax, _lam * _I_prev + e_t * _dt))
                v_unsat = self.v_nominal + _dv_max * _ehat + _kI * _I_new
            v_i = max(self.v_consensus_floor, min(self.v_consensus_ceiling, v_unsat))
            if v_unsat != v_i:
                _I_new = _I_prev
            self._pi_I[ns] = _I_new
            debug[ns].update({
                'e': e_t,
                'q': q_i,
                'offset': offset,
                'time_left': t_i,
                'eta_v': v_hat,
                'v_consensus': v_i,
                'eta_source': f'time_consensus:{src}:{rem_src}',
                'included': True,
            })
            result[ns] = v_i
        return result

    def _coordination_debug_payload(self, actual_p=None, p_bar=None):
        robots_debug = {
            ns: dict(values)
            for ns, values in self.arrival_debug.get('robots', {}).items()
        }
        payload = {
            't': time.time(),
            'coordination_mode': self.coordination_mode,
            'q_bar': self.arrival_debug.get('q_bar'),
            'p_bar': p_bar,
            'robots': robots_debug,
        }
        # [FIX-OBS] Plumbing status deteksi agen gagal [M4] + metrik konvergensi
        # otoritatif agar terekam di CSV (bukti F1/F2 ablation & latency deteksi).
        payload['failed'] = {
            ns: bool(self.agent_failed.get(ns, False)) for ns in ROBOT_NAMESPACES
        }
        payload['detection_enabled'] = bool(self.agent_failure_detection_enabled)
        payload['consensus_max_deviation'] = self._last_max_deviation
        payload['consensus_converged'] = (
            None if self._last_is_converged is None
            else bool(self._last_is_converged))
        if actual_p is not None:
            payload['progress'] = dict(actual_p)
        cg = getattr(self, '_latest_comm_graph', None)
        if cg is not None:
            payload['comm_graph'] = cg   # [DIST] topologi berganti utk analisis multi-agen
        return payload

    def _publish_coordination_debug_payload(self, payload):
        if not payload:
            return
        msg = String()
        msg.data = json.dumps(payload)
        self._coordination_debug_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # UDP RECEIVER — satu thread per robot
    # ═══════════════════════════════════════════════════════════════════════

    def _udp_listener(self, ns, port):
        """
        Thread: terima paket UDP dari udp_sender_node di robot ns.
        Paket format JSON:
          {
            'robot_ns'        : 'robot1',
            'pose'            : {x, y, yaw, sigma_x, sigma_y},
            'remaining_length': float,
            'path_length'     : float,
            'goal_reached'    : bool
          }
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', port))
        sock.settimeout(1.0)

        self.get_logger().info(f'[{ns}] UDP listener aktif di port {port}')

        while rclpy.ok():
            try:
                # [FIX-UDPBUF] Samakan dgn udp_receiver di robot (128KB). Buffer
                # lama 16KB hanya ~2.6KB di atas paket worst-case (path 300 titik +
                # fault_log_events + dynamic_obstacle_debug). Datagram > buffer akan
                # DIPOTONG recvfrom -> json.loads gagal -> SELURUH paket dibuang
                # (termasuk pose) -> pose hilang di PC. Perbesar utk hilangkan risiko.
                data, _ = sock.recvfrom(131072)
                packet  = json.loads(data.decode('utf-8'))

                # Update state dari paket yang diterima
                self._process_udp_packet(ns, packet)

            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                self.get_logger().warn(f'[{ns}] JSON error: {e}')
            except Exception as e:
                self.get_logger().warn(f'[{ns}] UDP recv error: {e}')

        sock.close()

    def _process_udp_packet(self, ns, packet):
        """Proses paket UDP dari robot — update state lokal."""
        with self._state_lock:

            # ── path_length ───────────────────────────────────────��─���──────
            path_len = packet.get('path_length')
            if path_len is not None and float(path_len) > self.min_path_length:
                prev_valid = self.data_valid[ns]
                self.path_length[ns] = float(path_len)
                self.data_valid[ns]  = True
                if not prev_valid:
                    self.get_logger().info(
                        f'[{ns}] Path baru: {path_len:.2f}m — consensus aktif')

            # ── remaining_length → hitung p_i ─────────────────────────────
            remaining = packet.get('remaining_length')
            if remaining is not None and self.data_valid[ns]:
                remaining = max(0.0, float(remaining))
                self.remaining[ns] = remaining
                total = self.path_length[ns]
                if total > self.min_path_length:
                    self.p[ns] = max(0.0, min(1.0, 1.0 - (remaining / total)))

            # ── goal_reached — bidirectional mirror dari robot ─────────────
            # Penting: harus bisa kembali ke False saat trial baru dimulai.
            # One-way lock (hanya True) menyebabkan p=1.0 stale antar trial.
            goal_reached_val = packet.get('goal_reached')
            if goal_reached_val is not None:
                with self._goal_reached_lock:
                    if bool(goal_reached_val) and not self.goal_reached[ns]:
                        self.goal_reached[ns] = True
                        self.p[ns]            = 1.0
                        self.get_logger().info(
                            f'[{ns}] Goal reached — p dikunci ke 1.0')
                    elif not bool(goal_reached_val) and self.goal_reached[ns]:
                        # Robot sudah reset (waypoint baru masuk) → ikuti
                        self.goal_reached[ns] = False
                        self.get_logger().info(
                            f'[{ns}] Goal reached RESET — trial baru dimulai')

            # ── mission-level metrics ──────────────────────────────────────
            mission_rem = packet.get('mission_remaining_length')
            if mission_rem is not None:
                self.mission_remaining[ns] = max(0.0, float(mission_rem))

            mission_tot = packet.get('mission_total_length')
            if mission_tot is not None and float(mission_tot) > self.min_path_length:
                self.mission_total[ns] = float(mission_tot)

            self.last_update[ns] = time.time()
            self.pose_valid[ns]  = True
            pose = packet.get('pose')
            if pose is not None:
                self.robot_pose[ns] = {
                    'robot': ns,
                    'x': float(pose.get('x', 0.0)),
                    'y': float(pose.get('y', 0.0)),
                    'theta': float(pose.get('yaw', 0.0)),
                    'stamp': self.last_update[ns],
                }
            # [M2] Track priority_stop untuk exclude dari p_bar/q_bar saat STOP
            pstop = packet.get('priority_stop_robot')
            if pstop is not None:
                self.robot_stopped[ns] = bool(pstop)
            # [M3] Track dwa_active untuk disertakan dalam peer_poses ke robot lain
            dwa_act = packet.get('dwa_active')
            if dwa_act is not None:
                self.robot_dwa_active[ns] = bool(dwa_act)
            # [MOD-21] Simpan cmd_vel terakhir agar bisa diteruskan ke peer (prediksi gerakan)
            cmd_vel_pkt = packet.get('cmd_vel')
            if cmd_vel_pkt is not None:
                self.robot_cmd_vel[ns] = {
                    'vx': float(cmd_vel_pkt.get('vx', 0.0)),
                    'vy': float(cmd_vel_pkt.get('vy', 0.0)),
                    'w':  float(cmd_vel_pkt.get('w',  0.0)),
                }
            dwa_vmax_eff = packet.get('dwa_vmax_eff')
            dwa_speed_mag = packet.get('dwa_speed_mag')
            if dwa_vmax_eff is not None:
                self.robot_dwa_vmax_eff[ns] = max(0.0, float(dwa_vmax_eff))
                self.robot_dwa_metric_update[ns] = self.last_update[ns]
            if dwa_speed_mag is not None:
                self.robot_dwa_speed_mag[ns] = max(0.0, float(dwa_speed_mag))
                self.robot_dwa_metric_update[ns] = self.last_update[ns]
            # [FIX-ROTETA] simpan heading_error (rad) utk term rotasi terminal ETA
            herr_in = packet.get('heading_error')
            if herr_in is not None and math.isfinite(float(herr_in)):
                self.robot_heading_err[ns] = float(herr_in)
        self._republish_telemetry(ns, packet)

    def _publish_amcl_pose(self, ns, pose_dict):
        """Rekonstruksi PoseWithCovarianceStamped dari pose dict UDP dan publish."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pose_dict['x'])
        msg.pose.pose.position.y = float(pose_dict['y'])
        yaw = float(pose_dict.get('yaw', 0.0))
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        sx  = float(pose_dict.get('sigma_x', 0.0))
        sy  = float(pose_dict.get('sigma_y', 0.0))
        cov = [0.0] * 36
        cov[0]  = sx * sx
        cov[7]  = sy * sy
        cov[35] = 0.001
        msg.pose.covariance = cov
        self.telemetry_pub[ns]['amcl_pose'].publish(msg)

    def _republish_telemetry(self, ns, packet):
        """Republish semua field dari UDP packet sebagai ROS topic di PC domain."""
        pub = self.telemetry_pub[ns]

        pose_dict = packet.get('pose')
        if pose_dict is not None:
            self._publish_amcl_pose(ns, pose_dict)

        path_len = packet.get('path_length')
        if path_len is not None:
            msg = Float32(); msg.data = float(path_len)
            pub['path_length'].publish(msg)

        remaining = packet.get('remaining_length')
        if remaining is not None:
            msg = Float32(); msg.data = max(0.0, float(remaining))
            pub['remaining_length'].publish(msg)

        goal_reached = packet.get('goal_reached')
        if goal_reached is not None:
            msg = Bool(); msg.data = bool(goal_reached)
            pub['goal_reached'].publish(msg)

        position_reached = packet.get('position_reached')  # [FIX-POSREACH]
        if position_reached is not None:
            msg = Bool(); msg.data = bool(position_reached)
            pub['position_reached'].publish(msg)

        fault_active = packet.get('fault_active')
        if fault_active is not None:
            msg = Bool(); msg.data = bool(fault_active)
            pub['fault_active'].publish(msg)

        for event in packet.get('fault_log_events') or []:
            msg = String(); msg.data = str(event)
            pub['fault_log'].publish(msg)

        waypoint_index = packet.get('waypoint_index')
        if waypoint_index is not None:
            msg = Int32(); msg.data = int(waypoint_index)
            pub['waypoint_index'].publish(msg)

        mission_rem = packet.get('mission_remaining_length')
        if mission_rem is not None:
            msg = Float32(); msg.data = max(0.0, float(mission_rem))
            pub['mission_remaining'].publish(msg)

        mission_tot = packet.get('mission_total_length')
        if mission_tot is not None:
            msg = Float32(); msg.data = float(mission_tot)
            pub['mission_total'].publish(msg)

        cmd_vel = packet.get('cmd_vel')
        if cmd_vel is not None:
            msg = Twist()
            msg.linear.x  = float(cmd_vel.get('vx', 0.0))
            msg.linear.y  = float(cmd_vel.get('vy', 0.0))
            msg.angular.z = float(cmd_vel.get('w',  0.0))
            pub['cmd_vel'].publish(msg)

        path_points = packet.get('path_points')
        if path_points:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            for pt in path_points:
                if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                    continue
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = float(pt[0])
                ps.pose.position.y = float(pt[1])
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)
            if path_msg.poses:
                pub['plan'].publish(path_msg)

        # [MOD-LOCALPLAN] teruskan local plan (DWA) ke ROS topic PC agar logger merekam
        local_plan_points = packet.get('local_plan_points')
        if local_plan_points:
            lp_msg = Path()
            lp_msg.header.stamp = self.get_clock().now().to_msg()
            lp_msg.header.frame_id = 'map'
            for pt in local_plan_points:
                if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                    continue
                ps = PoseStamped()
                ps.header = lp_msg.header
                ps.pose.position.x = float(pt[0])
                ps.pose.position.y = float(pt[1])
                ps.pose.orientation.w = 1.0
                lp_msg.poses.append(ps)
            if lp_msg.poses:
                pub['local_plan'].publish(lp_msg)

        dwa_mode = packet.get('dwa_mode')
        if dwa_mode is not None:
            msg = String(); msg.data = str(dwa_mode)
            pub['dwa_mode'].publish(msg)

        dwa_vmax_eff = packet.get('dwa_vmax_eff')
        if dwa_vmax_eff is not None:
            msg = Float32(); msg.data = float(dwa_vmax_eff)
            pub['dwa_vmax_eff'].publish(msg)

        omega_raw = packet.get('omega_raw')
        if omega_raw is not None:
            msg = Float32(); msg.data = float(omega_raw)
            pub['omega_raw'].publish(msg)

        omega_after_clamp = packet.get('omega_after_clamp')
        if omega_after_clamp is not None:
            msg = Float32(); msg.data = float(omega_after_clamp)
            pub['omega_clamped'].publish(msg)

        omega_global_limit = packet.get('omega_global_limit')
        if omega_global_limit is not None:
            msg = Float32(); msg.data = float(omega_global_limit)
            pub['omega_limit'].publish(msg)

        loc_hold = packet.get('localization_hold_active')
        if loc_hold is not None:
            msg = Bool(); msg.data = bool(loc_hold)
            pub['loc_hold'].publish(msg)

        dyn_debug = packet.get('dynamic_obstacle_debug')
        if dyn_debug:
            msg = String(); msg.data = str(dyn_debug)
            pub['dynobs_debug'].publish(msg)

        # ── Field tambahan dari robot (sebelumnya tidak di-bridge ke PC) ──
        tmode = packet.get('tracking_mode')
        if tmode is not None:
            msg = String(); msg.data = str(tmode)
            pub['tracking_mode'].publish(msg)

        herr = packet.get('heading_error')
        if herr is not None:
            msg = Float32(); msg.data = float(herr)
            pub['heading_error'].publish(msg)

        speed_mag = packet.get('dwa_speed_mag')
        if speed_mag is not None:
            msg = Float32(); msg.data = float(speed_mag)
            pub['dwa_speed_mag'].publish(msg)

        vmax_prio_robot = packet.get('vmax_priority_robot')
        if vmax_prio_robot is not None:
            msg = Float32(); msg.data = float(vmax_prio_robot)
            pub['vmax_prio_robot'].publish(msg)

        pstop_robot = packet.get('priority_stop_robot')
        if pstop_robot is not None:
            msg = Bool(); msg.data = bool(pstop_robot)
            pub['pstop_robot'].publish(msg)

        lane_off = packet.get('lane_offset_robot')
        if lane_off is not None:
            msg = Float32(); msg.data = float(lane_off)
            pub['lane_off_robot'].publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # UDP SENDER — kirim vmax ke tiap robot
    # ═══════════════════════════════════════════════════════════════════════

    def _peer_poses_for_locked(self, ns, now):
        peers = []
        for peer_ns, pose in self.robot_pose.items():
            if peer_ns == ns or pose is None:
                continue
            age = now - self.last_update.get(peer_ns, 0.0)
            cv = self.robot_cmd_vel.get(peer_ns)
            peers.append({
                'robot'     : peer_ns,
                'x'         : pose['x'],
                'y'         : pose['y'],
                'theta'     : pose['theta'],
                # [MOD-21] kecepatan badan peer utk prediksi gerakan di DWA
                'vx'        : (cv['vx'] if cv else 0.0),
                'vy'        : (cv['vy'] if cv else 0.0),
                'w'         : (cv['w']  if cv else 0.0),
                'age_s'     : age,
                'fresh'     : age < 1.0,
                'dwa_active': self.robot_dwa_active.get(peer_ns, False),
            })
        return peers

    def _peer_poses_snapshot_locked(self):
        now = time.time()
        return {
            ns: self._peer_poses_for_locked(ns, now)
            for ns in ROBOT_NAMESPACES
        }

    def _send_vmax_udp(self, ns, vmax, peers=None, experiment_running=None):
        """Kirim vmax_consensus ke udp_receiver_node di robot ns.
        priority_stop TIDAK dikirim dari sini — hanya priority_manager_node
        yang berhak mengirim priority_stop agar tidak terjadi race condition.

        experiment_state disertakan sebagai fallback: jika bridge UDP (port 9022)
        terblokir di robot tertentu, ctrl channel (port 9012) tetap menyampaikan
        state sehingga robot tidak stuck di HOLD_state_READY."""
        if experiment_running is None:
            with self._state_lock:
                experiment_running = self.experiment_running
        state = 'RUNNING' if experiment_running else 'READY'
        packet = {
            'vmax_consensus'  : vmax,
            'experiment_state': state,
        }
        if peers is None:
            with self._state_lock:
                peers = self._peer_poses_for_locked(ns, time.time())
        if peers:
            packet['peer_poses'] = peers
        try:
            ip   = self.robot_ip[ns]
            port = SEND_PORT_MAP[ns]
            self.send_sock.sendto(
                json.dumps(packet).encode('utf-8'),
                (ip, port)
            )
        except Exception as e:
            self.get_logger().warn(f'[{ns}] UDP send vmax failed: {e}')

    # ═══════════════════════════════════════════════════════════════════════
    # CONSENSUS LOOP — 10 Hz (IDENTIK versi lama, hanya output berubah)
    # ═══════════════════════════════════════════════════════════════════════

    def consensus_loop(self):
        with self._state_lock:
            actions = self._consensus_loop_locked()
            peer_snapshot = self._peer_poses_snapshot_locked()
            experiment_running = self.experiment_running
        self._publish_consensus_actions(
            actions, peer_snapshot, experiment_running)

    def _new_consensus_actions(self):
        return {
            'vmax_udp': {},
            'vmax_ros': {},
            'progress': {},
            'coord_debug': None,
        }

    def _publish_consensus_actions(self, actions, peer_snapshot, experiment_running):
        if not actions:
            return
        for ns, vmax in actions.get('vmax_udp', {}).items():
            self._send_vmax_udp(
                ns, vmax,
                peers=peer_snapshot.get(ns, []),
                experiment_running=experiment_running)
        for ns, vmax in actions.get('vmax_ros', {}).items():
            msg = Float32()
            msg.data = float(vmax)
            self.telemetry_pub[ns]['vmax_consensus'].publish(msg)
        for ns, progress in actions.get('progress', {}).items():
            msg = Float32()
            msg.data = float(progress)
            self.progress_pub[ns].publish(msg)
        self._publish_coordination_debug_payload(actions.get('coord_debug'))

    def _consensus_loop_locked(self):
        """
        Update rule average consensus discrete-time:
          p_i[k+1] = p_i[k] + ε · Σ_{j∈N_i} (p_j[k] - p_i[k])

        PENTING: v_max dihitung dari ACTUAL progress (remaining/total),
        bukan dari virtual consensus p yang sudah diratakan.
        Alasan: consensus update mendorong p_i → p̄, sehingga (p_i - p̄) ≈ 0
        setelah update — tidak ada koreksi kecepatan yang berarti.
        Virtual p tetap diupdate untuk metrik konvergensi di thesis.
        """
        now = time.time()
        actions = self._new_consensus_actions()

        # [M4] l4_sync_enabled=false → bypass semua logika konsensus,
        # kirim v_nominal ke semua robot. Dipakai untuk sub-variasi convoy async
        # (percobaan 3) di mana robot sengaja tidak disinkronkan waktu tiba.
        if not self.l4_sync_enabled:
            for ns in ROBOT_NAMESPACES:
                if self.data_valid.get(ns, False):
                    actions['vmax_udp'][ns] = self.v_nominal
            return actions

        # Consensus hanya boleh mengoreksi kecepatan saat trial berjalan.
        # Pada READY, mission_remaining/path sudah valid tetapi robot masih HOLD;
        # menghitung ETA dari kondisi diam membuat vcons saturasi/flip sebelum START.
        if not self.experiment_running:
            self._reset_arrival_debug()
            for ns in ROBOT_NAMESPACES:
                if self.data_valid.get(ns, False):
                    actions['vmax_udp'][ns] = self.v_nominal
                    actions['vmax_ros'][ns] = self.v_nominal
            return actions

        active_robots = [
            ns for ns in ROBOT_NAMESPACES
            if self.data_valid.get(ns, False)
            and (now - self.last_update[ns]) < 2.0
            and self.pose_valid[ns]
        ]
        if len(active_robots) < 2:
            return actions
        moving_robots = [ns for ns in active_robots if not self.goal_reached[ns]]
        unfinished_robots = [
            ns for ns in active_robots
            if not self.goal_reached.get(ns, False)
        ]

        # Step 1: Hitung actual progress — mission-level jika tersedia,
        # fallback ke per-segmen. Mission progress tidak reset saat ganti waypoint.
        # Jika experiment RUNNING, normalisasi relatif terhadap p0 saat start.
        actual_p = {}
        for ns in active_robots:
            p_raw = self._compute_actual_p(ns)
            actual_p[ns] = self._normalize_p(ns, p_raw) if self.experiment_running else p_raw

        # Step 2: Update virtual p_i via consensus (untuk metrik konvergensi)
        p_snapshot = dict(self.p)
        p_new      = dict(p_snapshot)

        for ns_i in moving_robots:
            consensus_term = sum(
                p_snapshot[ns_j] - p_snapshot[ns_i]
                for ns_j in moving_robots if ns_j != ns_i
            )
            p_new[ns_i] = max(0.0, min(1.0,
                p_snapshot[ns_i] + self.epsilon * consensus_term))

        for ns in moving_robots:
            self.p[ns] = p_new[ns]

        # Step 3: p_bar dari ACTUAL progress (bukan virtual p yang sudah rata)
        # [M2] Exclude robot yang sedang STOP dari rata-rata agar robot yang
        # berjalan tidak di-throttle oleh progress stasioner robot yang berhenti.
        # Contoh crossing: robot2/3 STOP di zona konflik → jangan perlambat robot1.
        # [M4b] Selain robot STOP, kecualikan juga agen GAGAL dari p_bar agar
        # progress robot mati/macet tidak mencemari rata-rata (konsisten dgn q̄).
        non_stopped = [ns for ns in active_robots
                       if (not self.robot_stopped.get(ns, False)
                           and not self.goal_reached.get(ns, False)
                           and not self.agent_failed.get(ns, False))]
        unfinished_alive = [ns for ns in unfinished_robots
                            if not self.agent_failed.get(ns, False)]
        active_alive = [ns for ns in active_robots
                        if not self.agent_failed.get(ns, False)]
        if len(non_stopped) >= 1:
            p_bar_robots = non_stopped
        elif len(unfinished_alive) >= 1:
            p_bar_robots = unfinished_alive
        elif len(active_alive) >= 1:
            p_bar_robots = active_alive
        else:
            p_bar_robots = active_robots  # semua gagal: fallback agar tak div-by-zero
        p_bar = sum(actual_p[ns] for ns in p_bar_robots) / len(p_bar_robots)
        # [SO] progress-rate (p_dot) per robot via beda-hingga + p_dot rata-rata.
        # Dipakai konsensus orde-2 (cocokkan laju) & evaluasi sinkronisasi laju.
        _now_pd = time.time()
        if not hasattr(self, '_pdot_prev_p'):
            self._pdot_prev_p = {}
            self._pdot_prev_t = {}
        self._pdot = {}
        for _ns in active_robots:
            _p = actual_p.get(_ns, 0.0)
            _pp = self._pdot_prev_p.get(_ns)
            _pt = self._pdot_prev_t.get(_ns)
            if _pp is not None and _pt is not None and (_now_pd - _pt) > 1e-3:
                self._pdot[_ns] = (_p - _pp) / (_now_pd - _pt)
            else:
                self._pdot[_ns] = 0.0
            self._pdot_prev_p[_ns] = _p
            self._pdot_prev_t[_ns] = _now_pd
        _bar_set = list(p_bar_robots) if p_bar_robots else []
        self._pdot_bar = (sum(self._pdot.get(_ns, 0.0) for _ns in _bar_set) / len(_bar_set)) if _bar_set else 0.0
        arrival_vmax = {}
        if self.coordination_mode == 'arrival_offset_consensus':
            arrival_vmax = self._compute_arrival_offset_vmax(active_robots)
        elif self.coordination_mode == 'consensus':
            arrival_vmax = self._compute_progress_consensus_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode == 'consensus_offset':
            arrival_vmax = self._compute_progress_consensus_offset_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode == 'consensus_so':
            arrival_vmax = self._compute_second_order_progress_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode == 'consensus_seg':
            arrival_vmax = self._compute_progress_consensus_seg_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode == 'consensus_so_seg':
            arrival_vmax = self._compute_second_order_progress_seg_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode == 'consensus_dist':
            arrival_vmax = self._compute_distributed_consensus_vmax(
                active_robots, actual_p)
        elif self.coordination_mode == 'consensus_ft':
            arrival_vmax = self._compute_finite_time_progress_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode == 'consensus_ftso':
            arrival_vmax = self._compute_ft_second_order_progress_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode == 'consensus_fxt':
            arrival_vmax = self._compute_fixed_time_progress_vmax(
                active_robots, actual_p, p_bar)
        elif self.coordination_mode in ('time_consensus', 'time_offset_consensus'):
            arrival_vmax = self._compute_remaining_time_consensus_vmax(
                active_robots)
        else:
            self._reset_arrival_debug()

        # Step 4: Hitung vmax per robot.
        # arrival_offset_consensus: sinkronisasi ETA relatif dengan offset per robot.
        # [FIX-CATCHUP] Begitu ADA peer yang sudah sampai goal, robot yang masih
        # jalan harus segera mengejar (sprint ke ceiling) agar selisih waktu-tiba
        # (delta arrival) mengecil. Tanpa ini, konsensus bisa menahan laggard saat
        # referensi rendezvous terdegradasi oleh robot yang sudah parkir. Slew-rate
        # di bawah membuat ramp halus; DWA + priority-stop tetap menjaga jarak aman.
        # [FIX-CATCHUP] Pindai SEMUA robot (bukan hanya active_robots): robot yang
        # sudah sampai bisa keluar dari active_robots saat parkir (telemetry stale
        # >2s / pose invalid). Pakai goal_reached penuh agar catch-up tetap nyala.
        # [OFFSET] Bila offset kedatangan memang disengaja (consensus_offset /
        # arrival_offset_consensus / offset eksplisit), JANGAN aktifkan catch-up
        # ceiling saat satu robot tiba — itu akan menghapus stagger yang diminta.
        _intentional_offset = (
            self.coordination_mode in ('consensus_offset', 'arrival_offset_consensus')
            or self._arrival_offset_explicit)
        any_arrived = any(self.goal_reached.values()) and not _intentional_offset
        for ns in ROBOT_NAMESPACES:
            if not self.data_valid[ns]:
                actions['vmax_udp'][ns] = self.v_nominal
                continue

            if self.goal_reached[ns]:
                actions['vmax_udp'][ns] = 0.0
                actions['vmax_ros'][ns] = 0.0
                self.last_vmax_cmd[ns] = 0.0   # [FIX-VRATE] jaga kontinuitas slew
                self.vmax_out_filt[ns] = 0.0   # [FIX-VOUTLPF] jaga kontinuitas EMA
                continue

            # [M2b] Robot sedang STOP oleh L5 priority_stop: kirim v_nominal
            # (bukan vmax hasil kalkulasi yang mungkin lebih tinggi karena robot
            # tertinggal saat berhenti). L5 tetap menghentikan DWA, jadi log L4
            # terlihat wajar (nominal) bukan anomali spike.
            if self.robot_stopped.get(ns, False):
                actions['vmax_udp'][ns] = self.v_nominal
                actions['vmax_ros'][ns] = self.v_nominal
                self.last_vmax_cmd[ns] = self.v_nominal   # [FIX-VRATE] jaga kontinuitas slew
                self.vmax_out_filt[ns] = self.v_nominal   # [FIX-VOUTLPF] jaga kontinuitas EMA
                continue

            v_max = arrival_vmax.get(ns, self.v_nominal)
            if any_arrived:
                # [FIX-CATCHUP] Peer sudah sampai goal → laggard langsung sprint ke
                # ceiling. Lebih tanggap: pakai slew catch-up yang lebih cepat dan
                # lewati EMA output supaya reaksi ~1-2 tick (bukan ramp lambat).
                v_max = self._rate_limit_vmax(
                    ns, self.v_consensus_ceiling, rate=self.v_consensus_catchup_rate)
                self.vmax_out_filt[ns] = v_max   # [FIX-VOUTLPF] jaga kontinuitas EMA
            else:
                # [FIX-VRATE] Slew-rate limit: cegah lompatan floor<->ceiling 1 tick.
                v_max = self._rate_limit_vmax(ns, v_max)
                # [FIX-VOUTLPF] EMA output: haluskan surging cap akhir (konservatif).
                v_max = self._smooth_output_vmax(ns, v_max)
            self.arrival_debug['robots'][ns]['v_consensus'] = v_max
            actions['vmax_udp'][ns] = v_max
            actions['vmax_ros'][ns] = v_max

        # Step 5: Publish actual progress untuk rqt_plot
        for ns in ROBOT_NAMESPACES:
            actions['progress'][ns] = actual_p.get(ns, 0.0)

        # Step 6: Cek konvergensi (dari actual progress)
        # [FIX-CONVSET] Ukur deviasi atas himpunan yg SAMA dgn p_bar (p_bar_robots).
        self._check_convergence(active_robots, p_bar, actual_p, eval_set=p_bar_robots)
        actions['coord_debug'] = self._coordination_debug_payload(actual_p, p_bar)
        return actions

    # CONVERGENCE TRACKING

    def _check_convergence(self, active_robots, p_bar, actual_p=None, eval_set=None):
        if len(active_robots) < 2:
            return

        p_vals = actual_p if actual_p is not None else self.p
        # [FIX-CONVSET] Ukur deviasi atas himpunan yang SAMA dengan p_bar.
        # Bug lama: p_bar dirata-rata atas p_bar_robots (kecualikan STOP/tiba/gagal)
        # tetapi deviasi diukur atas SEMUA active non-failed (termasuk robot yang
        # sudah TIBA dgn actual_p~1.0 atau STOP) -> |actual_p - p_bar| selalu besar
        # -> converged mustahil True & max_deviation mandek di floor (mis. ~0.071).
        # [M4b] Tetap kecualikan agen gagal agar metrik konvergensi bersih.
        base_set = eval_set if eval_set else active_robots
        eval_robots = [ns for ns in base_set
                       if not self.agent_failed.get(ns, False)]
        if len(eval_robots) < 2:
            # [FIX-CONVLOG] Konvergensi multi-robot tak terdefinisi utk <2 agen
            # berkoordinasi: kosongkan deviasi & status. Jangan tulis 0.0 karena
            # logger lama akan menafsirkan 0.0<threshold sebagai converged=True.
            self._last_max_deviation = None
            self._last_is_converged  = None
            return
        max_deviation = max(abs(p_vals.get(ns, 0.0) - p_bar) for ns in eval_robots)
        is_converged  = max_deviation < self.convergence_threshold
        # [FIX-OBS] Simpan untuk dipublikasikan ke coordination_debug payload.
        self._last_max_deviation = max_deviation
        self._last_is_converged  = is_converged
        now           = self.get_clock().now().nanoseconds / 1e9

        if not is_converged and self.was_converged:
            self.convergence_start_time = now
            self.was_converged          = False

        elif is_converged and not self.was_converged:
            if self.convergence_start_time is not None:
                convergence_time = now - self.convergence_start_time
                self.convergence_log.append({
                    'diverge_t'  : self.convergence_start_time,
                    'converge_t' : now,
                    'duration_s' : convergence_time,
                    'p_bar'      : p_bar,
                })
                self.get_logger().info(
                    f'[CONSENSUS] Konvergen | '
                    f'durasi={convergence_time:.3f}s | '
                    f'total={len(self.convergence_log)}')
            self.was_converged          = True
            self.convergence_start_time = None

    def get_convergence_stats(self):
        if not self.convergence_log:
            return {'count': 0, 'mean_s': 0.0, 'max_s': 0.0, 'min_s': 0.0}
        durations = [e['duration_s'] for e in self.convergence_log]
        return {
            'count'  : len(durations),
            'mean_s' : sum(durations) / len(durations),
            'max_s'  : max(durations),
            'min_s'  : min(durations),
            'log'    : self.convergence_log,
        }

    # STATUS REPORT — identik versi lama

    def status_report(self):
        with self._state_lock:
            self._status_report_locked()

    def _status_report_locked(self):
        active = [ns for ns in ROBOT_NAMESPACES if self.data_valid[ns]]
        if not active:
            self.get_logger().info(
                '[CONSENSUS] Menunggu data UDP dari robot...')
            return

        # Gunakan mission progress untuk status jika tersedia
        actual_p = {}
        for ns in active:
            if self.goal_reached[ns]:
                actual_p[ns] = 1.0
            elif (self.mission_remaining.get(ns) is not None
                  and self.mission_total.get(ns) is not None
                  and self.mission_total[ns] > self.min_path_length):
                actual_p[ns] = max(0.0, min(1.0,
                    1.0 - self.mission_remaining[ns] / self.mission_total[ns]))
            elif self.path_length[ns] > self.min_path_length:
                actual_p[ns] = max(0.0, min(1.0,
                    1.0 - self.remaining[ns] / self.path_length[ns]))
            else:
                actual_p[ns] = 0.0

        p_bar  = sum(actual_p[ns] for ns in active) / len(active)
        parts  = [f'p_avg={p_bar:.3f}']
        for ns in ROBOT_NAMESPACES:
            if self.data_valid[ns]:
                ap      = actual_p.get(ns, 0.0)
                e_i     = p_bar - ap
                reached = '✓' if self.goal_reached[ns] else ' '
                using_m = (self.mission_remaining.get(ns) is not None
                           and self.mission_total.get(ns) is not None
                           and self.mission_total[ns] > self.min_path_length)
                rem_str = (f'{self.mission_remaining[ns]:.2f}m[M]' if using_m
                           else f'{self.remaining[ns]:.2f}m')
                # Reconstruct vmax for display
                if self.goal_reached[ns]:
                    vmax_d = 0.0
                elif abs(e_i) < self.epsilon_deadband:
                    vmax_d = self.v_nominal
                else:
                    vmax_d = max(self.v_consensus_floor,
                                 min(self.v_consensus_ceiling,
                                     self.v_nominal + self.k_consensus * e_i))
                dbg = self.arrival_debug.get('robots', {}).get(ns, {})
                eta = dbg.get('ETA')
                t_left = dbg.get('time_left')
                eta_v = dbg.get('eta_v')
                err = dbg.get('e')
                if self.coordination_mode in ('time_consensus', 'time_offset_consensus') and t_left is not None:
                    eta_str = f'Trem={t_left:.1f}s'
                else:
                    eta_str = f'ETA={eta:.1f}s' if eta is not None else 'ETA=--'
                v_eta_str = f'vEta={eta_v:.2f}' if eta_v is not None else 'vEta=--'
                if err is None:
                    err_str = 'e=--'
                elif self.coordination_mode == 'arrival_offset_consensus':
                    err_str = f'eA={err:+.2f}s'
                elif self.coordination_mode in ('time_consensus', 'time_offset_consensus'):
                    err_str = f'eT={err:+.2f}s'
                else:
                    err_str = f'eP={err:+.3f}'
                parts.append(
                    f'{ns}: p={ap:.3f} {err_str} vmax={dbg.get("v_consensus", vmax_d):.3f}{reached}'
                    f' rem={rem_str} {eta_str} {v_eta_str}')
            else:
                parts.append(f'{ns}=-- (belum ada UDP)')

        conv_str = ('KONVERGEN' if self.was_converged
                    else f'DIVERGEN ({len(self.convergence_log)} events)')
        self.get_logger().info(
            f'[CONSENSUS] {" | ".join(parts)} | {conv_str}')

    # ═══════════════════════════════════════════════════════════════════════
    # CLEANUP
    # ═══════════════════════════════════════════════���═══════════════════════

    def destroy_node(self):
        self.send_sock.close()
        super().destroy_node()


# ═════════════════════════════════��═════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = ConsensusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Consensus node stopped')
        stats = node.get_convergence_stats()
        node.get_logger().info(
            f'Convergence stats: count={stats["count"]} | '
            f'mean={stats["mean_s"]:.3f}s | '
            f'max={stats["max_s"]:.3f}s')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
