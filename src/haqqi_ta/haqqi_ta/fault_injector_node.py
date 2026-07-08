#!/usr/bin/env python3
"""
Fault Injector Node — haqqi_ta
Support: Software Fault Injection (Planned / Reproducible)

Cara kerja:
  Fault injector TIDAK merusak hardware — ia menjadi "proxy" di antara
  sumber cmd_vel (modified_dwa_node) dan robot fisik dengan cara:

    modified_dwa_node → /{ns}/cmd_vel_raw  →  [Fault Injector]  →  /{ns}/cmd_vel
                                                      ↑
                                              jika fault aktif:
                                              publish Twist(0,0,0) saja

  Dengan demikian modified_dwa_node tidak perlu tahu apakah fault aktif —
  ia tetap menghitung dan publish ke cmd_vel_raw. Saat fault aktif, injector
  memblokir sinyal dan publish zero velocity ke cmd_vel yang sesungguhnya.

Skenario fault yang didukung:
  1. TIMED  — fault aktif selama durasi tetap (TTF deterministik)
  2. RANDOM — fault aktif selama durasi acak dalam range [ttf_min, ttf_max]
  3. PULSE  — fault aktif / tidak secara bergantian (stress test)

Semua skenario bisa dijadwalkan via:
  a. ROS parameter saat launch (untuk eksperimen reproducible)
  b. ROS topic trigger saat runtime: publish Bool ke /{ns}/fault_trigger
     (CATATAN: bukan ROS Service — implementasi pakai subscriber topic Bool)

Topic yang di-relay:
  /{ns}/cmd_vel_raw  (subscribe) — output dari modified_dwa_node
  /{ns}/cmd_vel      (publish)   — input ke driver motor robot

CATATAN PENTING untuk wiring:
  Di launch file, modified_dwa_node harus remap output-nya:
    remappings=[('/robot1/cmd_vel', '/robot1/cmd_vel_raw')]
  Sehingga hanya fault_injector yang publish ke /robot1/cmd_vel yang asli.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import random
import math
import time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


# Jenis fault yang tersedia
FAULT_MODE_TIMED  = 'timed'    # Durasi tetap
FAULT_MODE_RANDOM = 'random'   # Durasi acak dalam range
FAULT_MODE_PULSE  = 'pulse'    # Bergantian aktif/nonaktif
FAULT_MODE_MANUAL = 'manual'   # Diaktifkan manual via topic trigger
FAULT_MODE_NONE   = 'none'
FAULT_MODE_TIMED_PULSE = 'timed_pulse'


class FaultInjectorNode(Node):
    def __init__(self):
        super().__init__('fault_injector_node')

        # ── Parameter target robot ────────────────────────────────────────
        self.declare_parameter('robot_ns', 'robot1')
        self.ns = self.get_parameter('robot_ns').value

        # ── Parameter skenario fault ──────────────────────────────────────
        self.declare_parameter('fault_mode', FAULT_MODE_NONE)
        self.declare_parameter('fault_target_robot', 'robot2')
        self.declare_parameter('fault_start_s', 15.0)
        self.declare_parameter('fault_duration_s', 2.0)
        self.declare_parameter('fault_repeat_count', 3)
        self.declare_parameter('fault_interval_s', 10.0)
        self.declare_parameter('fault_use_trial_time', True)

        # Waktu setelah node start sebelum fault pertama diinjeksi (detik)
        self.declare_parameter('fault_delay',    8.0)

        # TIMED: durasi fault tetap
        self.declare_parameter('ttf_duration',   2.0)   # detik

        # RANDOM: range durasi fault acak
        self.declare_parameter('ttf_min',        1.0)   # detik
        self.declare_parameter('ttf_max',        3.0)   # detik

        # PULSE: durasi aktif dan durasi jeda
        self.declare_parameter('pulse_on',       2.0)   # detik fault aktif
        self.declare_parameter('pulse_off',      5.0)   # detik fault tidak aktif

        # Berapa kali fault diinjeksi (0 = tak terbatas)
        self.declare_parameter('fault_count',    1)

        # Seed random untuk reproducibility (0 = gunakan waktu sistem)
        self.declare_parameter('random_seed',    42)

        # Apakah langsung mulai saat node launch
        self.declare_parameter('auto_start',     True)

        # Jika False, injector jadi pure relay tanpa cek fault_active
        self.declare_parameter('enabled',        True)
        # hold_until_signal: nilai awal hold_active sebelum heartbeat /experiment_state
        # pertama diterima. Setelah itu hold_active selalu di-overwrite oleh callback
        # (state != 'RUNNING'), sehingga parameter ini HANYA mempengaruhi kondisi awal.
        self.declare_parameter('hold_until_signal', True)

        # [M6] Jenis EFEK kegagalan (orthogonal terhadap fault_mode = JADWAL).
        #   fail_stop : Twist(0,0,0) — berhenti total (perilaku lama, default)
        #   freeze    : ulangi cmd saat onset (controller hang / actuator stuck)
        #   degraded  : skala cmd dengan degraded_factor (motor/daya lemah)
        #   drift     : tambah bias konstan (miskalibrasi / sensor-drift)
        self.declare_parameter('fault_type', 'fail_stop')
        self.declare_parameter('degraded_factor', 0.4)
        self.declare_parameter('drift_vx', 0.0)
        self.declare_parameter('drift_vy', 0.0)
        self.declare_parameter('drift_wz', 0.3)

        self.fault_mode    = self.get_parameter('fault_mode').value
        self.fault_target_robot = self.get_parameter('fault_target_robot').value
        self.fault_start_s = float(self.get_parameter('fault_start_s').value)
        self.fault_duration_s = float(self.get_parameter('fault_duration_s').value)
        self.fault_repeat_count = int(self.get_parameter('fault_repeat_count').value)
        self.fault_interval_s = float(self.get_parameter('fault_interval_s').value)
        self.fault_use_trial_time = bool(self.get_parameter('fault_use_trial_time').value)
        self.fault_delay   = self.get_parameter('fault_delay').value
        self.ttf_duration  = self.get_parameter('ttf_duration').value
        self.ttf_min       = self.get_parameter('ttf_min').value
        self.ttf_max       = self.get_parameter('ttf_max').value
        self.pulse_on      = self.get_parameter('pulse_on').value
        self.pulse_off     = self.get_parameter('pulse_off').value
        self.fault_count   = self.get_parameter('fault_count').value
        self.random_seed   = self.get_parameter('random_seed').value
        self.auto_start    = self.get_parameter('auto_start').value
        self.enabled       = self.get_parameter('enabled').value
        # [M6] Parameter jenis efek kegagalan
        self.fault_type = (str(self.get_parameter('fault_type').value)
                           .strip().lower() or 'fail_stop')
        self.degraded_factor = max(0.0, float(
            self.get_parameter('degraded_factor').value))
        self.drift_vx = float(self.get_parameter('drift_vx').value)
        self.drift_vy = float(self.get_parameter('drift_vy').value)
        self.drift_wz = float(self.get_parameter('drift_wz').value)
        if self.ns != self.fault_target_robot and self.fault_mode == FAULT_MODE_TIMED_PULSE:
            self.get_logger().info(
                f'[FAULT] timed_pulse target={self.fault_target_robot}; /{self.ns} pure relay')

        # Inisialisasi RNG
        seed = self.random_seed if self.random_seed != 0 else None
        self.rng = random.Random(seed)
        self.get_logger().info(f'RNG seed: {seed} (None = system time)')

        # ── State mesin fault ─────────────────────────────────────────────
        self.fault_active       = False    # Apakah saat ini sedang dalam fault
        self._freeze_cmd        = None     # [M6] snapshot cmd saat onset (mode 'freeze')
        self.hold_active        = self.get_parameter('hold_until_signal').value
        self.fault_started      = False    # Apakah sequence fault sudah dimulai
        self.fault_inject_count = 0        # Berapa kali sudah diinjeksi
        self._inject_start      = None     # Kapan injeksi fault terakhir dimulai
        self.last_cmd_vel_raw   = Twist()  # cmd_vel terakhir dari DWA node
        # [FIX-9] Watchdog staleness cmd_vel_raw — safety jika DWA crash/mati
        self._last_cmd_vel_raw_t = None    # waktu terakhir cmd_vel_raw diterima

        # Experiment start tracking (dari /experiment_state)
        self.experiment_started = False
        self.fault_triggered    = False
        self.fault_start_time   = None    # Kapan state RUNNING diterima

        # ── Log kejadian untuk evaluasi ───────────────────────────────────
        # [{start_t, end_t, duration_s, ttf_planned, mode}]
        self.fault_events  = []
        self._current_event = None
        # Waktu deaktivasi fault terakhir — dipakai _schedule_pulse untuk
        # menghitung pulse_off tanpa bergantung pada _current_event (yang
        # di-clear oleh _deactivate_fault sebelum pulse_off dievaluasi).
        self._last_fault_end = None
        self._timed_pulse_active_index = None
        self._timed_pulse_actual_start = {}

        # ── Subscribers ───────────────────────────────────────────────────
        # cmd_vel_raw dari modified_dwa_node (setelah remap di launch file)
        self.create_subscription(
            Twist,
            f'/{self.ns}/cmd_vel_raw',
            self.cmd_vel_raw_callback,
            10)

        # Manual trigger: publish True ke topic ini untuk aktifkan fault
        self.create_subscription(
            Bool,
            f'/{self.ns}/fault_trigger',
            self.manual_trigger_callback,
            10)

        # Hold/release — robot diam sampai experiment_state == RUNNING
        self.create_subscription(
            String,
            '/experiment_state',
            self._experiment_state_cb,
            10)

        # /start_signal Bool (True=mulai, False=stop) — alternatif dari experiment_master_cli
        _sig_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Bool,
            '/start_signal',
            self.start_signal_callback,
            _sig_qos)

        # ── Publishers ────────────────────────────────────────────────────
        # cmd_vel ke driver motor (proxy output)
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{self.ns}/cmd_vel', 10)

        # Status fault ke modified_dwa_node dan consensus_node
        self.fault_status_pub = self.create_publisher(
            Bool, f'/{self.ns}/fault_active', 10)

        # Log string untuk monitoring
        self.fault_log_pub = self.create_publisher(
            String, f'/{self.ns}/fault_log', 10)

        # ── Timers ────────────────────────────────────────────────────────
        # Loop utama: relay / blokir cmd_vel
        self.create_timer(0.05, self.relay_loop)     # 20 Hz — lebih tinggi dari DWA

        # Loop scheduler: cek kapan harus aktifkan/matikan fault
        self.create_timer(0.1, self.scheduler_loop)  # 10 Hz

        # Status report
        self.create_timer(1.0, self.status_report)

        self.get_logger().info(
            f'Fault Injector ready | target=/{self.ns} | '
            f'mode={self.fault_mode} | delay={self.fault_delay}s | '
            f'count={self.fault_count} | auto_start={self.auto_start}')
        self.get_logger().info(f'  Fault type (efek): {self.fault_type}')

        if self.fault_mode == FAULT_MODE_NONE:
            self.get_logger().info('  Fault mode none: pure relay')
        elif self.fault_mode == FAULT_MODE_TIMED:
            self.get_logger().info(
                f'  TTF: {self.ttf_duration}s (deterministik)')
        elif self.fault_mode == FAULT_MODE_RANDOM:
            self.get_logger().info(
                f'  TTF: acak [{self.ttf_min}s, {self.ttf_max}s] '
                f'seed={seed}')
        elif self.fault_mode == FAULT_MODE_PULSE:
            self.get_logger().info(
                f'  Pulse: ON={self.pulse_on}s OFF={self.pulse_off}s')
        elif self.fault_mode == FAULT_MODE_TIMED_PULSE:
            self.get_logger().info(
                f'  Timed pulse: target={self.fault_target_robot} '
                f'start={self.fault_start_s}s dur={self.fault_duration_s}s '
                f'repeat={self.fault_repeat_count} interval={self.fault_interval_s}s')

    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def cmd_vel_raw_callback(self, msg):
        """Simpan cmd_vel terbaru dari DWA node."""
        self.last_cmd_vel_raw = msg
        self._last_cmd_vel_raw_t = time.time()  # [FIX-9] catat waktu terima

    def manual_trigger_callback(self, msg):
        """
        Trigger fault manual via topic (untuk mode MANUAL atau override).
        Publish True → aktifkan fault
        Publish False → matikan fault
        """
        if self.fault_mode == FAULT_MODE_MANUAL:
            if bool(msg.data) and not self.fault_active:
                ttf = self.ttf_duration
                self._activate_fault(ttf, reason='manual trigger')
            elif not bool(msg.data) and self.fault_active:
                self._deactivate_fault(reason='manual stop')

    def start_signal_callback(self, msg):
        prev = self.hold_active
        self.hold_active = not bool(msg.data)
        if bool(msg.data) and not self.experiment_started:
            self.experiment_started = True
            self.fault_start_time   = time.time()
            self.fault_triggered    = False
            self.fault_started      = False
            self.fault_inject_count = 0
            self._last_fault_end    = None
            self._timed_pulse_active_index = None
            self._timed_pulse_actual_start = {}
            self.get_logger().info(
                f'[FAULT] Eksperimen mulai — TTF countdown dimulai sekarang')
        elif not bool(msg.data):
            self.experiment_started = False
            self.fault_triggered    = False
            self.fault_start_time   = None
            self.fault_started      = False
            self.fault_inject_count = 0
            self._last_fault_end    = None
            self._timed_pulse_active_index = None
            self._timed_pulse_actual_start = {}
            if self.fault_active:
                self._deactivate_fault(reason='experiment stopped')
        if prev != self.hold_active:
            state = 'RELEASED' if not self.hold_active else 'HOLDING'
            self.get_logger().info(f'[HOLD] {state}')

    def _experiment_state_cb(self, msg):
        """Terima heartbeat /experiment_state. Robot hold kecuali state == RUNNING."""
        state    = msg.data
        prev_hold = self.hold_active
        self.hold_active = (state != 'RUNNING')

        if state == 'RUNNING' and not self.experiment_started:
            self.experiment_started = True
            self.fault_start_time   = time.time()
            self.get_logger().info(
                f'[FAULT] Eksperimen mulai, TTF countdown: {self.fault_delay}s')
        elif state == 'STOP':
            self.experiment_started  = False
            self.fault_triggered     = False
            self.fault_start_time    = None
            # Reset sequence state agar trial berikutnya mulai bersih
            self.fault_started       = False
            self.fault_inject_count  = 0
            self._last_fault_end     = None
            self._timed_pulse_active_index = None
            self._timed_pulse_actual_start = {}
            if self.fault_active:
                self._deactivate_fault(reason='experiment stopped')

        if prev_hold != self.hold_active:
            label = 'RELEASED' if not self.hold_active else 'HOLDING'
            self.get_logger().info(f'[HOLD] {label} (state={state})')

    # ═══════════════════════════════════════════════════════════════════════
    # RELAY LOOP — 20 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def _apply_fault_effect(self) -> Twist:
        """[M6] Hitung cmd_vel keluaran saat fault aktif, sesuai fault_type.
        Memungkinkan model kegagalan lebih kaya daripada sekadar fail-stop:
        - fail_stop : Twist(0,0,0) — berhenti total (perilaku lama).
        - freeze    : ulangi cmd saat onset (controller hang / actuator stuck).
        - degraded  : skala cmd dengan degraded_factor (motor/daya lemah).
        - drift     : tambah bias konstan (miskalibrasi / sensor-drift → menyimpang).
        """
        ft = self.fault_type
        if ft == 'freeze':
            return self._freeze_cmd if self._freeze_cmd is not None else Twist()
        if ft == 'degraded':
            out = Twist()
            f = self.degraded_factor
            out.linear.x  = self.last_cmd_vel_raw.linear.x  * f
            out.linear.y  = self.last_cmd_vel_raw.linear.y  * f
            out.angular.z = self.last_cmd_vel_raw.angular.z * f
            return out
        if ft == 'drift':
            out = Twist()
            out.linear.x  = self.last_cmd_vel_raw.linear.x  + self.drift_vx
            out.linear.y  = self.last_cmd_vel_raw.linear.y  + self.drift_vy
            out.angular.z = self.last_cmd_vel_raw.angular.z + self.drift_wz
            return out
        # default: fail_stop
        return Twist()

    def relay_loop(self):
        """
        Core logic: relay atau blokir cmd_vel.
        Jika fault aktif → publish Twist(0,0,0)
        Jika normal      → forward cmd_vel_raw apa adanya

        [FIX-9] Safety watchdog: jika DWA berhenti publish cmd_vel_raw > 0.5s
        (mis. node crash/mati), JANGAN terus relay command lama — publish zero.
        Tanpa ini, robot bisa terus melaju tak terkontrol dengan cmd terakhir
        saat DWA crash di tengah gerakan.
        """
        cmd_stale = (
            self._last_cmd_vel_raw_t is None
            or (time.time() - self._last_cmd_vel_raw_t) > 0.5)

        if not self.enabled:
            if cmd_stale:
                self.cmd_vel_pub.publish(Twist())
                self._warn_cmd_stale()
            else:
                self.cmd_vel_pub.publish(self.last_cmd_vel_raw)
            return

        if self.hold_active:
            # Hold pra-eksperimen — selalu diam
            self.cmd_vel_pub.publish(Twist())
        elif self.fault_active:
            # [M6] Terapkan EFEK kegagalan sesuai fault_type (default fail_stop = zero)
            self.cmd_vel_pub.publish(self._apply_fault_effect())
        elif cmd_stale:
            # [FIX-9] DWA tidak publish — safety stop
            self.cmd_vel_pub.publish(Twist())
            self._warn_cmd_stale()
        else:
            # Relay normal
            self.cmd_vel_pub.publish(self.last_cmd_vel_raw)

        # Publish status fault ke node lain
        status_msg      = Bool()
        status_msg.data = self.fault_active
        self.fault_status_pub.publish(status_msg)

    def _warn_cmd_stale(self):
        """[FIX-9] Warning throttled saat cmd_vel_raw dari DWA hilang."""
        now = time.time()
        last = getattr(self, '_last_cmd_stale_warn', 0.0)
        if now - last > 2.0:
            self.get_logger().error(
                f'[FAULT][{self.ns}] cmd_vel_raw dari DWA HILANG/STALE — '
                f'publish ZERO untuk keamanan. Cek apakah modified_dwa_node hidup!')
            self._last_cmd_stale_warn = now

    # ═══════════════════════════════════════════════════════════════════════
    # SCHEDULER LOOP — 10 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def scheduler_loop(self):
        """
        Kelola jadwal fault berdasarkan mode yang dipilih.
        Dipanggil 10 Hz — semua keputusan timing berbasis clock.
        """
        if self.fault_mode == FAULT_MODE_NONE:
            return
        if self.fault_mode == FAULT_MODE_MANUAL:
            return   # Manual mode: hanya trigger dari callback

        now = self.get_clock().now().nanoseconds / 1e9

        # Cek apakah sudah waktunya mulai sequence
        if not self.auto_start:
            return

        if not self.experiment_started or self.fault_start_time is None:
            return

        elapsed_since_start = time.time() - self.fault_start_time

        if self.fault_mode == FAULT_MODE_TIMED_PULSE:
            self._schedule_timed_pulse(elapsed_since_start)
            return

        if not self.fault_started:
            # Belum mulai — tunggu fault_delay
            if elapsed_since_start >= self.fault_delay:
                self.fault_started = True
                self.get_logger().info(
                    f'[INJECTOR] Delay {self.fault_delay}s selesai — '
                    f'memulai sequence fault mode={self.fault_mode}')
                self._trigger_next_fault(now)
            return

        # Sudah mulai — kelola sesuai mode
        if self.fault_mode == FAULT_MODE_TIMED:
            self._schedule_timed(now)
        elif self.fault_mode == FAULT_MODE_RANDOM:
            self._schedule_random(now)
        elif self.fault_mode == FAULT_MODE_PULSE:
            self._schedule_pulse(now)

    def _schedule_timed_pulse(self, elapsed_since_start):
        """Fault pulse fixed: 15-17, 25-27, 35-37 s by default."""
        if self.ns != self.fault_target_robot:
            if self.fault_active:
                self._deactivate_fault(reason='non-target robot')
            return
        active_index = None
        planned_start = None
        for k in range(max(0, self.fault_repeat_count)):
            start = self.fault_start_s + k * self.fault_interval_s
            end = start + self.fault_duration_s
            if start <= elapsed_since_start < end:
                active_index = k + 1
                planned_start = start
                break

        if active_index is not None and not self.fault_active:
            self._timed_pulse_active_index = active_index
            self._timed_pulse_actual_start[active_index] = elapsed_since_start
            self._activate_fault(
                self.fault_duration_s,
                reason='timed pulse',
                pulse_index=active_index,
                planned_start_s=planned_start)
        elif active_index is None and self.fault_active:
            self._deactivate_fault(reason='timed pulse recovered')

    def _schedule_timed(self, now):
        """Fault sekali selama ttf_duration detik."""
        if self.fault_active:
            elapsed = now - self._inject_start
            if elapsed >= self.ttf_duration:
                self._deactivate_fault(reason='timed TTF selesai')

    def _schedule_random(self, now):
        """Fault sekali (atau beberapa kali) dengan durasi acak."""
        if self.fault_active:
            elapsed = now - self._inject_start
            planned = self._current_event['ttf_planned'] if self._current_event else self.ttf_duration
            if elapsed >= planned:
                self._deactivate_fault(reason='random TTF selesai')

    def _schedule_pulse(self, now):
        """Fault bergantian ON/OFF — untuk stress test."""
        if self.fault_active:
            elapsed = now - self._inject_start
            if elapsed >= self.pulse_on:
                self._deactivate_fault(reason='pulse OFF')
        else:
            # Cek apakah sudah waktunya ON lagi.
            # Gunakan _last_fault_end bukan _current_event['end_t'] karena
            # _deactivate_fault() men-clear _current_event sebelum kita
            # sempat membacanya di sini → pulse_off tidak pernah dihitung.
            if self._last_fault_end is not None:
                off_elapsed = now - self._last_fault_end
                if off_elapsed >= self.pulse_off:
                    self._trigger_next_fault(now)
            else:
                # Pertama kali — langsung aktifkan
                self._trigger_next_fault(now)

    def _trigger_next_fault(self, now):
        """Aktifkan fault injeksi berikutnya jika belum mencapai batas."""
        # Cek batas count
        if self.fault_count > 0 and self.fault_inject_count >= self.fault_count:
            self.get_logger().info(
                f'[INJECTOR] Semua {self.fault_count} fault sudah diinjeksi — selesai')
            return

        # Tentukan durasi TTF
        if self.fault_mode == FAULT_MODE_RANDOM:
            ttf = self.rng.uniform(self.ttf_min, self.ttf_max)
        elif self.fault_mode == FAULT_MODE_PULSE:
            ttf = self.pulse_on
        else:
            ttf = self.ttf_duration

        self._activate_fault(ttf, reason=f'scheduled ({self.fault_mode})')

    def _activate_fault(self, ttf_duration, reason='', pulse_index=None, planned_start_s=None):
        """Aktifkan fault — catat event untuk evaluasi."""
        now = self.get_clock().now().nanoseconds / 1e9
        self.fault_active       = True
        self._inject_start      = now
        self.fault_inject_count += 1
        self._freeze_cmd        = self.last_cmd_vel_raw  # [M6] snapshot utk mode 'freeze'

        self._current_event = {
            'index'      : self.fault_inject_count,
            'start_t'    : now,
            'end_t'      : None,
            'duration_s' : None,
            'ttf_planned': ttf_duration,
            'mode'       : self.fault_mode,
            'fault_type' : self.fault_type,
            'reason'     : reason,
            'pulse_index': pulse_index if pulse_index is not None else self.fault_inject_count,
            'planned_start_s': planned_start_s if planned_start_s is not None else '',
            'actual_start_s': (
                self._timed_pulse_actual_start.get(pulse_index, '')
                if pulse_index is not None else ''),
        }

        self.get_logger().warn(
            f'[FAULT INJECT #{self.fault_inject_count}] AKTIF | '
            f'target=/{self.ns} | TTF={ttf_duration:.2f}s | {reason}')

        # Publish log string
        log_msg      = String()
        log_msg.data = (f'FAULT_START,{self.ns},{now:.3f},'
                        f'{ttf_duration:.3f},{self.fault_mode},'
                        f'{self._current_event["pulse_index"]},'
                        f'{self._current_event["planned_start_s"]},'
                        f'{self._current_event["actual_start_s"]},'
                        f'{self.fault_type}')
        self.fault_log_pub.publish(log_msg)

    def _deactivate_fault(self, reason=''):
        """Matikan fault — selesaikan event log."""
        now = self.get_clock().now().nanoseconds / 1e9
        self.fault_active    = False
        self._last_fault_end = now   # dipakai _schedule_pulse untuk menghitung pulse_off

        if self._current_event is not None:
            self._current_event['end_t']      = now
            self._current_event['duration_s'] = (
                now - self._current_event['start_t'])
            actual_end_s = ''
            if self.fault_start_time is not None:
                actual_end_s = time.time() - self.fault_start_time
            self._current_event['actual_end_s'] = actual_end_s
            self.fault_events.append(self._current_event)

            actual   = self._current_event['duration_s']
            planned  = self._current_event['ttf_planned']
            self.get_logger().info(
                f'[FAULT INJECT #{self._current_event["index"]}] SELESAI | '
                f'aktual={actual:.3f}s | direncanakan={planned:.3f}s | {reason}')

            # Publish log string
            log_msg      = String()
            log_msg.data = (f'FAULT_END,{self.ns},{now:.3f},'
                            f'{actual:.3f},{planned:.3f},'
                            f'{self._current_event.get("pulse_index", "")},'
                            f'{self._current_event.get("planned_start_s", "")},'
                            f'{self._current_event.get("actual_start_s", "")},'
                            f'{actual_end_s},'
                            f'{self._current_event.get("fault_type", "")}')
            self.fault_log_pub.publish(log_msg)

            self._current_event = None
            self._timed_pulse_active_index = None

    # ═══════════════════════════════════════════════════════════════════════
    # STATISTIK EVALUASI
    # ═══════════════════════════════════════════════════════════════════════

    def get_fault_stats(self):
        """
        Return statistik fault untuk experiment_logger.
        Metrik: jumlah injeksi, durasi aktual vs direncanakan.
        """
        if not self.fault_events:
            return {'count': 0}

        actuals  = [e['duration_s'] for e in self.fault_events
                    if e['duration_s'] is not None]
        planneds = [e['ttf_planned'] for e in self.fault_events]

        return {
            'count'           : len(self.fault_events),
            'mean_actual_s'   : sum(actuals)  / len(actuals)  if actuals  else 0.0,
            'mean_planned_s'  : sum(planneds) / len(planneds) if planneds else 0.0,
            'total_fault_time': sum(actuals),
            'mode'            : self.fault_mode,
            'fault_type'      : self.fault_type,
            'log'             : self.fault_events,
        }

    # ═══════════════════════════════════════════════════════════════════════
    # STATUS REPORT — 1 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def status_report(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.fault_active and self._inject_start:
            fault_elapsed = now - self._inject_start
            planned_str = (f'{self._current_event["ttf_planned"]:.1f}'
                           if self._current_event else '?')
            fault_str = f'FAULT AKTIF {fault_elapsed:.1f}s / {planned_str}s'
        else:
            fault_str = 'normal'

        if self.experiment_started and self.fault_start_time is not None:
            exp_elapsed = time.time() - self.fault_start_time
            elapsed_str = f'exp={exp_elapsed:.0f}s'
        else:
            elapsed_str = 'STANDBY'

        self.get_logger().info(
            f'[INJECTOR] /{self.ns} | {fault_str} | '
            f'injeksi={self.fault_inject_count}/{self.fault_count or "∞"} | '
            f'hold={self.hold_active} | {elapsed_str}')


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Fault Injector stopped')
        stats = node.get_fault_stats()
        if stats['count'] > 0:
            node.get_logger().info(
                f'Fault stats: count={stats["count"]} | '
                f'mean_actual={stats["mean_actual_s"]:.3f}s | '
                f'mean_planned={stats["mean_planned_s"]:.3f}s')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
