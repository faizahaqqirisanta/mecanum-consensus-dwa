#!/usr/bin/env python3
"""
Global Path Node — haqqi_ta
Layer 2: Dijkstra path planner on OccupancyGrid

Subscribe:
  /map                   — OccupancyGrid dari map_server (robot1)
  /{ns}/amcl_pose        — PoseWithCovarianceStamped
  /{ns}/goal_pose        — PoseStamped, goal dari user/eksperimen

Publish:
  /{ns}/plan             — nav_msgs/Path hasil Dijkstra
  /{ns}/path_length      — Float32, total panjang path (m)
  /{ns}/remaining_length — Float32, sisa jarak ke goal (m)
  /{ns}/goal_reached     — Bool, True saat robot sudah sampai
"""

import rclpy
from rclpy.node import Node
import math
import heapq
import os
import csv
import json
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float32, Bool, Int32, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class GlobalPathNode(Node):

    def __init__(self):
        super().__init__('global_path_node')

        self.declare_parameter('robot_ns',          'robot1')
        self.declare_parameter('map_topic',          '/lss_carto/map')
        self.declare_parameter('costmap_inflation',  0.50)
        self.declare_parameter('goal_tolerance',          0.10)   # m — error goal min 0.1
        self.declare_parameter('intermediate_tolerance',  0.30)
        self.declare_parameter('waypoint_tolerance',      -1.0)
        self.declare_parameter('heading_goal_tolerance',  0.12)
        self.declare_parameter('final_align_timeout_s',   20.0)
        self.declare_parameter('skip_passed_waypoints_enabled', True)
        self.declare_parameter('waypoint_passed_path_margin', 0.18)
        self.declare_parameter('waypoint_dwell_time',     0.3)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('waypoint_spacing',   0.08)
        self.declare_parameter('turn_penalty_m',     0.6)
        self.declare_parameter('path_smoothing_enabled', True)
        self.declare_parameter('smoothing_input_step_m', 0.04)
        self.declare_parameter('smoothing_output_spacing_m', 0.06)
        self.declare_parameter('smoothing_data_weight', 0.06)
        self.declare_parameter('smoothing_smooth_weight', 0.45)
        self.declare_parameter('smoothing_iterations', 350)
        self.declare_parameter('smoothing_chaikin_iterations', 2)
        self.declare_parameter('smoothing_chaikin_cut', 0.25)
        self.declare_parameter('smoothing_shortcut_enabled', True)
        self.declare_parameter('replan_on_initial_pose_jump', True)
        self.declare_parameter('initial_pose_replan_window_s', 4.0)
        self.declare_parameter('initial_pose_jump_threshold', 0.25)
        self.declare_parameter('freeze_path_updates_during_running', True)
        # Topic AMCL — '/amcl_pose' jika AMCL tanpa namespace,
        # '/{robot_ns}/amcl_pose' jika AMCL dijalankan dalam namespace.
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        # formation_offset: geser goal secara lateral (tegak lurus heading goal).
        # Positif = kiri heading, negatif = kanan heading.
        # Robot1: +0.40, Robot2: 0.00, Robot3: -0.40 (untuk skenario formasi).
        self.declare_parameter('formation_offset', 0.0)

        self.ns                   = self.get_parameter('robot_ns').value
        self.inflation_m          = self.get_parameter('costmap_inflation').value
        self.goal_tolerance       = self.get_parameter('goal_tolerance').value
        self.heading_goal_tolerance = self.get_parameter('heading_goal_tolerance').value
        self.final_align_timeout_s  = self.get_parameter('final_align_timeout_s').value
        self._final_pos_since       = None
        waypoint_tol              = self.get_parameter('waypoint_tolerance').value
        self.intermediate_tol     = (
            waypoint_tol if waypoint_tol > 0.0
            else self.get_parameter('intermediate_tolerance').value)
        self.skip_passed_waypoints_enabled = self.get_parameter(
            'skip_passed_waypoints_enabled').value
        self.waypoint_passed_path_margin = self.get_parameter(
            'waypoint_passed_path_margin').value
        self.waypoint_dwell_time  = self.get_parameter('waypoint_dwell_time').value
        self.occ_thresh           = self.get_parameter('occupied_threshold').value
        self.waypoint_spacing     = self.get_parameter('waypoint_spacing').value
        self.turn_penalty_m       = self.get_parameter('turn_penalty_m').value
        self.path_smoothing_enabled = self.get_parameter('path_smoothing_enabled').value
        self.smoothing_input_step_m = self.get_parameter('smoothing_input_step_m').value
        self.smoothing_output_spacing_m = self.get_parameter('smoothing_output_spacing_m').value
        self.smoothing_data_weight = self.get_parameter('smoothing_data_weight').value
        self.smoothing_smooth_weight = self.get_parameter('smoothing_smooth_weight').value
        self.smoothing_iterations = self.get_parameter('smoothing_iterations').value
        self.smoothing_chaikin_iterations = self.get_parameter('smoothing_chaikin_iterations').value
        self.smoothing_chaikin_cut = self.get_parameter('smoothing_chaikin_cut').value
        self.smoothing_shortcut_enabled = self.get_parameter(
            'smoothing_shortcut_enabled').value
        self.replan_on_initial_pose_jump = self.get_parameter(
            'replan_on_initial_pose_jump').value
        self.initial_pose_replan_window_s = self.get_parameter(
            'initial_pose_replan_window_s').value
        self.initial_pose_jump_threshold = self.get_parameter(
            'initial_pose_jump_threshold').value
        self.freeze_path_updates_during_running = bool(
            self.get_parameter('freeze_path_updates_during_running').value)
        map_topic             = self.get_parameter('map_topic').value

        # ── [ALGO-TRACE] Perekaman PROSES Dijkstra + peta untuk live-plot.
        #    Dinyalakan default dari robot launch (algo_trace_enabled:=true).
        self.declare_parameter('algo_trace_enabled', False)
        self.declare_parameter('algo_trace_dir', '/tmp/algo_trace')
        self.algo_trace_enabled = bool(self.get_parameter('algo_trace_enabled').value)
        self.algo_trace_dir = str(self.get_parameter('algo_trace_dir').value)
        self._plan_seq = 0
        self._map_dumped = False
        if self.algo_trace_enabled:
            try:
                os.makedirs(self.algo_trace_dir, exist_ok=True)
                self.get_logger().info(
                    f'[ALGO-TRACE] global_path aktif → {self.algo_trace_dir}')
            except Exception as e:
                self.get_logger().warn(f'[ALGO-TRACE] gagal buat dir: {e}')

        self.get_logger().info(
            f'GlobalPathNode starting — namespace: /{self.ns} | '
            f'wp_spacing={self.waypoint_spacing:.2f}m, '
            f'wp_tol={self.intermediate_tol:.2f}m, '
            f'goal_tol={self.goal_tolerance:.2f}m, '
            f'smoothing={"on" if self.path_smoothing_enabled else "off"}')

        # State
        self.map_info     = None
        self.raw_grid     = None   # np.int8 array (height, width)
        self.inflated     = None   # bool array, True = obstacle
        self._global_costmap_msg = None
        self.robot_x      = None
        self.robot_y      = None
        self.current_path = []     # list of PoseStamped
        self.path_length  = 0.0
        self._path_cumulative = [0.0]
        self._progress_idx = 0          # [FIX-REM] indeks titik-terdekat monotonic
        self._rem_back_window = 8       # [FIX-REM] toleransi mundur (indeks) utk noise lokalisasi
        self._rem_crosstrack_cap = 0.5  # [FIX-REM] batas koreksi lateral remaining (m)
        self.goal_reached = False

        # Goal yang datang sebelum map/pose siap — retry otomatis saat siap
        self._pending_goal_msg = None
        self._waypoints_plan_pending = False

        # Waypoint queue
        self.waypoint_queue        = []   # list of (x, y, yaw)
        self.current_waypoint_idx  = 0
        self._waypoint_dwell_until = 0.0  # timestamp selesai dwell antar waypoint

        # Mission-level progress tracking
        self._seg_lengths     = []    # panjang tiap segmen (estimasi Euclidean → diganti Dijkstra)
        self.completed_length = 0.0   # akumulasi panjang segmen yang sudah selesai
        self.mission_total    = 0.0   # total panjang misi (diperbarui saat Dijkstra jalan)
        self.experiment_state = 'STOP'
        self._mission_total_locked = None
        self._last_waypoints_wall = None
        self._last_plan_start = None

        # Subscribers
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        # AMCL QoS: RELIABLE+VOLATILE agar kompatibel dengan publisher AMCL
        # apapun (TRANSIENT_LOCAL maupun VOLATILE). Subscriber TRANSIENT_LOCAL
        # tidak akan tersambung ke publisher VOLATILE (silent no-comm).
        amcl_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,
            map_topic, self.map_callback, map_qos)
        amcl_topic = self.get_parameter('amcl_pose_topic').value
        self.create_subscription(PoseWithCovarianceStamped,
            amcl_topic, self.pose_callback, amcl_qos)
        self.create_subscription(PoseStamped,
            f'/{self.ns}/goal_pose', self.goal_callback, 10)
        self.create_subscription(PoseArray,
            f'/{self.ns}/waypoints', self.waypoints_callback, 10)
        self.create_subscription(String,
            '/experiment_state', self._experiment_state_cb, 10)
        # Publishers
        self.plan_pub      = self.create_publisher(Path,    f'/{self.ns}/plan',              10)
        self.length_pub    = self.create_publisher(Float32, f'/{self.ns}/path_length',       10)
        self.remaining_pub = self.create_publisher(Float32, f'/{self.ns}/remaining_length',  10)
        self.reached_pub   = self.create_publisher(Bool,    f'/{self.ns}/goal_reached',      10)
        # waypoint_index: indeks WP yang BARU SAJA tercapai (0-based)
        # Publish setiap kali WP tercapai; PC Master bedakan intermediate vs final
        # dari nilai indeks (idx < total-1 = intermediate, idx == total-1 = final)
        self.wp_index_pub          = self.create_publisher(Int32,   f'/{self.ns}/waypoint_index',           10)
        self.mission_total_pub     = self.create_publisher(Float32, f'/{self.ns}/mission_total_length',     10)
        self.mission_remaining_pub = self.create_publisher(Float32, f'/{self.ns}/mission_remaining_length', 10)
        _costmap_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.global_costmap_pub = self.create_publisher(
            OccupancyGrid, f'/{self.ns}/global_costmap', _costmap_qos)

        # 10 Hz progress publisher
        self.create_timer(0.1, self._publish_progress)
        # Republish planned path for late subscribers (udp_sender/logger/RViz).
        self.create_timer(1.0, self._republish_plan)
        # 1 Hz global costmap publisher (untuk RViz)
        self.create_timer(1.0, self._publish_global_costmap)

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────

    def _path_updates_locked(self):
        return (self.freeze_path_updates_during_running
                and self.experiment_state == 'RUNNING'
                and bool(self.current_path))

    def _experiment_state_cb(self, msg: String):
        prev = self.experiment_state
        self.experiment_state = msg.data
        if self.experiment_state == 'RUNNING' and prev != 'RUNNING':
            if self.mission_total > 0.0:
                self._mission_total_locked = float(self.mission_total)
                self.get_logger().info(
                    f'[{self.ns}] mission_total locked for RUNNING: '
                    f'{self._mission_total_locked:.2f}m')
        elif self.experiment_state in ('STOP', 'READY') and prev == 'RUNNING':
            self._mission_total_locked = None
            self.get_logger().info(f'[{self.ns}] mission_total lock released')

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.raw_grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.inflated = self._build_inflated(self.raw_grid, msg.info)
        self._rebuild_global_costmap_msg()
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, '
            f'res={msg.info.resolution:.3f} m/cell')
        self._publish_global_costmap()
        if self.algo_trace_enabled and not self._map_dumped:
            self._dump_map_trace()
        self._retry_pending_goal()
        self._retry_pending_waypoints()

    def _rebuild_global_costmap_msg(self):
        if self.map_info is None or self.inflated is None:
            self._global_costmap_msg = None
            return
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info = self.map_info
        msg.data = np.where(self.inflated.ravel(), 100, 0).astype(np.int8).tolist()
        self._global_costmap_msg = msg

    def _publish_global_costmap(self):
        if self._global_costmap_msg is None:
            self._rebuild_global_costmap_msg()
        if self._global_costmap_msg is None:
            return
        self._global_costmap_msg.header.stamp = self.get_clock().now().to_msg()
        self.global_costmap_pub.publish(self._global_costmap_msg)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z),
                                    1.0 - 2.0 * (q.z * q.z))
        self._retry_pending_goal()
        self._retry_pending_waypoints()
        self._maybe_replan_after_initial_pose()

    def waypoints_callback(self, msg: PoseArray):
        if self._path_updates_locked():
            self.get_logger().warn(
                f'[{self.ns}] Waypoints ignored while RUNNING; '
                f'keeping locked trial path ({len(self.current_path)} poses)',
                throttle_duration_sec=2.0)
            return
        self.waypoint_queue = []
        for pose in msg.poses:
            x   = pose.position.x
            y   = pose.position.y
            yaw = math.atan2(
                2.0 * (pose.orientation.w * pose.orientation.z),
                1.0 - 2.0 * (pose.orientation.z ** 2))
            self.waypoint_queue.append((x, y, yaw))
        self.current_waypoint_idx  = 0
        self._waypoint_dwell_until = 0.0
        self.goal_reached          = False
        self.current_path          = []
        self.path_length           = 0.0
        self._path_cumulative      = [0.0]
        self._waypoints_plan_pending = True

        # Estimasi Euclidean awal — diganti nilai Dijkstra aktual oleh _plan_remaining_chained
        self._seg_lengths = []
        self.completed_length = 0.0
        for i, (wx, wy, _) in enumerate(self.waypoint_queue):
            px = (self.robot_x if self.robot_x is not None else wx) if i == 0 \
                 else self.waypoint_queue[i - 1][0]
            py = (self.robot_y if self.robot_y is not None else wy) if i == 0 \
                 else self.waypoint_queue[i - 1][1]
            self._seg_lengths.append(math.hypot(wx - px, wy - py))
        self.mission_total = sum(self._seg_lengths)

        self.get_logger().info(
            f'Waypoints diterima: {len(self.waypoint_queue)} titik | '
            f'mission_total_est={self.mission_total:.2f}m')
        self._last_waypoints_wall = self.get_clock().now().nanoseconds / 1e9
        if self.waypoint_queue:
            self._plan_remaining_chained()

    def _maybe_replan_after_initial_pose(self):
        """
        Initialpose dari PC bisa tiba sedikit setelah waypoints.
        Jika path sempat dihitung dari pose lama, replan sekali dari pose AMCL
        yang baru agar path awal tidak meloncat/curam saat mission progress masih 0.
        """
        if not self.replan_on_initial_pose_jump:
            return
        if self._path_updates_locked():
            return
        if not self.waypoint_queue or not self.current_path:
            return
        if self.current_waypoint_idx != 0 or self.goal_reached:
            return
        if self._last_waypoints_wall is None or self._last_plan_start is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_waypoints_wall) > self.initial_pose_replan_window_s:
            return
        sx, sy = self._last_plan_start
        jump = math.hypot(self.robot_x - sx, self.robot_y - sy)
        if jump < self.initial_pose_jump_threshold:
            return
        self.get_logger().warn(
            f'[{self.ns}] Pose berubah {jump:.2f}m setelah waypoints — '
            f'replan dari initial pose terbaru')
        self._plan_remaining_chained()

    def _plan_remaining_chained(self):
        """Jalankan Dijkstra untuk semua segmen tersisa, gabung jadi satu path, publish sekali."""
        if self._path_updates_locked():
            self.get_logger().warn(
                f'[{self.ns}] Chained replan ignored while RUNNING; '
                f'keeping locked trial path ({len(self.current_path)} poses)',
                throttle_duration_sec=2.0)
            return
        if not self.waypoint_queue or self.inflated is None or self.robot_x is None:
            self._waypoints_plan_pending = True
            self.get_logger().warn(f'[{self.ns}] Chained plan: map/pose belum siap')
            return

        remaining_wps = self.waypoint_queue[self.current_waypoint_idx:]
        if not remaining_wps:
            return

        prev_x, prev_y = self.robot_x, self.robot_y
        self._last_plan_start = (prev_x, prev_y)
        combined_poses = []
        seg_lengths_actual = []
        offset = self.get_parameter('formation_offset').value
        n = len(remaining_wps)

        for i, (wx, wy, wyaw) in enumerate(remaining_wps):
            is_final   = (i == n - 1)
            global_idx = self.current_waypoint_idx + i

            if not is_final:
                nx, ny, _ = remaining_wps[i + 1]
                goal_yaw  = math.atan2(ny - wy, nx - wx)
            else:
                goal_yaw = wyaw

            gx, gy = wx, wy
            if abs(offset) > 0.001:
                gx += offset * (-math.sin(goal_yaw))
                gy += offset *   math.cos(goal_yaw)

            tmp_goal = PoseStamped()
            tmp_goal.pose.orientation.z = math.sin(goal_yaw / 2)
            tmp_goal.pose.orientation.w = math.cos(goal_yaw / 2)

            self.get_logger().info(
                f'[{self.ns}] Dijkstra WP{global_idx+1}: '
                f'({prev_x:.2f},{prev_y:.2f})→({gx:.2f},{gy:.2f})')

            seg = self._dijkstra(prev_x, prev_y, gx, gy, tmp_goal.pose.orientation, smooth=False)
            if seg is None:
                self.get_logger().error(
                    f'[{self.ns}] Dijkstra gagal untuk WP {global_idx+1} — abort chained plan')
                return

            seg_len = self._path_length(seg)
            seg_lengths_actual.append(seg_len)

            if combined_poses:
                seg = seg[1:]  # hapus duplikat titik junction
            combined_poses.extend(seg)

            prev_x, prev_y = gx, gy

        if not combined_poses:
            return

        # Perbarui _seg_lengths dengan panjang Dijkstra aktual
        for i, seg_len in enumerate(seg_lengths_actual):
            idx = self.current_waypoint_idx + i
            if idx < len(self._seg_lengths):
                old_est = self._seg_lengths[idx]
                self._seg_lengths[idx] = seg_len
                self.mission_total = max(0.0, self.mission_total + seg_len - old_est)
            else:
                self._seg_lengths.append(seg_len)
                self.mission_total += seg_len

        final_orientation = combined_poses[-1].pose.orientation
        combined_coords = [
            (p.pose.position.x, p.pose.position.y)
            for p in combined_poses
        ]
        # Smooth ulang path gabungan agar belokan antar waypoint tidak patah.
        # skip_shortcut=True menjaga planner tidak langsung melompati urutan WP.
        smoothed_coords = self._smooth_path(combined_coords, skip_shortcut=True)
        combined_poses = self._build_poses_from_coords(smoothed_coords, final_orientation)

        total_len = self._path_length(combined_poses)
        self.current_path = combined_poses
        self.path_length  = total_len
        self._cache_path_cumulative(combined_poses)
        self.goal_reached = False

        b_msg = Bool()
        b_msg.data = False
        self.reached_pub.publish(b_msg)

        plan_msg = Path()
        plan_msg.header.stamp    = self.get_clock().now().to_msg()
        plan_msg.header.frame_id = 'map'
        plan_msg.poses           = combined_poses
        self.plan_pub.publish(plan_msg)

        l_msg = Float32()
        l_msg.data = float(total_len)
        self.length_pub.publish(l_msg)

        self._waypoints_plan_pending = False
        self.get_logger().info(
            f'[{self.ns}] Chained path: {len(combined_poses)} poses, {total_len:.2f}m | '
            f'WP1→WP{len(self.waypoint_queue)}')

    def goal_callback(self, msg: PoseStamped):
        """Subscription callback /{ns}/goal_pose — goal tunggal dari user/eksperimen.

        Membersihkan waypoint mode agar queue lama tidak mempengaruhi
        logika 'goal reached' untuk goal tunggal ini.
        """
        if self._path_updates_locked():
            self.get_logger().warn(
                f'[{self.ns}] Goal ignored while RUNNING; keeping locked trial path',
                throttle_duration_sec=2.0)
            return
        # Clear waypoint mode: single-goal tidak boleh membaca queue lama.
        # _seg_lengths dan mission_total juga di-reset agar _publish_path
        # tidak menghitung delta dari misi waypoint sebelumnya.
        self.waypoint_queue        = []
        self.current_waypoint_idx  = 0
        self._waypoint_dwell_until = 0.0
        self._seg_lengths          = []
        self.completed_length      = 0.0
        self.mission_total         = 0.0
        self._plan_to_goal(msg)

    def _plan_to_goal(self, msg: PoseStamped):
        """Hitung dan publish path ke goal."""
        if self.inflated is None or self.robot_x is None:
            if self._pending_goal_msg is not None:
                self.get_logger().warn(
                    f'[{self.ns}] Pending goal lama tertimpa — '
                    f'hanya satu goal bisa antri saat map/pose belum siap')
            self.get_logger().warn(
                f'[{self.ns}] Goal diterima tapi map/pose belum siap — disimpan, retry otomatis')
            self._pending_goal_msg = msg
            return
        self._pending_goal_msg = None
        path, length = self._compute_path(msg)
        if path is None:
            self.get_logger().error('Dijkstra: no path found to goal')
            return
        self._publish_path(path, length)

    def _retry_pending_goal(self):
        """Coba jalankan goal tertunda setelah map dan pose tersedia."""
        if self._pending_goal_msg is not None and self.inflated is not None and self.robot_x is not None:
            self.get_logger().info(f'[{self.ns}] Retrying pending goal setelah map/pose siap')
            self._plan_to_goal(self._pending_goal_msg)

    def _retry_pending_waypoints(self):
        """Coba plan waypoints yang gagal karena map/pose belum siap saat diterima."""
        if (self._waypoints_plan_pending
                and self.waypoint_queue
                and self.inflated is not None
                and self.robot_x is not None):
            self.get_logger().info(
                f'[{self.ns}] Retry waypoint plan setelah map/pose tersedia '
                f'({len(self.waypoint_queue)} titik)')
            self._plan_remaining_chained()

    def _compute_path(self, msg: PoseStamped):
        """Jalankan Dijkstra, return (path, length) atau (None, 0.0)."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # formation_offset: geser goal secara lateral sebelum Dijkstra dihitung.
        # Ini memastikan global path sudah terpisah dan tidak tumpang tindih.
        offset = self.get_parameter('formation_offset').value
        if abs(offset) > 0.001:
            q   = msg.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            gx += offset * (-math.sin(yaw))
            gy += offset *   math.cos(yaw)

        self.get_logger().info(
            f'[{self.ns}] Hitung path ke ({gx:.2f},{gy:.2f}) '
            f'dari ({self.robot_x:.2f},{self.robot_y:.2f})'
            + (f' [offset={offset:+.2f}m]' if abs(offset) > 0.001 else ''))
        path = self._dijkstra(self.robot_x, self.robot_y, gx, gy, msg.pose.orientation)
        if path is None:
            return None, 0.0
        return path, self._path_length(path)

    def _publish_path(self, path, length):
        """Publish plan + path_length dan update state internal."""
        self.current_path = path
        self.path_length  = length
        self._cache_path_cumulative(path)

        # Ganti estimasi Euclidean segmen ini dengan panjang Dijkstra aktual
        idx = self.current_waypoint_idx
        if self._seg_lengths and idx < len(self._seg_lengths):
            old_est = self._seg_lengths[idx]
            self._seg_lengths[idx] = length
            self.mission_total = max(0.0, self.mission_total + length - old_est)

        # Selalu reset goal_reached dan publish False — tanpa kondisi.
        # Ini penting agar udp_sender_node dan priority_manager tidak
        # membaca cache goal_reached=True dari trial sebelumnya.
        self.goal_reached = False
        b_msg = Bool()
        b_msg.data = False
        self.reached_pub.publish(b_msg)

        plan_msg = Path()
        plan_msg.header.stamp    = self.get_clock().now().to_msg()
        plan_msg.header.frame_id = 'map'
        plan_msg.poses = path
        self.plan_pub.publish(plan_msg)

        l_msg = Float32()
        l_msg.data = float(length)
        self.length_pub.publish(l_msg)

        self.get_logger().info(
            f'[{self.ns}] Path published: {len(path)} waypoints, {length:.2f} m')

    # ──────────────────────────────────────────────────────────────────────
    # Progress timer
    # ──────────────────────────────────────────────────────────────────────

    def _republish_plan(self):
        if not self.current_path:
            return

        plan_msg = Path()
        plan_msg.header.stamp = self.get_clock().now().to_msg()
        plan_msg.header.frame_id = 'map'
        plan_msg.poses = self.current_path
        self.plan_pub.publish(plan_msg)

        l_msg = Float32()
        l_msg.data = float(self.path_length)
        self.length_pub.publish(l_msg)

    def _publish_progress(self):
        if not self.current_path or self.robot_x is None:
            return

        l_msg = Float32()
        l_msg.data = float(self.path_length)
        self.length_pub.publish(l_msg)

        remaining = self._remaining_length()
        r_msg = Float32()
        r_msg.data = float(remaining)
        self.remaining_pub.publish(r_msg)

        # Dengan chained path, remaining sudah mencakup seluruh sisa misi
        mr_msg = Float32()
        mr_msg.data = float(remaining)
        self.mission_remaining_pub.publish(mr_msg)
        mt_msg = Float32()
        if self.experiment_state == 'RUNNING':
            if self._mission_total_locked is None and self.mission_total > 0.0:
                self._mission_total_locked = float(self.mission_total)
                self.get_logger().info(
                    f'[{self.ns}] mission_total late-lock for RUNNING: '
                    f'{self._mission_total_locked:.2f}m')
            mt_msg.data = float(
                self._mission_total_locked
                if self._mission_total_locked is not None
                else self.mission_total)
        else:
            mt_msg.data = float(self.mission_total)
        self.mission_total_pub.publish(mt_msg)

        if self.goal_reached:
            return

        # Deteksi intermediate waypoints — hanya untuk logging & progress counter.
        # Waypoint diperlakukan sebagai guide: kalau robot sudah lewat di sepanjang
        # path, jangan paksa balik hanya untuk menyentuh titiknya presisi.
        self._advance_intermediate_waypoints()

        # Deteksi final goal: remaining kecil DAN jarak ke final waypoint kecil.
        # Validasi ganda mencegah false goal_reached saat remaining=0 tapi
        # robot masih jauh dari goal (akibat nearest-path-point di ujung path).
        is_final = (len(self.waypoint_queue) == 0 or
                    self.current_waypoint_idx == len(self.waypoint_queue) - 1)
        if is_final and self.waypoint_queue:
            final_x, final_y, final_yaw = self.waypoint_queue[-1]
        elif is_final and self.current_path:
            last = self.current_path[-1].pose
            final_x, final_y = last.position.x, last.position.y
            final_yaw = math.atan2(2.0 * (last.orientation.w * last.orientation.z),
                                   1.0 - 2.0 * (last.orientation.z ** 2))
        else:
            final_x, final_y, final_yaw = None, None, None

        dist_to_final = (math.hypot(final_x - self.robot_x, final_y - self.robot_y)
                         if final_x is not None else float('inf'))

        if is_final and remaining < self.goal_tolerance and dist_to_final < self.goal_tolerance:
            now = self.get_clock().now().nanoseconds / 1e9
            if self._final_pos_since is None:
                self._final_pos_since = now
            robot_yaw = getattr(self, 'robot_yaw', None)
            heading_ok = True
            if final_yaw is not None and robot_yaw is not None:
                herr = abs(math.atan2(math.sin(final_yaw - robot_yaw),
                                      math.cos(final_yaw - robot_yaw)))
                heading_ok = (herr < self.heading_goal_tolerance)
            timed_out = (now - self._final_pos_since) >= self.final_align_timeout_s
            if heading_ok or timed_out:
                self.goal_reached = True
                idx_msg = Int32()
                idx_msg.data = max(0, len(self.waypoint_queue) - 1)
                self.wp_index_pub.publish(idx_msg)
                b_msg = Bool()
                b_msg.data = True
                self.reached_pub.publish(b_msg)
                self.get_logger().info(
                    f'[{self.ns}] MISI SELESAI — semua {len(self.waypoint_queue)} '
                    f'waypoint tercapai!'
                    + ('' if heading_ok else ' [heading timeout]'))
        else:
            self._final_pos_since = None

    def _advance_intermediate_waypoints(self):
        if not self.waypoint_queue:
            return
        while self.current_waypoint_idx < len(self.waypoint_queue) - 1:
            idx = self.current_waypoint_idx
            wp_x, wp_y, _ = self.waypoint_queue[idx]
            dist_to_wp = math.hypot(wp_x - self.robot_x, wp_y - self.robot_y)

            if dist_to_wp <= self.intermediate_tol:
                self._mark_intermediate_waypoint_passed(
                    idx, 'within_tolerance', dist_to_wp)
                continue

            if (self.skip_passed_waypoints_enabled and
                    self._waypoint_is_behind_on_path(wp_x, wp_y)):
                self._mark_intermediate_waypoint_passed(
                    idx, 'passed_on_path', dist_to_wp)
                continue

            break

    def _mark_intermediate_waypoint_passed(self, idx, reason, dist_to_wp):
        idx_msg = Int32()
        idx_msg.data = idx
        self.wp_index_pub.publish(idx_msg)
        if self._seg_lengths and idx < len(self._seg_lengths):
            self.completed_length += self._seg_lengths[idx]
        self.get_logger().info(
            f'[{self.ns}] Waypoint {idx + 1}/{len(self.waypoint_queue)} '
            f'dilewati ({reason}, d={dist_to_wp:.2f}m, tol={self.intermediate_tol:.2f}m) | '
            f'completed={self.completed_length:.2f}m')
        self.current_waypoint_idx += 1

    def _waypoint_is_behind_on_path(self, wp_x, wp_y):
        if not self.current_path or self.robot_x is None:
            return False
        robot_idx = self._nearest_path_index(self.robot_x, self.robot_y)
        wp_idx = self._nearest_path_index(wp_x, wp_y)
        if robot_idx <= wp_idx:
            return False
        path_margin = self._path_distance_between_indices(wp_idx, robot_idx)
        return path_margin >= self.waypoint_passed_path_margin

    def _nearest_path_index(self, x, y):
        min_d = float('inf')
        min_idx = 0
        for i, p in enumerate(self.current_path):
            px = p.pose.position.x
            py = p.pose.position.y
            d = math.hypot(px - x, py - y)
            if d < min_d:
                min_d = d
                min_idx = i
        return min_idx

    def _path_distance_between_indices(self, start_idx, end_idx):
        if start_idx >= end_idx:
            return 0.0
        total = 0.0
        start_idx = max(0, start_idx)
        end_idx = min(end_idx, len(self.current_path) - 1)
        for i in range(start_idx, end_idx):
            p0 = self.current_path[i].pose.position
            p1 = self.current_path[i + 1].pose.position
            total += math.hypot(p1.x - p0.x, p1.y - p0.y)
        return total

    # ──────────────────────────────────────────────────────────────────────
    # Dijkstra
    # ──────────────────────────────────────────────────────────────────────

    def _dijkstra(self, sx, sy, gx, gy, goal_orientation, smooth=True):
        sr, sc = self._w2g(sx, sy)
        gr, gc = self._w2g(gx, gy)

        # Snap start/goal to nearest free cell if needed
        if not self._free(sr, sc):
            sr, sc = self._nearest_free(sr, sc)
            if sr is None:
                self.get_logger().error('Start cell blocked and no free cell nearby')
                return None

        if not self._free(gr, gc):
            gr, gc = self._nearest_free(gr, gc)
            if gr is None:
                self.get_logger().error('Goal cell blocked and no free cell nearby')
                return None

        MOVES = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
                 (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]

        # State: (r, c, dr, dc) — arah datang disertakan untuk penalti belok
        # dr=dc=0 berarti belum ada arah (start)
        start_state = (sr, sc, 0, 0)
        dist = {start_state: 0.0}
        prev = {}
        heap = [(0.0, sr, sc, 0, 0)]

        # [ALGO-TRACE] rekam urutan ekspansi node (cara Dijkstra merambat)
        _trace = self.algo_trace_enabled
        _explore = []
        _order = 0

        while heap:
            cost, r, c, pdr, pdc = heapq.heappop(heap)
            state = (r, c, pdr, pdc)
            if cost > dist.get(state, float('inf')):
                continue
            if _trace:
                _explore.append((_order, r, c, cost))
                _order += 1
            if r == gr and c == gc:
                break
            for dr, dc, w in MOVES:
                nr, nc = r + dr, c + dc
                if not self._free(nr, nc):
                    continue
                # Penalti belok: tambah turn_penalty_m jika arah berubah
                turn = 0.0 if (pdr == 0 and pdc == 0) or (dr == pdr and dc == pdc) \
                       else self.turn_penalty_m
                nc_cost = cost + w * self.map_info.resolution + turn
                nstate = (nr, nc, dr, dc)
                if nc_cost < dist.get(nstate, float('inf')):
                    dist[nstate] = nc_cost
                    prev[nstate] = state
                    heapq.heappush(heap, (nc_cost, nr, nc, dr, dc))

        # Cari state terbaik di goal cell
        goal_state = min(
            (s for s in dist if s[0] == gr and s[1] == gc),
            key=lambda s: dist[s],
            default=None)

        if goal_state is None and (gr, gc) != (sr, sc):
            return None

        # Reconstruct
        cells = []
        cur = goal_state if goal_state else start_state
        while cur in prev:
            cells.append((cur[0], cur[1]))
            cur = prev[cur]
        cells.append((sr, sc))
        cells.reverse()

        if _trace:
            try:
                self._plan_seq += 1
                self._write_dijkstra_trace(_explore, set(cells), sr, sc, gr, gc)
            except Exception as e:
                self.get_logger().warn(
                    f'[ALGO-TRACE] tulis trace Dijkstra gagal: {e}')

        # Downsample to waypoint_spacing. Spacing sengaja rapat karena DWA dan
        # auto-conflict-zone membaca path ini langsung.
        step = max(1, int(self.waypoint_spacing / self.map_info.resolution))
        sampled = cells[::step]
        if sampled[-1] != cells[-1]:
            sampled.append(cells[-1])

        # Smooth world coordinates sebelum build PoseStamped.
        # Saat dipanggil dari _plan_remaining_chained (smooth=False), smoothing
        # ditunda agar hanya dilakukan sekali pada combined path — menghindari
        # double-smooth yang membuang waktu dan bisa mendorong path ke obstacle.
        # Shortcut tetap dijalankan per-segmen untuk bersihkan zigzag grid Dijkstra,
        # tapi NOT pada combined path agar arc waypoint antar segmen tidak dilewati.
        raw_coords = [self._g2w(r, c) for r, c in sampled]
        if smooth:
            final_coords = self._smooth_path(raw_coords)
        else:
            final_coords = (self._shortcut_path(raw_coords)
                            if self.smoothing_shortcut_enabled else raw_coords)

        return self._build_poses_from_coords(final_coords, goal_orientation)

    def _build_poses_from_coords(self, coords, goal_orientation):
        """Build PoseStamped list; yaw intermediate mengikuti tangent path halus."""
        goal_yaw = self._quat_yaw(goal_orientation)
        poses = []
        now = self.get_clock().now().to_msg()
        heading_window = 8
        for i, (wx, wy) in enumerate(coords):
            if i < len(coords) - 1:
                prev_i = max(0, i - heading_window)
                next_i = min(len(coords) - 1, i + heading_window)
                px, py = coords[prev_i]
                nx, ny = coords[next_i]
                if next_i == prev_i:
                    nx, ny = coords[min(len(coords) - 1, i + 1)]
                    px, py = wx, wy
                yaw = math.atan2(ny - py, nx - px)
            else:
                yaw = goal_yaw
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp    = now
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(ps)
        return poses

    def _smooth_path(self, coords, skip_shortcut=False):
        """Path smoothing bertahap dengan validasi collision.

        Dijkstra tetap menentukan koridor aman. Tahap ini hanya membuat output
        lebih rapat dan membulatkan belokan untuk mengurangi rotasi mendadak di
        local planner.

        skip_shortcut=True: abaikan shortcut pada pemanggilan ini (dipakai oleh
        _plan_remaining_chained agar shortcut tidak melompati arc waypoint antar
        segmen). Shortcut per-segmen sudah dijalankan lebih awal di _dijkstra.
        """
        if len(coords) <= 2 or not self.path_smoothing_enabled:
            return coords

        use_shortcut = self.smoothing_shortcut_enabled and not skip_shortcut
        base = (self._shortcut_path(coords) if use_shortcut else coords)
        dense = self._densify_coords(base, self.smoothing_input_step_m)
        if len(dense) <= 2:
            return coords

        # Coba agresif dulu, lalu turun perlahan kalau path memotong obstacle.
        attempts = [
            (self.smoothing_data_weight,
             self.smoothing_smooth_weight,
             int(self.smoothing_iterations),
             int(self.smoothing_chaikin_iterations),
             float(self.smoothing_chaikin_cut)),
            (max(self.smoothing_data_weight, 0.10),
             min(self.smoothing_smooth_weight, 0.35),
             max(180, int(self.smoothing_iterations * 0.65)),
             max(1, int(self.smoothing_chaikin_iterations) - 1),
             min(float(self.smoothing_chaikin_cut), 0.20)),
            (0.18, 0.22, 140, 0, 0.0),
        ]

        for alpha, beta, iterations, chaikin_iters, chaikin_cut in attempts:
            candidate = dense
            if chaikin_iters > 0 and chaikin_cut > 0.0:
                candidate = self._chaikin_path(candidate, chaikin_iters, chaikin_cut)
            candidate = self._gradient_smooth(candidate, alpha, beta, iterations)
            candidate = self._resample_coords(candidate, self.smoothing_output_spacing_m)
            if self._path_is_free(candidate):
                return candidate

        self.get_logger().warn(
            f'[{self.ns}] Smoothing path ditolak oleh collision check — pakai Dijkstra mentah')
        return coords

    def _gradient_smooth(self, coords, alpha, beta, iterations):
        """Classic gradient path smoothing dengan endpoint tetap."""
        orig = [[float(x), float(y)] for x, y in coords]
        s = [[float(x), float(y)] for x, y in coords]
        for _ in range(max(0, int(iterations))):
            for i in range(1, len(s) - 1):
                s[i][0] += alpha * (orig[i][0] - s[i][0]) + beta * (s[i-1][0] + s[i+1][0] - 2*s[i][0])
                s[i][1] += alpha * (orig[i][1] - s[i][1]) + beta * (s[i-1][1] + s[i+1][1] - 2*s[i][1])
        return [(p[0], p[1]) for p in s]

    def _chaikin_path(self, coords, iterations, cut):
        """Corner cutting ringan untuk membulatkan polyline Dijkstra."""
        result = list(coords)
        cut = max(0.0, min(0.45, float(cut)))
        for _ in range(max(0, int(iterations))):
            if len(result) <= 2:
                break
            new_path = [result[0]]
            for p0, p1 in zip(result[:-1], result[1:]):
                qx = (1.0 - cut) * p0[0] + cut * p1[0]
                qy = (1.0 - cut) * p0[1] + cut * p1[1]
                rx = cut * p0[0] + (1.0 - cut) * p1[0]
                ry = cut * p0[1] + (1.0 - cut) * p1[1]
                new_path.append((qx, qy))
                new_path.append((rx, ry))
            new_path.append(result[-1])
            result = new_path
        return result

    def _shortcut_path(self, coords):
        """Hilangkan anak tangga Dijkstra jika segmen lurusnya tetap bebas."""
        if len(coords) <= 2:
            return coords

        simplified = [coords[0]]
        i = 0
        last_idx = len(coords) - 1
        while i < last_idx:
            j = last_idx
            while j > i + 1:
                if self._segment_free(
                        coords[i][0], coords[i][1],
                        coords[j][0], coords[j][1]):
                    break
                j -= 1
            simplified.append(coords[j])
            i = j

        return simplified

    def _densify_coords(self, coords, max_step):
        """Tambahkan titik interpolasi agar smoothing punya resolusi cukup."""
        if len(coords) <= 1:
            return coords
        max_step = max(0.01, float(max_step))
        dense = [coords[0]]
        for x1, y1 in coords[1:]:
            x0, y0 = dense[-1]
            dist = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(math.ceil(dist / max_step)))
            for k in range(1, n + 1):
                t = k / n
                dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        return dense

    def _resample_coords(self, coords, spacing):
        """Resample path ke spacing seragam supaya yaw antar pose tidak meloncat."""
        if len(coords) <= 2:
            return coords
        spacing = max(0.01, float(spacing))
        sampled = [coords[0]]
        carry = spacing
        x0, y0 = coords[0]
        for x1, y1 in coords[1:]:
            seg_len = math.hypot(x1 - x0, y1 - y0)
            while seg_len >= carry and seg_len > 1e-9:
                t = carry / seg_len
                nx = x0 + t * (x1 - x0)
                ny = y0 + t * (y1 - y0)
                sampled.append((nx, ny))
                x0, y0 = nx, ny
                seg_len = math.hypot(x1 - x0, y1 - y0)
                carry = spacing
            carry -= seg_len
            x0, y0 = x1, y1
        if sampled[-1] != coords[-1]:
            sampled.append(coords[-1])
        return sampled

    def _path_is_free(self, coords):
        if len(coords) <= 1:
            return True
        for x, y in coords:
            r, c = self._w2g(x, y)
            if not self._free(r, c):
                return False
        for p0, p1 in zip(coords[:-1], coords[1:]):
            if not self._segment_free(p0[0], p0[1], p1[0], p1[1]):
                return False
        return True

    def _segment_free(self, x1, y1, x2, y2):
        """Bresenham line check: True jika semua sel pada segmen (x1,y1)→(x2,y2) bebas obstacle."""
        r1, c1 = self._w2g(x1, y1)
        r2, c2 = self._w2g(x2, y2)
        dr = abs(r2 - r1)
        dc = abs(c2 - c1)
        sr = 1 if r2 > r1 else -1
        sc = 1 if c2 > c1 else -1
        r, c = r1, c1
        err  = dr - dc
        for _ in range(dr + dc + 2):
            if not self._free(r, c):
                return False
            if r == r2 and c == c2:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r   += sr
            if e2 < dr:
                err += dr
                c   += sc
        return True

    # ──────────────────────────────────────────────────────────────────────
    # Map helpers
    # ──────────────────────────────────────────────────────────────────────

    def _build_inflated(self, grid, info):
        radius_cells = int(math.ceil(self.inflation_m / info.resolution))
        obstacle = ((grid >= self.occ_thresh) | (grid < 0))
        dist_map = distance_transform_edt(~obstacle)
        inflated = dist_map <= radius_cells
        return inflated

    def _free(self, row, col):
        h, w = self.inflated.shape
        if row < 0 or row >= h or col < 0 or col >= w:
            return False
        return not self.inflated[row, col]

    def _w2g(self, x, y):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        return int((y - oy) / res), int((x - ox) / res)

    def _g2w(self, row, col):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        return ox + (col + 0.5) * res, oy + (row + 0.5) * res

    def _nearest_free(self, row, col, max_r=15):
        for r in range(1, max_r + 1):
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    if abs(dr) == r or abs(dc) == r:
                        if self._free(row + dr, col + dc):
                            return row + dr, col + dc
        return None, None

    # ──────────────────────────────────────────────────────────────────────
    # Path length helpers
    # ──────────────────────────────────────────────────────────────────────

    def _path_length(self, path):
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i].pose.position.x - path[i-1].pose.position.x
            dy = path[i].pose.position.y - path[i-1].pose.position.y
            total += math.hypot(dx, dy)
        return total

    def _cache_path_cumulative(self, path):
        cumulative = [0.0]
        for i in range(1, len(path)):
            dx = path[i].pose.position.x - path[i-1].pose.position.x
            dy = path[i].pose.position.y - path[i-1].pose.position.y
            cumulative.append(cumulative[-1] + math.hypot(dx, dy))
        self._path_cumulative = cumulative
        # [FIX-REM] Geometri path baru -> restart pelacakan kemajuan. Path selalu
        # dibangun mulai dari posisi robot terkini, jadi idx kembali ke 0 itu benar.
        self._progress_idx = 0

    def _remaining_length(self):
        if not self.current_path or self.robot_x is None:
            return 0.0
        n = len(self.current_path)
        # [FIX-REM] Cari titik terdekat secara MAJU (monotonic) dgn jendela mundur
        # kecil. Tanpa batasan ini, idx bisa melompat MUNDUR saat path mendekati
        # dirinya sendiri (junction/belokan) atau saat lokalisasi/scan lidar goyang,
        # sehingga remaining melonjak naik secara keliru (mis. 3m -> 10m).
        start = max(0, self._progress_idx - self._rem_back_window)
        min_d, idx = float('inf'), start
        for i in range(start, n):
            p = self.current_path[i]
            d = math.hypot(p.pose.position.x - self.robot_x,
                           p.pose.position.y - self.robot_y)
            if d < min_d:
                min_d, idx = d, i
        # Commit kemajuan: idx tidak boleh mundur.
        self._progress_idx = max(self._progress_idx, idx)
        idx = self._progress_idx
        if len(self._path_cumulative) == n:
            path_total = self._path_cumulative[-1]
            total = path_total - self._path_cumulative[idx]
        else:
            total = 0.0
            for i in range(idx, n - 1):
                dx = self.current_path[i+1].pose.position.x - self.current_path[i].pose.position.x
                dy = self.current_path[i+1].pose.position.y - self.current_path[i].pose.position.y
                total += math.hypot(dx, dy)
            path_total = total
        # [FIX-REM] Koreksi lateral (cross-track) DIBATASI agar lompatan lokalisasi
        # tidak menggelembungkan remaining; lalu clamp ke [0, total] sehingga
        # remaining tidak pernah melebihi panjang jalur.
        remaining = total + min(min_d, self._rem_crosstrack_cap)
        return max(0.0, min(remaining, path_total))

    @staticmethod
    def _quat_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))


    def _write_dijkstra_trace(self, explore, final_set, sr, sc, gr, gc):
        """[ALGO-TRACE] Urutan ekspansi node Dijkstra + cost-to-come ke CSV."""
        ts = time.strftime('%Y%m%d_%H%M%S')
        fname = os.path.join(
            self.algo_trace_dir,
            f'dijkstra_explore_{self.ns}_{self._plan_seq:03d}_{ts}.csv')
        with open(fname, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['expand_order', 'x', 'y', 'grid_r', 'grid_c',
                        'cost_to_come', 'on_final_path', 'is_start', 'is_goal'])
            for order, r, c, cost in explore:
                wx, wy = self._g2w(r, c)
                w.writerow([order, f'{wx:.4f}', f'{wy:.4f}', r, c,
                            f'{cost:.4f}',
                            1 if (r, c) in final_set else 0,
                            1 if (r, c) == (sr, sc) else 0,
                            1 if (r, c) == (gr, gc) else 0])
        self.get_logger().info(
            f'[ALGO-TRACE] Dijkstra trace: {fname} ({len(explore)} node dibuka)')

    def _dump_map_trace(self):
        """Simpan peta arena sekali per run sebagai latar animasi MATLAB."""
        if self.map_info is None or self.raw_grid is None or self.inflated is None:
            return
        res = self.map_info.resolution
        ox  = self.map_info.origin.position.x
        oy  = self.map_info.origin.position.y
        h, w_ = self.raw_grid.shape
        meta = {'ns': self.ns, 'resolution': float(res),
                'origin_x': float(ox), 'origin_y': float(oy),
                'width': int(w_), 'height': int(h),
                'occupied_threshold': int(self.occ_thresh),
                'inflation_m': float(self.inflation_m)}
        with open(os.path.join(self.algo_trace_dir,
                               f'map_meta_{self.ns}.json'), 'w') as f:
            json.dump(meta, f, indent=2)
        occ = np.argwhere((self.raw_grid >= self.occ_thresh) | (self.raw_grid < 0))
        with open(os.path.join(self.algo_trace_dir,
                               f'map_obstacles_{self.ns}.csv'), 'w', newline='') as f:
            wri = csv.writer(f)
            wri.writerow(['x', 'y', 'grid_r', 'grid_c'])
            for r, c in occ:
                wx = ox + (c + 0.5) * res
                wy = oy + (r + 0.5) * res
                wri.writerow([f'{wx:.4f}', f'{wy:.4f}', int(r), int(c)])
        infl = np.argwhere(self.inflated)
        with open(os.path.join(self.algo_trace_dir,
                               f'map_inflated_{self.ns}.csv'), 'w', newline='') as f:
            wri = csv.writer(f)
            wri.writerow(['x', 'y', 'grid_r', 'grid_c'])
            for r, c in infl:
                wx = ox + (c + 0.5) * res
                wy = oy + (r + 0.5) * res
                wri.writerow([f'{wx:.4f}', f'{wy:.4f}', int(r), int(c)])
        self._map_dumped = True
        self.get_logger().info(
            f'[ALGO-TRACE] Map dump: {len(occ)} obstacle, {len(infl)} inflated cell')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
