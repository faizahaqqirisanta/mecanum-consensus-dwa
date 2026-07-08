#!/usr/bin/env python3
"""
UDP Sender Node — haqqi_ta
Dijalankan DI ROBOT — kirim data ke PC Master via UDP.

Basis: robot1_udp_sender.py (modifikasi untuk 3 robot + consensus)

Data yang dikirim ke PC Master (10 Hz):
  - amcl_pose  : x, y, yaw, covariance sigma
  - remaining_length : sisa jarak ke goal
  - path_length      : total panjang path
  - goal_reached     : True hanya saat FINAL waypoint tercapai
  - waypoint_index   : indeks WP terakhir yang tercapai (-1 = belum)

Port UDP per robot (PC Master listen di semua port ini):
  robot1 → PC Master port 9001
  robot2 → PC Master port 9002
  robot3 → PC Master port 9003

Data yang DITERIMA dari PC Master via udp_receiver_node:
  - vmax_consensus  : v_max dari consensus
  - priority_stop   : stop command dari priority manager
  (diterima oleh udp_receiver_node.py yang terpisah)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool, Int32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import socket
import json
import math
import time
import os


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


# Port mapping per robot → PC Master (consensus_node)
ROBOT_PORT_MAP = {
    'robot1': 9001,
    'robot2': 9002,
    'robot3': 9003,
}

# Port mapping per robot → PC Master (priority_manager_node)
ROBOT_PORT_PRIORITY_MAP = {
    'robot1': 9031,
    'robot2': 9032,
    'robot3': 9033,
}


class UDPSenderNode(Node):
    def __init__(self):
        super().__init__('udp_sender_node')

        # ── Parameter ─────────────────────────────────────────────────────
        self.declare_parameter('robot_ns', 'robot1')
        # [MOD-IPENV] IP PC bisa diganti dari satu tempat lewat env var PC_MASTER_IP
        #   contoh: export PC_MASTER_IP=192.168.1.50  (sebelum ros2 launch / di tutorial_run.sh)
        #   kalau env tidak diset, pakai default lama 192.168.0.34.
        self.declare_parameter('pc_master_ip',
                               os.environ.get('PC_MASTER_IP', '192.168.0.34'))
        self.declare_parameter('send_rate', 10.0)   # Hz
        self.declare_parameter('path_sample_step_m', 0.05)   # 0.10→0.05: resolusi lebih rapat
        self.declare_parameter('path_max_points_udp', 300)  # 80→300: cover path ~15m penuh
        self.declare_parameter('path_udp_period_s', 1.0)
        # [MOD-LOCALPLAN] kirim local_plan (DWA) ke PC agar logger bisa merekamnya
        self.declare_parameter('local_plan_max_points_udp', 80)
        # Topic AMCL — '/amcl_pose' jika AMCL tanpa namespace,
        # '/{robot_ns}/amcl_pose' jika AMCL dijalankan dalam namespace.
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')

        self.ns            = self.get_parameter('robot_ns').value
        self.pc_master_ip  = self.get_parameter('pc_master_ip').value
        self.send_rate     = self.get_parameter('send_rate').value
        self.path_sample_step_m = float(self.get_parameter('path_sample_step_m').value)
        self.path_max_points_udp = int(self.get_parameter('path_max_points_udp').value)
        self.path_udp_period_s = max(0.1, float(
            self.get_parameter('path_udp_period_s').value))
        self.amcl_topic    = self.get_parameter('amcl_pose_topic').value
        self.local_plan_max_points_udp = int(
            self.get_parameter('local_plan_max_points_udp').value)

        # Tentukan port berdasarkan robot_ns
        self.pc_master_port    = ROBOT_PORT_MAP.get(self.ns, 9001)
        self.pc_priority_port  = ROBOT_PORT_PRIORITY_MAP.get(self.ns, 9031)

        # ── UDP Socket ────────────────────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── Data cache ────────────────────────────────────────────────────
        self.pose_data                = None   # {x, y, yaw, sigma_x, sigma_y}
        self.remaining_length         = None
        self.path_length              = None
        self.goal_reached             = False
        self.position_reached         = False  # [FIX-POSREACH] waktu tiba posisi (DWA)
        self.waypoint_index           = -1
        self.fault_active             = False
        self.fault_log_events         = []
        self.mission_remaining_length = None
        self.mission_total_length     = None
        self.cmd_vel_data             = None   # {vx, vy, w}
        self.path_points              = []     # [[x, y], ...] downsampled /{ns}/plan
        self._path_points_dirty       = False
        self.local_plan_points        = []     # [MOD-LOCALPLAN] [[x,y],...] dari /{ns}/local_plan (DWA)
        self._last_path_udp_wall      = 0.0
        self.dwa_mode                 = 'NO_DWA'  # [FIX-5] default NO_DWA bukan IDLE — lebih jelas jika DWA node mati
        self.dwa_vmax_eff             = 0.0
        self.omega_raw                = 0.0
        self.omega_after_clamp        = 0.0
        self.omega_global_limit       = 0.0
        self.localization_hold_active = False
        self.dynamic_obstacle_debug   = ''
        self._dynamic_obstacle_debug_burst = 0
        # Field tambahan untuk analisis yang sebelumnya tidak dikirim ke PC
        self.tracking_mode            = 'IDLE'  # fine-grained: HOLO/HOLO_BLK/DWA/BKTRK/APPR
        self.dwa_active               = False   # [M3] True saat mode DWA/HOLO_BLK aktif
        self.heading_error            = 0.0     # rad — heading robot vs path tangent
        self.dwa_speed_mag            = 0.0     # m/s — kecepatan aktual dari DWA
        self.vmax_priority_robot      = 0.0     # m/s — vmax_priority yg diterima robot
        self.priority_stop_robot      = False   # bool — priority_stop yg diterima robot
        self.lane_offset_robot        = 0.0     # m — crossing_lane_offset aktif
        # [FIX-6] DWA heartbeat watchdog
        self.dwa_alive_ok             = False    # True jika heartbeat diterima < 2s lalu
        self._dwa_alive_last_t        = None     # waktu terakhir heartbeat diterima

        # ── QoS ───────────────────────────────────────────────────────────
        # AMCL Nav2 umumnya TRANSIENT_LOCAL sehingga subscriber yang start
        # belakangan perlu transient QoS agar langsung mendapat pose terakhir.
        # Subscriber VOLATILE tetap dipasang sebagai fallback publisher biasa.
        amcl_qos_volatile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        amcl_qos_transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.amcl_callback,
            amcl_qos_volatile
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.amcl_callback,
            amcl_qos_transient
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/remaining_length',
            self.remaining_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/path_length',
            self.path_length_callback,
            10
        )

        self.create_subscription(
            Bool,
            f'/{self.ns}/goal_reached',
            self.goal_reached_callback,
            10
        )

        self.create_subscription(
            Bool,
            f'/{self.ns}/position_reached',
            lambda msg: setattr(self, 'position_reached', bool(msg.data)),  # [FIX-POSREACH]
            10
        )

        self.create_subscription(
            Int32,
            f'/{self.ns}/waypoint_index',
            self.waypoint_index_callback,
            10
        )

        self.create_subscription(
            Bool,
            f'/{self.ns}/fault_active',
            self.fault_active_callback,
            10
        )

        self.create_subscription(
            String,
            f'/{self.ns}/fault_log',
            self.fault_log_callback,
            10
        )

        self.create_subscription(
            String,
            f'/{self.ns}/dwa_mode',
            self.dwa_mode_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/dwa_vmax_eff',
            self.dwa_vmax_eff_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/omega_raw',
            self.omega_raw_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/omega_after_clamp',
            self.omega_after_clamp_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/omega_global_limit',
            self.omega_global_limit_callback,
            10
        )

        self.create_subscription(
            Bool,
            f'/{self.ns}/localization_hold_active',
            self.localization_hold_callback,
            10
        )

        self.create_subscription(
            String,
            f'/{self.ns}/dynamic_obstacle_debug',
            self.dynamic_obstacle_debug_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/mission_remaining_length',
            self.mission_remaining_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/mission_total_length',
            self.mission_total_callback,
            10
        )

        self.create_subscription(
            Twist,
            f'/{self.ns}/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Path,
            f'/{self.ns}/plan',
            self.path_callback,
            10
        )

        # [MOD-LOCALPLAN] local path dari DWA → diteruskan ke PC via UDP
        self.create_subscription(
            Path,
            f'/{self.ns}/local_plan',
            self.local_plan_callback,
            10
        )

        self.create_subscription(
            String,
            f'/{self.ns}/tracking_mode',
            self._tracking_mode_cb,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/heading_error',
            lambda msg: setattr(self, 'heading_error', float(msg.data)),
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/dwa_speed_mag',
            lambda msg: setattr(self, 'dwa_speed_mag', float(msg.data)),
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/vmax_priority',
            lambda msg: setattr(self, 'vmax_priority_robot', float(msg.data)),
            10
        )

        self.create_subscription(
            Bool,
            f'/{self.ns}/priority_stop',
            lambda msg: setattr(self, 'priority_stop_robot', bool(msg.data)),
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.ns}/crossing_lane_offset',
            lambda msg: setattr(self, 'lane_offset_robot', float(msg.data)),
            10
        )

        # [FIX-6] Subscribe heartbeat dari DWA node
        self.create_subscription(
            Bool,
            f'/{self.ns}/dwa_alive',
            self._dwa_alive_cb,
            10)

        # ── Timer ─────────────────────────────────────────────────────────
        period = 1.0 / self.send_rate
        self.create_timer(period, self.send_udp)
        # [FIX-6] Watchdog: cek apakah DWA masih hidup tiap 2s
        self.create_timer(2.0, self._check_dwa_alive)

        self.get_logger().info(
            f'UDP Sender ready | robot={self.ns} | '
            f'target={self.pc_master_ip}:{self.pc_master_port} | '
            f'{self.send_rate}Hz'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def amcl_callback(self, msg):
        p   = msg.pose.pose
        cov = list(msg.pose.covariance)
        self.pose_data = {
            'x'       : p.position.x,
            'y'       : p.position.y,
            'yaw'     : quat_to_yaw(p.orientation),
            'sigma_x' : math.sqrt(max(0.0, cov[0])),
            'sigma_y' : math.sqrt(max(0.0, cov[7])),
        }

    def remaining_callback(self, msg):
        self.remaining_length = float(msg.data)

    def path_length_callback(self, msg):
        self.path_length = float(msg.data)

    def goal_reached_callback(self, msg):
        self.goal_reached = bool(msg.data)

    def waypoint_index_callback(self, msg):
        self.waypoint_index = int(msg.data)

    def fault_active_callback(self, msg):
        self.fault_active = bool(msg.data)

    def fault_log_callback(self, msg):
        event = str(msg.data).strip()
        if event:
            self.fault_log_events.append(event)

    def mission_remaining_callback(self, msg):
        self.mission_remaining_length = float(msg.data)

    def mission_total_callback(self, msg):
        self.mission_total_length = float(msg.data)

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = {
            'vx': msg.linear.x,
            'vy': msg.linear.y,
            'w' : msg.angular.z,
        }

    def _tracking_mode_cb(self, msg):
        """[M3] Update tracking_mode dan derivasi dwa_active."""
        self.tracking_mode = str(msg.data)
        self.dwa_active = self.tracking_mode in ('HOLO_BLK', 'DWA', 'BLOCKED',
                                                  'DYN_AVOID', 'WAIT_PEER_DWA')

    def dwa_mode_callback(self, msg):
        self.dwa_mode = str(msg.data)

    # [FIX-6] DWA heartbeat watchdog ─────────────────────────────────────
    def _dwa_alive_cb(self, msg):
        """Terima heartbeat 1Hz dari DWA node."""
        import time as _time
        self._dwa_alive_last_t = _time.time()
        if not self.dwa_alive_ok:
            self.dwa_alive_ok = True
            self.get_logger().info(
                f'[UDP] DWA node {self.ns} ALIVE — heartbeat diterima')

    def _check_dwa_alive(self):
        """Watchdog 2s: jika tidak ada heartbeat, set status NO_DWA."""
        import time as _time
        if self._dwa_alive_last_t is None:
            if self.dwa_alive_ok:
                self.dwa_alive_ok = False
                self.get_logger().error(
                    f'[UDP] DWA node {self.ns} TIDAK DITEMUKAN — '
                    f'pastikan modified_dwa_node sudah di-launch!')
            self.dwa_mode = 'NO_DWA'
            return
        age = _time.time() - self._dwa_alive_last_t
        if age > 2.0:
            if self.dwa_alive_ok:
                self.dwa_alive_ok = False
                self.get_logger().error(
                    f'[UDP] DWA node {self.ns} MATI '
                    f'(heartbeat terakhir {age:.1f}s lalu) — cek crash!')
            self.dwa_mode = 'NO_DWA'
        else:
            self.dwa_alive_ok = True

    def dwa_vmax_eff_callback(self, msg):
        self.dwa_vmax_eff = float(msg.data)

    def omega_raw_callback(self, msg):
        self.omega_raw = float(msg.data)

    def omega_after_clamp_callback(self, msg):
        self.omega_after_clamp = float(msg.data)

    def omega_global_limit_callback(self, msg):
        self.omega_global_limit = float(msg.data)

    def localization_hold_callback(self, msg):
        self.localization_hold_active = bool(msg.data)

    def dynamic_obstacle_debug_callback(self, msg):
        self.dynamic_obstacle_debug = str(msg.data)
        self._dynamic_obstacle_debug_burst = 1

    def path_callback(self, msg):
        points = []
        last = None
        for pose in msg.poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if last is None:
                points.append([x, y])
                last = (x, y)
                continue
            if math.hypot(x - last[0], y - last[1]) >= self.path_sample_step_m:
                points.append([x, y])
                last = (x, y)
            if len(points) >= self.path_max_points_udp:
                break
        if msg.poses:
            last_pose = msg.poses[-1].pose.position
            last_pt = [float(last_pose.x), float(last_pose.y)]
            if not points or math.hypot(points[-1][0] - last_pt[0], points[-1][1] - last_pt[1]) > 1e-3:
                if len(points) >= self.path_max_points_udp:
                    points[-1] = last_pt
                else:
                    points.append(last_pt)
        if points != self.path_points:
            self.path_points = points
            self._path_points_dirty = True

    def local_plan_callback(self, msg):
        # [MOD-LOCALPLAN] local plan DWA biasanya pendek (trajektori prediksi),
        # cukup dikirim apa adanya dengan cap jumlah titik.
        points = []
        for pose in msg.poses:
            points.append([float(pose.pose.position.x),
                           float(pose.pose.position.y)])
            if len(points) >= self.local_plan_max_points_udp:
                break
        self.local_plan_points = points

    # ═══════════════════════════════════════════════════════════════════════
    # UDP SEND — 10 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def send_udp(self):
        if self.pose_data is None:
            return

        wall_now = time.time()
        send_path = (
            bool(self.path_points)
            and (self._path_points_dirty
                 or wall_now - self._last_path_udp_wall >= self.path_udp_period_s))
        path_points_udp = self.path_points if send_path else []
        dyn_debug = (
            self.dynamic_obstacle_debug
            if self._dynamic_obstacle_debug_burst > 0 else '')

        # [FIX-POSEDELIVERY] Paket INTI kecil (pose + skalar liveness) dikirim
        # tiap siklus TERPISAH dari paket penuh. Paket penuh bisa membengkak
        # (path_points s/d 300 titik, fault_log_events, dynamic_obstacle_debug)
        # lalu terpotong di socket recvfrom PC -> json.loads gagal -> SELURUH
        # paket (termasuk pose) dibuang -> pose/telemetri stale di CLI & monitor.
        # Paket inti dijamin jauh di bawah batas datagram sehingga pose + metrik
        # liveness yang dipakai CLI (pose, path_length) dan sync_monitor
        # (mission_*, dwa_*, goal/pstop/fault) SELALU sampai ke PC.
        core = {
            'robot_ns'                : self.ns,
            'pose'                    : self.pose_data,
            'remaining_length'        : self.remaining_length,
            'path_length'             : self.path_length,
            'goal_reached'            : self.goal_reached,
            'position_reached'        : self.position_reached,
            'waypoint_index'          : self.waypoint_index,
            'mission_remaining_length': self.mission_remaining_length,
            'mission_total_length'    : self.mission_total_length,
            'cmd_vel'                 : self.cmd_vel_data,
            'fault_active'            : self.fault_active,
            'dwa_mode'                : self.dwa_mode,
            'dwa_vmax_eff'            : self.dwa_vmax_eff,
            'dwa_speed_mag'           : self.dwa_speed_mag,
            'vmax_priority_robot'     : self.vmax_priority_robot,
            'priority_stop_robot'     : self.priority_stop_robot,
            'dwa_active'              : self.dwa_active,
        }
        try:
            core_data = json.dumps(core).encode('utf-8')
            self.sock.sendto(core_data, (self.pc_master_ip, self.pc_master_port))
            self.sock.sendto(core_data, (self.pc_master_ip, self.pc_priority_port))
        except Exception as e:
            self.get_logger().warn(f'UDP core send failed: {e}')

        packet = {
            'robot_ns'                : self.ns,
            'pose'                    : self.pose_data,
            'remaining_length'        : self.remaining_length,
            'path_length'             : self.path_length,
            'goal_reached'            : self.goal_reached,
            'position_reached'        : self.position_reached,  # [FIX-POSREACH]
            'waypoint_index'          : self.waypoint_index,
            'fault_active'            : self.fault_active,
            'fault_log_events'        : list(self.fault_log_events),
            'mission_remaining_length': self.mission_remaining_length,
            'mission_total_length'    : self.mission_total_length,
            'cmd_vel'                 : self.cmd_vel_data,
            'path_points'             : path_points_udp,
            'local_plan_points'       : self.local_plan_points,   # [MOD-LOCALPLAN]
            'dwa_mode'                : self.dwa_mode,
            'dwa_vmax_eff'            : self.dwa_vmax_eff,
            'omega_raw'               : self.omega_raw,
            'omega_after_clamp'       : self.omega_after_clamp,
            'omega_global_limit'      : self.omega_global_limit,
            'localization_hold_active': self.localization_hold_active,
            'dynamic_obstacle_debug'  : dyn_debug,
            'tracking_mode'           : self.tracking_mode,
            'heading_error'           : self.heading_error,
            'dwa_speed_mag'           : self.dwa_speed_mag,
            'vmax_priority_robot'     : self.vmax_priority_robot,
            'priority_stop_robot'     : self.priority_stop_robot,
            'lane_offset_robot'       : self.lane_offset_robot,
            'dwa_alive_ok'            : self.dwa_alive_ok,    # [FIX-6]
            'dwa_active'              : self.dwa_active,       # [M3]
        }

        data = json.dumps(packet).encode('utf-8')
        try:
            # Kirim ke consensus_node
            self.sock.sendto(data, (self.pc_master_ip, self.pc_master_port))
            # Kirim ke priority_manager
            self.sock.sendto(data, (self.pc_master_ip, self.pc_priority_port))
            self.fault_log_events.clear()
            if send_path:
                self._path_points_dirty = False
                self._last_path_udp_wall = wall_now
            if self._dynamic_obstacle_debug_burst > 0:
                self._dynamic_obstacle_debug_burst -= 1
        except Exception as e:
            self.get_logger().warn(f'UDP send failed: {e}')


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = UDPSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UDP Sender stopped')
    except ExternalShutdownException:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
