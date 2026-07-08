#!/usr/bin/env python3
"""
UDP Bridge PC — haqqi_ta
Dijalankan di PC MASTER.

Menjembatani ROS topic di PC Master (domain 44) ke robot
yang masing-masing punya domain berbeda (40, 41, 42).

Data yang dikirim ke tiap robot via UDP:
  - /robot*/goal_pose   → PoseStamped
  - /robot*/initialpose → PoseWithCovarianceStamped
  - /robot*/waypoints   → PoseArray
  - /experiment_state   → String (heartbeat READY/RUNNING/STOP, broadcast ke semua robot)
  - /experiment_scenario → String (scenario aktif dari CLI, broadcast ke semua robot)

Port PC Master → Robot (kontrol):
  robot1 ← 9021
  robot2 ← 9022
  robot3 ← 9023

Packet format:
  8 bytes header (4 type + 4 length) + payload
  type: 'GOAL', 'IPOS', 'STAT', 'WAYP', 'SCEN'
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import String

import socket
import json
import struct
import math
import os


ROBOT_NAMESPACES = ['robot1', 'robot2', 'robot3']

# Port robot untuk terima data dari PC Master
ROBOT_CTRL_PORT = {
    'robot1': 9021,
    'robot2': 9022,
    'robot3': 9023,
}

# IP robot — [MOD-IPENV] bisa diubah dari satu tempat lewat env var (opsional)
ROBOT_IP_DEFAULT = {
    'robot1': os.environ.get('ROBOT1_IP', '192.168.0.91'),
    'robot2': os.environ.get('ROBOT2_IP', '192.168.0.88'),
    'robot3': os.environ.get('ROBOT3_IP', '192.168.0.82'),
}


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_packet(ptype: str, payload: bytes) -> bytes:
    """
    Buat paket dengan header 8 byte:
      4 byte: type string (misal b'MAP_')
      4 byte: panjang payload (unsigned int)
    """
    assert len(ptype) <= 4
    header_type = ptype.encode().ljust(4, b'_')
    header_len  = struct.pack('>I', len(payload))
    return header_type + header_len + payload


class UDPBridgePCNode(Node):
    def __init__(self):
        super().__init__('udp_bridge_pc')

        # ── Parameter ─────────────────────────────────────────────────────
        self.declare_parameter('robot1_ip', ROBOT_IP_DEFAULT['robot1'])
        self.declare_parameter('robot2_ip', ROBOT_IP_DEFAULT['robot2'])
        self.declare_parameter('robot3_ip', ROBOT_IP_DEFAULT['robot3'])
        self.declare_parameter('active_robots', ['robot1', 'robot2', 'robot3'])
        self.declare_parameter('state_udp_redundancy', 3)

        self.robot_ip = {
            'robot1': self.get_parameter('robot1_ip').value,
            'robot2': self.get_parameter('robot2_ip').value,
            'robot3': self.get_parameter('robot3_ip').value,
        }
        self.active_robots = list(
            self.get_parameter('active_robots').value)
        self.state_udp_redundancy = max(
            1, int(self.get_parameter('state_udp_redundancy').value))

        # ── UDP Socket ────────────────────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 131072)

        # ── Subscribers ───────────────────────────────────────────────────

        # Experiment state heartbeat — forward ke semua robot @ 10 Hz
        self.create_subscription(
            String,
            '/experiment_state',
            self._experiment_state_cb,
            10)

        self.create_subscription(
            String,
            '/experiment_scenario',
            self._experiment_scenario_cb,
            10)

        # Goal dan initialpose per robot
        for ns in ROBOT_NAMESPACES:
            self.create_subscription(
                PoseStamped,
                f'/{ns}/goal_pose',
                lambda msg, n=ns: self.goal_callback(msg, n),
                10)

            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{ns}/initialpose',
                lambda msg, n=ns: self.initialpose_callback(msg, n),
                10)

            self.create_subscription(
                PoseArray,
                f'/{ns}/waypoints',
                lambda msg, n=ns: self.waypoints_callback(msg, n),
                10)

        self.get_logger().info(
            f'UDP Bridge PC ready | robots={self.active_robots} | '
            f'ports={[ROBOT_CTRL_PORT[ns] for ns in self.active_robots]}')

    # ═══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════════════

    def _experiment_state_cb(self, msg: String):
        """Forward /experiment_state heartbeat ke semua robot via UDP."""
        payload = json.dumps({
            'type'              : 'experiment_state',
            'experiment_state'  : msg.data,
        }).encode('utf-8')
        packet = make_packet('STAT', payload)

        for ns in self.active_robots:
            try:
                for _ in range(self.state_udp_redundancy):
                    self.sock.sendto(
                        packet,
                        (self.robot_ip[ns], ROBOT_CTRL_PORT[ns]))
            except Exception as e:
                self.get_logger().warn(
                    f'Gagal kirim experiment_state ke {ns}: {e}')

    def _experiment_scenario_cb(self, msg: String):
        """Forward scenario aktif agar node robot tidak perlu relaunch per skenario."""
        scenario = str(msg.data).strip()
        if not scenario:
            return
        payload = json.dumps({
            'type'     : 'experiment_scenario',
            'scenario' : scenario,
        }).encode('utf-8')
        packet = make_packet('SCEN', payload)

        for ns in self.active_robots:
            try:
                for _ in range(self.state_udp_redundancy):
                    self.sock.sendto(
                        packet,
                        (self.robot_ip[ns], ROBOT_CTRL_PORT[ns]))
            except Exception as e:
                self.get_logger().warn(
                    f'Gagal kirim experiment_scenario ke {ns}: {e}')

    def goal_callback(self, msg: PoseStamped, ns: str):
        """Kirim goal_pose ke robot yang bersangkutan."""
        if ns not in self.active_robots:
            return

        yaw = quat_to_yaw(msg.pose.orientation)
        payload = json.dumps({
            'type' : 'goal_pose',
            'ns'   : ns,
            'x'    : msg.pose.position.x,
            'y'    : msg.pose.position.y,
            'yaw'  : yaw,
        }).encode('utf-8')
        packet = make_packet('GOAL', payload)

        try:
            self.sock.sendto(
                packet,
                (self.robot_ip[ns], ROBOT_CTRL_PORT[ns]))
            self.get_logger().info(
                f'Goal → {ns}: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f})')
        except Exception as e:
            self.get_logger().warn(f'Gagal kirim goal ke {ns}: {e}')

    def initialpose_callback(self, msg: PoseWithCovarianceStamped, ns: str):
        """Kirim initialpose ke robot yang bersangkutan."""
        if ns not in self.active_robots:
            return

        yaw = quat_to_yaw(msg.pose.pose.orientation)
        payload = json.dumps({
            'type' : 'initialpose',
            'ns'   : ns,
            'x'    : msg.pose.pose.position.x,
            'y'    : msg.pose.pose.position.y,
            'yaw'  : yaw,
            'cov'  : self._tight_initial_covariance(msg.pose.covariance),
        }).encode('utf-8')
        packet = make_packet('IPOS', payload)

        try:
            self.sock.sendto(
                packet,
                (self.robot_ip[ns], ROBOT_CTRL_PORT[ns]))
            self.get_logger().info(
                f'InitialPose → {ns}: ({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f})')
        except Exception as e:
            self.get_logger().warn(f'Gagal kirim initialpose ke {ns}: {e}')

    @staticmethod
    def _tight_initial_covariance(cov):
        cov = list(cov)
        if len(cov) != 36:
            cov = [0.0] * 36
        cov[0] = min(float(cov[0] or 0.04), 0.04)
        cov[7] = min(float(cov[7] or 0.04), 0.04)
        cov[35] = min(float(cov[35] or 0.03), 0.03)
        return cov

    def waypoints_callback(self, msg: PoseArray, ns: str):
        """Kirim daftar waypoints ke robot (PoseArray → paket WAYP)."""
        if ns not in self.active_robots:
            return

        poses_data = []
        for pose in msg.poses:
            yaw = quat_to_yaw(pose.orientation)
            poses_data.append({'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw})

        payload = json.dumps({
            'type'  : 'waypoints',
            'ns'    : ns,
            'poses' : poses_data,
        }).encode('utf-8')
        packet = make_packet('WAYP', payload)

        try:
            self.sock.sendto(
                packet,
                (self.robot_ip[ns], ROBOT_CTRL_PORT[ns]))
            self.get_logger().info(
                f'Waypoints → {ns}: {len(poses_data)} titik')
        except Exception as e:
            self.get_logger().warn(f'Gagal kirim waypoints ke {ns}: {e}')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = UDPBridgePCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UDP Bridge PC stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
