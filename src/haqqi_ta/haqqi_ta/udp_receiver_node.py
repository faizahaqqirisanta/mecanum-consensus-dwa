#!/usr/bin/env python3
"""
UDP Receiver Node — haqqi_ta (versi multi-domain)
Dijalankan DI ROBOT.

Menerima dua jenis data via UDP:

1. Dari PC Master (udp_bridge_pc) di port 9021/9022/9023:
   - GOAL  : PoseStamped → publish /{ns}/goal_pose
   - IPOS  : PoseWithCovarianceStamped → publish /{ns}/initialpose
   - STAT  : String → publish /experiment_state
   - SCEN  : String → publish /experiment_scenario

2. Dari consensus_node dan priority_manager (PC Master)
   di port 9011/9012/9013:
   - vmax_consensus : Float32 → publish /{ns}/vmax_consensus
   - priority_stop  : Bool    → publish /{ns}/priority_stop
   - vmax_priority  : Float32 → publish /{ns}/vmax_priority
   - conflict_zone_detail : String → publish /conflict_zone_detail

Port mapping:
  Robot terima dari PC Master (bridge): 9021/9022/9023
  Robot terima dari consensus/priority: 9011/9012/9013
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import Float32, Bool, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import socket
import json
import struct
import math
import threading


# Port robot untuk terima dari PC Master bridge
BRIDGE_PORT_MAP = {
    'robot1': 9021,
    'robot2': 9022,
    'robot3': 9023,
}

# Port robot untuk terima dari consensus/priority
CTRL_PORT_MAP = {
    'robot1': 9011,
    'robot2': 9012,
    'robot3': 9013,
}


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class UDPReceiverNode(Node):
    def __init__(self):
        super().__init__('udp_receiver_node')

        # ── Parameter ─────────────────────────────────────────────────────
        self.declare_parameter('robot_ns', 'robot1')
        self.declare_parameter('listen_ip', '0.0.0.0')
        # v_nominal dideklarasikan untuk backward-compat YAML tapi tidak dipakai saat runtime
        self.declare_parameter('v_nominal', 0.15)

        self.ns   = self.get_parameter('robot_ns').value
        listen_ip = self.get_parameter('listen_ip').value
        self.last_experiment_state = None

        bridge_port = BRIDGE_PORT_MAP.get(self.ns, 9021)
        ctrl_port   = CTRL_PORT_MAP.get(self.ns, 9011)

        # ── Publishers ────────────────────────────────────────────────────

        # Dari bridge PC Master
        self.goal_pub = self.create_publisher(
            PoseStamped,
            f'/{self.ns}/goal_pose', 10)

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{self.ns}/initialpose', 10)
        # Kompatibilitas AMCL: launch per-robot baru memakai /robotX/initialpose,
        # sementara beberapa launch lama/default AMCL masih mendengar /initialpose.
        self.initialpose_global_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose', 10)

        self.experiment_state_pub = self.create_publisher(
            String, '/experiment_state', 10)
        self.experiment_scenario_pub = self.create_publisher(
            String, '/experiment_scenario', 10)

        self.waypoints_pub = self.create_publisher(
            PoseArray, f'/{self.ns}/waypoints', 10)

        # Dari consensus_node dan priority_manager
        self.vmax_consensus_pub = self.create_publisher(
            Float32, f'/{self.ns}/vmax_consensus', 10)

        self.pstop_pub = self.create_publisher(
            Bool, f'/{self.ns}/priority_stop', 10)

        self.vmax_priority_pub = self.create_publisher(
            Float32, f'/{self.ns}/vmax_priority', 10)

        self.peer_pose_pub = self.create_publisher(
            String, f'/{self.ns}/peer_robot_poses', 10)

        self.lane_offset_pub = self.create_publisher(
            Float32, f'/{self.ns}/crossing_lane_offset', 10)
        self.conflict_zone_detail_pub = self.create_publisher(
            String, '/conflict_zone_detail', 10)

        # ── UDP Sockets ───────────────────────────────────────────────────
        self.running = True

        # Socket 1: terima dari bridge PC (map, goal, initialpose, signal)
        self.bridge_sock = self._make_socket(listen_ip, bridge_port)

        # Socket 2: terima dari consensus/priority (vmax, stop)
        self.ctrl_sock = self._make_socket(listen_ip, ctrl_port)

        # ── Listener Threads ──────────────────────────────────────────────
        threading.Thread(
            target=self._bridge_listener,
            daemon=True).start()

        threading.Thread(
            target=self._ctrl_listener,
            daemon=True).start()

        self.get_logger().info(
            f'UDP Receiver ready | robot={self.ns} | '
            f'bridge_port={bridge_port} | ctrl_port={ctrl_port}')

    def _make_socket(self, ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)
        sock.bind((ip, port))
        sock.settimeout(1.0)
        return sock

    # ═══════════════════════════════════════════════════════════════════════
    # BRIDGE LISTENER — terima dari udp_bridge_pc
    # ═══════════════════════════════════════════════════════════════════════

    def _bridge_listener(self):
        """
        Terima paket dari udp_bridge_pc.
        Format: 8 byte header (4 type + 4 length) + payload
        """
        while self.running and rclpy.ok():
            try:
                data, _ = self.bridge_sock.recvfrom(131072)  # 128KB buffer

                if len(data) < 8:
                    continue

                ptype = data[:4].decode('utf-8', errors='ignore').rstrip('_')
                plen  = struct.unpack('>I', data[4:8])[0]
                if len(data) < 8 + plen:
                    self.get_logger().warn(
                        f'Truncated UDP packet: type={ptype} '
                        f'expected={8 + plen} got={len(data)} — dibuang')
                    continue
                payload = data[8:8 + plen]

                if ptype == 'GOAL':
                    self._handle_goal(payload)
                elif ptype == 'IPOS':
                    self._handle_initialpose(payload)
                elif ptype == 'STAT':
                    self._handle_experiment_state(payload)
                elif ptype == 'SCEN':
                    self._handle_experiment_scenario(payload)
                elif ptype == 'WAYP':
                    self._handle_waypoints(payload)


            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f'Bridge recv error: {e}')

    def _handle_goal(self, payload: bytes):
        """Publish goal_pose."""
        try:
            d   = json.loads(payload.decode('utf-8'))
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.pose.position.x = d['x']
            msg.pose.position.y = d['y']
            z, w = yaw_to_quat(d.get('yaw', 0.0))
            msg.pose.orientation.z = z
            msg.pose.orientation.w = w
            self.goal_pub.publish(msg)
            self.get_logger().info(
                f'Goal diterima: ({d["x"]:.2f}, {d["y"]:.2f})')
        except Exception as e:
            self.get_logger().warn(f'Gagal proses goal: {e}')

    def _handle_initialpose(self, payload: bytes):
        """Publish initialpose."""
        try:
            d   = json.loads(payload.decode('utf-8'))
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id      = 'map'
            msg.header.stamp         = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = d['x']
            msg.pose.pose.position.y = d['y']
            z, w = yaw_to_quat(d.get('yaw', 0.0))
            msg.pose.pose.orientation.z = z
            msg.pose.pose.orientation.w = w
            msg.pose.covariance = d.get('cov', [0.04,0,0,0,0,0,
                                                0,0.04,0,0,0,0,
                                                0,0,0,0,0,0,
                                                0,0,0,0,0,0,
                                                0,0,0,0,0,0,
                                                0,0,0,0,0,0.03])
            self.initialpose_pub.publish(msg)
            self.initialpose_global_pub.publish(msg)
            self.get_logger().info(
                f'InitialPose diterima: ({d["x"]:.2f}, {d["y"]:.2f})')
        except Exception as e:
            self.get_logger().warn(f'Gagal proses initialpose: {e}')

    def _handle_experiment_state(self, payload: bytes):
        """Republish /experiment_state dari PC Master ke robot local topic."""
        try:
            d   = json.loads(payload.decode('utf-8'))
            msg = String()
            msg.data = str(d.get('experiment_state', 'STOP'))
            self.experiment_state_pub.publish(msg)
            if msg.data != self.last_experiment_state:
                self.get_logger().info(
                    f'Experiment state diterima: {self.last_experiment_state} → {msg.data}')
                self.last_experiment_state = msg.data
        except Exception as e:
            self.get_logger().warn(f'Gagal proses experiment_state: {e}')

    def _handle_experiment_scenario(self, payload: bytes):
        """Republish scenario aktif dari PC Master ke robot local topic."""
        try:
            d = json.loads(payload.decode('utf-8'))
            scenario = str(d.get('scenario', '')).strip()
            if not scenario:
                return
            msg = String()
            msg.data = scenario
            self.experiment_scenario_pub.publish(msg)
            self.get_logger().info(f'Scenario diterima: {scenario}')
        except Exception as e:
            self.get_logger().warn(f'Gagal proses experiment_scenario: {e}')

    def _handle_waypoints(self, payload: bytes):
        """Publish /{ns}/waypoints (PoseArray) dari PC Master."""
        try:
            d   = json.loads(payload.decode('utf-8'))
            msg = PoseArray()
            msg.header.frame_id = 'map'
            msg.header.stamp    = self.get_clock().now().to_msg()
            for wp in d.get('poses', []):
                pose = Pose()
                pose.position.x = float(wp['x'])
                pose.position.y = float(wp['y'])
                z, w = yaw_to_quat(float(wp.get('yaw', 0.0)))
                pose.orientation.z = z
                pose.orientation.w = w
                msg.poses.append(pose)
            self.waypoints_pub.publish(msg)
            self.get_logger().info(
                f'Waypoints diterima: {len(msg.poses)} titik')
        except Exception as e:
            self.get_logger().warn(f'Gagal proses waypoints: {e}')

    # ═══════════════════════════════════════════════════════════════════════
    # CTRL LISTENER — terima dari consensus_node dan priority_manager
    # ═══════════════════════════════════════════════════════════════════════

    def _ctrl_listener(self):
        """
        Terima vmax_consensus, priority_stop, vmax_priority
        dari consensus_node dan priority_manager di PC Master.
        Format: JSON biasa (tidak ada header)
        """
        while self.running and rclpy.ok():
            try:
                data, _ = self.ctrl_sock.recvfrom(2048)
                packet  = json.loads(data.decode('utf-8'))

                if 'vmax_consensus' in packet:
                    msg      = Float32()
                    msg.data = float(packet['vmax_consensus'])
                    self.vmax_consensus_pub.publish(msg)

                # Fallback: experiment_state via ctrl channel jika bridge (port 9022) terblokir.
                # Dikirim oleh consensus_node bersama vmax agar state tetap sampai.
                if 'experiment_state' in packet:
                    state = str(packet['experiment_state'])
                    msg      = String()
                    msg.data = state
                    self.experiment_state_pub.publish(msg)
                    if state != self.last_experiment_state:
                        self.get_logger().info(
                            f'[ctrl-fallback] Experiment state: '
                            f'{self.last_experiment_state} → {state}')
                        self.last_experiment_state = state

                if 'priority_stop' in packet:
                    msg      = Bool()
                    msg.data = bool(packet['priority_stop'])
                    self.pstop_pub.publish(msg)

                if 'vmax_priority' in packet:
                    msg      = Float32()
                    msg.data = float(packet['vmax_priority'])
                    self.vmax_priority_pub.publish(msg)

                if 'peer_poses' in packet:
                    msg      = String()
                    msg.data = json.dumps(packet.get('peer_poses', []))
                    self.peer_pose_pub.publish(msg)

                if 'lane_offset' in packet:
                    msg      = Float32()
                    msg.data = float(packet['lane_offset'])
                    self.lane_offset_pub.publish(msg)

                if 'conflict_zone_detail' in packet:
                    msg      = String()
                    msg.data = json.dumps(packet.get('conflict_zone_detail') or {})
                    self.conflict_zone_detail_pub.publish(msg)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f'Ctrl recv error: {e}')

    # ═══════════════════════════════════════════════════════════════════════
    # CLEANUP
    # ═══════════════════════════════════════════════════════════════════════

    def destroy_node(self):
        self.running = False
        self.bridge_sock.close()
        self.ctrl_sock.close()
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = UDPReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('UDP Receiver stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
