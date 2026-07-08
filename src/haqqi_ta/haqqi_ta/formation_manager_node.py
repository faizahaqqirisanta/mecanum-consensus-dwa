#!/usr/bin/env python3
"""
Formation Manager Node — haqqi_ta
Dijalankan di PC Master.

Menerima satu goal anchor via /formation/goal_pose, lalu distribute
ke masing-masing robot dengan layout:

  formation_layout='circle' (default):
    Robot ditempatkan mengelilingi anchor. Jarak sudut antar robot otomatis
    360/N derajat, sehingga 3 robot membentuk beda sudut 120°.

  formation_layout='line':
    Gunakan offset_mode lama.

  offset_mode='lateral' (line mode, convoy/crossing):
    Offset tegak lurus heading goal — formasi tetap relatif terhadap arah gerak.
    R1 kiri, R2 anchor, R3 kanan.

  offset_mode='fixed_y' (line mode, merge/rendezvous):
    Offset di sumbu Y global — semua robot menuju titik x yang sama,
    terpisah 0.30m di Y. Mencegah path saling silang saat robots datang
    dari arah berbeda ke satu titik.

Orientasi akhir default:
  final_orientation_mode='face_anchor'
    Tiap robot menghadap kembali ke goal anchor. Robot yang tepat di anchor
    memakai yaw anchor sebagai fallback.

Catatan skenario:
  - convoy/crossing : kirim satu /formation/goal_pose, offset_mode=lateral
  - merge/rendezvous: kirim satu /formation/goal_pose, offset_mode=fixed_y
  - split (1_to_many): kirim langsung ke /robot*/goal_pose dari CLI
"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped


class FormationManagerNode(Node):

    def __init__(self):
        super().__init__('formation_manager_node')

        self.declare_parameter('formation_spacing', 0.30)
        self.declare_parameter('formation_radius',  0.30)
        self.declare_parameter('formation_layout',  'circle')
        self.declare_parameter('formation_start_angle_deg', 90.0)
        self.declare_parameter('active_robots',     ['robot1', 'robot2', 'robot3'])
        # 'lateral' : offset tegak lurus heading (convoy/crossing)
        # 'fixed_y' : offset di sumbu Y global   (merge/rendezvous)
        self.declare_parameter('offset_mode',       'fixed_y')
        # 'face_anchor': tiap robot menghadap titik goal anchor
        # 'copy_anchor': semua robot memakai yaw dari /formation/goal_pose
        self.declare_parameter('final_orientation_mode', 'face_anchor')

        self.spacing       = float(self.get_parameter('formation_spacing').value)
        self.radius        = float(self.get_parameter('formation_radius').value)
        self.layout        = str(self.get_parameter('formation_layout').value).strip().lower()
        self.start_angle   = math.radians(float(
            self.get_parameter('formation_start_angle_deg').value))
        self.active_robots = list(self.get_parameter('active_robots').value)
        self.offset_mode   = self.get_parameter('offset_mode').value
        self.final_orientation_mode = str(
            self.get_parameter('final_orientation_mode').value).strip()

        self._goal_pubs = {}
        for ns in self.active_robots:
            self._goal_pubs[ns] = self.create_publisher(
                PoseStamped, f'/{ns}/goal_pose', 10)

        self.create_subscription(
            PoseStamped, '/formation/goal_pose',
            self._goal_cb, 10)

        self.get_logger().info(
            f'FormationManagerNode ready | layout={self.layout} | '
            f'spacing={self.spacing}m | radius={self.radius}m | '
            f'mode={self.offset_mode} | '
            f'orientation={self.final_orientation_mode} | '
            f'robots={self.active_robots}')

    # ──────────────────────────────────────────────────────────────────────

    @staticmethod
    def _yaw_to_quat(yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _formation_yaw(self, anchor_x, anchor_y, goal_x, goal_y, anchor_yaw):
        mode = self.final_orientation_mode
        if mode in ('copy_anchor', 'anchor_yaw'):
            return anchor_yaw
        if mode in ('face_anchor', 'face_goal', 'face_gathering_point'):
            dx = anchor_x - goal_x
            dy = anchor_y - goal_y
            if math.hypot(dx, dy) > 1e-3:
                return math.atan2(dy, dx)
        return anchor_yaw

    def _formation_offsets(self, anchor_yaw):
        if self.layout in ('circle', 'circular', 'radial', 'around_anchor'):
            n = max(1, len(self.active_robots))
            step = 2.0 * math.pi / n
            return {
                ns: (
                    self.radius * math.cos(self.start_angle + idx * step),
                    self.radius * math.sin(self.start_angle + idx * step),
                )
                for idx, ns in enumerate(self.active_robots)
            }

        center = (len(self.active_robots) - 1) / 2.0
        scalar_by_robot = {
            ns: (center - idx) * self.spacing
            for idx, ns in enumerate(self.active_robots)
        }

        if self.offset_mode == 'lateral':
            lx = -math.sin(anchor_yaw)
            ly = math.cos(anchor_yaw)
            return {
                ns: (scalar * lx, scalar * ly)
                for ns, scalar in scalar_by_robot.items()
            }

        if self.offset_mode == 'fixed_x':
            return {
                ns: (scalar, 0.0)
                for ns, scalar in scalar_by_robot.items()
            }

        return {
            ns: (0.0, scalar)
            for ns, scalar in scalar_by_robot.items()
        }

    def _goal_cb(self, msg: PoseStamped):
        gx  = msg.pose.position.x
        gy  = msg.pose.position.y
        q   = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        per_robot = self._formation_offsets(yaw)

        for ns in self.active_robots:
            dx, dy = per_robot.get(ns, (0.0, 0.0))
            goal_x = gx + dx
            goal_y = gy + dy
            final_yaw = self._formation_yaw(gx, gy, goal_x, goal_y, yaw)
            z_q, w_q = self._yaw_to_quat(final_yaw)

            goal = PoseStamped()
            goal.header.stamp    = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = z_q
            goal.pose.orientation.w = w_q
            self._goal_pubs[ns].publish(goal)
            self.get_logger().info(
                f'[FORM/{self.layout}/{self.offset_mode}] {ns} → '
                f'({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}) '
                f'Δ=({dx:+.3f}, {dy:+.3f}) '
                f'yaw={math.degrees(final_yaw):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = FormationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('FormationManagerNode stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
