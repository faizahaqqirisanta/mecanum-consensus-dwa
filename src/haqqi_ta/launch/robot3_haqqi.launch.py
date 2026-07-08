#!/usr/bin/env python3
"""
robot3_haqqi.launch.py — haqqi_ta
Dijalankan DI robot3 fisik.

Urutan startup di robot3:
  Terminal 1: ros2 launch yahboomcar_multi yahboomcar_bringup_multi.launch.xml robot_name:=robot3
  Terminal 2: ros2 launch yahboomcar_multi ms200_scan_robot3.launch.py
  Terminal 3: ros2 launch haqqi_ta robot3_haqqi.launch.py

Wiring cmd_vel:
  modified_dwa_node → /robot3/cmd_vel_raw → [fault_injector] → /robot3/cmd_vel → driver motor

  fault_enabled default=true → injector menahan cmd_vel sampai START.
  fault_mode:=none berarti pure relay saat RUNNING tanpa injeksi fault.

Node di PC master (bukan di sini):
  consensus_node, priority_manager_node, experiment_logger_node
"""

import os
import socket
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('192.168.0.1', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '192.168.0.210'




NS = 'robot3'


def generate_launch_description():

    # ── Path file parameter ────────────────────────────────────────────────
    haqqi_pkg      = get_package_share_directory('haqqi_ta')
    dwa_param_file = os.path.join(haqqi_pkg, 'param', f'dwa_{NS}_params.yaml')

    yahboomcar_pkg = get_package_share_directory('yahboomcar_multi')
    map_file       = os.path.join(yahboomcar_pkg, 'maps', 'yahboom_map_lss_carto.yaml')

    # ══════════════════════════════════════════════════════════════════════
    # BLOK 3: GLOBAL PATH NODE (L2 — Dijkstra)
    # Publish: /{NS}/plan, /{NS}/remaining_length,
    #          /{NS}/path_length, /{NS}/goal_reached
    # ══════════════════════════════════════════════════════════════════════
    global_path_node = Node(
        package='haqqi_ta',
        executable='global_path_node',
        name='global_path_node',
        namespace=NS,
        output='screen',
        parameters=[{
            'robot_ns'          : NS,
            'use_sim_time'      : False,
            'map_topic'         : '/map',
            'costmap_inflation' : 0.40,
            'goal_tolerance'     : 0.15,
            'waypoint_tolerance' : 0.30,
            'skip_passed_waypoints_enabled': True,
            'waypoint_passed_path_margin'  : 0.18,
            'waypoint_spacing'   : 0.05,
            'path_smoothing_enabled'      : True,
            'smoothing_input_step_m'      : 0.03,
            'smoothing_output_spacing_m'  : 0.04,
            'smoothing_data_weight'       : 0.04,
            'smoothing_smooth_weight'     : 0.55,
            'smoothing_iterations'        : 500,
            'smoothing_chaikin_iterations': 3,
            'smoothing_chaikin_cut'       : 0.30,
            'smoothing_shortcut_enabled'  : True,
            'amcl_pose_topic'   : '/amcl_pose',
            'formation_offset'  : 0.00,   # digeser oleh formation_manager_node, bukan di sini
        }],
        remappings=[
            ('plan', f'/{NS}/plan'),
        ]
    )

    # ══════════════════════════════════════════════════════════════════════
    # BLOK 4: MODIFIED DWA NODE (L3)
    # Output ke /{NS}/cmd_vel_raw — fault_injector yang relay ke cmd_vel
    # ══════════════════════════════════════════════════════════════════════
    modified_dwa_node = Node(
        package='haqqi_ta',
        executable='modified_dwa_node',
        name='modified_dwa_node',
        namespace=NS,
        output='screen',
        parameters=[
            dwa_param_file,
            {
                'corner_slowdown_enabled': False,
                'localization_guard_enabled': LaunchConfiguration('localization_guard_enabled'),   # [FIX-LOC-EN] override-able via launch arg
                'localization_consistency_guard_enabled': False,
            },
            {'robot_ns': NS, 'amcl_pose_topic': '/amcl_pose',
             'scenario': LaunchConfiguration('scenario'),
             'avoidance_mode': LaunchConfiguration('avoidance_mode'),
             'dynamic_robot_obstacle_enabled': LaunchConfiguration('dynamic_robot_obstacle_enabled'),
             'peer_pose_timeout_s': LaunchConfiguration('peer_pose_timeout_s'),
             'robot_obstacle_radius': LaunchConfiguration('robot_obstacle_radius'),
             'robot_obstacle_margin': LaunchConfiguration('robot_obstacle_margin'),
             'robot_obstacle_influence_radius': LaunchConfiguration('robot_obstacle_influence_radius'),
             'robot_path_blocking_radius': LaunchConfiguration('robot_path_blocking_radius'),
             'bypass_offset': LaunchConfiguration('bypass_offset'),
             'bypass_clear_distance': LaunchConfiguration('bypass_clear_distance'),
             'dynamic_obstacle_weight': LaunchConfiguration('dynamic_obstacle_weight'),
             'peer_blocking_max_dist_m': LaunchConfiguration('peer_blocking_max_dist_m'),
             'ekf_warmup_steps': LaunchConfiguration('ekf_warmup_steps')},
        ],
        remappings=[
            (f'/{NS}/cmd_vel', f'/{NS}/cmd_vel_raw'),  # WAJIB
        ]
    )

    # ══════════════════════════════════════════════════════════════════════
    # BLOK 5: FAULT INJECTOR NODE (Support)
    # fault_enabled=true + fault_mode=none → hold sampai START, lalu relay normal.
    # fault_enabled=false hanya untuk debug bypass fault/hold.
    # ══════════════════════════════════════════════════════════════════════
    fault_injector_node = Node(
        package='haqqi_ta',
        executable='fault_injector_node',
        name='fault_injector_node',
        output='screen',
        parameters=[{
            'robot_ns'    : NS,
            'enabled'     : LaunchConfiguration('fault_enabled'),
            'auto_start'  : True,
            'fault_mode'  : LaunchConfiguration('fault_mode'),
            'fault_target_robot': LaunchConfiguration('fault_target_robot'),
            'fault_start_s': LaunchConfiguration('fault_start_s'),
            'fault_duration_s': LaunchConfiguration('fault_duration_s'),
            'fault_repeat_count': LaunchConfiguration('fault_repeat_count'),
            'fault_interval_s': LaunchConfiguration('fault_interval_s'),
            'fault_delay' : LaunchConfiguration('fault_delay'),
            'ttf_duration': LaunchConfiguration('ttf_duration'),
            'ttf_min'     : LaunchConfiguration('ttf_min'),
            'ttf_max'     : LaunchConfiguration('ttf_max'),
            'fault_count' : LaunchConfiguration('fault_count'),
            'fault_type'  : LaunchConfiguration('fault_type'),
            'degraded_factor': LaunchConfiguration('degraded_factor'),
            'drift_vx'    : LaunchConfiguration('drift_vx'),
            'drift_vy'    : LaunchConfiguration('drift_vy'),
            'drift_wz'    : LaunchConfiguration('drift_wz'),
            'random_seed' : 42,
        }]
    )


    # ══════════════════════════════════════════════════════════════════════
    # BLOK 6: UDP SENDER NODE
    # ══════════════════════════════════════════════════════════════════════
    udp_sender_node = Node(
        package='haqqi_ta',
        executable='udp_sender_node',
        name='udp_sender_node',
        namespace=NS,
        output='screen',
        parameters=[{
            'robot_ns'        : NS,
            'pc_master_ip'    : LaunchConfiguration('pc_master_ip'),
            'send_rate'       : 10.0,
            'amcl_pose_topic' : '/amcl_pose',
        }]
    )

    # ══════════════════════════════════════════════════════════════════════
    # BLOK 7: UDP RECEIVER NODE
    # ══════════════════════════════════════════════════════════════════════
    udp_receiver_node = Node(
        package='haqqi_ta',
        executable='udp_receiver_node',
        name='udp_receiver_node',
        namespace=NS,
        output='screen',
        parameters=[{
            'robot_ns': NS,
            'v_nominal': 0.15,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════
    # BLOK 1: MAP SERVER (nav2_map_server + lifecycle_manager)
    # ══════════════════════════════════════════════════════════════════════
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_node',
        output='screen',
        parameters=[{'yaml_filename': map_file}])

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart'  : True,
            'node_names' : ['map_server_node'],
        }])

    # ══════════════════════════════════════════════════════════════════════
    # TIMING STARTUP
    # t=0s  map_server_node, map_lifecycle_node  (AMCL butuh /map)
    # t=2s  global_path_node (butuh /map dari map_server)
    # t=3s  modified_dwa_node
    # t=4s  fault_injector_node, udp_sender_node, udp_receiver_node
    # ══════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        DeclareLaunchArgument(
            'pc_master_ip',
            default_value='192.168.0.34',
            description='IP address PC Master'),
        DeclareLaunchArgument('scenario', default_value='convoy',
                              description='convoy | crossing | merge | split'),
        DeclareLaunchArgument('avoidance_mode', default_value='costmap',
                              description='none | costmap | reactive'),
        DeclareLaunchArgument('fault_enabled', default_value='true'),
        DeclareLaunchArgument('localization_guard_enabled', default_value='true',
                              description='Aktifkan localization guard (COV_INVALID/POSE_STALE halt). true|false'),
        DeclareLaunchArgument('fault_mode',    default_value='none'),
        DeclareLaunchArgument('fault_target_robot', default_value='robot2'),
        DeclareLaunchArgument('fault_start_s', default_value='15.0'),
        DeclareLaunchArgument('fault_duration_s', default_value='2.0'),
        DeclareLaunchArgument('fault_repeat_count', default_value='3'),
        DeclareLaunchArgument('fault_interval_s', default_value='10.0'),
        DeclareLaunchArgument('fault_delay',   default_value='8.0'),
        DeclareLaunchArgument('ttf_duration',  default_value='2.0'),
        DeclareLaunchArgument('ttf_min',       default_value='1.0'),
        DeclareLaunchArgument('ttf_max',       default_value='3.0'),
        DeclareLaunchArgument('fault_count',   default_value='1'),
        DeclareLaunchArgument('fault_type',    default_value='fail_stop',
                              description='fail_stop | freeze | degraded | drift'),
        DeclareLaunchArgument('degraded_factor', default_value='0.4'),
        DeclareLaunchArgument('drift_vx',      default_value='0.0'),
        DeclareLaunchArgument('drift_vy',      default_value='0.0'),
        DeclareLaunchArgument('drift_wz',      default_value='0.3'),
        DeclareLaunchArgument('dynamic_robot_obstacle_enabled', default_value='true'),
        DeclareLaunchArgument('peer_pose_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('robot_obstacle_radius', default_value='0.20'),
        DeclareLaunchArgument('robot_obstacle_margin', default_value='0.15'),
        DeclareLaunchArgument('robot_obstacle_influence_radius', default_value='0.80'),
        DeclareLaunchArgument('robot_path_blocking_radius', default_value='0.35'),
        DeclareLaunchArgument('bypass_offset', default_value='0.45'),
        DeclareLaunchArgument('bypass_clear_distance', default_value='0.70'),
        DeclareLaunchArgument('dynamic_obstacle_weight', default_value='2.0'),
        DeclareLaunchArgument('peer_blocking_max_dist_m', default_value='0.0',
                              description='[M1] Max dist (m) peer blocks path. 0=disabled'),
        DeclareLaunchArgument('ekf_warmup_steps', default_value='0',
                              description='[M5] AMCL steps before collision check. 0=disabled'),
        # t=0s — map server untuk AMCL
        TimerAction(period=0.0, actions=[map_server_node, map_lifecycle_node]),

        # t=2s — global path node (Dijkstra)
        TimerAction(period=2.0, actions=[global_path_node]),

        # t=3s — DWA mulai setelah map tersedia
        TimerAction(period=3.0, actions=[modified_dwa_node]),

        # t=4s — UDP nodes + fault injector
        TimerAction(period=4.0, actions=[fault_injector_node,
                                          udp_sender_node,
                                          udp_receiver_node]),
    ])
