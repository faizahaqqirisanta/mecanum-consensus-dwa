#!/usr/bin/env python3
"""
multi_robot_bringup.launch.py — haqqi_ta
Dijalankan di PC MASTER (laptop/komputer, bukan di robot fisik).

Tanggung jawab file ini:
  - udp_bridge_pc         → forward /experiment_state + goal/pose ke robot via UDP
  - consensus_node        → sinkronisasi sisa-waktu/progress L4
  - priority_manager_node → stop-and-go hierarkis L5
  - experiment_logger_node → rekam semua metrik evaluasi

Yang TIDAK dijalankan di sini (sudah jalan di masing-masing robot):
  - yahboomcar_bringup_multi  → driver motor, EKF, IMU, TF  (di robot)
  - ms200_scan_robot*.launch  → LiDAR publisher              (di robot)
  - robot*_haqqi.launch.py    → AMCL, DWA, path, fault       (di robot)

Prasyarat jaringan:
  - PC master dan semua robot harus di jaringan yang sama (WiFi/LAN)
  - Setiap robot pakai ROS_DOMAIN_ID yang BERBEDA (robot1=40, robot2=41, robot3=42, PC=44)
    → Isolasi DDS; semua komunikasi lintas domain via UDP bridge
  - Disarankan set RMW_IMPLEMENTATION=rmw_fastrtps_cpp untuk stabilitas DDS

Urutan startup yang benar (semua terminal di PC master):
  [Di setiap robot fisik TERLEBIH DAHULU]:
    ros2 launch yahboomcar_multi yahboomcar_bringup_multi.launch.xml robot_name:=robot1
    ros2 launch yahboomcar_multi ms200_scan_robot1.launch.py
    ros2 launch haqqi_ta robot1_haqqi.launch.py

  [Setelah semua robot siap, jalankan di PC master]:
    ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge

Argumen launch:
  scenario      : convoy | crossing | merge | split
  experiment_name : nama prefix folder output logger
  output_dir    : direktori output CSV logger
  v_nominal     : kecepatan nominal robot (m/s)
  d_emergency   : jarak hard stop antar robot (m)
  d_warning     : jarak mulai melambat antar robot (m)
  d_clear       : jarak hysteresis kembali normal (m)
  t_max_stop    : Priority Override Timeout (detik)
  epsilon       : step size consensus (< 0.5)
  k_consensus   : gain proportional consensus
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAMESPACES = ['robot1', 'robot2', 'robot3']


def generate_launch_description():

    # ══════════════════════════════════════════════════════════════════════
    # ARGUMEN LAUNCH
    # ══════════════════════════════════════════════════════════════════════

    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='merge',
        description='Skenario: convoy | crossing | merge | split')

    exp_name_arg = DeclareLaunchArgument(
        'experiment_name', default_value='run',
        description='Prefix nama folder output logger')

    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value=os.path.expanduser('~/experiment_logs'),
        description='Direktori output file CSV logger')

    enable_formation_manager_arg = DeclareLaunchArgument(
        'enable_formation_manager', default_value='false',
        description='true untuk menjalankan formation_manager_node berbasis /formation/goal_pose')

    log_coordination_debug_arg = DeclareLaunchArgument(
        'log_coordination_debug', default_value='false',
        description='true untuk menulis coordination_debug_log.csv')
    log_dwa_mode_arg = DeclareLaunchArgument(
        'log_dwa_mode', default_value='false',
        description='true untuk menulis dwa_mode_log.csv')
    log_dynamic_obstacle_debug_arg = DeclareLaunchArgument(
        'log_dynamic_obstacle_debug', default_value='true',
        description='true untuk menulis dynamic_obstacle_log.csv')
    log_path_debug_arg = DeclareLaunchArgument(
        'log_path_debug', default_value='true',
        description='true untuk menulis path_debug_log.csv')
    log_conflict_detail_arg = DeclareLaunchArgument(
        'log_conflict_detail', default_value='true',
        description='true untuk menulis conflict_detail_log.csv')
    log_local_plan_arg = DeclareLaunchArgument(
        'log_local_plan', default_value='true',
        description='true untuk menulis local_plan_log.csv')

    # ── IP Robot — satu tempat untuk semua node UDP ───────────────────────
    robot1_ip_arg = DeclareLaunchArgument(
        'robot1_ip', default_value='192.168.0.91',
        description='IP robot1')
    robot2_ip_arg = DeclareLaunchArgument(
        'robot2_ip', default_value='192.168.0.88',
        description='IP robot2')
    robot3_ip_arg = DeclareLaunchArgument(
        'robot3_ip', default_value='192.168.0.82',
        description='IP robot3')

    # ── Parameter konsensus (L4) ──────────────────────────────────────────
    v_nominal_arg = DeclareLaunchArgument(
        'v_nominal', default_value='0.30',
        description='Kecepatan nominal robot (m/s), selaras dengan max_vel_x DWA')

    vnom_pathlen_scaling_arg = DeclareLaunchArgument(
        'vnom_pathlen_scaling', default_value='true',
        description='Baseline v per-robot ~ panjang lintasan: robot terpanjang pakai v_nominal, '
                    'yang lebih pendek diperlambat (v_i = v_nominal * L_i / L_max)')

    v_consensus_floor_arg = DeclareLaunchArgument(
        'v_consensus_floor', default_value='0.015',
        description='Batas bawah v_max dari consensus')

    v_consensus_ceiling_arg = DeclareLaunchArgument(
        'v_consensus_ceiling', default_value='0.50',
        description='Batas atas v_max consensus untuk robot yang tertinggal')

    epsilon_arg = DeclareLaunchArgument(
        'epsilon', default_value='0.03',
        description='Step size consensus (ε < 1/3 untuk 3 robot fully-connected)')

    epsilon_deadband_arg = DeclareLaunchArgument(
        'epsilon_deadband', default_value='0.02',   # [FIX-ARRSYNC] 0.03->0.02 engage koreksi lebih awal
        description='Deadband: |e_i| < epsilon_deadband → kirim v_nominal (tidak ada koreksi)')

    k_consensus_arg = DeclareLaunchArgument(
        'k_consensus', default_value='1.0',   # [FIX-ARRSYNC] 0.70->1.0 leader melambat lebih agresif menunggu laggard
        description='Gain bidirectional: vmax = clip(v_nominal + Kc*e_i, floor, ceiling)')

    # [FIX-TIMECONS] Konsensus sisa-waktu (time-to-go) — feedback, bukan ETA open-loop.
    # Aktif bila coordination_mode:=time_consensus (atau arrival_mode:=time_consensus).
    k_time_arg = DeclareLaunchArgument(
        'k_time', default_value='0.20',
        description='Gain konsensus sisa-waktu: vmax = clip(v_nominal + k_time*(t_remain_i - t_bar), floor, ceiling)')

    epsilon_time_s_arg = DeclareLaunchArgument(
        'epsilon_time_s', default_value='0.30',
        description='Deadband selisih sisa-waktu (detik): |t_remain_i - t_bar| < epsilon_time_s → v_nominal')

    feasibility_aware_eta_arg = DeclareLaunchArgument(
        'feasibility_aware_eta_enabled', default_value='true',
        description='true: estimasi sisa waktu consensus memakai v efektif DWA saat tersedia')

    eta_v_eff_floor_arg = DeclareLaunchArgument(
        'eta_v_eff_floor', default_value='0.03',
        description='Floor v_hat ETA agar remaining/v_hat tidak terlalu sensitif saat robot pelan')

    eta_v_eff_timeout_s_arg = DeclareLaunchArgument(
        'eta_v_eff_timeout_s', default_value='1.0',
        description='Timeout telemetry DWA untuk feasibility-aware ETA')

    eta_v_filter_alpha_arg = DeclareLaunchArgument(
        'eta_v_filter_alpha', default_value='0.20',
        description='EMA alpha untuk v_hat ETA; 1.0 = tanpa filter')

    v_consensus_max_rate_arg = DeclareLaunchArgument(
        'v_consensus_max_rate', default_value='0.50',
        description='Slew-rate vmax consensus (m/s per detik); <=0 = nonaktif')

    v_consensus_output_filter_alpha_arg = DeclareLaunchArgument(
        'v_consensus_output_filter_alpha', default_value='0.10',
        description='EMA alpha untuk output akhir vmax consensus; 1.0 = tanpa filter')

    progress_speed_enabled_arg = DeclareLaunchArgument(
        'progress_speed_enabled', default_value='true',
        description='true: v_hat ETA memakai ds/dt remaining; false: pakai DWA speed magnitude')

    progress_speed_max_jump_m_arg = DeclareLaunchArgument(
        'progress_speed_max_jump_m', default_value='0.5',
        description='Lompatan remaining di atas nilai ini dianggap replan/AMCL jump')

    eta_rot_term_enabled_arg = DeclareLaunchArgument(
        'eta_rot_term_enabled', default_value='false',
        description='true: tambahkan estimasi waktu rotasi terminal ke ETA')

    eta_rot_omega_max_arg = DeclareLaunchArgument(
        'eta_rot_omega_max', default_value='0.20',
        description='Laju rotasi efektif untuk estimasi ETA rotasi terminal')

    eta_rot_activation_m_arg = DeclareLaunchArgument(
        'eta_rot_activation_m', default_value='0.30',
        description='Sisa jarak terminal saat ETA rotasi mulai dihitung')

    eta_rot_time_cap_s_arg = DeclareLaunchArgument(
        'eta_rot_time_cap_s', default_value='8.0',
        description='Batas atas tambahan ETA rotasi terminal')

    convergence_threshold_arg = DeclareLaunchArgument(
        'convergence_threshold', default_value='0.02',
        description='|p_i - p_bar| < threshold → dianggap konvergen')

    # ── Time/progress consensus (L4) ─────────────────────────────────────
    # arrival_mode='time_consensus' → sinkronisasi sisa-waktu berbasis remaining;
    #                                 arrival_schedule YAML dipakai sebagai offset
    #                                 bila arrival_offset_robot* kosong.
    # arrival_mode='time_offset_consensus' → alias eksplisit untuk time_consensus + offset.
    # arrival_mode='consensus' → mode lama berbasis progress.
    # arrival_mode='arrival_offset_consensus' → mode lama ETA relatif terhadap q_bar.
    # target_arrival_r*: hanya untuk logger/evaluator, bukan kontrol scheduler.
    arrival_mode_arg = DeclareLaunchArgument(
        'arrival_mode', default_value='time_consensus',
        description='Default = time_consensus. Offset dibaca dari arrival_schedule YAML '
                    'bila arrival_offset_robot* kosong, atau isi arrival_offset_robot* manual.')

    coordination_mode_arg = DeclareLaunchArgument(
        'coordination_mode', default_value='consensus_offset',
        description='[ARR-OFFSET] default = consensus_offset (progress consensus; offset masuk ke baseline v_nominal DAN target consensus). Offset manual via arrival_offset_robot*.')

    # arrival_offset_robot*: offset waktu tiba relatif per robot.
    # 0.0 = tanpa offset (default semua skenario kecuali variasi convoy).
    # [M4] L4 sync switch: false → kirim v_nominal tanpa throttling (variasi convoy async).
    l4_sync_enabled_arg = DeclareLaunchArgument(
        'l4_sync_enabled', default_value='true',
        description='[M4] true=L4 consensus aktif; false=bypass (robot dapat v_nominal, no sync)')

    arrival_offset_r1_arg = DeclareLaunchArgument(
        'arrival_offset_robot1', default_value='0.0')
    arrival_offset_r2_arg = DeclareLaunchArgument(
        'arrival_offset_robot2', default_value='10.0')   # [ARR-OFFSET] convoy stagger 10s: R2 tiba ~10s setelah R1
    arrival_offset_r3_arg = DeclareLaunchArgument(
        'arrival_offset_robot3', default_value='20.0')   # [ARR-OFFSET] convoy stagger 10s: R3 tiba ~20s setelah R1
    k_arrival_consensus_arg = DeclareLaunchArgument(
        'k_arrival_consensus', default_value='0.06')
    eta_v_ref_arg = DeclareLaunchArgument(
        'eta_v_ref', default_value='0.0')

    target_arrival_r1_arg = DeclareLaunchArgument(
        'target_arrival_r1', default_value='0.0',
        description='Target arrival time robot1 untuk logger/evaluator (s dari RUNNING)')

    target_arrival_r2_arg = DeclareLaunchArgument(
        'target_arrival_r2', default_value='0.0',
        description='Target arrival time robot2 untuk logger/evaluator (s dari RUNNING)')

    target_arrival_r3_arg = DeclareLaunchArgument(
        'target_arrival_r3', default_value='0.0',
        description='Target arrival time robot3 untuk logger/evaluator (s dari RUNNING)')

    # ── Parameter priority manager (L5) ───────────────────────────────────
    # Default eksperimen fisik ringan tapi tetap menjaga clearance minimum 0.30m.
    d_emergency_arg = DeclareLaunchArgument(
        'd_emergency', default_value='0.30',
        description='Jarak EMERGENCY stop antar robot (m)')

    d_warning_arg = DeclareLaunchArgument(
        'd_warning', default_value='0.50',
        description='Jarak mulai decelerate (m)')

    d_clear_arg = DeclareLaunchArgument(
        'd_clear', default_value='0.80',
        description='Jarak hysteresis kembali normal (m)')

    d_hard_collision_arg = DeclareLaunchArgument(
        'd_hard_collision', default_value='0.30',
        description='Last-resort: stop KEDUA robot jika jarak <= nilai ini (m)')

    t_max_stop_arg = DeclareLaunchArgument(
        't_max_stop', default_value='4.0',
        description='Priority Override Timeout (detik) — beri margin > TTF max')

    t_override_arg = DeclareLaunchArgument(
        't_override', default_value='3.0',
        description='Durasi override aktif (detik)')

    v_warning_ratio_arg = DeclareLaunchArgument(
        'v_warning_ratio', default_value='0.20',
        description='Rasio v_max saat warning zone terhadap v_nominal')

    trial_timeout_arg = DeclareLaunchArgument(
        'trial_timeout_s', default_value='0.0',
        description='Auto-stop setelah N detik (0 = nonaktif). Pakai 90.0 untuk split.')

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance', default_value='0.15',
        description='Radius goal untuk position_success (m)')

    lane_negotiation_enabled_arg = DeclareLaunchArgument(
        'lane_negotiation_enabled', default_value='false',
        description='false → matikan head-on lane offset, cocok untuk tes zone/gap murni')

    auto_conflict_zone_enabled_arg = DeclareLaunchArgument(
        'auto_conflict_zone_enabled', default_value='false')
    path_horizon_m_arg = DeclareLaunchArgument(
        'path_horizon_m', default_value='3.0')
    path_sample_step_m_arg = DeclareLaunchArgument(
        'path_sample_step_m', default_value='0.10')
    conflict_path_distance_arg = DeclareLaunchArgument(
        'conflict_path_distance', default_value='0.45')
    conflict_cluster_radius_arg = DeclareLaunchArgument(
        'conflict_cluster_radius', default_value='0.50')
    auto_zone_radius_arg = DeclareLaunchArgument(
        'auto_zone_radius', default_value='0.75')
    auto_detect_radius_arg = DeclareLaunchArgument(
        'auto_detect_radius', default_value='1.80')
    auto_hold_radius_arg = DeclareLaunchArgument(
        'auto_hold_radius', default_value='0.90')
    auto_clear_radius_arg = DeclareLaunchArgument(
        'auto_clear_radius', default_value='1.05')
    auto_gap_s_arg = DeclareLaunchArgument(
        'auto_gap_s', default_value='2.0')
    min_conflict_angle_deg_arg = DeclareLaunchArgument(
        'min_conflict_angle_deg', default_value='45.0')

    dynamic_robot_obstacle_enabled_arg = DeclareLaunchArgument(
        'dynamic_robot_obstacle_enabled', default_value='true')
    peer_pose_timeout_s_arg = DeclareLaunchArgument(
        'peer_pose_timeout_s', default_value='1.0')
    robot_obstacle_radius_arg = DeclareLaunchArgument(
        'robot_obstacle_radius', default_value='0.20')
    robot_obstacle_margin_arg = DeclareLaunchArgument(
        'robot_obstacle_margin', default_value='0.15')
    robot_obstacle_influence_radius_arg = DeclareLaunchArgument(
        'robot_obstacle_influence_radius', default_value='0.80')
    robot_path_blocking_radius_arg = DeclareLaunchArgument(
        'robot_path_blocking_radius', default_value='0.35')
    bypass_offset_arg = DeclareLaunchArgument(
        'bypass_offset', default_value='0.45')
    bypass_clear_distance_arg = DeclareLaunchArgument(
        'bypass_clear_distance', default_value='0.70')
    dynamic_obstacle_weight_arg = DeclareLaunchArgument(
        'dynamic_obstacle_weight', default_value='2.0')

    approach_stall_timeout_arg = DeclareLaunchArgument(
        'approach_stall_timeout_s', default_value='6.0',
        description='Detik sebelum owner yang tidak maju dipaksa CLEARING')  # [FIX-DEADLOCK] 8->6

    approach_progress_min_arg = DeclareLaunchArgument(
        'approach_progress_min_m', default_value='0.15',
        description='Kemajuan minimum menuju zona dalam stall window (m)')

    owner_cooldown_arg = DeclareLaunchArgument(
        'owner_cooldown_s', default_value='15.0',
        description='Cooldown robot setelah timeout/stall — tidak bisa jadi owner (detik)')

    owner_stuck_release_arg = DeclareLaunchArgument(
        'owner_stuck_release_s', default_value='5.0',
        description='Tambahan tahan OWNER_STUCK sebelum anti-deadlock release (detik)')  # [FIX-DEADLOCK] 8->5

    priority_mode_arg = DeclareLaunchArgument(
        'priority_mode', default_value='eta',
        description='eta untuk right-of-way dinamis, static untuk urutan skenario')

    priority_order_arg = DeclareLaunchArgument(
        'priority_order', default_value='',
        description='Override urutan prioritas, contoh: robot3,robot2,robot1')

    agent_failure_detection_arg = DeclareLaunchArgument(
        'agent_failure_detection_enabled', default_value='false',
        description='true untuk exclude agen gagal/stall dari consensus dan priority ETA')

    final_goal_proximity_arg = DeclareLaunchArgument(
        'final_goal_proximity', default_value='0.25',
        description='Jarak ke final goal (m) — robot dianggap sudah selesai, skip zone')

    goal_stable_time_arg = DeclareLaunchArgument(
        'goal_stable_time', default_value='1.5',
        description='Detik posisi harus stabil dalam toleransi sebelum position_success dicatat')

    # ══════════════════════════════════════════════════════════════════════
    # NODE 0: UDP BRIDGE PC
    # Forward /experiment_state heartbeat + goal/initialpose ke robot via UDP
    # WAJIB jalan agar sinyal dari experiment_master_cli sampai ke robot
    # ══════════════════════════════════════════════════════════════════════
    udp_bridge_node = Node(
        package='haqqi_ta',
        executable='udp_bridge_pc',
        name='udp_bridge_pc',
        output='screen',
        parameters=[{
            'robot1_ip'     : LaunchConfiguration('robot1_ip'),
            'robot2_ip'     : LaunchConfiguration('robot2_ip'),
            'robot3_ip'     : LaunchConfiguration('robot3_ip'),
            'active_robots' : ['robot1', 'robot2', 'robot3'],
        }]
    )

    # ═══════════════════════════════════════════════════════════════════════
    # NODE 1: CONSENSUS NODE (L4)
    # Satu node untuk semua robot — subscribe ke /robot*/remaining_length
    # dan /robot*/path_length, publish /robot*/vmax_consensus
    # ═══════════════════════════════════════════════════════════════════════
    consensus_node = Node(
        package='haqqi_ta',
        executable='consensus_node',
        name='consensus_node',
        output='screen',
        parameters=[{
            'v_nominal'              : LaunchConfiguration('v_nominal'),
            'vnom_pathlen_scaling'   : LaunchConfiguration('vnom_pathlen_scaling'),
            'v_consensus_floor'      : LaunchConfiguration('v_consensus_floor'),
            'v_consensus_ceiling'    : LaunchConfiguration('v_consensus_ceiling'),
            'epsilon'                : LaunchConfiguration('epsilon'),
            'epsilon_deadband'       : LaunchConfiguration('epsilon_deadband'),
            'k_consensus'            : LaunchConfiguration('k_consensus'),
            'k_time'                 : LaunchConfiguration('k_time'),
            'epsilon_time_s'         : LaunchConfiguration('epsilon_time_s'),
            'feasibility_aware_eta_enabled': LaunchConfiguration('feasibility_aware_eta_enabled'),
            'eta_v_eff_floor'        : LaunchConfiguration('eta_v_eff_floor'),
            'eta_v_eff_timeout_s'    : LaunchConfiguration('eta_v_eff_timeout_s'),
            'eta_v_filter_alpha'     : LaunchConfiguration('eta_v_filter_alpha'),
            'v_consensus_max_rate'   : LaunchConfiguration('v_consensus_max_rate'),
            'v_consensus_output_filter_alpha': LaunchConfiguration('v_consensus_output_filter_alpha'),
            'progress_speed_enabled' : LaunchConfiguration('progress_speed_enabled'),
            'progress_speed_max_jump_m': LaunchConfiguration('progress_speed_max_jump_m'),
            'eta_rot_term_enabled'   : LaunchConfiguration('eta_rot_term_enabled'),
            'eta_rot_omega_max'      : LaunchConfiguration('eta_rot_omega_max'),
            'eta_rot_activation_m'   : LaunchConfiguration('eta_rot_activation_m'),
            'eta_rot_time_cap_s'     : LaunchConfiguration('eta_rot_time_cap_s'),
            'convergence_threshold'  : LaunchConfiguration('convergence_threshold'),
            'consensus_rate'         : 20.0,
            'min_path_length'        : 0.1,
            'goal_tolerance'         : LaunchConfiguration('goal_tolerance'),
            'scenario'               : LaunchConfiguration('scenario'),
            'arrival_mode'           : LaunchConfiguration('arrival_mode'),
            'coordination_mode'      : LaunchConfiguration('coordination_mode'),
            'target_arrival_robot1'  : LaunchConfiguration('target_arrival_r1'),
            'target_arrival_robot2'  : LaunchConfiguration('target_arrival_r2'),
            'target_arrival_robot3'  : LaunchConfiguration('target_arrival_r3'),
            'l4_sync_enabled'        : LaunchConfiguration('l4_sync_enabled'),
            'arrival_offset_robot1'  : LaunchConfiguration('arrival_offset_robot1'),
            'arrival_offset_robot2'  : LaunchConfiguration('arrival_offset_robot2'),
            'arrival_offset_robot3'  : LaunchConfiguration('arrival_offset_robot3'),
            'k_arrival_consensus'    : LaunchConfiguration('k_arrival_consensus'),
            'eta_v_ref'              : LaunchConfiguration('eta_v_ref'),
            'agent_failure_detection_enabled': LaunchConfiguration('agent_failure_detection_enabled'),
            'robot1_ip'              : LaunchConfiguration('robot1_ip'),
            'robot2_ip'              : LaunchConfiguration('robot2_ip'),
            'robot3_ip'              : LaunchConfiguration('robot3_ip'),
        }]
    )

    # ══════════════════════════════════════════════════════════════════════
    # NODE 2: PRIORITY MANAGER NODE (L5)
    # Satu node untuk semua robot — subscribe ke /robot*/amcl_pose,
    # publish /robot*/priority_stop dan /robot*/vmax_priority
    # ══════════════════════════════════════════════════════════════════════
    priority_manager_node = Node(
        package='haqqi_ta',
        executable='priority_manager_node',
        name='priority_manager_node',
        output='screen',
        parameters=[{
            'd_emergency'      : LaunchConfiguration('d_emergency'),
            'd_warning'        : LaunchConfiguration('d_warning'),
            'd_clear'          : LaunchConfiguration('d_clear'),
            'd_hard_collision' : LaunchConfiguration('d_hard_collision'),
            't_max_stop'       : LaunchConfiguration('t_max_stop'),
            't_override'       : LaunchConfiguration('t_override'),
            'v_warning_ratio'  : LaunchConfiguration('v_warning_ratio'),
            # [FIX-RELAXCLAMP] v_nominal priority 0.30 -> 0.50 (= max_vel_x).
            # Plafon warning-clamp di priority_manager = v_nominal; menaikkannya
            # melepas kunci 0.30 sehingga catch-up consensus (s.d. 0.488) bisa
            # terealisasi di robot. consensus_node tetap v_nominal=0.30 (headroom
            # kontrol). RISIKO: robot lebih cepat saat strafe dekat peer -> AMCL
            # bisa lebih mudah diverge; pantau tracking_mode + heading_error.
            'v_nominal'        : 0.50,
            'scenario'                  : LaunchConfiguration('scenario'),
            'lane_negotiation_enabled'  : LaunchConfiguration('lane_negotiation_enabled'),
            'approach_stall_timeout_s'  : LaunchConfiguration('approach_stall_timeout_s'),
            'approach_progress_min_m'   : LaunchConfiguration('approach_progress_min_m'),
            'owner_cooldown_s'          : LaunchConfiguration('owner_cooldown_s'),
            'owner_stuck_release_s'     : LaunchConfiguration('owner_stuck_release_s'),
            'priority_mode'             : LaunchConfiguration('priority_mode'),
            'priority_order'            : LaunchConfiguration('priority_order'),
            'agent_failure_detection_enabled': LaunchConfiguration('agent_failure_detection_enabled'),
            'final_goal_proximity'      : LaunchConfiguration('final_goal_proximity'),
            'auto_conflict_zone_enabled': LaunchConfiguration('auto_conflict_zone_enabled'),
            'path_horizon_m'            : LaunchConfiguration('path_horizon_m'),
            'path_sample_step_m'        : LaunchConfiguration('path_sample_step_m'),
            'conflict_path_distance'    : LaunchConfiguration('conflict_path_distance'),
            'conflict_cluster_radius'   : LaunchConfiguration('conflict_cluster_radius'),
            'auto_zone_radius'          : LaunchConfiguration('auto_zone_radius'),
            'auto_detect_radius'        : LaunchConfiguration('auto_detect_radius'),
            'auto_hold_radius'          : LaunchConfiguration('auto_hold_radius'),
            'auto_clear_radius'         : LaunchConfiguration('auto_clear_radius'),
            'auto_gap_s'                : LaunchConfiguration('auto_gap_s'),
            'min_conflict_angle_deg'    : LaunchConfiguration('min_conflict_angle_deg'),
            'check_rate'                : 10.0,
            'robot1_ip'                 : LaunchConfiguration('robot1_ip'),
            'robot2_ip'                 : LaunchConfiguration('robot2_ip'),
            'robot3_ip'                 : LaunchConfiguration('robot3_ip'),
        }]
    )

    # ══════════════════════════════════════════════════════════════════════
    # NODE 3: FORMATION MANAGER NODE
    # Subscribe /formation/goal_pose → distribusi goal ke semua robot
    # dengan lateral offset per robot (R1=+spacing, R2=anchor, R3=-spacing)
    # ══════════════════════════════════════════════════════════════════════
    formation_offset_mode_arg = DeclareLaunchArgument(
        'formation_offset_mode', default_value='fixed_y',
        description='lateral (convoy/crossing) | fixed_y (merge/rendezvous)')

    formation_spacing_arg = DeclareLaunchArgument(
        'formation_spacing', default_value='0.30',
        description='Jarak offset line-mode antar robot (m)')

    formation_layout_arg = DeclareLaunchArgument(
        'formation_layout', default_value='circle',
        description='circle untuk 360/N derajat, atau line untuk offset_mode lama')

    formation_radius_arg = DeclareLaunchArgument(
        'formation_radius', default_value='0.30',
        description='Radius formasi circle dari anchor (m)')

    formation_start_angle_arg = DeclareLaunchArgument(
        'formation_start_angle_deg', default_value='90.0',
        description='Sudut robot pertama pada formasi circle (deg)')

    formation_manager_node = Node(
        package='haqqi_ta',
        executable='formation_manager_node',
        name='formation_manager_node',
        condition=IfCondition(LaunchConfiguration('enable_formation_manager')),
        output='screen',
        parameters=[{
            'formation_spacing' : LaunchConfiguration('formation_spacing'),
            'formation_layout'  : LaunchConfiguration('formation_layout'),
            'formation_radius'  : LaunchConfiguration('formation_radius'),
            'formation_start_angle_deg': LaunchConfiguration('formation_start_angle_deg'),
            'offset_mode'       : LaunchConfiguration('formation_offset_mode'),
            'active_robots'     : ['robot1', 'robot2', 'robot3'],
        }]
    )

    # ══════════════════════════════════════════════════════════════════════
    # NODE 4: EXPERIMENT LOGGER NODE
    # Merekam semua metrik evaluasi ke CSV
    # ══════════════════════════════════════════════════════════════════════
    experiment_logger_node = Node(
        package='haqqi_ta',
        executable='experiment_logger_node',
        name='experiment_logger_node',
        output='screen',
        parameters=[{
            'output_dir'             : LaunchConfiguration('output_dir'),
            'experiment_name'        : LaunchConfiguration('experiment_name'),
            'scenario'               : LaunchConfiguration('scenario'),
            'arrival_mode'           : LaunchConfiguration('arrival_mode'),
            'coordination_mode'      : LaunchConfiguration('coordination_mode'),
            'd_emergency'            : LaunchConfiguration('d_emergency'),
            'goal_tolerance'         : LaunchConfiguration('goal_tolerance'),
            'trial_timeout_s'        : LaunchConfiguration('trial_timeout_s'),
            'goal_stable_time'       : LaunchConfiguration('goal_stable_time'),
            'target_arrival_robot1'  : LaunchConfiguration('target_arrival_r1'),
            'target_arrival_robot2'  : LaunchConfiguration('target_arrival_r2'),
            'target_arrival_robot3'  : LaunchConfiguration('target_arrival_r3'),
            'log_coordination_debug' : LaunchConfiguration('log_coordination_debug'),
            'log_dwa_mode'           : LaunchConfiguration('log_dwa_mode'),
            'log_dynamic_obstacle_debug': LaunchConfiguration('log_dynamic_obstacle_debug'),
            'log_path_debug'         : LaunchConfiguration('log_path_debug'),
            'log_conflict_detail'    : LaunchConfiguration('log_conflict_detail'),
            'log_local_plan'         : LaunchConfiguration('log_local_plan'),
            'log_rate'               : 10.0,
            'auto_stop_on_all_goal'  : True,
        }]
    )

    # ════════════════════════════════════════���═════════════════════════════
    # TIMING STARTUP
    #
    # t=0s  consensus_node      → langsung aktif, tunggu data dari robot
    # t=0s  priority_manager    → langsung aktif, tunggu pose dari robot
    # t=2s  experiment_logger   → sedikit delay agar node lain sudah publish
    #
    # Catatan: node-node ini tidak bergantung satu sama lain saat startup,
    # ketiganya bisa jalan bersamaan. Delay logger hanya untuk memastikan
    # header CSV tidak kosong di baris pertama.
    # ══════��═══════════════════════════════════════════════════════════════

    log_start = LogInfo(
        msg='[multi_robot_bringup] PC Master nodes starting...'
             ' Pastikan semua robot sudah menjalankan robot*_haqqi.launch.py')

    log_ready = LogInfo(
        msg='[multi_robot_bringup] Semua node PC Master aktif. '
             'Sistem siap untuk eksperimen.')

    return LaunchDescription([
        # Argumen — IP robot (satu tempat untuk semua node UDP)
        robot1_ip_arg,
        robot2_ip_arg,
        robot3_ip_arg,

        # Argumen — eksperimen
        scenario_arg,
        exp_name_arg,
        output_dir_arg,
        enable_formation_manager_arg,
        formation_layout_arg,
        formation_radius_arg,
        formation_start_angle_arg,
        log_coordination_debug_arg,
        log_dwa_mode_arg,
        log_dynamic_obstacle_debug_arg,
        log_path_debug_arg,
        log_conflict_detail_arg,
        log_local_plan_arg,
        v_nominal_arg,
        vnom_pathlen_scaling_arg,
        v_consensus_floor_arg,
        v_consensus_ceiling_arg,
        epsilon_arg,
        epsilon_deadband_arg,
        k_consensus_arg,
        k_time_arg,
        epsilon_time_s_arg,
        feasibility_aware_eta_arg,
        eta_v_eff_floor_arg,
        eta_v_eff_timeout_s_arg,
        eta_v_filter_alpha_arg,
        v_consensus_max_rate_arg,
        v_consensus_output_filter_alpha_arg,
        progress_speed_enabled_arg,
        progress_speed_max_jump_m_arg,
        eta_rot_term_enabled_arg,
        eta_rot_omega_max_arg,
        eta_rot_activation_m_arg,
        eta_rot_time_cap_s_arg,
        convergence_threshold_arg,
        l4_sync_enabled_arg,
        arrival_mode_arg,
        coordination_mode_arg,
        target_arrival_r1_arg,
        target_arrival_r2_arg,
        target_arrival_r3_arg,
        arrival_offset_r1_arg,
        arrival_offset_r2_arg,
        arrival_offset_r3_arg,
        k_arrival_consensus_arg,
        eta_v_ref_arg,
        d_emergency_arg,
        d_warning_arg,
        d_clear_arg,
        d_hard_collision_arg,
        t_max_stop_arg,
        t_override_arg,
        v_warning_ratio_arg,
        trial_timeout_arg,
        goal_tolerance_arg,
        lane_negotiation_enabled_arg,
        auto_conflict_zone_enabled_arg,
        path_horizon_m_arg,
        path_sample_step_m_arg,
        conflict_path_distance_arg,
        conflict_cluster_radius_arg,
        auto_zone_radius_arg,
        auto_detect_radius_arg,
        auto_hold_radius_arg,
        auto_clear_radius_arg,
        auto_gap_s_arg,
        min_conflict_angle_deg_arg,
        dynamic_robot_obstacle_enabled_arg,
        robot_obstacle_radius_arg,
        robot_obstacle_margin_arg,
        robot_obstacle_influence_radius_arg,
        robot_path_blocking_radius_arg,
        bypass_offset_arg,
        bypass_clear_distance_arg,
        peer_pose_timeout_s_arg,
        dynamic_obstacle_weight_arg,
        approach_stall_timeout_arg,
        approach_progress_min_arg,
        owner_cooldown_arg,
        owner_stuck_release_arg,
        priority_mode_arg,
        priority_order_arg,
        agent_failure_detection_arg,
        final_goal_proximity_arg,
        goal_stable_time_arg,
        formation_offset_mode_arg,
        formation_spacing_arg,

        # Log info
        log_start,

        # t=0s — UDP bridge, consensus, priority aktif; formation optional via arg
        udp_bridge_node,
        consensus_node,
        priority_manager_node,
        formation_manager_node,

        # t=2s — logger mulai setelah semua node publish
        TimerAction(period=2.0, actions=[
            experiment_logger_node,
            log_ready,
        ]),
    ])
