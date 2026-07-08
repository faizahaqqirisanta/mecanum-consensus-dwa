# Runbook — haqqi_ta

Language: English | [Bahasa Indonesia](RUNBOOK.id.md) | [Language list](RUNBOOK.md)

This document is the complete operational guide for the coordination system of three Yahboom RDK X5 mecanum robots (ROS 2 Humble). For a project overview, see [../README.en.md](../README.en.md). Official hardware documentation: https://www.yahboom.net/study/RDK-X5-Robot

Scope: setup, build, three-robot synchronization, startup, the 16-trial matrix, monitoring, logs, troubleshooting, and the role of the `analysis/` folder (MATLAB scripts) and `docs/` folder (documentation/runbook). The IP addresses and `ROS_DOMAIN_ID` values below are the author's example configuration; adapt them to your network.

This document is not a script to be run all at once. The command blocks below are a reference to be run selectively and separately across several terminals or machines (the master PC and Robots 1/2/3 according to `ROS_DOMAIN_ID`). Running them top to bottom in sequence will fail. Copy each section according to the machine's role and the experiment stage.

## Scope and audience

This guide is intended for experiment operators (laboratory assistants or students) who run the experiments on three physical robots, and for repository readers who want to understand how the system works. The `docs/` folder contains the operating documentation; the `analysis/` folder contains MATLAB scripts that convert CSV logs into result figures. Read the preflight list before operating the robots.

## Installation from a bare system

This section covers installation from a system with neither Ubuntu nor ROS 2. If Ubuntu 22.04 and ROS 2 Humble are already installed, skip to the preflight list or Section 1.

Hardware prerequisite: a full experiment requires three physical Yahboom RDK X5 robots together with their stock stack and packages (motor driver, IMU, EKF/`robot_localization`, LiDAR), which are not included in this repository and are available through the official Yahboom documentation: https://www.yahboom.net/study/RDK-X5-Robot. There is no simulation mode yet; without hardware, the code can only be reviewed and figures reproduced from data. The steps below prepare a single machine (the master PC or each robot).

Step 1 — Install Ubuntu 22.04 LTS
- Download the Ubuntu 22.04 LTS ISO from the official Ubuntu site.
- Create a bootable USB (for example with Rufus or BalenaEtcher) and install it (dual-boot with Windows is possible).
- ROS 2 Humble is officially supported only on Ubuntu 22.04; do not use 20.04 or 24.04.

Step 2 — Install ROS 2 Humble (concise; refer to docs.ros.org if the repository or key changes)
```bash
sudo apt update && sudo apt install -y locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LANG=en_US.UTF-8
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
```

Step 3 — Environment, colcon, and rosdep
```bash
source /opt/ros/humble/setup.bash          # run in each new terminal (or add to ~/.bashrc)
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null; rosdep update
pip3 install numpy scipy pyyaml            # dependencies for the Python scripts
```

Step 4 — Create the workspace and add the code
```bash
mkdir -p ~/yahboomcar_ws/src
cp -r /path/to/this-repo/src/* ~/yahboomcar_ws/src/
cd ~/yahboomcar_ws
```

Step 5 — Install package dependencies (rosdep), then build
```bash
rosdep install --from-paths src --ignore-src -r -y   # install ROS dependencies automatically
colcon build --symlink-install
source install/setup.bash
```

Step 6 — Quick check
```bash
ros2 pkg executables haqqi_ta | head        # nodes should be listed
ros2 run haqqi_ta experiment_master_cli     # open the CLI (full function needs other nodes running)
```

For the three-robot and master-PC configuration (master PC at `~/TA/yahboomcar_ws`, each robot at `~/yahboomcar_ws`, cross-machine synchronization, and `ROS_DOMAIN_ID`), continue to Section 1 below. The Yahboom driver stack must already be present on each robot from its stock image.

## Preflight list (mandatory before running)

Verify every item before `START`. Most experiment failures originate from Tier 1 and Tier 2 items.

Tier 1 — Environment (errors here cause the robot to be mislocalized):
- [ ] Map matches the room. The robot launch uses `yahboom_map_lss_carto.yaml`. Ensure this map is identical to the current test room. If the room differs or changes, create a new map and update the map filename in `robotN_haqqi.launch.py` (variable `map_file`).
- [ ] Scenario coordinates match the map. `start_pose` and `waypoints` in `scenarios.yaml` were measured for the author's room. If the map origin differs, adjust the coordinates.
- [ ] IP and `ROS_DOMAIN_ID` are correct. PC `192.168.0.34` (domain 44), Robot1/2/3 `.91/.88/.82` (domains 40/41/42). If the network uses DHCP, assign static IPs or check `hostname -I` on each robot and override `robotN_ip:=<ip>` when launching the PC. All machines must be on one subnet.
- [ ] Robots placed exactly at their start points. The CLI publishes `initialpose` automatically from `start_pose`; if the physical position differs from `start_pose`, the AMCL estimate is wrong and the robot is mislocalized. Mark each robot's start point on the floor.

Tier 2 — Safety:
- [ ] Test the E-STOP before the robots move (CLI [6] EMERGENCY STOP). Ensure all operators know how to trigger it.
- [ ] Keep the area clear; stay away from moving robots (speeds up to `0.50 m/s`).
- [ ] Keep `localization_guard_enabled=true` (default); the robot will HOLD if localization is unreliable.
- [ ] Understand fault injection: for the `cons_fault` type, robot2 is stopped deliberately at about t = 15 s. This is part of the experiment, not a malfunction.

Tier 3 — System readiness (via the CLI menu):
- [ ] Environment sourced correctly on each machine (ROS 2 Humble and the workspace).
- [ ] Startup order: map, AMCL, DWA, UDP (see Sections 1 and 2).
- [ ] [3] Readiness Check fully satisfied (scenario loaded, `state=READY`, each robot's AMCL pose fresh, path formed) before [4] START.
- [ ] Set `experiment_name:=<name>` so log folders are easy to find (output is timestamped automatically: `name_YYYYMMDD_HHMMSS`).
- [ ] Between trials: [6] EMERGENCY STOP then [7] Reset Trial, and close `multi_robot_bringup` so logs do not mix.

## Architecture glossary

| Layer | Node | Function |
|---|---|---|
| L2 | `global_path_node` | Global path planning (Dijkstra) → `/{robot}/plan` |
| L3 | `modified_dwa_node` | Local planner (Modified DWA), output `cmd_vel_raw` |
| L4 | `consensus_node` | Average consensus: align arrival times across robots |
| L5 | `priority_manager_node` | Hierarchical stop-and-go in conflict zones |
| — | `fault_injector_node` | Software fault injection (robustness testing) |
| — | `experiment_logger_node` | Record all metrics to CSV |
| — | `udp_*` / `sync_monitor_node` | Cross-domain bridge and synchronization monitor |

## Code map (for further development)

Suggested reading order in `src/haqqi_ta/haqqi_ta/`:
1. `experiment_master_cli.py` — orchestrator; all trials are controlled from here (Load, Readiness, START, Reset). The most convenient entry point for understanding the flow.
2. `global_path_node.py` (L2) then `modified_dwa_node.py` (L3) — from the global plan to motion commands.
3. `consensus_node.py` (L4) — the core contribution: progress and arrival-time alignment.
4. `priority_manager_node.py` (L5) — intersection conflict resolution.
5. `fault_injector_node.py` and `experiment_logger_node.py` — robustness testing and metric recording.
6. `udp_*` / `sync_monitor_node.py` — cross-domain communication between machines (rarely changed).

Where to change things:
- Scenario behavior and waypoints: `param/scenarios.yaml`.
- Per-robot DWA tuning: `param/dwa_robot{1,2,3}_params.yaml`.
- Consensus parameters: `param/consensus_params.yaml`.
- Adding a coordination mode: add logic in `consensus_node.py` (the mode list is in Section 4) and register its name.
- Launch arguments: `launch/robot{1,2,3}_haqqi.launch.py` (per robot) and `launch/multi_robot_bringup.launch.py` (PC).

Safe to change: YAML parameters and launch arguments. Requires care: the topic contracts and UDP message formats between nodes; changing one side requires the other side to match.

```bash
# RUN TUTORIAL - haqqi_ta
# Multi-Agent Coordination Control: 3 Yahboom RDK X5 Mecanum Robots
# ================================================================

# ================================================================
# 0. FIXED CONFIGURATION
# ================================================================
#
# ROS_DOMAIN_ID:
#   Master PC : 44
#   Robot1    : 40
#   Robot2    : 41
#   Robot3    : 42
#
# IP:
#   Master PC : 192.168.0.34
#   Robot1    : 192.168.0.91
#   Robot2    : 192.168.0.88
#   Robot3    : 192.168.0.82
#
# Workspace:
#   Master PC : ~/TA/yahboomcar_ws
#   Robot     : ~/yahboomcar_ws
#   Staging   : ~/TA/yahboomcar_ws1/2/3
#
# Notes:
#   - Launch from the PC only from ~/TA/yahboomcar_ws.
#   - yahboomcar_ws1/2/3 are staging sources only, to be synced to the physical robots.
#   - The robot scenario is sent on [1] Load Scenario in experiment_master_cli.
#   - The PC multi_robot_bringup is still given scenario:=... for consensus/priority/logger.


# ================================================================
# 1. SETUP AND BUILD
# ================================================================

# Master PC:
export ROS_DOMAIN_ID=44
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/TA/yahboomcar_ws/install/setup.bash

# Build the master PC after changes to PC code / main source:
cd ~/TA/yahboomcar_ws
colcon build --symlink-install --packages-select haqqi_ta
source install/setup.bash

# Physical robots:
#   Robot1: export ROS_DOMAIN_ID=40
#   Robot2: export ROS_DOMAIN_ID=41
#   Robot3: export ROS_DOMAIN_ID=42
#   source /opt/ros/humble/setup.bash
#   source ~/yahboomcar_ws/install/setup.bash

# Sync source to robot staging from the PC main source:
for ws in yahboomcar_ws1 yahboomcar_ws2 yahboomcar_ws3; do
  rsync -a --delete --exclude='__pycache__/' ~/TA/yahboomcar_ws/src/haqqi_ta/         ~/TA/${ws}/src/haqqi_ta/
  rsync -a --delete --exclude='__pycache__/' ~/TA/yahboomcar_ws/src/yahboomcar_multi/ ~/TA/${ws}/src/yahboomcar_multi/
done

# Sync staging to the physical robots:
rsync -a ~/TA/yahboomcar_ws1/src/haqqi_ta/         sunrise@192.168.0.91:~/yahboomcar_ws/src/haqqi_ta/
rsync -a ~/TA/yahboomcar_ws1/src/yahboomcar_multi/ sunrise@192.168.0.91:~/yahboomcar_ws/src/yahboomcar_multi/
rsync -a ~/TA/yahboomcar_ws2/src/haqqi_ta/         sunrise@192.168.0.88:~/yahboomcar_ws/src/haqqi_ta/
rsync -a ~/TA/yahboomcar_ws2/src/yahboomcar_multi/ sunrise@192.168.0.88:~/yahboomcar_ws/src/yahboomcar_multi/
rsync -a ~/TA/yahboomcar_ws3/src/haqqi_ta/         sunrise@192.168.0.82:~/yahboomcar_ws/src/haqqi_ta/
rsync -a ~/TA/yahboomcar_ws3/src/yahboomcar_multi/ sunrise@192.168.0.82:~/yahboomcar_ws/src/yahboomcar_multi/

# Build on the physical robot after syncing code/launch/yaml:
cd ~/yahboomcar_ws
colcon build --symlink-install --packages-select haqqi_ta
source install/setup.bash

# Important:
#   - .py files take effect quickly with symlink-install.
#   - YAML/launch under install/share are copies; rebuild the robot or copy manually to install/share.
#   - Staging ws1/ws2/ws3 do not need building; build only the PC workspace and the physical robots.


# ================================================================
# 2. ROBOT STARTUP
# ================================================================
#
# ---------- RViz monitor for 3 robots ----------
export ROS_DOMAIN_ID=40
source ~/TA/yahboomcar_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/TA/yahboomcar_ws/src/yahboomcar_rviz/rviz/robot1_nav.rviz \
  --ros-args -r __node:=rviz2_robot1

export ROS_DOMAIN_ID=41
source ~/TA/yahboomcar_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/TA/yahboomcar_ws/src/yahboomcar_rviz/rviz/robot2_nav.rviz \
  --ros-args -r __node:=rviz2_robot2

export ROS_DOMAIN_ID=42
source ~/TA/yahboomcar_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/TA/yahboomcar_ws/src/yahboomcar_rviz/rviz/robot3_nav.rviz \
  --ros-args -r __node:=rviz2_robot3

# Run from the PC on domain 44. This view uses telemetry republished by
# consensus_node on the PC: /robot*/amcl_pose, /robot*/plan, /robot*/waypoints.
# This launch also starts the PC map_server so /map is available on domain 44.
export ROS_DOMAIN_ID=44
source ~/TA/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_rviz pc_multi_robot_monitor.launch.py
# To open only RViz without the PC map_server:
# ros2 launch yahboomcar_rviz pc_multi_robot_monitor.launch.py use_map_server:=false

# The older per-robot RViz can still be used to debug raw scan/TF.
# Use ROS_DOMAIN_ID=40/41/42 and the files robot1_nav.rviz/robot2_nav.rviz/robot3_nav.rviz.

# Run 3 terminals on each robot.
# Embedded default parameters need not be rewritten:
#   pc_master_ip=192.168.0.34, avoidance_mode=costmap, fault_mode=none.
#
# The scenario need not be set in the robot launch. On CLI Load Scenario, the PC
# sends /experiment_scenario via UDP and modified_dwa_node on the robot updates
# its active scenario.
#
# The only variations that must differ on the robot:
#   - localize mode  : localization_guard_enabled (see INPUT in Terminal 3)
#   - fault          : only for the 'cons_fault' type, robot2 timed_pulse

# ================================================================
# TERMINAL 1 — Laser bringup (each robot)
# ================================================================
ros2 launch yahboomcar_multi laser_bringup_multi.launch.xml robot_name:=robot1
ros2 launch yahboomcar_multi laser_bringup_multi.launch.xml robot_name:=robot2
ros2 launch yahboomcar_multi laser_bringup_multi.launch.xml robot_name:=robot3

# ================================================================
# TERMINAL 2 — AMCL (each robot)
# ================================================================
ros2 launch yahboomcar_multi robot1_amcl_launch.py
ros2 launch yahboomcar_multi robot2_amcl_launch.py
ros2 launch yahboomcar_multi robot3_amcl_launch.py

# ================================================================
# TERMINAL 3 — Robot bringup (haqqi_ta)
#   localize guard: set the same value for all three robots (true=ON / false=OFF)
#   fault: ONLY the target robot is given fault_mode (for cons_fault)
# ================================================================

# ---- cons_nofault / baseline / offset_arrival : ALL robots without fault ----
ros2 launch haqqi_ta robot1_haqqi.launch.py localization_guard_enabled:=false
ros2 launch haqqi_ta robot2_haqqi.launch.py localization_guard_enabled:=false
ros2 launch haqqi_ta robot3_haqqi.launch.py localization_guard_enabled:=false

# ---- cons_fault : fault ONLY on the target robot (example robot1) ----
ros2 launch haqqi_ta robot3_haqqi.launch.py \
  fault_mode:=timed_pulse \
  fault_target_robot:=robot3 \
  localization_guard_enabled:=false
ros2 launch haqqi_ta robot2_haqqi.launch.py localization_guard_enabled:=false
ros2 launch haqqi_ta robot3_haqqi.launch.py localization_guard_enabled:=false
#   timed_pulse defaults: start=15s, duration=2s, repeat=3, interval=10s.
#   Variation: add fault_start_s:=20.0 fault_duration_s:=3.0 etc.

# ================================================================
# 3. MASTER PC STARTUP
# ================================================================

# PC Terminal 1: experiment CLI.
ros2 run haqqi_ta experiment_master_cli
#
# Main menu:
#   [1] Load Scenario          [2] Preview Scenario
#   [3] Readiness Check        [4] START Trial
#   [5] Monitor Live           [6] EMERGENCY STOP
#   [7] Reset Trial            [8] Manual Goal / Waypoints
#   [9] Fault Injection        [0] Exit
#
# Per-trial flow:
#   1. Choose the robot2 variation for the type (fault only for cons_fault).
#   2. Start multi_robot_bringup on the PC for the trial (Section 4).
#   3. In experiment_master_cli: [1] Load Scenario, the same as the PC launch.
#      The CLI sends initialpose, waypoints, final yaw, and scenario to the robots.
#   4. [3] Readiness Check until pose/path/state OK.
#   5. [4] START Trial, wait for completion.
#   6. [6] EMERGENCY STOP, then [7] Reset Trial before moving to the next trial.
#   7. Close multi_robot_bringup before the next trial so logs/parameters do not mix.

# PC Terminal 2: lightweight monitor.
ros2 run haqqi_ta sync_monitor_node

# ================================================================
# 4. EXPERIMENT MATRIX  (4 TYPES x 4 PATH SHAPES = 16 trials)
# ================================================================
#
#   Experiment TYPES:
#     - baseline        : L4 coordination OFF (l4_sync_enabled:=false). Robots run
#                         independently without consensus. NO fault.
#     - cons_nofault    : consensus ON (coordination_mode:=$COORD_MODE). NO fault.
#     - cons_fault      : consensus ON + robot2 fault (timed_pulse, from the robot launch).
#     - offset_arrival  : consensus ON + relative arrival offset between robots.
#
#   Path SHAPES: convoy | crossing | merge | split
#
# ----------------------------------------------------------------
# >>> EXPERIMENT INPUT (set once on the master PC before a trial) <<<
# ----------------------------------------------------------------
#
# Choose the CONSENSUS MODE (used by cons_nofault / cons_fault / offset_arrival):
#   consensus | consensus_offset | consensus_so | consensus_seg | consensus_so_seg |
#   consensus_dist | consensus_ft | consensus_ftso | consensus_fxt |
#   time_consensus | time_offset_consensus | arrival_offset_consensus
export COORD_MODE=consensus

# Arrival offset (seconds) for the offset_arrival type:
export OFF_R1=0.0
export OFF_R2=10.0
export OFF_R3=20.0

# Output and robot IPs (usually fixed):
export OUT=/home/$(whoami)/experiment_logs
export R1=192.168.0.91 ; export R2=192.168.0.88 ; export R3=192.168.0.82

# Common arguments for all trials (full logging + IPs). Remove the two log_* for a light run.
export COMMON="output_dir:=${OUT} log_dwa_mode:=true log_coordination_debug:=true robot1_ip:=${R1} robot2_ip:=${R2} robot3_ip:=${R3}"
# split needs a longer timeout:
export SPLIT_EXTRA="trial_timeout_s:=90.0"
#
# Localize-mode note: set on the robot (Terminal 3) via LOCALIZE_GUARD.
# Fault note: for cons_fault, robot2 MUST be launched with fault_mode:=timed_pulse.


# ---------------- TYPE 1: baseline (L4 OFF, no fault) ----------------
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_baseline_01   l4_sync_enabled:=false ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_baseline_01 l4_sync_enabled:=false ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_baseline_01    l4_sync_enabled:=false ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_baseline_01    l4_sync_enabled:=false ${COMMON} ${SPLIT_EXTRA}


# ---------------- TYPE 2: cons_nofault (consensus ON, no fault) ----------------
# Robot2 is launched WITHOUT fault.
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_cons_nofault_01   coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_cons_nofault_01 coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_cons_nofault_01    coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_cons_nofault_01    coordination_mode:=${COORD_MODE} ${COMMON} ${SPLIT_EXTRA}


# ---------------- TYPE 3: cons_fault (consensus ON, robot2 fault) ----------------
# Robot2 is launched WITH fault_mode:=timed_pulse (see Terminal 3).
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_cons_fault_01   coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_cons_fault_01 coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_cons_fault_01    coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_cons_fault_01    coordination_mode:=${COORD_MODE} ${COMMON} ${SPLIT_EXTRA}


# ---------------- TYPE 4: offset_arrival (consensus ON + arrival offset) ----------------
# Relative offset: a robot with a larger offset tends to be slowed. Robot2 WITHOUT fault.
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_offset_01   coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_offset_01 coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_offset_01    coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_offset_01    coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON} ${SPLIT_EXTRA}


# ================================================================
# 5. CURRENT PROGRAM NOTES
# ================================================================
#
# Consensus:
#   - coordination_mode is chosen via the INPUT $COORD_MODE (Section 4).
#   - baseline uses l4_sync_enabled:=false (consensus fully disabled).
#   - If arrival_offset_robot* are empty, the active scenario YAML arrival_schedule
#     is read as the relative offset. time_offset_consensus is only an explicit alias.
#   - For offset_arrival, set at least one nonzero arrival_offset_robot*.
#   - Time-consensus formula:
#       t_i = mission_remaining_i / v_est_i
#       q_i = t_i - arrival_offset_i
#       v_i = clamp(v_nominal + k_time * (q_i - q_bar))
#   - On crossing/split, the arrival_schedule does differ per robot; the vcons plot
#     is reasonably non-identical because the YAML offset is also applied.
#   - The priority layer does not lock the baseline to v_nominal when there is no conflict:
#     vmax_priority_ceiling=0.50 gives consensus room to catch up up to the DWA limit.
#
# Localize mode:
#   - Set per robot via localization_guard_enabled (INPUT $LOCALIZE_GUARD).
#   - true  = robot HOLDs if AMCL sigma is invalid / pose is stale (safe, conservative).
#   - false = guard off; the robot keeps moving even if localization is doubtful.
#   - Check the loc_hold fraction via the loc_hold_frac / sigma_invalid_frac metrics.
#
# Fault:
#   - Only the cons_fault type uses a fault; robot2 timed_pulse.
#   - The fault is configured in the robot2 launch, not in the PC multi_robot_bringup.
#   - fault_mode:=none still holds cmd_vel until START, then relays normally.
#
# Final orientation:
#   - Convoy and merge use final_orientation_mode: face_gathering_point.
#   - The final yaw is computed at Load Scenario by experiment_master_cli.
#   - During final alignment, DWA uses a latched odom-yaw as the final rotation reference.
#
# DWA/local planner:
#   - The robot DWA scenario is updated from the CLI via /experiment_scenario.
#   - [MOD-21] Peer avoidance uses motion prediction (cmd_vel over UDP) + a heading-aware
#     OBB footprint. Knobs: footprint_margin_m, peer_predict_horizon_s, peer_predict_enabled.
#   - DYNAV/DYN_AVOID is active for side-stepping dynamic obstacles if the robot may move.
#   - HOLO_BLK has a detailed reason in the log/debug.
#
# Conflict zone:
#   - Manual zones are active for crossing and merge from priority_manager_node.py.
#   - convoy and split do not use manual conflict zones.
#   - merge already has priority_stop_enabled:=false (see scenarios.yaml).

# ================================================================
# 6. QUICK MONITORING
# ================================================================

# Master PC domain 44:
ros2 topic echo /experiment_state
ros2 topic echo /robot1/vmax_consensus
ros2 topic echo /robot2/vmax_consensus
ros2 topic echo /robot3/vmax_consensus
ros2 topic echo /robot1/priority_stop
ros2 topic echo /robot2/priority_stop
ros2 topic echo /robot3/priority_stop
ros2 topic echo /conflict_zone_state
ros2 topic echo /conflict_zone_detail
ros2 topic echo /coordination_debug          # contains comm_graph for consensus_dist mode

# Each robot domain:
ros2 topic echo /robot1/cmd_vel
ros2 topic echo /robot1/dwa_vmax_eff
ros2 topic echo /robot1/dwa_speed_mag
ros2 topic echo /robot1/dwa_mode
ros2 topic echo /robot1/tracking_mode
ros2 topic echo /amcl_pose
ros2 topic echo /scan

# Parameter sanity check after robot launch:
ros2 param get /robot1/modified_dwa_node scenario
ros2 param get /robot1/modified_dwa_node avoidance_mode
ros2 param get /robot1/modified_dwa_node localization_guard_enabled

# PC parameter sanity check:
ros2 param get /consensus_node arrival_mode
ros2 param get /consensus_node coordination_mode
ros2 param get /priority_manager_node scenario


# ================================================================
# 7. RESULT LOGS
# ================================================================

ls ~/experiment_logs/
cat ~/experiment_logs/merge_cons_fault_01_*/experiment_summary.txt

# Key files:
#   experiment_summary.txt        default ON
#   goal_result.csv               default ON
#   pose_log.csv                  default ON
#   velocity_log.csv              default ON
#   consensus_log.csv             default ON
#   mission_log.csv               default ON
#   crosstrack_log.csv            default ON
#   interrobot_log.csv            default ON
#   fault_event_log.csv           default ON, fault_active edge
#   stop_event_log.csv            default ON, priority_stop edge
#   conflict_log.csv              default ON
#   path_log.csv                  default ON, global path
#   local_plan_log.csv            default ON, log_local_plan:=true
#   dynamic_obstacle_log.csv      default ON, log_dynamic_obstacle_debug:=true
#   conflict_detail_log.csv       default ON, log_conflict_detail:=true
#   path_debug_log.csv            default ON, log_path_debug:=true
#   dwa_mode_log.csv              default OFF, needs log_dwa_mode:=true
#   coordination_debug_log.csv    default OFF, needs log_coordination_debug:=true
#
# Additional metrics from analyze_experiments.py:
#   sigma_invalid_frac            fraction of AMCL samples with invalid sigma
#   loc_hold_frac                 fraction of samples with localization_hold_active
#   run_quality_score             1.0 clean, lower is worse
#   run_quality_ok                quick filter for whether a run is worth analyzing


# ================================================================
# 8. CORE TROUBLESHOOTING
# ================================================================

# Robot idle after START:
#   - Check /experiment_state on the robot must be RUNNING.
#   - Check the fault_injector log: fault_mode none must release after RUNNING.
#   - Check /robotX/cmd_vel and /robotX/cmd_vel_raw.
#   - Check the motor driver subscribes to /robotX/cmd_vel, not the global /cmd_vel.
#   - If localize mode is ON, check localization_hold_active (may HOLD due to AMCL sigma).

# Scenario appears mismatched:
#   - After CLI Load Scenario, check the scenario on the robot:
#       ros2 param get /robotX/modified_dwa_node scenario
#     or see the DWA log: [SCENARIO] convoy -> crossing.
#   - Check the scenario on the PC:
#       ros2 param get /priority_manager_node scenario
#       ros2 param get /consensus_node scenario
#   - Stop all old trial nodes before switching scenario.

# Path does not appear:
#   - Ensure AMCL has converged.
#   - Check /robotX/mission_remaining_length > 0.
#   - Check the global_path_node log: path found or Dijkstra failed.

# Fault event empty but velocity_log shows fault_active:
#   - Check /robot2/fault_active.
#   - Check robot2 was indeed launched with fault_mode:=timed_pulse (cons_fault only).
#   - The logger reads the fault_active edge, so also check fault_event_log.csv.

# Convoy robot stops too long near a peer:
#   - Check dwa_mode_log.csv: HOLO_BLK reason, DYNAV, CROSSING_EVADE, PEER_ESCAPE.
#   - Check dynamic_obstacle_log.csv: peer_blocks_path/front_blocked/holo_blk_reason.
#   - Check interrobot_log.csv for min_dist and signs of too-tight separation.
#   - [MOD-21] If it seems too timid, reduce peer_predict_horizon_s or footprint_margin_m.

# Crossing/merge stuck in the zone:
#   - Check /conflict_zone_detail for the GO/SLOW/YIELD/HOLD command.
#   - The owner must change after CLEARING and the gap_s elapses.
#   - If a non-owner stays DYNAV during HOLD, check tracking_mode and the robot scenario.

# Final orientation not clean:
#   - Check goal_result.csv heading_error / state_success.
#   - Check velocity_log.csv for FINAL_ALIGNING mode and omega.
#   - If AMCL is static near the goal, final-align must still follow the odom-yaw latch.

# Shutdown:
#   - CLI [6] EMERGENCY STOP first.
#   - Ctrl+C sync_monitor, experiment_master_cli, multi_robot_bringup.
#   - On the robot: Ctrl+C robot*_haqqi, AMCL, laser_bringup.
```

---

## Quick troubleshooting (symptom table)

For causes and detailed steps, see Section 8 (Core Troubleshooting) in the cheat-sheet above.

| Symptom | Possible cause | Quick action |
|---|---|---|
| Robot idle after START | `/experiment_state` not yet RUNNING, or HOLD due to localize guard | Check `/experiment_state`; check `localization_hold_active`; compare `/robotX/cmd_vel` vs `cmd_vel_raw` |
| Robot mislocalized / wrong position | Physical position not equal to `start_pose`, or map does not match the room | Place the robot exactly on the marking; verify the map file matches the room |
| Path does not appear | AMCL not converged / Dijkstra failed | Wait for AMCL to converge; check `mission_remaining_length>0`; check the `global_path_node` log |
| PC receives no robot data | Wrong IP / different subnet / IP changed by DHCP | Check `hostname -I` on each robot, override `robotN_ip:=`; ensure one subnet |
| Fault not recorded | robot2 not launched with `fault_mode:=timed_pulse` | Only `cons_fault` uses a fault; check `/robot2/fault_active` |
| Readiness Check red | pose stale / no path / `state` not READY | Fix the red items; do not START before all are green |
