# Consensus-Based Coordination Control for Three Mecanum Mobile Robots

Language: English | [Bahasa Indonesia](README.id.md)

This repository contains the software implementation and supporting material for the undergraduate thesis of Faiza Haqqi (Institut Teknologi Sepuluh Nopember). The system coordinates three mecanum wheeled robots (Yahboom RDK X5) on ROS 2 Humble to achieve simultaneous arrival, maintenance of a safe inter-robot distance, and resolution of trajectory conflicts, including under injected actuator faults.

## Method overview

The system is organized into functional layers:

- Trajectory-progress consensus: a distributed consensus protocol that aligns progress and estimated time of arrival (ETA) across robots through state exchange, minimizing arrival-time differences.
- Local navigation: a modified Dynamic Window Approach (DWA) variant for obstacle avoidance and trajectory tracking.
- ETA-based priority management: resolution of conflicts at trajectory crossing points through a hierarchical stop-and-go scheme.
- Fault injection and detection: deactivation of one robot's actuator at scheduled intervals to evaluate system robustness and recovery.

Test scenarios: split, convoy, merge, and crossing, each with and without consensus and with and without fault, plus convoy with an arrival offset.

## How to read this document

- Operating procedure: [docs/RUNBOOK.en.md](docs/RUNBOOK.en.md) (covers Ubuntu installation through program execution).
- Source code: start with `src/haqqi_ta/`.
- Reproducing the result figures: the Analysis section (requires MATLAB).

## Repository layout

```
.
├─ src/                    ROS 2 code (copied into the workspace at build time)
│  ├─ haqqi_ta/          This project's core package: nodes, launch, parameters, scripts
│  ├─ yahboomcar_multi/  Multi-robot launch and test-environment maps
│  └─ yahboomcar_rviz/   (optional) URDF and meshes for RViz visualization
├─ analysis/             MATLAB scripts that turn CSV logs into the thesis figures
├─ docs/                 Documentation: RUNBOOK (step-by-step operating guide)
└─ README.md             Language selector (full versions: README.id.md and README.en.md)
```

## Requirements

- Ubuntu 22.04 and ROS 2 Humble
- Python 3.10 (numpy, scipy, pyyaml) for the nodes and scripts
- MATLAB R2020a or newer for data analysis and figure generation (no extra toolboxes)
- 3x Yahboom RDK X5 (mecanum wheels). Hardware documentation and stock stack: https://www.yahboom.net/study/RDK-X5-Robot

A full experiment requires three physical robots together with Yahboom's stock stack (motor driver, IMU, EKF/robot_localization, LiDAR), which is not included in this repository and is available at the link above. There is no simulation mode yet. Without hardware, the code can still be reviewed and figures reproduced from existing data.


## Status, evidence, and limitations

This documentation is structured so that technical readers and external reviewers can assess system readiness directly:

- System status: a research prototype tested on three physical Yahboom RDK X5 robots.
- Available evidence: ROS 2 code, parameters, launch files, experiment scripts, the RUNBOOK, and MATLAB scripts for reproducing figures from CSV logs.
- Replication requirements: three Yahboom RDK X5 robots with Yahboom's stock packages, a stable local network, ROS 2 Humble, and MATLAB for result analysis.
- Current limitations: no simulation mode is provided, large raw datasets are not included in the ZIP, and IP/`ROS_DOMAIN_ID` settings must be adapted to the user's network.

This section is intentionally explicit because the repository is intended not only for running the code, but also for assessing whether the system is ready to be replicated, extended, or demonstrated further.

## Running (quick)

For installation from a bare system (no Ubuntu or ROS 2 yet), see [docs/RUNBOOK.en.md](docs/RUNBOOK.en.md).

```bash
mkdir -p ~/yahboomcar_ws/src && cp -r src/* ~/yahboomcar_ws/src/
cd ~/yahboomcar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch haqqi_ta multi_robot_bringup.launch.py
ros2 run haqqi_ta experiment_master_cli
```

Parameters are in `src/haqqi_ta/param/`, maps in `src/yahboomcar_multi/maps/`. This quick procedure uses a single workspace. The multi-machine configuration (master PC at `~/TA/yahboomcar_ws`, each robot at `~/yahboomcar_ws`) is described in RUNBOOK Section 1.

## Analysis and figure reproduction (MATLAB)

The `analysis/` folder contains MATLAB scripts that turn each run's CSV log files into the figures used in the thesis. These scripts require MATLAB R2020a or newer and need no extra toolboxes. The `docs/` folder contains the RUNBOOK, a step-by-step guide for operating the system.

Raw data is not included because it is large; distribute it via GitHub Release or Zenodo and link it in this section.

- `compare_scenarios.m`: cross-scenario comparison. `compare_scenarios('data fix')` produces figures 3.10, 4.8a–4.8f, and 4.43 in the `gambar_ta/` folder.
- `analyze_run.m`: single-run diagnostics. `analyze_run('4.convoy_cons_fault_01_...')` produces figures 01–08 and an optional video file.

Primary evaluation metrics: progress convergence time, inter-robot progress deviation, minimum inter-robot distance (0.30 m safety threshold), and the number of safety violations.

## Third-party packages

The `yahboomcar_multi` and `yahboomcar_rviz` packages are derived from Yahboom's stock packages for the RDK X5 (https://www.yahboom.net/study/RDK-X5-Robot); their source and version must be cited on publication. `yahboomcar_rviz` is for RViz visualization only (about 40 MB of meshes) and is not required to run the experiments.

The Yahboom stock stack running on each robot (motor driver, IMU, EKF, LiDAR) comes from the device's stock image and is documented on the Yahboom site above; it is not included in this repository.

## License

MIT (see LICENSE). Citation of the related thesis is expected when the code or data is used.
