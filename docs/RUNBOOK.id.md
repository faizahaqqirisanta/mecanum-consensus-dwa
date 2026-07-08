# Panduan Menjalankan (Runbook) — haqqi_ta

Bahasa: Indonesia | [English](RUNBOOK.en.md) | [Daftar bahasa](RUNBOOK.md)

Dokumen ini adalah panduan operasional lengkap untuk sistem koordinasi tiga robot Yahboom RDK X5 mecanum (ROS 2 Humble). Untuk ringkasan proyek, lihat [../README.id.md](../README.id.md). Dokumentasi perangkat keras resmi: https://www.yahboom.net/study/RDK-X5-Robot

Cakupan: setup, build, sinkronisasi tiga robot, startup, matriks 16 trial, monitoring, log, troubleshooting, serta posisi folder `analysis/` (skrip MATLAB) dan `docs/` (dokumentasi/runbook). Alamat IP dan `ROS_DOMAIN_ID` di bawah adalah contoh konfigurasi penulis; sesuaikan dengan jaringan Anda.

Dokumen ini bukan skrip yang dijalankan sekaligus. Blok perintah di bawah adalah rujukan yang dijalankan secara selektif dan terpisah pada beberapa terminal atau mesin berbeda (PC master dan Robot 1/2/3 sesuai `ROS_DOMAIN_ID`). Menjalankannya berurutan dari atas ke bawah akan gagal. Salin per bagian sesuai peran mesin dan tahap eksperimen.

## Cakupan dan pembaca

Panduan ini ditujukan bagi operator eksperimen (asisten atau mahasiswa laboratorium) yang menjalankan eksperimen pada tiga robot fisik, serta pembaca repositori yang ingin memahami cara kerja sistem. Folder `docs/` berisi dokumen panduan; folder `analysis/` berisi skrip MATLAB untuk mengubah log CSV menjadi gambar hasil. Baca daftar preflight sebelum mengoperasikan robot.

## Instalasi dari sistem kosong

Bagian ini ditujukan untuk instalasi dari sistem tanpa Ubuntu maupun ROS 2. Jika Ubuntu 22.04 dan ROS 2 Humble sudah terpasang, lanjut ke daftar preflight atau Bagian 1.

Prasyarat perangkat keras: eksperimen penuh membutuhkan tiga robot fisik Yahboom RDK X5 beserta stack dan package bawaannya (driver motor, IMU, EKF/`robot_localization`, LiDAR) yang tidak disertakan dalam repositori ini dan tersedia melalui dokumentasi resmi Yahboom: https://www.yahboom.net/study/RDK-X5-Robot. Mode simulasi belum tersedia; tanpa perangkat keras, kode hanya dapat ditinjau dan gambar direproduksi dari data. Langkah berikut menyiapkan satu mesin (PC master atau tiap robot).

**Langkah 1 — Pasang Ubuntu 22.04 LTS**
- Unduh ISO **Ubuntu 22.04 LTS** dari situs resmi Ubuntu.
- Buat USB bootable (mis. Rufus / BalenaEtcher) lalu install (bisa *dual-boot* dengan Windows).
- ROS 2 Humble **hanya didukung resmi di Ubuntu 22.04** — jangan pakai 20.04 / 24.04.

**Langkah 2 — Pasang ROS 2 Humble** *(ringkas; rujuk docs.ros.org bila repo/key berubah)*
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

**Langkah 3 — Environment, colcon, & rosdep**
```bash
source /opt/ros/humble/setup.bash          # jalankan tiap buka terminal (atau taruh di ~/.bashrc)
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null; rosdep update
pip3 install numpy scipy pyyaml            # dependency skrip Python
```

**Langkah 4 — Buat workspace & masukkan kode**
```bash
mkdir -p ~/yahboomcar_ws/src
cp -r /path/ke/repo-ini/src/* ~/yahboomcar_ws/src/
cd ~/yahboomcar_ws
```

**Langkah 5 — Pasang dependency package (rosdep) lalu build**
```bash
rosdep install --from-paths src --ignore-src -r -y   # pasang dependency ROS otomatis
colcon build --symlink-install
source install/setup.bash
```

**Langkah 6 — Uji cepat**
```bash
ros2 pkg executables haqqi_ta | head        # node harus terdaftar
ros2 run haqqi_ta experiment_master_cli     # buka CLI (fungsi penuh butuh node lain hidup)
```

Untuk konfigurasi tiga robot dan PC master (PC master di `~/TA/yahboomcar_ws`, tiap robot di `~/yahboomcar_ws`, sinkronisasi antar-mesin, dan `ROS_DOMAIN_ID`), lanjut ke Bagian 1 di bawah. Stack driver Yahboom harus sudah tersedia di tiap robot dari image bawaannya.

## Daftar preflight (wajib sebelum menjalankan)

Verifikasi seluruh item sebelum `START`. Sebagian besar kegagalan eksperimen berasal dari item Tier 1 dan Tier 2.

Tier 1 — Lingkungan (kesalahan di sini menyebabkan robot salah posisi):
- [ ] Peta sesuai ruangan. Launch robot memakai `yahboom_map_lss_carto.yaml`. Pastikan peta ini identik dengan ruangan pengujian saat ini. Bila ruangan berbeda atau berubah, buat peta baru lalu perbarui nama berkas peta pada `robotN_haqqi.launch.py` (variabel `map_file`).
- [ ] Koordinat skenario sesuai peta. `start_pose` dan `waypoints` pada `scenarios.yaml` diukur untuk ruangan penulis. Bila origin peta berbeda, sesuaikan koordinatnya.
- [ ] IP dan `ROS_DOMAIN_ID` benar. PC `192.168.0.34` (domain 44), Robot1/2/3 `.91/.88/.82` (domain 40/41/42). Bila jaringan memakai DHCP, tetapkan IP statik atau periksa `hostname -I` pada tiap robot lalu override `robotN_ip:=<ip>` saat launch PC. Seluruh mesin harus berada pada satu subnet.
- [ ] Robot ditempatkan tepat pada titik awal. CLI mem-publish `initialpose` secara otomatis dari `start_pose`; bila posisi fisik tidak sama dengan `start_pose`, estimasi AMCL keliru dan robot salah posisi. Tandai titik awal tiap robot pada lantai.

Tier 2 — Keselamatan:
- [ ] Uji E-STOP sebelum robot bergerak (CLI [6] EMERGENCY STOP). Pastikan seluruh operator memahami cara memicunya.
- [ ] Area bebas hambatan; jaga jarak dari robot yang bergerak (kecepatan hingga `0.50 m/s`).
- [ ] `localization_guard_enabled=true` (default) dibiarkan aktif; robot akan menahan gerak (HOLD) bila lokalisasi tidak andal.
- [ ] Pahami injeksi fault: pada jenis `cons_fault`, robot2 dihentikan secara sengaja sekitar t = 15 s. Ini bagian dari eksperimen, bukan kerusakan.

Tier 3 — Kesiapan sistem (melalui menu CLI):
- [ ] Environment ter-`source` dengan benar di tiap mesin (ROS 2 Humble dan workspace).
- [ ] Urutan startup: map, AMCL, DWA, UDP (lihat Bagian 1 dan 2).
- [ ] [3] Readiness Check seluruhnya terpenuhi (scenario dimuat, `state=READY`, pose AMCL tiap robot mutakhir, path terbentuk) sebelum [4] START.
- [ ] Tetapkan `experiment_name:=<nama>` agar folder log mudah ditelusuri (output diberi timestamp otomatis: `nama_YYYYMMDD_HHMMSS`).
- [ ] Antar trial: [6] EMERGENCY STOP lalu [7] Reset Trial, dan tutup `multi_robot_bringup` agar log tidak tercampur.

## Glosarium arsitektur

| Layer | Node | Fungsi singkat |
|---|---|---|
| L2 | `global_path_node` | Perencanaan jalur global (Dijkstra) → `/{robot}/plan` |
| L3 | `modified_dwa_node` | Local planner (Modified DWA), keluaran `cmd_vel_raw` |
| L4 | `consensus_node` | Average consensus: samakan waktu tiba antar robot |
| L5 | `priority_manager_node` | Stop-and-go hierarkis di zona konflik |
| — | `fault_injector_node` | Injeksi fault perangkat lunak (uji ketahanan) |
| — | `experiment_logger_node` | Rekam semua metrik ke CSV |
| — | `udp_*` / `sync_monitor_node` | Jembatan lintas-domain & monitor sinkronisasi |

## Peta kode (untuk pengembangan lanjutan)

Urutan baca yang disarankan di `src/haqqi_ta/haqqi_ta/`:
1. **`experiment_master_cli.py`** — orkestrator; dari sini semua trial dikendalikan (Load → Readiness → START → Reset). Titik masuk paling enak untuk memahami alur.
2. **`global_path_node.py`** (L2) → **`modified_dwa_node.py`** (L3) — dari rencana global ke perintah gerak.
3. **`consensus_node.py`** (L4) — inti kontribusi: penyelarasan progres / waktu tiba.
4. **`priority_manager_node.py`** (L5) — resolusi konflik persimpangan.
5. **`fault_injector_node.py`** + **`experiment_logger_node.py`** — uji ketahanan & perekaman metrik.
6. **`udp_*`** / **`sync_monitor_node.py`** — komunikasi lintas-domain antar mesin (jarang perlu diubah).

Di mana mengubah sesuatu:
- **Perilaku skenario / waypoint** → `param/scenarios.yaml`.
- **Tuning DWA per robot** → `param/dwa_robot{1,2,3}_params.yaml`.
- **Parameter konsensus** → `param/consensus_params.yaml`.
- **Menambah mode koordinasi** → tambahkan logika di `consensus_node.py` (daftar mode ada di cheat-sheet Bagian 4) lalu daftarkan namanya.
- **Argumen launch** → `launch/robot{1,2,3}_haqqi.launch.py` (per robot) & `launch/multi_robot_bringup.launch.py` (PC).

Aman diubah: parameter YAML dan argumen launch. Perlu kehati-hatian: kontrak topik dan format pesan UDP antar node; perubahan pada satu sisi mengharuskan sisi lain ikut menyesuaikan.

```bash
# TUTORIAL RUN PROGRAM - haqqi_ta
# Multi-Agent Coordination Control: 3 Robot Yahboom RDK X5 Mecanum
# ================================================================

# ================================================================
# 0. KONFIGURASI TETAP
# ================================================================
#
# ROS_DOMAIN_ID:
#   PC Master : 44
#   Robot1    : 40
#   Robot2    : 41
#   Robot3    : 42
#
# IP:
#   PC Master : 192.168.0.34
#   Robot1    : 192.168.0.91
#   Robot2    : 192.168.0.88
#   Robot3    : 192.168.0.82
#
# Workspace:
#   PC Master : ~/TA/yahboomcar_ws
#   Robot     : ~/yahboomcar_ws
#   Staging   : ~/TA/yahboomcar_ws1/2/3
#
# Catatan:
#   - Launch dari PC hanya dari ~/TA/yahboomcar_ws.
#   - yahboomcar_ws1/2/3 hanya source staging untuk disync ke robot fisik.
#   - Scenario robot dikirim saat [1] Load Scenario di experiment_master_cli.
#   - PC multi_robot_bringup tetap diberi scenario:=... untuk consensus/priority/logger.


# ================================================================
# 1. SETUP DAN BUILD
# ================================================================

# PC Master:
export ROS_DOMAIN_ID=44
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/TA/yahboomcar_ws/install/setup.bash

# Build PC Master setelah ada perubahan kode PC/source utama:
cd ~/TA/yahboomcar_ws
colcon build --symlink-install --packages-select haqqi_ta
source install/setup.bash

# Robot fisik:
#   Robot1: export ROS_DOMAIN_ID=40
#   Robot2: export ROS_DOMAIN_ID=41
#   Robot3: export ROS_DOMAIN_ID=42
#   source /opt/ros/humble/setup.bash
#   source ~/yahboomcar_ws/install/setup.bash

# Sync source ke staging robot dari PC source utama:
for ws in yahboomcar_ws1 yahboomcar_ws2 yahboomcar_ws3; do
  rsync -a --delete --exclude='__pycache__/' ~/TA/yahboomcar_ws/src/haqqi_ta/         ~/TA/${ws}/src/haqqi_ta/
  rsync -a --delete --exclude='__pycache__/' ~/TA/yahboomcar_ws/src/yahboomcar_multi/ ~/TA/${ws}/src/yahboomcar_multi/
done

# Sync staging ke robot fisik:
rsync -a ~/TA/yahboomcar_ws1/src/haqqi_ta/         sunrise@192.168.0.91:~/yahboomcar_ws/src/haqqi_ta/
rsync -a ~/TA/yahboomcar_ws1/src/yahboomcar_multi/ sunrise@192.168.0.91:~/yahboomcar_ws/src/yahboomcar_multi/
rsync -a ~/TA/yahboomcar_ws2/src/haqqi_ta/         sunrise@192.168.0.88:~/yahboomcar_ws/src/haqqi_ta/
rsync -a ~/TA/yahboomcar_ws2/src/yahboomcar_multi/ sunrise@192.168.0.88:~/yahboomcar_ws/src/yahboomcar_multi/
rsync -a ~/TA/yahboomcar_ws3/src/haqqi_ta/         sunrise@192.168.0.82:~/yahboomcar_ws/src/haqqi_ta/
rsync -a ~/TA/yahboomcar_ws3/src/yahboomcar_multi/ sunrise@192.168.0.82:~/yahboomcar_ws/src/yahboomcar_multi/

# Build di robot fisik setelah sync kode/launch/yaml:
cd ~/yahboomcar_ws
colcon build --symlink-install --packages-select haqqi_ta
source install/setup.bash

# Penting:
#   - File .py efektif cepat dengan symlink-install.
#   - YAML/launch di install/share adalah copy; rebuild robot atau copy manual ke install/share.
#   - Staging ws1/ws2/ws3 tidak perlu dibuild; build hanya PC workspace dan robot fisik.


# ================================================================
# 2. STARTUP ROBOT
# ================================================================
#
# ---------- RViz monitor 3 robot ----------
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

# Jalankan dari PC domain 44. Tampilan ini memakai telemetry yang direpublish
# consensus_node di PC: /robot*/amcl_pose, /robot*/plan, /robot*/waypoints.
# Launch ini juga menjalankan map_server PC agar /map tersedia di domain 44.
export ROS_DOMAIN_ID=44
source ~/TA/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_rviz pc_multi_robot_monitor.launch.py
# Jika hanya ingin membuka RViz tanpa map_server PC:
# ros2 launch yahboomcar_rviz pc_multi_robot_monitor.launch.py use_map_server:=false

# RViz per-robot lama masih bisa dipakai untuk debug scan/TF mentah.
# Gunakan ROS_DOMAIN_ID=40/41/42 dan file robot1_nav.rviz/robot2_nav.rviz/robot3_nav.rviz.

# Jalankan 3 terminal di tiap robot.
# Parameter default yang sudah tertanam tidak perlu ditulis ulang:
#   pc_master_ip=192.168.0.34, avoidance_mode=costmap, fault_mode=none.
#
# Scenario tidak perlu diisi di robot launch. Saat CLI Load Scenario, PC
# mengirim /experiment_scenario via UDP dan modified_dwa_node di robot update
# scenario aktifnya.
#
# Variasi yang perlu dibedakan di robot hanya:
#   - localize mode  : localization_guard_enabled (lihat INPUT di Terminal 3)
#   - fault          : hanya untuk jenis 'cons_fault', robot2 timed_pulse

# ================================================================
# TERMINAL 1 — Laser bringup (tiap robot)
# ================================================================
ros2 launch yahboomcar_multi laser_bringup_multi.launch.xml robot_name:=robot1
ros2 launch yahboomcar_multi laser_bringup_multi.launch.xml robot_name:=robot2
ros2 launch yahboomcar_multi laser_bringup_multi.launch.xml robot_name:=robot3

# ================================================================
# TERMINAL 2 — AMCL (tiap robot)
# ================================================================
ros2 launch yahboomcar_multi robot1_amcl_launch.py
ros2 launch yahboomcar_multi robot2_amcl_launch.py
ros2 launch yahboomcar_multi robot3_amcl_launch.py

# ================================================================
# TERMINAL 3 — Robot bringup (haqqi_ta)
#   localize guard: set nilai sama untuk ketiga robot (true=ON / false=OFF)
#   fault: HANYA robot target yang diberi fault_mode (untuk cons_fault)
# ================================================================

# ---- cons_nofault / baseline / offset_arrival : SEMUA robot tanpa fault ----
ros2 launch haqqi_ta robot1_haqqi.launch.py localization_guard_enabled:=false
ros2 launch haqqi_ta robot2_haqqi.launch.py localization_guard_enabled:=false
ros2 launch haqqi_ta robot3_haqqi.launch.py localization_guard_enabled:=false

# ---- cons_fault : fault HANYA di robot target (contoh robot1) ----
ros2 launch haqqi_ta robot3_haqqi.launch.py \
  fault_mode:=timed_pulse \
  fault_target_robot:=robot3 \
  localization_guard_enabled:=false
ros2 launch haqqi_ta robot2_haqqi.launch.py localization_guard_enabled:=false
ros2 launch haqqi_ta robot3_haqqi.launch.py localization_guard_enabled:=false
#   Default timed_pulse: start=15s, duration=2s, repeat=3, interval=10s.
#   Variasi: tambah fault_start_s:=20.0 fault_duration_s:=3.0 dst.

# ================================================================
# 3. STARTUP PC MASTER
# ================================================================

# Terminal PC 1: CLI eksperimen.
ros2 run haqqi_ta experiment_master_cli
#
# Menu utama:
#   [1] Load Scenario          [2] Preview Scenario
#   [3] Readiness Check        [4] START Trial
#   [5] Monitor Live           [6] EMERGENCY STOP
#   [7] Reset Trial            [8] Manual Goal / Waypoints
#   [9] Fault Injection        [0] Keluar
#
# Alur tiap trial:
#   1. Pilih variasi robot2 sesuai jenis (fault hanya untuk cons_fault).
#   2. Start multi_robot_bringup di PC sesuai trial (Bagian 4).
#   3. Di experiment_master_cli: [1] Load Scenario yang sama dengan PC launch.
#      CLI akan mengirim initialpose, waypoints, final yaw, dan scenario ke robot.
#   4. [3] Readiness Check sampai pose/path/state OK.
#   5. [4] START Trial, tunggu selesai.
#   6. [6] EMERGENCY STOP, lalu [7] Reset Trial sebelum pindah trial.
#   7. Tutup multi_robot_bringup sebelum pindah trial agar log/parameter tidak campur.

# Terminal PC 2: monitor ringkas.
ros2 run haqqi_ta sync_monitor_node

# ================================================================
# 4. MATRIKS PERCOBAAN  (4 JENIS x 4 BENTUK LINTASAN = 16 trial)
# ================================================================
#
#   JENIS percobaan:
#     - baseline        : koordinasi L4 OFF (l4_sync_enabled:=false). Robot jalan
#                         mandiri tanpa consensus. TANPA fault.
#     - cons_nofault    : consensus ON (coordination_mode:=$COORD_MODE). TANPA fault.
#     - cons_fault      : consensus ON + fault robot2 (timed_pulse, dari robot launch).
#     - offset_arrival  : consensus ON + arrival offset relatif antar robot.
#
#   BENTUK lintasan: convoy | crossing | merge | split
#
# ----------------------------------------------------------------
# >>> INPUT PERCOBAAN (atur sekali di PC Master sebelum trial) <<<
# ----------------------------------------------------------------
#
# Pilih MODE CONSENSUS (dipakai cons_nofault / cons_fault / offset_arrival):
#   consensus | consensus_offset | consensus_so | consensus_seg | consensus_so_seg |
#   consensus_dist | consensus_ft | consensus_ftso | consensus_fxt |
#   time_consensus | time_offset_consensus | arrival_offset_consensus
export COORD_MODE=consensus

# Arrival offset (detik) untuk jenis offset_arrival:
export OFF_R1=0.0
export OFF_R2=10.0
export OFF_R3=20.0

# Output & IP robot (umumnya tetap):
export OUT=/home/$(whoami)/experiment_logs
export R1=192.168.0.91 ; export R2=192.168.0.88 ; export R3=192.168.0.82

# Argumen umum semua trial (logging penuh + IP). Hapus dua log_* utk run ringan.
export COMMON="output_dir:=${OUT} log_dwa_mode:=true log_coordination_debug:=true robot1_ip:=${R1} robot2_ip:=${R2} robot3_ip:=${R3}"
# split butuh timeout lebih panjang:
export SPLIT_EXTRA="trial_timeout_s:=90.0"
#
# Catatan localize mode: diatur di robot (Terminal 3) via LOCALIZE_GUARD.
# Catatan fault: untuk cons_fault, robot2 HARUS dilaunch dengan fault_mode:=timed_pulse.


# ---------------- JENIS 1: baseline (L4 OFF, tanpa fault) ----------------
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_baseline_01   l4_sync_enabled:=false ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_baseline_01 l4_sync_enabled:=false ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_baseline_01    l4_sync_enabled:=false ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_baseline_01    l4_sync_enabled:=false ${COMMON} ${SPLIT_EXTRA}


# ---------------- JENIS 2: cons_nofault (consensus ON, tanpa fault) ----------------
# Robot2 dilaunch TANPA fault.
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_cons_nofault_01   coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_cons_nofault_01 coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_cons_nofault_01    coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_cons_nofault_01    coordination_mode:=${COORD_MODE} ${COMMON} ${SPLIT_EXTRA}


# ---------------- JENIS 3: cons_fault (consensus ON, fault robot2) ----------------
# Robot2 dilaunch DENGAN fault_mode:=timed_pulse (lihat Terminal 3).
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_cons_fault_01   coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_cons_fault_01 coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_cons_fault_01    coordination_mode:=${COORD_MODE} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_cons_fault_01    coordination_mode:=${COORD_MODE} ${COMMON} ${SPLIT_EXTRA}


# ---------------- JENIS 4: offset_arrival (consensus ON + arrival offset) ----------------
# Offset relatif: robot dgn offset lebih besar cenderung diperlambat. Robot2 TANPA fault.
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=convoy   experiment_name:=convoy_offset_01   coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=crossing experiment_name:=crossing_offset_01 coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=merge    experiment_name:=merge_offset_01    coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON}
ros2 launch haqqi_ta multi_robot_bringup.launch.py scenario:=split    experiment_name:=split_offset_01    coordination_mode:=${COORD_MODE} arrival_offset_robot1:=${OFF_R1} arrival_offset_robot2:=${OFF_R2} arrival_offset_robot3:=${OFF_R3} ${COMMON} ${SPLIT_EXTRA}


# ================================================================
# 5. CATATAN PROGRAM SAAT INI
# ================================================================
#
# Consensus:
#   - coordination_mode dipilih lewat INPUT $COORD_MODE (Bagian 4).
#   - baseline memakai l4_sync_enabled:=false (consensus dimatikan total).
#   - Jika arrival_offset_robot* kosong, arrival_schedule YAML skenario aktif
#     dibaca sebagai offset relatif. time_offset_consensus hanya alias eksplisit.
#   - Untuk offset_arrival, isi minimal satu arrival_offset_robot* nonzero.
#   - Rumus time-consensus:
#       t_i = mission_remaining_i / v_est_i
#       q_i = t_i - arrival_offset_i
#       v_i = clamp(v_nominal + k_time * (q_i - q_bar))
#   - Pada crossing/split, arrival_schedule memang beda per robot; grafik vcons
#     wajar tidak identik karena offset YAML ikut dipakai.
#   - Priority layer tidak mengunci baseline ke v_nominal saat tidak ada konflik:
#     vmax_priority_ceiling=0.50 memberi ruang catch-up consensus sampai batas DWA.
#
# Localize mode:
#   - Diatur per-robot via localization_guard_enabled (INPUT $LOCALIZE_GUARD).
#   - true  = robot HOLD jika AMCL sigma invalid / pose stale (aman, konservatif).
#   - false = guard mati; robot tetap jalan walau lokalisasi meragukan.
#   - Cek fraksi loc_hold lewat metrik loc_hold_frac / sigma_invalid_frac.
#
# Fault:
#   - Hanya jenis cons_fault yang memakai fault; robot2 timed_pulse.
#   - Fault dikonfigurasi di robot2 launch, bukan di multi_robot_bringup PC.
#   - fault_mode:=none tetap menahan cmd_vel sampai START, lalu relay normal.
#
# Final orientation:
#   - Convoy dan merge memakai final_orientation_mode: face_gathering_point.
#   - Yaw akhir dihitung saat Load Scenario oleh experiment_master_cli.
#   - Saat final alignment, DWA memakai latch odom-yaw sebagai referensi rotasi akhir.
#
# DWA/local planner:
#   - Scenario DWA robot diupdate dari CLI melalui /experiment_scenario.
#   - [MOD-21] Penghindaran peer memakai prediksi gerakan (cmd_vel UDP) + footprint
#     OBB ber-heading. Knob: footprint_margin_m, peer_predict_horizon_s, peer_predict_enabled.
#   - DYNAV/DYN_AVOID aktif untuk side-step obstacle dinamis jika robot boleh jalan.
#   - HOLO_BLK punya reason detail di log/debug.
#
# Conflict zone:
#   - Manual zones aktif untuk crossing dan merge dari priority_manager_node.py.
#   - convoy dan split tidak memakai conflict zone manual.
#   - merge sudah priority_stop_enabled:=false (lihat scenarios.yaml).

# ================================================================
# 6. MONITORING CEPAT
# ================================================================

# PC Master domain 44:
ros2 topic echo /experiment_state
ros2 topic echo /robot1/vmax_consensus
ros2 topic echo /robot2/vmax_consensus
ros2 topic echo /robot3/vmax_consensus
ros2 topic echo /robot1/priority_stop
ros2 topic echo /robot2/priority_stop
ros2 topic echo /robot3/priority_stop
ros2 topic echo /conflict_zone_state
ros2 topic echo /conflict_zone_detail
ros2 topic echo /coordination_debug          # berisi comm_graph utk mode consensus_dist

# Robot domain masing-masing:
ros2 topic echo /robot1/cmd_vel
ros2 topic echo /robot1/dwa_vmax_eff
ros2 topic echo /robot1/dwa_speed_mag
ros2 topic echo /robot1/dwa_mode
ros2 topic echo /robot1/tracking_mode
ros2 topic echo /amcl_pose
ros2 topic echo /scan

# Parameter sanity check setelah launch robot:
ros2 param get /robot1/modified_dwa_node scenario
ros2 param get /robot1/modified_dwa_node avoidance_mode
ros2 param get /robot1/modified_dwa_node localization_guard_enabled

# Parameter sanity check PC:
ros2 param get /consensus_node arrival_mode
ros2 param get /consensus_node coordination_mode
ros2 param get /priority_manager_node scenario


# ================================================================
# 7. LOG HASIL
# ================================================================

ls ~/experiment_logs/
cat ~/experiment_logs/merge_cons_fault_01_*/experiment_summary.txt

# File penting:
#   experiment_summary.txt        default ON
#   goal_result.csv               default ON
#   pose_log.csv                  default ON
#   velocity_log.csv              default ON
#   consensus_log.csv             default ON
#   mission_log.csv               default ON
#   crosstrack_log.csv            default ON
#   interrobot_log.csv            default ON
#   fault_event_log.csv           default ON, edge fault_active
#   stop_event_log.csv            default ON, edge priority_stop
#   conflict_log.csv              default ON
#   path_log.csv                  default ON, global path
#   local_plan_log.csv            default ON, log_local_plan:=true
#   dynamic_obstacle_log.csv      default ON, log_dynamic_obstacle_debug:=true
#   conflict_detail_log.csv       default ON, log_conflict_detail:=true
#   path_debug_log.csv            default ON, log_path_debug:=true
#   dwa_mode_log.csv              default OFF, butuh log_dwa_mode:=true
#   coordination_debug_log.csv    default OFF, butuh log_coordination_debug:=true
#
# Metrik tambahan dari analyze_experiments.py:
#   sigma_invalid_frac            fraksi sampel AMCL sigma tidak valid
#   loc_hold_frac                 fraksi sampel localization_hold_active
#   run_quality_score             1.0 bersih, makin kecil makin buruk
#   run_quality_ok                filter cepat run layak dianalisis atau tidak


# ================================================================
# 8. TROUBLESHOOTING INTI
# ================================================================

# Robot diam setelah START:
#   - Cek /experiment_state di robot harus RUNNING.
#   - Cek fault_injector log: fault_mode none harus release setelah RUNNING.
#   - Cek /robotX/cmd_vel dan /robotX/cmd_vel_raw.
#   - Cek driver motor subscribe ke /robotX/cmd_vel, bukan /cmd_vel global.
#   - Jika localize mode ON, cek localization_hold_active (mungkin HOLD krn AMCL sigma).

# Scenario terlihat tidak cocok:
#   - Setelah CLI Load Scenario, cek scenario di robot:
#       ros2 param get /robotX/modified_dwa_node scenario
#     atau lihat log DWA: [SCENARIO] convoy -> crossing.
#   - Cek scenario di PC:
#       ros2 param get /priority_manager_node scenario
#       ros2 param get /consensus_node scenario
#   - Stop semua node trial lama sebelum pindah scenario.

# Path tidak muncul:
#   - Pastikan AMCL sudah konvergen.
#   - Cek /robotX/mission_remaining_length > 0.
#   - Cek global_path_node log: path ditemukan atau Dijkstra gagal.

# Fault event kosong tapi velocity_log menunjukkan fault_active:
#   - Cek /robot2/fault_active.
#   - Cek robot2 memang launch dengan fault_mode:=timed_pulse (hanya cons_fault).
#   - Logger membaca edge fault_active, jadi cek juga fault_event_log.csv.

# Convoy robot berhenti terlalu lama dekat peer:
#   - Cek dwa_mode_log.csv: HOLO_BLK reason, DYNAV, CROSSING_EVADE, PEER_ESCAPE.
#   - Cek dynamic_obstacle_log.csv: peer_blocks_path/front_blocked/holo_blk_reason.
#   - Cek interrobot_log.csv untuk min_dist dan indikasi separasi terlalu rapat.
#   - [MOD-21] Jika terasa terlalu penakut, kecilkan peer_predict_horizon_s atau footprint_margin_m.

# Crossing/merge macet di zona:
#   - Cek /conflict_zone_detail untuk cmd GO/SLOW/YIELD/HOLD.
#   - Owner harus berganti setelah CLEARING dan gap_s selesai.
#   - Jika non-owner tetap DYNAV saat HOLD, cek tracking_mode dan scenario robot.

# Final orientation tidak rapi:
#   - Cek goal_result.csv heading_error / state_success.
#   - Cek velocity_log.csv mode FINAL_ALIGNING dan omega.
#   - Jika AMCL diam dekat goal, final-align tetap harus mengikuti odom-yaw latch.

# Shutdown:
#   - CLI [6] EMERGENCY STOP dulu.
#   - Ctrl+C sync_monitor, experiment_master_cli, multi_robot_bringup.
#   - Di robot: Ctrl+C robot*_haqqi, AMCL, laser_bringup.
```

---

## Troubleshooting cepat (tabel gejala)

Untuk penyebab dan langkah rinci, lihat Bagian 8 (Troubleshooting Inti) pada cheat-sheet di atas.

| Gejala | Kemungkinan sebab | Aksi cepat |
|---|---|---|
| Robot diam setelah START | `/experiment_state` belum RUNNING, atau HOLD karena localize guard | Cek `/experiment_state`; cek `localization_hold_active`; bandingkan `/robotX/cmd_vel` vs `cmd_vel_raw` |
| Robot nyasar / posisi ngawur | Posisi fisik ≠ `start_pose`, atau peta tidak cocok ruangan | Letakkan robot tepat di marking; verifikasi file peta = ruangan |
| Path tidak muncul | AMCL belum konvergen / Dijkstra gagal | Tunggu AMCL konvergen; cek `mission_remaining_length>0`; cek log `global_path_node` |
| PC tidak menerima data robot | IP salah / beda subnet / IP berubah karena DHCP | Cek `hostname -I` tiap robot, override `robotN_ip:=`; pastikan satu subnet |
| Fault tak terekam | robot2 tidak dilaunch `fault_mode:=timed_pulse` | Hanya `cons_fault` yang pakai fault; cek `/robot2/fault_active` |
| Readiness Check merah | pose stale / path belum ada / `state` ≠ READY | Perbaiki item yang merah; jangan START sebelum semua hijau |
