# Sistem Kontrol Koordinasi Konsensus untuk Tiga Mobile Robot Mecanum

Bahasa: Indonesia | [English](README.en.md)

Repositori ini berisi implementasi perangkat lunak dan bahan pendukung Tugas Akhir Faiza Haqqi (Institut Teknologi Sepuluh Nopember). Sistem mengoordinasikan tiga robot beroda mecanum (Yahboom RDK X5) di atas ROS 2 Humble untuk mencapai kedatangan serempak, pemeliharaan jarak aman antarrobot, dan penyelesaian konflik lintasan, termasuk pada kondisi kegagalan aktuator yang diinjeksikan.

## Ringkasan metode

Sistem disusun dalam beberapa lapisan fungsi:

- Konsensus progres lintasan: protokol konsensus terdistribusi yang menyelaraskan progres dan estimasi waktu tiba (ETA) antarrobot melalui pertukaran state, sehingga selisih waktu kedatangan diminimalkan.
- Navigasi lokal: varian Dynamic Window Approach (DWA) termodifikasi untuk penghindaran halangan dan pelacakan lintasan.
- Manajemen prioritas berbasis ETA: resolusi konflik pada titik silang lintasan melalui skema stop-and-go hierarkis.
- Injeksi dan deteksi fault: penonaktifan aktuator satu robot pada interval terjadwal untuk mengevaluasi ketahanan dan pemulihan sistem.

Skenario pengujian: split, convoy, merge, dan crossing, masing-masing dengan dan tanpa konsensus serta dengan dan tanpa fault, ditambah convoy dengan offset kedatangan.

## Cara membaca dokumen ini

- Prosedur menjalankan: [docs/RUNBOOK.id.md](docs/RUNBOOK.id.md) (mencakup instalasi Ubuntu hingga eksekusi program).
- Kode sumber: mulai dari `src/haqqi_ta/`.
- Reproduksi gambar hasil: bagian Analisis (memerlukan MATLAB).

## Struktur repositori

```
.
├─ src/                    Kode ROS 2 (disalin ke workspace saat build)
│  ├─ haqqi_ta/          Package inti proyek ini: node, launch, parameter, scripts
│  ├─ yahboomcar_multi/  Launch multi-robot dan peta lingkungan uji
│  └─ yahboomcar_rviz/   (opsional) URDF dan mesh untuk visualisasi RViz
├─ analysis/             Skrip MATLAB yang mengubah CSV log menjadi gambar hasil TA
├─ docs/                 Dokumentasi: RUNBOOK (panduan menjalankan langkah demi langkah)
└─ README.md             Halaman pemilih bahasa (versi lengkap: README.id.md dan README.en.md)
```

## Prasyarat

- Ubuntu 22.04 dan ROS 2 Humble
- Python 3.10 (numpy, scipy, pyyaml) untuk node dan skrip
- MATLAB R2020a atau lebih baru untuk analisis data dan pembuatan gambar (tanpa toolbox tambahan)
- 3 unit Yahboom RDK X5 (roda mecanum). Dokumentasi perangkat keras dan stack bawaan: https://www.yahboom.net/study/RDK-X5-Robot

Eksperimen penuh membutuhkan tiga robot fisik beserta stack bawaan Yahboom (driver motor, IMU, EKF/robot_localization, LiDAR) yang tidak disertakan dalam repositori ini dan tersedia melalui tautan di atas. Mode simulasi belum tersedia. Tanpa perangkat keras, kode tetap dapat ditinjau dan gambar dapat direproduksi dari data yang sudah ada.


## Status, bukti, dan batasan

Dokumentasi ini disusun agar pembaca teknis maupun reviewer eksternal dapat menilai kesiapan sistem secara langsung:

- Status sistem: prototipe riset yang sudah diuji pada tiga robot fisik Yahboom RDK X5.
- Bukti yang tersedia: kode ROS 2, parameter, launch file, skrip eksperimen, RUNBOOK, serta skrip MATLAB untuk mereproduksi gambar dari log CSV.
- Kebutuhan replikasi: tiga robot Yahboom RDK X5 dengan package bawaan dari Yahboom, jaringan lokal yang stabil, ROS 2 Humble, dan MATLAB untuk analisis hasil.
- Batasan saat ini: mode simulasi belum disediakan, data mentah berukuran besar tidak disertakan di ZIP, dan konfigurasi IP/`ROS_DOMAIN_ID` perlu disesuaikan dengan jaringan pengguna.

Bagian ini sengaja ditulis eksplisit karena repositori tidak hanya ditujukan untuk menjalankan kode, tetapi juga untuk menilai apakah sistem siap direplikasi, dikembangkan, atau didemonstrasikan lebih lanjut.

## Menjalankan (ringkas)

Untuk instalasi dari sistem kosong (belum ada Ubuntu maupun ROS 2), lihat [docs/RUNBOOK.id.md](docs/RUNBOOK.id.md).

```bash
mkdir -p ~/yahboomcar_ws/src && cp -r src/* ~/yahboomcar_ws/src/
cd ~/yahboomcar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch haqqi_ta multi_robot_bringup.launch.py
ros2 run haqqi_ta experiment_master_cli
```

Parameter berada di `src/haqqi_ta/param/`, peta di `src/yahboomcar_multi/maps/`. Perintah ringkas ini menggunakan satu workspace. Konfigurasi multi-mesin (PC master di `~/TA/yahboomcar_ws`, tiap robot di `~/yahboomcar_ws`) dijelaskan pada RUNBOOK Bagian 1.

## Analisis dan reproduksi gambar (MATLAB)

Folder `analysis/` berisi skrip MATLAB yang mengubah berkas CSV log dari tiap run menjadi gambar hasil pada Tugas Akhir. Skrip ini memerlukan MATLAB R2020a atau lebih baru dan tidak membutuhkan toolbox tambahan. Folder `docs/` berisi RUNBOOK, yaitu panduan menjalankan sistem langkah demi langkah.

Data mentah tidak disertakan karena berukuran besar; distribusikan melalui GitHub Release atau Zenodo lalu tautkan pada bagian ini.

- `compare_scenarios.m`: perbandingan antar-skenario. Perintah `compare_scenarios('data fix')` menghasilkan Gambar 3.10, 4.8a–4.8f, dan 4.43 pada folder `gambar_ta/`.
- `analyze_run.m`: diagnostik satu run. Perintah `analyze_run('4.convoy_cons_fault_01_...')` menghasilkan gambar 01–08 dan berkas video opsional.

Metrik evaluasi utama: waktu konvergensi progres, deviasi progres antarrobot, jarak minimum antarrobot (ambang keselamatan 0,30 m), dan jumlah pelanggaran keselamatan.

## Catatan package pihak ketiga

Package `yahboomcar_multi` dan `yahboomcar_rviz` diturunkan dari package bawaan Yahboom untuk RDK X5 (https://www.yahboom.net/study/RDK-X5-Robot); sumber dan versinya wajib dicantumkan saat publikasi. `yahboomcar_rviz` hanya untuk visualisasi RViz (mesh sekitar 40 MB) dan tidak diperlukan untuk eksekusi eksperimen.

Stack bawaan Yahboom yang berjalan di tiap robot (driver motor, IMU, EKF, LiDAR) berasal dari image bawaan perangkat dan didokumentasikan pada situs Yahboom di atas; stack tersebut tidak termasuk dalam repositori ini.

## Lisensi

MIT (lihat LICENSE). Sitasi Tugas Akhir terkait diharapkan bila kode atau data digunakan.
