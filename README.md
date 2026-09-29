# 🤖 AMR-POLEBOT-WS

> **Official Autonomous Mobile Robot (AMR) Development Workspace & Mission Control Stack**  
> **Laboratorium Robotika & Otomasi — Politeknik Manufaktur Bandung (POLMAN Bandung)**

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-3498DB?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?style=flat-square&logo=ubuntu)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=flat-square)](LICENSE)
[![Build](https://img.shields.io/badge/Build-Colcon_Symlink-brightgreen?style=flat-square)](https://colcon.readthedocs.io/)
[![Hardware](https://img.shields.io/badge/Hardware-SocketCAN_500kbps-orange?style=flat-square)](https://www.kernel.org/doc/Documentation/networking/can.txt)

---

## 📋 Overview

**AMR-POLEBOT** adalah platform Autonomous Mobile Robot (AMR) kelas industri bertipe *differential drive* yang dirancang dan dikembangkan di **Politeknik Manufaktur Bandung**. Workspace ini mengintegrasikan seluruh tumpukan perangkat lunak (*software stack*) berbasis **ROS 2 Jazzy**, meliputi sistem lokalisasi probabilistik (AMCL), perencanaan navigasi cerdas (Nav2), arsitektur multi-mode kontrol motor (Native DWB, S-Curve PID, & Sliding Mode Controller), editor zona virtual berbasis web, hingga dashboard kendali misi 3D Three.js modern yang dapat diakses secara nirkabel dari berbagai perangkat.

| Spesifikasi Sistem | Keterangan / Parameter Teknis |
|--------------------|--------------------------------|
| **Sistem Operasi** | Ubuntu 24.04 LTS (Noble Numbat) |
| **Framework Robot** | ROS 2 Jazzy Jalisco |
| **Sistem Penggerak** | 2x Motor BLDC Direct Drive TongYi + 4x Caster Wheels (*Differential Drive*) |
| **Protokol Motor** | CANopen CiA 402 via SocketCAN (`can0` @ 500 kbps, node ID 10 & 11) |
| **Dimensi Kinematika** | Jarak Roda (*Wheel Base*) $W = 0.5473\text{ m}$, Radius Roda $R = 0.079\text{ m}$ ($158\text{ mm}$ tyre) |
| **Sensor Utama** | Autonics LSC Series 2D LiDAR (Ethernet UDP 192.168.0.1, jarak $25\text{ m}$, FoV $270^\circ$) |
| **Lokalisasi & SLAM** | AMCL + SLAM Toolbox (Koreksi scan-matching LiDAR real-time) |
| **Kecepatan Navigasi** | Linier $v_{\max} = 0.14\text{ m/s}$, Sudut $\omega_{\max} = 0.18\text{ rad/s}$ ($10.3^\circ/\text{s}$, akselerasi $0.35\text{ m/s}^2$) |
| **Mission Dashboard** | Web-based 3D Three.js GUI (Port `5050`) + Offline Mobile Joystick (Port `8000`) |

---

## 🌟 Fitur Utama & Inovasi Riset

### 1. 🧠 Modular Multi-Mode Motion Control
Dapat diganti secara instan (1-klik) dari Web Dashboard tanpa perlu me-restart tumpukan navigasi:
* **⚡ Native DWB (Standard OK — Default)**: Controller Nav2 DWB Local Planner standar industri dengan critic *RotateToGoal*, *PathAlign*, dan *GoalAlign* yang telah teruji stabil di lapangan.
* **📈 PID Profiled Pure Pursuit**: Controller berbasis profil gerakan S-Curve jerk-limited ($j_{\max} = 0.16\text{ m/s}^3$) dengan gain scheduling ($K_p = 3.20$ cruise, $K_p = 3.00$ deccel) dan kompensasi feedforward kelengkungan rute.
* **🎛️ Sliding Mode Controller (SMC)**: Controller nonlinier berbasis riset *Alipour et al. (2019)* dengan sliding surface polar error $S_1(\rho)$ & $S_2(\varphi)$, hyperbolic tangent boundary layer switching, dan parameter $\lambda_1 = 0.5, \lambda_2 = 1.5, K_1 = 2.0, K_2 = 10.0$.
* **🛡️ Standard OK (Drawback / Emergency Fallback)**: Tombol darurat yang dalam 1-klik seketika mematikan proses riset dan mengembalikan kontrol ke baseline Native DWB.

### 2. 🧭 Multi-Method Global Path Planning
* **Nav2 Navfn A\***: Global grid planner standar Nav2 berbasis costmap 2D berlapis.
* **BFS Grid Planner**: Algoritma Breadth-First Search wavefront grid 8-konektivitas deterministik.
* **A\* Euclidean Optimal**: Algoritma A* terbobot dengan fungsi penalti jarak rintangan (*distance-transform clearance field*).

### 3. 🖥️ Interactive Web Mission Control & Dashboard (Port `5050`)
* **3D Three.js Viewport**: Rendering model 3D STL robot, peta 2D Occupancy Grid, costmap global/lokal real-time, dan point cloud LiDAR.
* **Adobe-Style Zone Editor**: Alat interaktif untuk menggambar poligon zona larangan melintas (*Keepout Filter Masks*) dan zona pembatas kecepatan (*Speed Restriction Zones*) secara visual langsung di atas peta.
* **📍 RViz-Style Sequential Waypoint Route**:
  * Menempatkan rangkaian titik target rute secara berurutan (Point 1, 2, 3...) di peta 3D.
  * Dilengkapi nomor badge billboard melingkar yang selalu menghadap kamera dan garis rute sian neon.
  * Manajer daftar titik (koordinat $X, Y, \theta$, tombol hapus per-titik 🗑️, tombol Clear 🧹).
  * Eksekusi misi rute via action native `/navigate_through_poses` dengan fitur pembatalan instan (⏹️ Cancel).
* **Responsive Multi-Device Access**: Terbuka ke jaringan hotspot/LAN (`0.0.0.0`) sehingga operator dapat mengontrol robot dari laptop, tablet, maupun smartphone.

### 4. 🛡️ Behavior Tree Khusus Ruangan Sempit (*Confined Space Recovery*)
* Behavior Tree kustom (`polebot_obstacle_stop_and_backup.xml`) yang dirancang khusus untuk lingkungan lab/koridor sempit.
* Saat terhalang rintangan, robot hanya melakukan **1 kali jeda ($0.5\text{s}$) dan 1 kali mundur perlahan ($0.12\text{ m}$ pada $0.05\text{ m/s}$)** lalu langsung mengakhiri target navigasi untuk mencegah risiko menabrak dinding di belakangnya.

---

## 📦 Struktur Package

```
AMR-POLEBOT-WS/
├── src/
│   ├── polebot_bringup/          # Launch file bringup robot, CAN interface, & konfigurasi motor
│   ├── polebot_description/      # Model robot URDF, Xacro, mesh 3D STL, & geometri roda
│   ├── polebot_navigation/       # Konfigurasi Nav2 (nav2_params.yaml), Behavior Tree, & costmap filters
│   ├── polebot_slam/             # Konfigurasi pemetaan SLAM Toolbox 2D
│   ├── polebot_sensors/          # Driver sensor (Autonics LSC LiDAR Ethernet & USB Camera)
│   ├── polebot_web_interface/    # Dashboard Nav2 Web 3D (Python backend, Three.js, REST API, & ROSBridge)
│   ├── polebot_web_teleop/       # Aplikasi web joystick mobile teleop offline (Port 8000)
│   ├── polebot_research_control/ # Folder terisolasi modul riset (SMC, PID Profiled, BFS/A* planner)
│   ├── tongyi_canopen_driver/    # Driver C++ SocketCAN DS402 motor TongYi & odometri presisi
│   └── polebot_simulation/       # Dunia simulasi Gazebo & testing track
├── maps/                         # File peta static YAML/PGM, keepout mask, & speed mask
├── docker/                       # Dockerfiles untuk deployment terisolasi
├── scripts/                      # Skrip utilitas kalibrasi odometri & setup CAN
└── README.md
```

---

## 🚀 Panduan Memulai Cepat (Quick Start)

### 1. Prasyarat Sistem
Pastikan lingkungan Anda menggunakan **Ubuntu 24.04 LTS** dan **ROS 2 Jazzy**:
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop-full \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rosbridge-server \
  can-utils iproute2
```

### 2. Kompilasi Workspace
```bash
cd ~/Desktop/AMR-POLEBOT-WS
source /opt/ros/jazzy/setup.bash

# Kompilasi seluruh package dengan symlink
colcon build --symlink-install

# Source overlay workspace
source install/setup.bash
```

### 3. Menjalankan Web Mission Control
Jalankan satu perintah launch untuk mengaktifkan seluruh backend, rosbridge, dan antarmuka web:
```bash
ros2 launch polebot_web_interface web_interface.launch.py
```

Buka peramban (*browser*) di laptop robot atau perangkat lain di jaringan yang sama:
* **Dashboard Nav2 Utama**: `http://localhost:5050`
* **Mobile Joystick Teleop**: `http://localhost:8000`

> 💡 **Akses Nirkabel dari HP / Tablet Operator:**  
> Jika robot terhubung ke Hotspot Wi-Fi bersama, cari IP robot dengan `hostname -I` (contoh: `10.86.182.19`), lalu buka dari peramban HP:  
> **`http://10.86.182.19:5050`** (WebSocket ROSBridge port `9090` akan otomatis terhubung).

### 4. Menjalankan Motor & Navigasi dari Dashboard
1. Buka dashboard di `http://localhost:5050`.
2. Klik tombol **⚡ Start Motor** pada sidebar kiri (otomatis menginisialisasi bus CAN `can0` @ 500 kbps dan mengaktifkan driver TongYi dengan parameter kinematika terkalibrasi).
3. Pilih peta yang diinginkan (misal: `Lab_Robotik.yaml`), lalu klik **Load Map & Start Nav2**.
4. Berikan estimasi posisi awal (*2D Pose Estimate*) jika diperlukan.
5. Pilih mode target navigasi:
   * **🎯 Single Goal**: Klik & drag di peta untuk menetapkan tujuan tunggal.
   * **📍 Waypoint Route**: Klik tab *Route* atau pilih tool *📍 Waypoints* untuk membuat rute sekuensial multi-titik ala RViz, lalu klik **▶️ Start Route**.

---

## 👥 Tim Peneliti & Kontributor Proyek

Pengembangan AMR-POLEBOT didukung oleh kolaborasi tim dosen peneliti dan mahasiswa dari **Laboratorium Robotika & Otomasi, Jurusan Teknik Mekatronika — Politeknik Manufaktur Bandung (POLMAN Bandung)**:

### 🎓 Dosen Pembina & Peneliti Utama (*Faculty Advisors & Main Researchers*)

| No | Nama Peneliti / Dosen | Peran & Bidang Kepakaran | Afiliasi / Profil |
|:--:|-----------------------|--------------------------|-------------------|
| 1 | **Ismail, M.T.** | **Kepala Laboratorium Robotika & Otomasi (Ka.Lab)**<br>Pengarah Proyek & Sistem Mekatronika | Politeknik Manufaktur Bandung |
| 2 | **Andri Wiyono, M.T.** | **Dosen Peneliti Sistem Kontrol**<br>Arsitektur Penggerak & Diferensial AMR | Politeknik Manufaktur Bandung |
| 3 | **Siti Rodiah, M.T.** | **Dosen Peneliti Sistem Cerdas & Navigasi**<br>Algoritma Perencanaan Jalur & Lokalisasi | [@rdhst](https://github.com/rdhst) |
| 4 | **Nur Jamiludin Ramadhan, M.T.** | **Dosen Peneliti Sistem Robotika & Otomasi**<br>Integrasi Sensor, Firmware, & Sistem Kendali | [@nj-ramadhan](https://github.com/nj-ramadhan) |
| 5 | **Wahyu Caesarendra, Ph.D.** | **Senior Researcher & Scientific Advisor**<br>Autonomous Navigation & Intelligent Systems | [@WhyAC](https://github.com/WhyAC) |
| 6 | **Pipit Anggraeni, M.T.** | **Dosen Peneliti Mekatronika**<br>Instrumentasi & Pengujian Dinamika Robot | Politeknik Manufaktur Bandung |
| 7 | **Noval, M.T.** | **Dosen Peneliti Sistem Tertanam**<br>Hardware Komunikasi CAN Bus & Power Management | Politeknik Manufaktur Bandung |
| 8 | **Adhitya, M.T.** | **Dosen Peneliti Instrumentasi Robotika**<br>Sensor Perception, LiDAR Safety, & Kalibrasi | Politeknik Manufaktur Bandung |

### 🛠️ Tim Pengembang & Integrator Mahasiswa (*Engineering Developers*)

| Kontributor | Peran | Area Pengembangan Teknis |
|-------------|-------|--------------------------|
| **MiraeNK** | Lead Developer | Interfacing, Web Mission Control Dashboard, Navigation Tuning, & Odometry Calibration |
| **Iridnes** | Developer | Motor Control, Kinematics, & Diff Drive Integration |
| **RkZx** | Developer | SLAM 2D Mapping, Sensor Setup, & Simulation Validation |

---

## 📄 Lisensi

Proyek ini dilisensikan di bawah **Apache License 2.0** — silakan baca berkas [LICENSE](LICENSE) untuk ketentuan lengkapnya.

---

<div align="center">
  <strong>Laboratorium Robotika & Otomasi</strong><br>
  <strong>Politeknik Manufaktur Bandung (POLMAN Bandung)</strong><br>
  Jl. Kanayakan No. 21, Dago, Kecamatan Coblong, Kota Bandung, Jawa Barat 40135<br>
  <em>AMR-POLEBOT Autonomous Mobile Robot Project</em>
</div>
