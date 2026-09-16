# POLEBOT Web Interface

Antarmuka web ROS 2 untuk pemetaan, navigasi, dan kontrol POLEBOT. Workspace Navigation menampilkan scene 3D, layer peta/LiDAR/path/costmap, kesiapan Nav2, dan status goal.

## Menjalankan

Dari workspace yang sudah dibangun dan environment ROS 2 yang sudah di-source:

```bash
source install/setup.bash
ros2 launch polebot_web_interface web_interface.launch.py
```

Buka [dashboard lokal](http://localhost:5050), lalu pilih Navigation. Gunakan **Ctrl+Shift+R** setelah perubahan frontend. Launch ini menjalankan backend dashboard (5050), server aset (8000), rosbridge (9090), dan rosapi; robot serta stack SLAM/Nav2 harus dijalankan sesuai konfigurasi sistem.

## Alur navigasi

1. Periksa koneksi, map, pose robot, dan kesiapan Nav2.
2. Pilih alat goal, klik-tahan untuk posisi, lalu seret untuk arah/yaw.
3. Lepaskan untuk melihat preview; sesuaikan heading bila perlu.
4. Tekan **Send** untuk mengirim goal. Pantau feedback dan gunakan Cancel untuk meminta pembatalan.

Initial pose menggunakan interaksi posisi dan arah yang sama. Kontrol opacity dan minimum visible cost hanya mengubah tampilan costmap; tidak mengubah parameter inflation Nav2. Kalibrasi viewer 3D dan LiDAR yang sudah ada dipertahankan.

## Pengujian dan batas validasi

Dari direktori paket ini, dengan Node.js tersedia:

```bash
node test/navigation_workspace.test.cjs
```

Sebanyak 11 pengujian frontend telah lulus dalam sesi implementasi menggunakan Node bawaan lingkungan pengembangan. Layout desktop/mobile diperiksa memakai fixture sintetis; fixture tersebut tidak disertakan sebagai fitur aplikasi. Integrasi robot/Nav2 nyata masih perlu divalidasi, terutama protokol action rosbridge, lifecycle, TF, costmap, eksekusi goal, dan pembatalan. Tampilan ini terinspirasi RViz/Foxglove dan tidak menyediakan seluruh fitur Foxglove.

Lihat [PRODUCT.md](PRODUCT.md) untuk kebutuhan dan batas fungsi, serta [DESIGN.md](DESIGN.md) untuk token dan pola UI. Metadata panel desain tersedia di [.impeccable/design.json](.impeccable/design.json).
