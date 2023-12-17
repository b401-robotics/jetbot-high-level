# Dokumentasi

Ada 2 package yaitu package Detection dan Middleware

## Detection

Hal yang berhubungan deteksi dan membutuhkan OpenCV ada disini

### [icp.cpp](../src/detection/src/icp.cpp)

- Deteksi daerah sekitar menggunakan lidar 1 channel
- Transformasi point cloud lidar menjadi point cloud terhadap lapangan
- Penggambaran lapangan dan menjadikannya point cloud referensi
- Algoritma ICP sampai jadi delta Estimasti terhadap Odometry

### [qr.cpp](../src/detection/src/qr.cpp)

- Menerima data dari kamera
- Deteksi QR dan hasilnya dicocokkan dengan kode yang sudah ditentukan

## Middleware

### [pid.h](../src/middleware/include/middleware/pid.h)

Ini adalah header yang berisi tentang sistem kontrol PID yang mengunakan limit saturasi output

### [simple_fsm.h](../src/middleware/include/middleware/simple_fsm.h)

Ini adalah header yang berisi tentang Finite State Machine lengkap dengan timeout dan reentry nya

### [gyro.cpp](../src/middleware/src/gyro.cpp)

Ini adalah interface untuk gyro MPU6050 yang diambil data gyro yaw nya. Terdapat juga sistem auto kalibrasi gyro

### [keyboard_input.cpp](../src/middleware/src/keyboard_input.cpp)

Ini adalah program untuk menangkap input pada keyboard berbasis termios dan ioctl

### [motor.cpp](../src/middleware/src/motor.cpp)

Ini adalah program yang akan mengkonversi kecepatan lokal robot x,y,yaw menjadi kecepatan masing-masing roda.

### [high_level_control.cpp](../src/middleware/src/high_level_control.cpp)

- Odometry
- Motion untuk bergerak ke titik tujuan
- Fusion Odometry dan ICP menggunakan Complementary Filter
- Game state (FSM) untuk membentuk algoritma permainan
- Komunikasi dengan Basesation lewat rosbridge-server
- Kontrol utama dan kontrol paling tinggi ada disini
