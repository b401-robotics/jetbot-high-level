# Hide and Seek using Jetbot

## Hal yang perlu dilakukan

Karena baterai RTC Jetson Nano nya jetboard telah mati, jadi setiap kali menghidupkan jetbot harus selalu memperbaiki waktunya.

```
sudo timedatectl set-time "2023-12-23 08:08:08"
```

Sebenarnya hal itu bisa diatasi oleh NTP, namun NTP di ITS kebetulan tidak bisa.

## Compile

Untuk compile workspace ini cukup gunakan `catkin_make`

```
catkin_make
```

## Penggunaan

Untuk menggunakan workspace ini jalnkan terlebih dahulu `run.sh` pada ${home}/workspace/catkin_ws lalu jalankan `run.sh` disini

```
chmod +x run.sh # Jika run.sh tidak bisa dijalankan "permission denied"

./run.sh
```

Untuk dokumentasi lebih lanjut ada di [dokumentasi](doc/README.md)

## Demonstrasi Video
[![Watch the video]](https://drive.google.com/file/d/1ieZr84CJkuApwBAylvSIfFp5AeSWpBLe/view?usp=drivesdk)
