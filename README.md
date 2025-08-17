# RobotServoArm
Library untuk mempermudah dalam project Arm Robot yang menggunakan Servo.

# 🤖 RobotServoArm  
**Library *single-header* untuk lengan robot berbasis servo**  
> Satu baris kode = satu gerakan! Kuasai inverse kinematics, rekam pose, & kontrol gripper **tanpa ribet**.

---

## ✨ Fitur Utama

| Fitur | Penjelasan Singkat |
|---|---|
| **Inverse Kinematics** | Hitung sudut otomatis dari koordinat X-Y-Z (planar 2-link) |
| **Gripper Otomatis** | Tutup / buka dengan parameter **gaya** (0–100 %) |
| **Rekam & Putar Pose** | Simpan 8 pose di memori → putar balik dengan satu perintah |
| **Gerakan Halus** | Easing kubik agar servo tidak kaku / getar |
| **Multi-board** | Arduino Uno, Nano, ESP32, STM32, dll. |

---

## 🚀 Instalasi Cepat

1. **Unduh** file `RobotServoArm.hpp` lalu letakkan di folder `src/` project Anda.
2. **Tidak ada library tambahan** kecuali `Servo.h` bawaan Arduino (ESP32 pakai `ESP32Servo`).

---

## 🔌 Skema Kabel (contoh 4 servo)

| Servo | Fungsi | Pin Arduino (contoh) |
|---|---|---|
| Base | Putar dasar lengan | D5 |
| Shoulder | Angkat / turun | D6 |
| Elbow | Lengan bawah | D9 |
| Gripper | Jepit / lepas | D10 |
| Vcc | 5 V | 5 V |
| GND | GND | GND |

---

## 📖 Contoh Program

### A. Gerak ke titik & jepit
```cpp
#include <RobotServoArm.hpp>

RobotServoArm arm;

void setup() {
  Serial.begin(115200);
  // Mulai: base, shoulder, elbow, gripper
  arm.begin(5, 6, 9, 10);
}

void loop() {
  arm.moveTo(120, 0, 80);  // gerak ke x=120, y=0, z=80 mm
  delay(1000);
  arm.grip(0.7);           // jepit 70 % kekuatan
  delay(500);
  arm.moveTo(120, 0, 40);  // angkat lebih tinggi
  delay(1000);
  arm.release();           // buka gripper
  arm.home();              // kembali posisi awal
  delay(2000);
}
```

### B. Rekam & putar pose
```cpp
void setup() {
  arm.begin(5, 6, 9, 10);

  // 1. Manual gerakkan lengan ke posisi A
  delay(3000);
  arm.recordPose(0);  // simpan di slot 0

  // 2. Gerakkan ke posisi B
  delay(3000);
  arm.recordPose(1);  // simpan di slot 1

  // 3. Putar otomatis
  arm.playPose(0, 1500);  // gerak ke posisi A selama 1,5 detik
  delay(1000);
  arm.playPose(1, 1500);  // gerak ke posisi B
}
```

---

## 📚 Referensi API (Ringkas)

| Fungsi | Parameter | Keterangan |
|---|---|---|
| `begin(bPin, sPin, ePin, gPin)` | nomor pin | Inisialisasi 4 servo & batas pulsa |
| `moveTo(x, y, z, ms)` | mm, mm, mm, durasi | Gerak ke koordinat dalam durasi tertentu |
| `grip(f)` | 0–1 (0 %–100 %) | Tutup gripper dengan gaya tertentu |
| `release()` | – | Buka gripper |
| `home()` | – | Kembali ke posisi tengah (90°,90°,90°) |
| `recordPose(slot)` | 0–7 | Simpan pose saat ini |
| `playPose(slot, ms)` | 0–7, durasi | Putar pose tersimpan |

---

## 🤝 Berkontribusi
- Fork → buat branch `fitur-baru` → Pull Request.  
- Gunakan `pio check` sebelum commit agar lint bersih.

---

## 📄 Lisensi
**MIT License** – bebas dipakai untuk proyek pribadi, komersial, maupun edukasi.
