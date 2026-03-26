# 🚲 Bike-Light-Controller (Smart Cycling System)

This repository hosts a specialized embedded system designed for conventional bicycles, developed as a final project for my **Bachelor's Degree in Electronic and Telecommunications Engineering**.

The system transforms a standard bike into a "smart" vehicle by automating safety lighting through real-time sensor data fusion.

---

## 🚀 Overview
The **Bike-Light-Controller** manages visibility and signaling automatically. Using an **ESP32-S3** and the **Arduino IDE** framework, the system processes environmental light levels and motion dynamics to ensure rider safety.

### ✨ Key Features
* **Intelligent Brake Light:** Automatically triggers high-intensity rear LEDs when deceleration is detected via the **MPU6050**.
* **Adaptive Headlight:** Adjusts front lighting based on ambient lux levels measured by the **TEMT6000**.
* **Custom Hardware:** Professional PCB designed using **KiCad 9.0**, utilizing official **Espressif hardware libraries** for the ESP32-S3.
* **Real-time Processing:** Low-latency response to motion and lighting changes.

---

## 🛠️ Tech Stack

### Hardware Components
| Component | Function |
| :--- | :--- |
| **MCU** | **ESP32-S3** (Xtensa® 32-bit LX7 Dual-Core) |
| **IMU** | **MPU6050** (6-axis Accelerometer & Gyroscope) |
| **Light Sensor** | **TEMT6000** (High-precision ambient light sensor) |
| **PCB Design** | **KiCad 9.0** (Schematic & Layout) |

### Software & Firmware
* **Language:** C++ / Arduino Framework
* **IDE:** Arduino IDE 2.3.8
* **Communication:** I2C Protocol (Sensors) / MQTT (Optional for IoT telemetry)
* **Libraries:** `Adafruit_MPU6050`, `Adafruit_Sensor`, `Wire.h`

---

## 📂 Repository Structure

```text
├── Firmware/        # Source code (.ino) for the ESP32-S3
├── Hardware/        # KiCad 9.0 Project files (Schematics, PCB, Gerbers)
├── Docs/            # Datasheets, project report, and bibliography
└── Assets/          # Images, circuit diagrams, and 3D renders
