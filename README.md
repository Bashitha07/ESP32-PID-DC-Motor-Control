# PID-Based DC Motor Speed Control using ESP32 and L298N

A closed-loop digital speed control system designed for an N20 Gear Motor using an ESP32 microcontroller, L298N motor driver, and a quadrature Hall sensor encoder. Developed as the Final Assignment for **IE2074 - Control Theory** at the Sri Lanka Institute of Information Technology (SLIIT).

---

## 👥 Authors & Group Details
* **Group No:** 14
* **Module:** IE2074 - Control Theory
* **Degree Program:** BSc. (Hons) in Computer Systems Engineering

| Name | Student ID (IT Number) |
| :--- | :--- |
| **B.A. Sellapperuma** | IT24103485 |
| **R.M.V.S. Amarasinghe** | IT24103092 |

---

## 🚀 Project Overview
This project implements a real-time digital **PID (Proportional-Integral-Derivative)** controller running on an ESP32 microcontroller. Speed feedback is captured via hardware interrupts attached to the Hall sensor encoder of an N20 DC gear motor. A TCP-over-WiFi interface allows live monitoring and real-time parameter tuning ($Kp$, $Ki$, $Kd$, and setpoint) via a Python/GUI dashboard without needing to reflash the firmware.

### Key Performance Metrics:
* **Rise Time ($10\%-90\%$):** ~0.45 seconds
* **Settling Time ($\pm2\%$):** ~1.2 seconds
* **Maximum Overshoot:** ~8%
* **Steady-State Error (Full PID):** < 1 RPM ($\approx 0\%$)

---

## 🛠️ Hardware Components
* **Microcontroller:** ESP32 Dev Board (Dual-core, 240 MHz, 3.3V logic)
* **Motor Driver:** L298N Dual H-Bridge Motor Driver Module
* **Actuator:** N20 Gear Motor (6V, 300 RPM) with quadrature Hall encoder
* **Power Supply:** 9V Battery (Motor) + USB 5V (ESP32 Logic, common ground)

---

## 📐 System Architecture & Tuning
1. **Sensor Feedback:** Channel A of the Hall encoder triggers an interrupt service routine (ISR) on GPIO 5 on every rising edge (with a 1 ms software debounce window).
2. **Control Loop:** Executes every $20\text{ ms}$ (`SAMPLE_TIME_MS`), calculating the error between the target RPM and measured RPM.
3. **PWM Generation:** The ESP32 LEDC peripheral outputs a 5 kHz PWM signal to the L298N ENA pin to regulate motor speed.
4. **Anti-Windup:** Integral accumulation is clamped (`±437`) to prevent saturation-induced overshoot during startup transients.

### Final Tuned Parameters:
* $Kp = 2$
* $Ki = 1$
* $Kd = 0.02$

---

## 📂 Repository Structure
```text
ESP32-PID-DC-Motor-Control/
│
├── firmware/
│   └── motor_pid_controller.ino      # Arduino C/C++ firmware for ESP32
│
├── docs/
│   └── Group No.14 - IE2074 - CT - Final Assignment.pdf
│
├── images/                           # Circuit diagrams and dashboard response plots
│   ├── circuit_diagram.png
│   └── step_response.png
│
└── README.md                         # Project documentation
