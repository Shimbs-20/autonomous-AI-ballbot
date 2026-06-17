# 🤖 Hazard Detection Ball Bot — Cairo University GP 2026

<div align="center">

![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%20%2B%20STM32F411-red?style=for-the-badge&logo=raspberry-pi)
![ROS2](https://img.shields.io/badge/ROS2-Humble%2FJazzy-blue?style=for-the-badge&logo=ros)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green?style=for-the-badge)
![YOLOv8](https://img.shields.io/badge/Vision-YOLOv8%20NCNN-purple?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

**A self-balancing omnidirectional robot on a spherical wheel with real-time hazard detection, autonomous SLAM navigation, and a live web dashboard.**

*Cairo University · Faculty of Engineering · Mechatronics Graduation Project · Spring 2026*

</div>

---

## 📋 Table of Contents

- [Overview](#overview)
- [Demo](#demo)
- [Team](#team)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Software Stack](#software-stack)
- [Repository Structure](#repository-structure)
- [Quick Start](#quick-start)
- [Control System](#control-system)
- [Navigation](#navigation)
- [Hazard Detection](#hazard-detection)
- [Web Dashboard](#web-dashboard)
- [Results](#results)
- [Bill of Materials](#bill-of-materials)
- [Acknowledgements](#acknowledgements)

---

## Overview

The Hazard Detection Ball Bot is an inverted-pendulum robot that balances on a single 350mm sphere driven by three omnidirectional wheels. It combines:

- **Real-time dynamic balancing** using a custom EKF (8-state) and LQI controller running at 200 Hz on STM32F411
- **Autonomous SLAM mapping** using a 360° LiDAR and slam_toolbox on Raspberry Pi 4
- **Hazard detection** using a custom-trained YOLOv8 NCNN model (fire, smoke, electrical hazard, person, obstacle) at 33+ FPS
- **Full ROS2 Navigation Stack** with MPPI holonomic controller for autonomous goal-seeking
- **Live web dashboard** accessible from any browser on the local network

The robot operates in four modes:
| Mode | Description |
|------|-------------|
| `0 STANDBY` | Motors off, safe state |
| `1 BALANCE` | Stationary self-balancing (LQI active) |
| `2 TELEOP` | Balance + velocity commands from Pi |
| `3 FLOOR DRIVE` | Direct omni-drive on casters (for SLAM mapping without balance risk) |

---

## Demo

> 📹 **[Demo Video — YouTube](#)** ← add link after recording

| SLAM Mapping | Hazard Detection | Dashboard |
|:---:|:---:|:---:|
| ![mapping](docs/images/slam_map.png) | ![detection](docs/images/hazard_detection.png) | ![dashboard](docs/images/dashboard.png) |


---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 4B                              │
│                                                                     │
│  LD06 LiDAR ──→ /scan ──→ slam_toolbox ──→ /map                    │
│                              ↑                                      │
│  /odom ←── stm32_bridge ←── UART2 ←─────────────────────┐         │
│               │                                           │         │
│               └──→ twist_mux ──→ /cmd_vel ───────────────┘         │
│                      ↑              ↑                               │
│               PS4 joystick    Nav2 MPPI                             │
│                                                                     │
│  IMX219 ──→ /image_raw ──→ YOLOv8 NCNN ──→ /hazard_detections     │
│                                 └──→ /image_annotated               │
│                                                                     │
│  web_video_server (port 8080) + rosbridge (port 9090)               │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ UART2 / 115200 baud
                               │ TX: 16-byte [AA][55][mode][vx][vy][yaw][XOR]
                               │ RX: 27-byte [BB][66][x][y][θ][vx][vy][yaw][XOR]
┌──────────────────────────────┴──────────────────────────────────────┐
│                    STM32F411 @ 96 MHz / FreeRTOS                    │
│                                                                     │
│  MPU6050 ──→ EKF (200 Hz, 8-state) ──→ LQI K[3×10]               │
│                        ↑                     ↓                      │
│  Encoders (TIM2/3/4) ──┘           convert_torque_to_speed          │
│                                             ↓                       │
│  TIM11 ISR (70021 Hz) ─── Bresenham ──→ 3× DM860H ──→ NEMA34      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Hardware

### Key Specifications

| Component | Specification |
|-----------|--------------|
| **Compute (embedded)** | STM32F411CEU6 @ 96 MHz, 128KB RAM |
| **Compute (high-level)** | Raspberry Pi 4B, 4GB RAM |
| **Motors** | 3× NEMA34 86BYGH127, 8.5 N·m holding torque |
| **Motor Drivers** | 3× DM860H, 48V / 7.2A, 16-microstep |
| **IMU** | MPU6050, ±16g / ±2000°/s, DLPF 44 Hz |
| **LiDAR** | LD06, 360°, 12m range, 230,400 baud |
| **Camera** | IMX219 (Pi Camera v2), 1080p, 30 FPS |
| **Ball** | 350mm diameter, rubber-tape coated, µ≈0.50 |
| **Battery** | 48V / 11Ah NMC (EV Raid) |
| **Total Height** | 925 mm |
| **Total Mass** | ~17 kg |

### Kinematic Geometry

```
Wheel placement:  γ₁=120°, γ₂=240°, γ₃=0° (equal spacing)
Zenith angle:     α = 45°
Ball radius:      Rk = 0.175 m
Wheel radius:     Rw = 0.0625 m
```

---

## Software Stack

```
Firmware (STM32):          ROS2 (Raspberry Pi):
  FreeRTOS                   ROS2 Humble / Jazzy
  STM32 HAL                  slam_toolbox (async SLAM)
  EKF (custom C)             Nav2 (MPPI Omni)
  LQI controller             AMCL localization
  USART2 DMA                 twist_mux
  Bresenham step engine      web_video_server
                             rosbridge_websocket
Vision:                    Dashboard:
  OpenCV                     HTML5 / JavaScript
  Ultralytics YOLOv8         ROSLIB.js
  NCNN inference             Web Video Server MJPEG
```

---

## Repository Structure

```
ballbot/
├── README.md
├── LICENSE
├── .gitignore
│
├── firmware/                   STM32CubeIDE project
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c          Main control loop (Modes 0-3)
│   │   │   ├── lqr.c           LQI controller + EKF interface
│   │   │   ├── Ekf.c           Extended Kalman Filter (8-state)
│   │   │   └── mpu6050.c       IMU driver
│   │   └── Inc/
│   │       ├── lqr.h
│   │       └── Ekf.h
│   └── README.md
│
├── ros2_ws/                    ROS2 workspace
│   └── src/
│       ├── ballbot_controller/ Bridge, teleop, mode control
│       ├── ballbot_mapping/    SLAM toolbox config + launch
│       ├── ballbot_navigation/ Nav2 params, behavior tree
│       ├── camera_pi/          Camera node + YOLOv8 detector
│       └── gp_description/     URDF + mesh files
│
├── matlab/                     Design and simulation
│   ├── system_model/           Nonlinear ball-bot dynamics
│   ├── lqi_design/             LQI gain matrix K[3×10] derivation
│   ├── ekf_design/             EKF Q/R tuning + verification
│   └── simulation/             Full closed-loop Simulink model
│
├── hardware/
│   ├── cad/                    SolidWorks assembly (.SLDASM, .STEP)
│   ├── pcb/                    Custom power board schematics
│   ├── BOM.md                  Complete bill of materials
│   └── README.md
│
└── dashboard/
    └── dashboard.html          Standalone web dashboard
```

---

## Quick Start

### Prerequisites

```bash
# On Raspberry Pi 4B — Bookworm 64-bit
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
    ros-humble-slam-toolbox ros-humble-nav2-bringup \
    ros-humble-rosbridge-server ros-humble-web-video-server

pip install ultralytics opencv-python picamera2
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run — Hardware bringup (Terminal 1)

```bash
# Testing without STM32
ros2 launch ballbot_controller pi_controller.launch.py

# Real robot — stationary balance
ros2 launch ballbot_controller pi_controller.launch.py \
    use_fake_odom:=false robot_mode:=1

# Real robot — SLAM mapping on floor (safest)
ros2 launch ballbot_controller pi_controller.launch.py \
    use_fake_odom:=false robot_mode:=3
```

### Run — SLAM mapping (Terminal 2)

```bash
ros2 launch ballbot_mapping slam.launch.py use_slam:=true

# Drive to map:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save when done:
ros2 run nav2_map_server map_saver_cli -f ~/maps/room
```

### Run — Autonomous navigation (Terminal 2)

```bash
ros2 launch ballbot_mapping slam.launch.py \
    use_slam:=false map_yaml:=~/maps/room.yaml

ros2 launch ballbot_navigation nav2.launch.py \
    use_slam:=false shelf_test:=false
# Then: RViz → 2D Pose Estimate → 2D Nav Goal
```

### Run — Camera + Hazard detection (Terminal 3)

```bash
ros2 launch camera_pi hazard.launch.py
# Dashboard: http://ballbot.local:8080
```

### Switch robot modes at runtime

```bash
ros2 topic pub --once /ballbot/mode std_msgs/msg/UInt8 '{data: 1}'  # BALANCE
ros2 topic pub --once /ballbot/mode std_msgs/msg/UInt8 '{data: 2}'  # TELEOP
ros2 topic pub --once /ballbot/mode std_msgs/msg/UInt8 '{data: 3}'  # FLOOR DRIVE
```

---

## Control System

### EKF — 8-State Observer

State vector: `[φ, θ, dφ, dθ, Vx, Vy, bias_x, bias_y]`

- **Sensor fusion**: MPU6050 gyro + accelerometer + 3 wheel encoders
- **Accelerometer spike rejection**: rejects readings outside [0.2g, 15g]
- **Motor-aware noise**: R_accel scales with motor speed (30→250)
- **Execution time**: ~0.4ms @96MHz (Joseph-form covariance update)

### LQI Controller — K[3×10]

```
Torque = −K × [φ, θ, ψ, Vx, Vy, dφ, dθ, ω_z, ∫Vx, ∫Vy]ᵀ
```

Gains tuned in MATLAB for: m=17kg, Rk=0.175m, Rw=0.0625m, α=45°, J_wheel=0.0032 kg·m²

### Inverse Kinematics (Mode 3 — Floor Drive)

Pseudo-inverse of forward kinematic matrix (verified: K·K⁺=I₂ₓ₂):

```
ω₁ = −1.617·vx − 0.933·vy + 1.980·ω_z
ω₂ = +1.617·vx − 0.933·vy + 1.980·ω_z
ω₃ = +0.000·vx + 1.867·vy + 1.980·ω_z
```

---

## Navigation

- **SLAM**: slam_toolbox async_online, 5cm/cell, Ceres solver
- **Localization**: AMCL with OmniMotionModel
- **Planner**: SmacPlanner2D (A*, holonomic — no heading constraint)
- **Controller**: MPPI, 300 trajectories, `motion_model: Omni`, vy_max=0.5 m/s
- **Behavior tree**: ClearCostmap → Spin → BackUp → Wait
- **Velocity arbitration**: twist_mux — PS4(100) > keyboard(50) > Nav2(10)

---

## Hazard Detection

| Class | mAP@0.5 | Triggers E-Stop |
|-------|---------|-----------------|
| 🔥 Fire | 94.2% | ✅ |
| 💨 Smoke | 91.8% | ✅ |
| ⚡ Electrical Hazard | 88.7% | ✅ |
| 👤 Person | 96.5% | ❌ |
| 🚧 Obstacle | 85.3% | ❌ |

- **Model**: YOLOv8n converted to NCNN for Pi 4 CPU inference
- **Input**: 416×416 letterbox (aspect-ratio preserving)
- **Output**: 640×480 annotated MJPEG to web dashboard
- **Throughput**: 33.4 FPS average on Raspberry Pi 4B

---

## Web Dashboard

Accessible at `http://ballbot.local:8080` or `http://<Pi-IP>:8080`

**Features:**
- Live MJPEG camera feed (annotated with bounding boxes)
- Real-time SLAM map rendering from `/map` OccupancyGrid
- LiDAR scan overlay on map
- Odometry trail
- Mode control buttons (0/1/2/3) via `/ballbot/mode`
- Hazard detection cards with proximity indicator
- CPU temperature gauge
- Emergency stop overlay

**Dependencies:** rosbridge_websocket (port 9090), web_video_server (port 8080)

---

## Results

| Metric | Value |
|--------|-------|
| Control loop frequency | 200 Hz |
| EKF execution time | ~0.4 ms |
| Stepper ISR frequency | 70,021 Hz |
| SLAM map resolution | 5 cm/cell |
| Nav2 goal success rate | 100% (lab trials) |
| Detection FPS | 33.4 FPS |
| Detection classes | 5 |
| Total system mass | 17 kg |
| Battery endurance | ~2 hours |

---

## Bill of Materials

See [`hardware/BOM.md`](hardware/BOM.md) for the complete list.

**Major components:**

| Item | Qty | Est. Cost |
|------|-----|-----------|
| STM32F411 Black Pill | 1 | $5 |
| Raspberry Pi 4B 4GB | 1 | $55 |
| NEMA34 stepper 86BYGH127 | 3 | $45 each |
| DM860H stepper driver | 3 | $30 each |
| MPU6050 IMU module | 1 | $3 |
| LD06 LiDAR | 1 | $90 |
| Pi Camera Module v2 (IMX219) | 1 | $25 |
| 48V 11Ah NMC battery pack | 1 | $120 |
| Omni roller ball 350mm | 1 | $80 |
| **Total (approx.)** | | **~$1,480** |

---

## Acknowledgements

This project was completed as the Graduation Project 2 (GP2) for the Mechatronics program at Cairo University Faculty of Engineering, Spring 2026.

We thank Dr. Hossam H. Ammar and Dr. Moataz El-Sisi for their guidance throughout the project.

**References:**
- Lauwers, T. B. et al., "A Dynamically Stable Single-Wheeled Mobile Robot with Inverse Mouse-Ball Drive" — ICRA 2006
- Geyer, H., "Ballbot: The Fast, Omnidirectional Balancing Mobile Robot" — CMU 2010
- Welch, G. and Bishop, G., "An Introduction to the Kalman Filter" — UNC 1995

---

<div align="center">
Cairo University · Faculty of Engineering · Spring 2026
</div>
