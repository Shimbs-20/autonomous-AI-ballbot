# STM32 Firmware

**Platform**: STM32F411CEU6 (Black Pill) @ 96 MHz  
**RTOS**: FreeRTOS (CMSIS-RTOS v1)  
**IDE**: STM32CubeIDE 1.14+

---

## File Map

| File | Purpose |
|------|---------|
| `Core/Src/main.c` | Hardware init, all 4 robot modes, stepper ISR |
| `Core/Src/lqr.c` | LQI gain matrix, EKF interface, watchdog |
| `Core/Inc/lqr.h` | Public API: `controlLoop()`, `setControlMode()`, extern state vars |
| `Core/Src/Ekf.c` | 8-state EKF (predict + Joseph-form update) |
| `Core/Src/mpu6050.c` | MPU6050 I2C driver (400 kHz) |

---

## Robot Modes

| Mode | Value | Description |
|------|-------|-------------|
| `STANDBY` | 0 | Motors off, EKF reset |
| `BALANCE` | 1 | LQI active, velocity refs = 0 |
| `TELEOP` | 2 | LQI + velocity commands from Pi, binary odom @ 20 Hz |
| `FLOOR_DRIVE` | 3 | Direct inverse kinematics, no balance, binary odom @ 20 Hz |

---

## UART Protocol

**Pi → STM32** (16 bytes, 50 Hz):
```
[0xAA][0x55][mode:u8][vx:f32 LE][vy:f32 LE][yaw:f32 LE][XOR of bytes 2-14]
```

**STM32 → Pi** (27 bytes, 20 Hz — Modes 2 & 3 only):
```
[0xBB][0x66][x:f32][y:f32][θ:f32][vx:f32][vy:f32][yaw:f32][XOR of bytes 2-25]
```

---

## Key Parameters to Tune

```c
// lqr.c
const float LPF_ALPHA   = 0.5f;   // Torque LPF — higher = faster response
const float MAX_INTEGRAL = 0.3f;  // Anti-windup on velocity integral

// main.c
#define TORQUE_TO_OMEGA   5.0f    // (rad/s)/Nm — increase if response is slow
#define MAX_RAMP          500     // Steps/loop — increase for faster acceleration
#define PHI_OFFSET       -0.022f  // Calibrate to your robot's mounting angle
#define THETA_OFFSET     -0.185f  // Calibrate to your robot's mounting angle
```

---

## Flashing

1. Open project in STM32CubeIDE
2. Connect ST-Link to SWD header (SWDIO, SWCLK, GND, 3.3V)
3. **Run → Debug** (first flash) or **Run → Run** (subsequent)

> ⚠️ Keep robot still for 2 seconds after power-on — gyro bias calibration runs automatically.
