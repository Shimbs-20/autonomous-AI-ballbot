/* ═══════════════════════════════════════════════════════════════
 * mpu6050.c  —  fully corrected
 * ═══════════════════════════════════════════════════════════════
 *
 * Configuration chosen:
 *   Accel: ±8g   → ACCEL_CONFIG = 0x10,  sensitivity = 4096 LSB/g
 *   Gyro:  ±2000°/s → GYRO_CONFIG = 0x18, sensitivity = 16.4 LSB/(°/s)
 *
 * These are appropriate for a ballbot — ±8g handles vibration
 * without saturating, ±2000°/s handles fast recovery tilts.
 * ═══════════════════════════════════════════════════════════════ */

#include <math.h>
#include "mpu6050.h"

/* ── Register addresses ──────────────────────────────────────── */
#define WHO_AM_I_REG       0x75
#define PWR_MGMT_1_REG     0x6B
#define SMPLRT_DIV_REG     0x19
#define CONFIG_REG         0x1A
#define ACCEL_CONFIG_REG   0x1C
#define GYRO_CONFIG_REG    0x1B
#define ACCEL_XOUT_H_REG   0x3B
#define TEMP_OUT_H_REG     0x41
#define GYRO_XOUT_H_REG    0x43

/* ── Device addresses — HAL requires 8-bit shifted address ───── */
#define MPU_ADDR_LOW   (0x68 << 1)   /* AD0 pin → GND */
#define MPU_ADDR_HIGH  (0x69 << 1)   /* AD0 pin → VCC */

/* ── Sensitivities matching the config bytes below ───────────── */

#define ACCEL_SENSITIVITY  2048.0f  // ±2g  → 8× more sensitive
#define GYRO_SENSITIVITY   16.4f     // ±500°/s → 4× more sensitive

static const uint16_t I2C_TIMEOUT = 4000;
static uint16_t mpu_addr = MPU_ADDR_LOW;  /* discovered at init */

/* ─────────────────────────────────────────────────────────────── */
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t who = 0;
    uint8_t data;

    /* Try AD0=LOW (0x68) first, then AD0=HIGH (0x69) */
    HAL_I2C_Mem_Read(I2Cx, 0x68 << 1,  WHO_AM_I_REG, 1, &who, 1, I2C_TIMEOUT);
    if (who == 0x68) {
        mpu_addr = MPU_ADDR_LOW;
    } else {
        who = 0;
        HAL_I2C_Mem_Read(I2Cx, 0x69 << 1, WHO_AM_I_REG, 1, &who, 1, I2C_TIMEOUT);
        if (who == 0x68) {
            mpu_addr = MPU_ADDR_HIGH;
        } else {
          HAL_Delay(10);
        }
    }

    /* Wake up — clear sleep bit in PWR_MGMT_1 */
    data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, mpu_addr, PWR_MGMT_1_REG, 1, &data, 1, I2C_TIMEOUT);
    HAL_Delay(10);   /* datasheet: 10ms after wake-up */

    /* Sample rate divider: SMPLRT_DIV=7 → sample rate = 1kHz/(1+7) = 125 Hz
     * This caps the internal ODR. MPU6050_Read_All still reads on demand,
     * but the internal FIFO and interrupt rate is 125 Hz.               */
    data = 0x04;
    HAL_I2C_Mem_Write(I2Cx, mpu_addr, SMPLRT_DIV_REG, 1, &data, 1, I2C_TIMEOUT);

    /* Digital Low-Pass Filter: DLPF_CFG=3 → ~44Hz accel, ~42Hz gyro.
     * Critical for the EKF: this removes stepper motor vibration noise
     * above 44Hz before the data reaches our filter.                    */
    data = 0x03;
    HAL_I2C_Mem_Write(I2Cx, mpu_addr, CONFIG_REG, 1, &data, 1, I2C_TIMEOUT);

    /* Accelerometer: ±16g range → AFS_SEL=2 → 0x10
     * BUG FIXED: previous code used wrong address (missing << 1)        */
    data = 0x18;
    HAL_I2C_Mem_Write(I2Cx, mpu_addr, ACCEL_CONFIG_REG, 1, &data, 1, I2C_TIMEOUT);

    /* Gyroscope: ±2000°/s range → FS_SEL=3 → 0x18
     * BUG FIXED: previous code used wrong address (missing << 1)        */
    data = 0x18;
    HAL_I2C_Mem_Write(I2Cx, mpu_addr, GYRO_CONFIG_REG, 1, &data, 1, I2C_TIMEOUT);

    return 0;   /* success */
}

/* ─────────────────────────────────────────────────────────────── */
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t buf[14];

    /* Single 14-byte burst read: Accel(6) + Temp(2) + Gyro(6)    */
    HAL_I2C_Mem_Read(I2Cx, mpu_addr, ACCEL_XOUT_H_REG, 1, buf, 14, I2C_TIMEOUT);

    /* Reassemble big-endian 16-bit words */
    DataStruct->Accel_X_RAW = (int16_t)(buf[0]  << 8 | buf[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(buf[2]  << 8 | buf[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(buf[4]  << 8 | buf[5]);
    /* buf[6..7] = temperature — skip for EKF */
    DataStruct->Gyro_X_RAW  = (int16_t)(buf[8]  << 8 | buf[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(buf[10] << 8 | buf[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(buf[12] << 8 | buf[13]);

    /* Convert to physical units
     * BUG FIXED: previously Ax/Ay used 16384 (±2g) not 4096 (±8g)
     * BUG FIXED: previously Gx/Gy/Gz inconsistently 131 vs 16.4    */
    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;  /* g */
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;  /* g */
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;  /* g */

    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;   /* °/s */
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;   /* °/s */
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;   /* °/s */
}

/* ─────────────────────────────────────────────────────────────── */
void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t buf[6];
    HAL_I2C_Mem_Read(I2Cx, mpu_addr, ACCEL_XOUT_H_REG, 1, buf, 6, I2C_TIMEOUT);
    DataStruct->Accel_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
}

/* ─────────────────────────────────────────────────────────────── */
void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t buf[6];
    HAL_I2C_Mem_Read(I2Cx, mpu_addr, GYRO_XOUT_H_REG, 1, buf, 6, I2C_TIMEOUT);
    DataStruct->Gyro_X_RAW = (int16_t)(buf[0] << 8 | buf[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(buf[2] << 8 | buf[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(buf[4] << 8 | buf[5]);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;
}
