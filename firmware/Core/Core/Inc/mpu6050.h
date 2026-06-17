//// mpu6050_fixed.h (replace or add)
//#ifndef __MPU6050__H
//#define __MPU6050__H
//
//#include "stm32f4xx_hal.h"
//#include <math.h>
//#include <string.h>
//
//#ifndef M_PI
//#define M_PI 3.14159265358979323846f
//#endif
//
//// try both AD0 low and high (shifted for HAL)
//#define MPU6050_ADDR_68 (0x68 << 1)
//#define MPU6050_ADDR_69 (0x69 << 1)
//
//#define WHO_AM_I_REG         0x75
//#define PWR_MGMT_1_REG       0x6B
//#define SMPLRT_DIV_REG       0x19
//#define CONFIG_REG           0x1A
//#define GYRO_CONFIG_REG      0x1B
//#define ACCEL_CONFIG_REG     0x1C
//#define ACCEL_XOUT_H_REG     0x3B
//
//#define SMPLRT_DIV_VAL       0x07
//#define DLPF_CONFIG_VAL      0x03
//#define ACCEL_CONFIG_VAL     0x00
//#define GYRO_CONFIG_VAL      0x00
//
//#define ACCEL_SENSITIVITY    16384.0f
//#define GYRO_SENSITIVITY     131.0f
//#define COMP_FILTER_ALPHA    0.98f
//
//typedef struct {
//    I2C_HandleTypeDef *i2c_handle;
//    uint16_t dev_addr; // actual detected device address (8-bit shifted)
//    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
//    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
//    float Ax, Ay, Az;
//    float Gx, Gy, Gz;
//    float AngleRoll, AnglePitch;
//} MPU6050_Data_t;
//
//HAL_StatusTypeDef MPU6050_Init(MPU6050_Data_t *mpu, I2C_HandleTypeDef *hi2c);
//HAL_StatusTypeDef MPU6050_Read_All(MPU6050_Data_t *mpu);
//void MPU6050_Calculate_Angles(MPU6050_Data_t *mpu, float dt);
//
//#endif


/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#include"main.h"
#include <stdint.h>

// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif /* INC_GY521_H_ */

