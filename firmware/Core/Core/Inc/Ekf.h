/*
 * Ekf.h
 *
 *  Created on: Mar 23, 2026
 *      Author: Dell
 */

/* Ekf.h */
#ifndef INC_EKF_H_
#define INC_EKF_H_

#include <math.h>

// --- THE SHIELD STARTS HERE ---
#ifndef MATLAB_MEX_FILE
    // Only the STM32 is allowed to read these
    #include "stm32f4xx_hal.h"
    #include "arm_math.h"
#else
    // Simulink reads this dummy variable instead
    typedef void* I2C_HandleTypeDef;
    typedef float float32_t;           // <-- THE MISSING LINK!
#endif
// --- THE SHIELD ENDS HERE ---

// The 8 state variables
typedef struct {
    float phi;
    float theta;
    float dphi;
    float dtheta;
    float vx;
    float vy;
    float bx;
    float by;
} BallbotStates;

void BallbotEKF_Init(void);

void BallbotEKF_Update(float gyro_x, float gyro_y,
                       float accel_x, float accel_y,
                       float enc_vx, float enc_vy, float dt,
                       BallbotStates* out_states);

#endif /* INC_EKF_H_ */
