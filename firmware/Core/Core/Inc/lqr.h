/*
 * lqr.h
 *
 *  Created on: Mar 24, 2026
 *      Author: Dell
 */

/* lqr.h */

#ifndef INC_LQR_H_
#define INC_LQR_H_

#include <stdint.h>
#include "Ekf.h"

// --- THE SHIELD STARTS HERE ---
#ifndef MATLAB_MEX_FILE
    // Only the STM32 is allowed to read these
    #include "stm32f4xx_hal.h"
    #include "arm_math.h"
    #include "main.h"
    #include "mpu6050.h"
    #include "usbd_cdc_if.h"
	#include "usb_device.h"
#else
    // Simulink reads this dummy variable instead
    typedef void* I2C_HandleTypeDef;
    typedef float float32_t;           // <-- THE MISSING LINK!
#endif
// --- THE SHIELD ENDS HERE ---

extern float current_state[10];

void updateReferencesFromROS(float ros_v_x, float ros_v_y, float ros_yaw_rate, uint32_t current_time_ms);

// The I2C variable is gone, and the Simulink output port is added!
void controlLoop(float dt, uint32_t current_time_ms,float* out_motor_torques);

#endif /* INC_LQR_H_ */
