#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

/*
 * NOTE: Your main.c file must have these pins defined in CubeMX:
 *
 * Motor 1 (TIM3_CH1): PA6_PWM, PB13_DIR, PB14_DIR
 * Motor 2 (TIM3_CH2): PA7_PWM, PB1_DIR,  PB2_DIR
 * Motor 3 (TIM3_CH3): PB0_PWM, PA11_DIR, PA12_DIR
 */

// This struct holds the pins for one motor
typedef struct {
    TIM_HandleTypeDef* htim;       // Timer handle (&htim1)
    uint32_t            channel;    // Timer channel (TIM_CHANNEL_1)
    GPIO_TypeDef* port_in1;   // Direction Pin 1 Port
    uint16_t            pin_in1;    // Direction Pin 1
    GPIO_TypeDef* port_in2;   // Direction Pin 2 Port
    uint16_t            pin_in2;    // Direction Pin 2
} Motor_t;

/**
 * @brief  Initializes all motors and starts their PWM channels.
 */
void Motor_Init(void);


void Motor_SetSpeed(Motor_t* motor, int32_t speed);

/**
 * @brief
 */
void Motor_StopAll(void);

// Global motor objects, accessible from your tasks
extern Motor_t motor1;
extern Motor_t motor2;
extern Motor_t motor3;

#endif /* INC_MOTOR_H_ */
