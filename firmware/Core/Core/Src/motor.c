//#include "motor.h"
//#include <stdlib.h>
//
//// Bring in the timer handle from main.c
////extern TIM_HandleTypeDef htim1;
//
//// Define the 3 global motor objects
//Motor_t motor1;
//Motor_t motor2;
//Motor_t motor3;
//
//// Internal helper function (not in .h file)
//static void Motor_SetDirection(Motor_t* motor, int32_t speed);
//
//void Motor_Init(void) {
//    // --- Motor 1 Configuration ---
//    motor1.htim = &htim1;
//    motor1.channel = TIM_CHANNEL_1;
//    motor1.port_in1 = GPIOB;
//    motor1.pin_in1 = GPIO_PIN_13;
//    motor1.port_in2 = GPIOB;
//    motor1.pin_in2 = GPIO_PIN_14;
//
//    // --- Motor 2 Configuration ---
//    motor2.htim = &htim1;
//    motor2.channel = TIM_CHANNEL_2;
//    motor2.port_in1 = GPIOB;
//    motor2.pin_in1 = GPIO_PIN_1;
//    motor2.port_in2 = GPIOB;
//    motor2.pin_in2 = GPIO_PIN_2;
//
//    // --- Motor 3 Configuration ---
//    motor3.htim = &htim1;
//    motor3.channel = TIM_CHANNEL_3;
//    motor3.port_in1 = GPIOA;
//    motor3.pin_in1 = GPIO_PIN_6;
//    motor3.port_in2 = GPIOA;
//    motor3.pin_in2 = GPIO_PIN_7;
//
//    // Start all PWM channels
//    HAL_TIM_PWM_Start(motor1.htim, motor1.channel);
//    HAL_TIM_PWM_Start(motor2.htim, motor2.channel);
//    HAL_TIM_PWM_Start(motor3.htim, motor3.channel);
//
//    Motor_StopAll();
//}
//
//void Motor_SetSpeed(Motor_t* motor, int32_t speed) {
//    // 1. Set the motor direction
//    Motor_SetDirection(motor, speed);
//
//    // 2. Set the speed (absolute value)
//    uint32_t pwm_pulse = (uint32_t)abs(speed);
//
//    // 3. Saturate the speed
//    // The PWM_MAX is 20000, but your old file used 1000.
//    // We will scale it. Your timer is 19999.
//    // We will assume a 0-1000 input range for simplicity.
////    if (pwm_pulse > 1000) {
////        pwm_pulse = 1000;
////    }
//
//    // Scale 0-1000 to 0-19999
//    // (This is faster than floating point)
//    pwm_pulse = pwm_pulse * 19999 / 1000;
//
//    // 4. Set the PWM duty cycle
//    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm_pulse);
//}
//
//static void Motor_SetDirection(Motor_t* motor, int32_t speed) {
//    if (speed > 0) {
//        // Forward
//        HAL_GPIO_WritePin(motor->port_in1, motor->pin_in1, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(motor->port_in2, motor->pin_in2, GPIO_PIN_RESET);
//    } else if (speed < 0) {
//        // Backward
//        HAL_GPIO_WritePin(motor->port_in1, motor->pin_in1, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(motor->port_in2, motor->pin_in2, GPIO_PIN_SET);
//    } else {
//        // Stop (Brake)
//        HAL_GPIO_WritePin(motor->port_in1, motor->pin_in1, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(motor->port_in2, motor->pin_in2, GPIO_PIN_RESET);
//    }
//}
//
//void Motor_StopAll(void) {
//    Motor_SetSpeed(&motor1, 0);
//    Motor_SetSpeed(&motor2, 0);
//    Motor_SetSpeed(&motor3, 0);
//}
