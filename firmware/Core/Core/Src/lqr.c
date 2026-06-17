/*
 * lqr.c — Corrected for hardware testing
 *
 * Changes from previous version:
 * 1. error_state[8,9] now receives integral values (was zeroed — LQI disabled)
 * 2. MIX_MATRIX removed (K_LQI already outputs per-motor torques)
 * 3. MAX_INTEGRAL reduced to 0.5 for first balance test
 */

#include "lqr.h"

//const float K_LQI[3][10] = {
//    {-10.6863f,  18.3029f,  0.1281f,   0.5576f, 0.0001f, -2.3690f,   4.1997f, 0.0892f,  0.1128f,  0.0000f},
//    {-10.5151f, -18.4017f,  0.1281f,  -0.2787f, 0.4830f, -2.4691f,  -4.1418f, 0.0892f, -0.0564f,  0.0977f},
//    { 21.2013f,  0.0988f,   0.1281f,  -0.2789f,-0.4831f,  4.8382f,  -0.0580f, 0.0892f, -0.0564f, -0.0977f}
//};

//const float K_LQI[3][10] = {
//    {  8.5684f, -14.8026f,  1.2356f, -1.6587f, -0.9600f,  1.3350f, -2.2820f,  0.2620f, -0.9557f, -0.5521f},
//    {  8.5684f,  14.8026f,  1.2356f,  1.6587f, -0.9600f,  1.3350f,  2.2820f,  0.2620f,  0.9557f, -0.5521f},
//    {-17.1368f,   0.0000f,  1.2356f, -0.0000f,  1.9200f, -2.6699f,  0.0000f,  0.2620f, -0.0000f,  1.1042f}
//};

//3mlt update lel weight
//const float K_LQI[3][10] = {
//    { 10.2284f, -17.6871f,  0.7909f, -1.5998f, -0.9252f,  1.6026f, -2.7477f,  0.1921f, -0.7434f, -0.4294f},
//    { 10.2284f,  17.6871f,  0.7909f,  1.5998f, -0.9252f,  1.6026f,  2.7477f,  0.1921f,  0.7434f, -0.4294f},
//    {-20.4568f,  -0.0000f,  0.7909f, -0.0000f,  1.8503f, -3.2052f, -0.0000f,  0.1921f, -0.0000f,  0.8588f}
//};

//3mlt update lel J kman
//const float K_LQI[3][10] = {
//    { 10.3031f, -17.8132f,  0.7909f, -1.6111f, -0.9318f,  1.6756f, -2.8705f,  0.1921f, -0.7447f, -0.4301f},
//    { 10.3031f,  17.8132f,  0.7909f,  1.6111f, -0.9318f,  1.6756f,  2.8705f,  0.1921f,  0.7447f, -0.4301f},
//    {-20.6061f,  -0.0000f,  0.7909f, -0.0000f,  1.8637f, -3.3512f, -0.0000f,  0.1921f, -0.0000f,  0.8602f}
//};

//3mlt update lel Jw
//const float K_LQI[3][10] = {
//    { 10.3882f, -17.9611f,  0.7943f, -1.6190f, -0.9364f,  1.7045f, -2.9207f,  0.2073f, -0.7450f, -0.4303f},
//    { 10.3882f,  17.9611f,  0.7943f,  1.6190f, -0.9364f,  1.7045f,  2.9207f,  0.2073f,  0.7450f, -0.4303f},
//    {-20.7765f,   0.0000f,  0.7943f,  0.0000f,  1.8727f, -3.4090f,  0.0000f,  0.2073f,  0.0000f,  0.8606f}
//};

//3mlt update lel Jw tany
const float K_LQI[3][10] = {
    { 10.4241, -18.0232,  0.7953, -1.6223, -0.9382,  1.7166, -2.9418,  0.2133, -0.7452, -0.4304 },
    { 10.4241,  18.0232,  0.7953,  1.6223, -0.9382,  1.7166,  2.9418,  0.2133,  0.7452, -0.4304 },
    {-20.8481, -0.0000,  0.7953,  0.0000,  1.8765, -3.4333,  0.0000,  0.2133,  0.0000,  0.8608 }
};

// Physical Limits
const float MAX_MOTOR_TORQUE = 1.2f;
const float MAX_YAW_RATE     = 1.0f;
const float LPF_ALPHA        = 0.2f;
const float MAX_INTEGRAL     = 0.1f;   // reduced from 2.0 — prevents windup on first test
const uint32_t WATCHDOG_TIMEOUT_MS = 500;
extern float integral_error_x;
extern float integral_error_y;


float current_state[10] = {0};
float reference_state[10] = {0};
float error_state[10] = {0};

float target_world_x = 0.0f;
float target_world_y = 0.0f;
float target_yaw_rate = 0.0f;
uint32_t last_ros_msg_time = 0;

float integral_error_x = 0.0f;
float integral_error_y = 0.0f;
float prev_filtered_torque[3] = {0.0f, 0.0f, 0.0f};

float physical_torque[3] = {0};

void updateReferencesFromROS(float ros_v_x, float ros_v_y, float ros_yaw_rate, uint32_t current_time_ms) {
    last_ros_msg_time = current_time_ms;
    target_world_x = ros_v_x;
    target_world_y = ros_v_y;
    if (ros_yaw_rate > MAX_YAW_RATE)  ros_yaw_rate = MAX_YAW_RATE;
    if (ros_yaw_rate < -MAX_YAW_RATE) ros_yaw_rate = -MAX_YAW_RATE;
    target_yaw_rate = ros_yaw_rate;
}

void controlLoop(float dt, uint32_t current_time_ms, float* out_motor_torques) {

    // Watchdog — zero velocity targets if no ROS message for 500ms
//    if ((current_time_ms - last_ros_msg_time) > WATCHDOG_TIMEOUT_MS) {
//        target_world_x = 0.0f;
//        target_world_y = 0.0f;
//        target_yaw_rate = 0.0f;
//    }

    // ── STEP 1: Compute error for the 8 inner states ──
    for (int i = 0; i < 8; i++) {
        error_state[i] = current_state[i] - reference_state[i];
    }

    // ── STEP 2: Integrate velocity error (states 3=Vx, 4=Vy) ──
    integral_error_x += error_state[3] * dt;
    integral_error_y += error_state[4] * dt;

    // Clamp integrals to prevent windup
    if (integral_error_x >  MAX_INTEGRAL) integral_error_x =  MAX_INTEGRAL;
    if (integral_error_x < -MAX_INTEGRAL) integral_error_x = -MAX_INTEGRAL;
    if (integral_error_y >  MAX_INTEGRAL) integral_error_y =  MAX_INTEGRAL;
    if (integral_error_y < -MAX_INTEGRAL) integral_error_y = -MAX_INTEGRAL;

    error_state[8] = integral_error_x;
    error_state[9] = integral_error_y;


    // ── STEP 3: Matrix multiply — torques = -K * error ──
    for (int i = 0; i < 3; i++) {
        physical_torque[i] = 0.0f;
        for (int j = 0; j < 10; j++) {
            physical_torque[i] -= K_LQI[i][j] * error_state[j];
        }
    }

    // ── STEP 4: Proportional torque scaling (preserves direction ratios) ──
    float max_req_torque = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (fabsf(physical_torque[i]) > max_req_torque) {
            max_req_torque = fabsf(physical_torque[i]);
        }
    }
    if (max_req_torque > MAX_MOTOR_TORQUE) {
        float scale = MAX_MOTOR_TORQUE / max_req_torque;
        for (int i = 0; i < 3; i++) {
            physical_torque[i] *= scale;
        }
    }

    // ── STEP 5: Low-pass filter to smooth motor commands ──
    for (int i = 0; i < 3; i++) {
        float smoothed = (LPF_ALPHA * physical_torque[i]) + ((1.0f - LPF_ALPHA) * prev_filtered_torque[i]);
        prev_filtered_torque[i] = smoothed;
        physical_torque[i] = smoothed;
    }

    out_motor_torques[0] = physical_torque[0];
    out_motor_torques[1] = physical_torque[1];
    out_motor_torques[2] = physical_torque[2];
}
