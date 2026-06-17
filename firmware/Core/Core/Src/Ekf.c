#include "Ekf.h"

static uint8_t is_initialized = 0;
static const float g = 9.81f;

static float32_t x_data[ 8 ] = {0};        // State
static float32_t P_data[ 64 ] = {0};       // Covariance
static float32_t Q_data[ 64 ] = {0};       // Process Noise
static float32_t R_data[ 16 ] = {0};       // Measurement Noise

static float32_t F_data[ 64 ] = {0};       // Physics Jacobian
static float32_t H_data[ 32 ] = {0};       // Sensor Jacobian
static float32_t y_data[ 4 ] = {0};        // Innovation (Surprise)
static float32_t z_data[ 4 ] = {0};        // Actual Sensors
static float32_t hx_data[ 4 ] = {0};       // Predicted Sensors
static float32_t S_data[ 16 ] = {0};       // Innovation Covariance
static float32_t S_inv_data[ 16 ] = {0};   // Inverse of S
static float32_t K_data[ 32 ] = {0};       // Kalman Gain

static float32_t t8x8_A[ 64 ] = {0};
static float32_t t8x8_B[ 64 ] = {0};
static float32_t t8x8_C[ 64 ] = {0};
static float32_t t4x8[ 32 ] = {0};
static float32_t t8x4_A[ 32 ] = {0};
static float32_t t8x4_B[ 32 ] = {0};
static float32_t t4x4[ 16 ] = {0};
static float32_t t8x1[ 8 ] = {0};
static float32_t I_data[ 64 ] = {0};

static arm_matrix_instance_f32 mat_x, mat_P, mat_Q, mat_R;
static arm_matrix_instance_f32 mat_F, mat_H, mat_y, mat_z, mat_hx;
static arm_matrix_instance_f32 mat_S, mat_S_inv, mat_K, mat_I;
static arm_matrix_instance_f32 mat_t8x8_A, mat_t8x8_B, mat_t8x8_C;
static arm_matrix_instance_f32 mat_t4x8, mat_t8x4_A, mat_t8x4_B;
static arm_matrix_instance_f32 mat_t4x4, mat_t8x1;


void BallbotEKF_Init(void) {

	arm_mat_init_f32(&mat_x, 8, 1, x_data);
    arm_mat_init_f32(&mat_P, 8, 8, P_data);
    arm_mat_init_f32(&mat_Q, 8, 8, Q_data);
    arm_mat_init_f32(&mat_R, 4, 4, R_data);
    arm_mat_init_f32(&mat_F, 8, 8, F_data);
    arm_mat_init_f32(&mat_H, 4, 8, H_data);
    arm_mat_init_f32(&mat_y, 4, 1, y_data);
    arm_mat_init_f32(&mat_z, 4, 1, z_data);
    arm_mat_init_f32(&mat_hx, 4, 1, hx_data);
    arm_mat_init_f32(&mat_S, 4, 4, S_data);
    arm_mat_init_f32(&mat_S_inv, 4, 4, S_inv_data);
    arm_mat_init_f32(&mat_K, 8, 4, K_data);
    arm_mat_init_f32(&mat_I, 8, 8, I_data);

    arm_mat_init_f32(&mat_t8x8_A, 8, 8, t8x8_A);
    arm_mat_init_f32(&mat_t8x8_B, 8, 8, t8x8_B);
    arm_mat_init_f32(&mat_t8x8_C, 8, 8, t8x8_C);
    arm_mat_init_f32(&mat_t4x8, 4, 8, t4x8);
    arm_mat_init_f32(&mat_t8x4_A, 8, 4, t8x4_A);
    arm_mat_init_f32(&mat_t8x4_B, 8, 4, t8x4_B);
    arm_mat_init_f32(&mat_t4x4, 4, 4, t4x4);
    arm_mat_init_f32(&mat_t8x1, 8, 1, t8x1);

    for(int i = 0; i < 64; i++) {
    	P_data[ i ] = 0;
    	Q_data[ i ] = 0;
    	F_data[ i ] = 0;
    	I_data[ i ] = 0; }
    for(int i = 0; i < 32; i++) {
    	H_data[ i ] = 0;
        K_data[ i ] = 0; }

    for(int i = 0; i < 16; i++) {
    	R_data[ i ] = 0;
        S_data[ i ] = 0; }

    for(int i = 0; i < 8;  i++) {
    	x_data[ i ] = 0; }

    // (Diagonal index for NxN matrix is i * (N + 1))
    for(int i = 0; i < 8; i++) {
        P_data[ i * 9 ] = 0.1f;   // P Diagonal
        Q_data[ i * 9 ] = 0.001f; // Q Diagonal (Tune this!)
        I_data[ i * 9 ] = 1.0f;   // Identity Matrix Diagonal
    }

    // P Matrix
    P_data[ 0 ] = 0.01f;   P_data[ 9 ] = 0.01f;   // Roll/Pitch Angles
    P_data[ 18 ] = 0.01f;  P_data[ 27 ] = 0.01f;  // Angular Rates
    P_data[ 36 ] = 0.25f;  P_data[ 45 ] = 0.25f;  // Linear Velocities (High initial doubt)
    P_data[ 54 ] = 1e-7f;  P_data[ 63 ] = 1e-7f;  // Gyro Biases (Near zero, as recommended)

    // Q Matrix (the process Noise - how much the physics model drifts)
    Q_data[ 0 ] = 0.0001f; Q_data[ 9 ] = 0.0001f; // Gyro angle noise 0.0005
    Q_data[ 18 ] = 0.001f; Q_data[ 27 ] = 0.001f; // Gyro rate noise0.001
    Q_data[ 36 ] = 0.5f;  Q_data[ 45 ] = 0.5f;  // Velocity integration noise
    Q_data[ 54 ] = 1e-6f;  Q_data[ 63 ] = 1e-6f;  // Biases drift VERY slowly over time

    // R Diagonal (Accel X, Accel Y, Enc Vx, Enc Vy) the sensor errors
    R_data[ 0 ]  = 5.0f;
    R_data[ 5 ]  = 5.0f;
    R_data[ 10 ] = 1.0f;
    R_data[ 15 ] = 1.0f;

    is_initialized = 1;
}


void BallbotEKF_Update(float gyro_x, float gyro_y,
                       float accel_x, float accel_y,
                       float enc_vx, float enc_vy,
                       float dt,
                       BallbotStates* out_states)
{
    if (!is_initialized) BallbotEKF_Init();

    float phi   = x_data[ 0 ];
    float theta = x_data[ 1 ];
    float vx    = x_data[ 4 ];
    float vy    = x_data[ 5 ];
    float bx    = x_data[ 6 ];
    float by    = x_data[ 7 ];

    float accel_magnitude = sqrtf(accel_x*accel_x + accel_y*accel_y);
//    float safe_ax, safe_ay;
//    if (accel_magnitude > 147.0f || accel_magnitude < 2.0f) {
//        // Use predicted accel from current angle estimate — zero innovation
//        safe_ax = g * sinf(x_data[1]);                        // g*sin(theta)
//        safe_ay = -g * sinf(x_data[0]) * cosf(x_data[1]);    // -g*sin(phi)*cos(theta)
//    } else {
//        safe_ax = accel_x;
//        safe_ay = accel_y;
//    }

    float safe_enc_vx = fmaxf(-5.2f, fminf(5.2f, enc_vx));
    float safe_enc_vy = fmaxf(-4.5f, fminf(4.5f, enc_vy));
    // Update State (x = f(x))
    x_data[ 0 ] = phi + (gyro_x - bx) * dt;
    x_data[ 1 ] = theta + (gyro_y - by) * dt;
    x_data[ 2 ] = gyro_x - bx;
    x_data[ 3 ] = gyro_y - by;
    x_data[ 4 ] = vx;
    x_data[ 5 ] = vy;



    // Build F Matrix Jacobian
    for(int i=0; i<64; i++) F_data[ i ] = 0.0f;
    for(int i=0; i<8; i++)  F_data[ i*8 + i ] = 1.0f; // Eye diagonal

    F_data[ 0*8 + 6 ] = -dt;
    F_data[ 1*8 + 7 ] = -dt;
    F_data[ 2*8 + 2 ] = 0.0f;
    F_data[ 2*8 + 6 ] = -1.0f;
    F_data[ 3*8 + 3 ] = 0.0f;
    F_data[ 3*8 + 7 ] = -1.0f;

    // Predict Covariance (P = F * P * F^T + Q)
    arm_mat_trans_f32(&mat_F, &mat_t8x8_A);                 // t8x8_A = F^T
    arm_mat_mult_f32(&mat_F, &mat_P, &mat_t8x8_B);          // t8x8_B = F * P
    arm_mat_mult_f32(&mat_t8x8_B, &mat_t8x8_A, &mat_P);     // P = F * P * F^T
    arm_mat_add_f32(&mat_P, &mat_Q, &mat_P);                // P = P + Q


    // Build H Matrix Jacobian
    for(int i=0; i<32; i++) H_data[ i ] = 0.0f;
    H_data[ 0*8 + 1 ] = g * cosf(x_data[ 1 ]);
    H_data[ 1*8 + 0 ] = -g * cosf(x_data[ 0 ]) * cosf(x_data[ 1 ]);
    H_data[ 1*8 + 1 ] = g * sinf(x_data[ 0 ]) * sinf(x_data[ 1 ]);
    H_data[ 2*8 + 4 ] = 1.0f;
    H_data[ 3*8 + 5 ] = 1.0f;

    // Calculate Expected Sensors and Innovation (y = z - h_x)
    hx_data[ 0 ] = g * sinf(x_data[ 1 ]);
    hx_data[ 1 ] = -g * sinf(x_data[ 0 ]) * cosf(x_data[ 1 ]);
    hx_data[ 2 ] = x_data[ 4 ];
    hx_data[ 3 ] = x_data[ 5 ];

    // Add after building hx_data, before arm_mat_sub_f32:

    // Use safe values in measurement vector
    z_data[ 0 ] = accel_x;   // NOT accel_x
    z_data[ 1 ] = accel_y;   // NOT accel_y
    z_data[ 2 ] = safe_enc_vx;
    z_data[ 3 ] = safe_enc_vy;

    arm_mat_sub_f32(&mat_z, &mat_hx, &mat_y);

    // Innovation Covariance (S = H * P * H^T + R)
    arm_mat_trans_f32(&mat_H, &mat_t8x4_A);                 // t8x4_A = H^T
    arm_mat_mult_f32(&mat_H, &mat_P, &mat_t4x8);            // t4x8 = H * P
    arm_mat_mult_f32(&mat_t4x8, &mat_t8x4_A, &mat_S);       // S = H * P * H^T
    arm_mat_add_f32(&mat_S, &mat_R, &mat_S);                // S = S + R

    // Kalman Gain (K = P * H^T * S_inv) -- WITH SAFETY CHECK
    if (arm_mat_inverse_f32(&mat_S, &mat_S_inv) != ARM_MATH_SUCCESS) {
    	return;
    }
    arm_mat_mult_f32(&mat_P, &mat_t8x4_A, &mat_t8x4_B);     // t8x4_B = P * H^T
    arm_mat_mult_f32(&mat_t8x4_B, &mat_S_inv, &mat_K);      // K = (P * H^T) * S_inv

    // Update State (x = x + K * y)
    arm_mat_mult_f32(&mat_K, &mat_y, &mat_t8x1);            // t8x1 = K * y
    arm_mat_add_f32(&mat_x, &mat_t8x1, &mat_x);             // x = x + K * y

    // Update Covariance (Joseph Form)
    // Part 1: (I - K*H)
    arm_mat_mult_f32(&mat_K, &mat_H, &mat_t8x8_A);          // t8x8_A = K * H
    arm_mat_sub_f32(&mat_I, &mat_t8x8_A, &mat_t8x8_B);      // t8x8_B = (I - K*H)

    // Part 2: (I - K*H) * P * (I - K*H)^T
    arm_mat_mult_f32(&mat_t8x8_B, &mat_P, &mat_t8x8_A);     // t8x8_A = (I - K*H) * P
    arm_mat_trans_f32(&mat_t8x8_B, &mat_t8x8_C);            // t8x8_C = (I - K*H)^T
    arm_mat_mult_f32(&mat_t8x8_A, &mat_t8x8_C, &mat_t8x8_B);// t8x8_B = Part 2 Result

    // Part 3: K * R * K^T
    arm_mat_mult_f32(&mat_K, &mat_R, &mat_t8x4_A);          // t8x4_A = K * R
    arm_mat_trans_f32(&mat_K, &mat_t4x8);                   // t4x8 = K^T
    arm_mat_mult_f32(&mat_t8x4_A, &mat_t4x8, &mat_t8x8_A);  // t8x8_A = Part 3 Result

    // Final P = Part 2 + Part 3
    arm_mat_add_f32(&mat_t8x8_B, &mat_t8x8_A, &mat_P);

    if (P_data[ 0]>1.0f) P_data[ 0]=1.0f;  /* phi */
    if (P_data[ 9]>1.0f) P_data[ 9]=1.0f;  /* theta */
    if (P_data[36]>4.0f) P_data[36]=4.0f;  /* vx */
    if (P_data[45]>4.0f) P_data[45]=4.0f;  /* vy */

    if (out_states != NULL) {
        out_states->phi    = x_data[ 0 ];
        out_states->theta  = x_data[ 1 ];
        out_states->dphi   = x_data[ 2 ];
        out_states->dtheta = x_data[ 3 ];
        out_states->vx     = x_data[ 4 ];
        out_states->vy     = x_data[ 5 ];
        out_states->bx     = x_data[ 6 ];
        out_states->by     = x_data[ 7 ];
    }
}
