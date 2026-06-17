%clear; clc; close all;

disp('--- Starting EKF Symbolic Derivation ---');
syms phi theta dphi dtheta vx vy bx by real      % 8 states
syms omega_x omega_y accel_x accel_y real        % IMU inputs
syms ax_actual ay_actual real                    % Encoder inputs
syms dt g real                                   
x_state = [phi; theta; dphi; dtheta; vx; vy; bx; by];

f_x = [ phi + (omega_x - bx) * dt;                                 % New Roll
        theta + (omega_y - by) * dt;                               % New Pitch
        omega_x - bx;                                              % New Roll Rate
        omega_y - by;                                              % New Pitch Rate
        vx + (accel_x - g * sin(theta)) * dt;                      % New Vel X
        vy + (accel_y + g * sin(phi) * cos(theta)) * dt;           % New Vel Y
        bx;                                                        % Bias X 
        by ];                                                      % Bias Y

h_x = [ g * sin(theta) + ax_actual;                                % Expected Accel X
       -g * sin(phi) * cos(theta) + ay_actual;                     % Expected Accel Y
        vx;                                                        % Expected Enc X
        vy ];                                                      % Expected Enc Y

disp('Calculating Jacobian F (Prediction Matrix)...');
F_matrix = jacobian(f_x, x_state);

disp('Calculating Jacobian H (Measurement Matrix)...');
H_matrix = jacobian(h_x, x_state);

disp('--- F MATRIX ---');
pretty(F_matrix) 
disp('--- H MATRIX ---');
pretty(H_matrix)

% =========================================================================
dt = 0.002;             % 200 Hz control loop
t = 0:dt:10;           
N = length(t);          
g = 9.81;

clear optimized_ballbot_EKF1; 

true_phi   = 0.05 * sin(2*pi*0.3*t);                % Roll wobbling at 0.3 Hz
true_theta = 0.15 * cos(2*pi*0.5*t);                % Pitch wobbling at 0.5 Hz
true_vx    = 0.5 * t;                               % Constant acceleration to 5 m/s
true_vy    = 0.2 * sin(2*pi*0.2*t);                 % Slight side-to-side drift

% Take derivatives to get true rates and physical accelerations
true_omega_x = 0.05 * (2*pi*0.3) * cos(2*pi*0.3*t); % d/dt of phi
true_omega_y = -0.15 * (2*pi*0.5) * sin(2*pi*0.5*t);% d/dt of theta
true_ax      = 0.5 * ones(1, N);                    % d/dt of vx
true_ay      = 0.2 * (2*pi*0.2) * cos(2*pi*0.2*t);  % d/dt of vy

% True static biases for the gyroscope
true_bx = 0.02; 
true_by = -0.015;

% Define how terrible our sensors are (Standard Deviations)
noise_gyro  = 0.05;  % rad/s
noise_accel = 0.6;    % m/s^2 (High noise from motor vibrations!)
noise_enc   = 0.02;   % m/s

gyro_meas  = zeros(2, N);
accel_meas = zeros(2, N);
enc_meas   = zeros(2, N);

for k = 1:N
    % Gyro measures True Rate + Bias + Noise
    gyro_meas(1,k) = true_omega_x(k) + true_bx + randn() * noise_gyro;
    gyro_meas(2,k) = true_omega_y(k) + true_by + randn() * noise_gyro;
    
    % Accel measures Gravity Projection + Physical Accel + Vibration Noise
    accel_meas(1,k) = g * sin(true_theta(k)) + true_ax(k) + randn() * noise_accel;
    accel_meas(2,k) = -g * sin(true_phi(k)) * cos(true_theta(k)) + true_ay(k) + randn() * noise_accel;
    
    % Encoders measure True Velocity + Noise
    enc_meas(1,k) = true_vx(k) + randn() * noise_enc;
    enc_meas(2,k) = true_vy(k) + randn() * noise_enc;
end

% PART 3: RUN THE EKF
ekf_states = zeros(8, N);
disp('Running EKF Simulation...');
for k = 1:N
    [x_hat, Pout] = optimized_ballbot_EKF1(gyro_meas(:,k), accel_meas(:,k), enc_meas(:,k), dt);
    
    % Log the state estimates
    ekf_states(:,k) = x_hat;
end
disp('Simulation Complete!');

% PART 4: PLOT THE RESULTS
% =========================================================================

% --- Plot 1: Pitch Angle (Theta) ---
subplot(2,2,1); hold on; grid on;
plot(t, true_theta, 'g', 'LineWidth', 2);
plot(t, ekf_states(2,:), 'b--', 'LineWidth', 1.5);
title('Pitch Angle (\theta)');
xlabel('Time (s)'); ylabel('rad');
legend('True Physics', 'EKF Estimate');

% --- Plot 2: Forward Velocity (Vx) ---
subplot(2,2,2); hold on; grid on;
plot(t, true_vx, 'g', 'LineWidth', 2);
plot(t, enc_meas(1,:), 'r.', 'MarkerSize', 2); % Show raw noisy encoder data
plot(t, ekf_states(5,:), 'b--', 'LineWidth', 1.5);
title('Forward Velocity (v_x)');
xlabel('Time (s)'); ylabel('m/s');
legend('True Speed', 'Raw Encoder', 'EKF Estimate');

% --- Plot 3: Gyro Bias Tracking (By) ---
subplot(2,2,3); hold on; grid on;
plot(t, true_by * ones(1,N), 'g', 'LineWidth', 2);
plot(t, ekf_states(8,:), 'b--', 'LineWidth', 1.5);
title('Gyro Y Bias Estimation');
xlabel('Time (s)'); ylabel('rad/s');
legend('True Bias', 'EKF Estimate');

% --- Plot 4: Roll Angle (Phi) ---
subplot(2,2,4); hold on; grid on;
plot(t, true_phi, 'g', 'LineWidth', 2);
plot(t, ekf_states(1,:), 'b--', 'LineWidth', 1.5);
title('Roll Angle (\phi)');
xlabel('Time (s)'); ylabel('rad');
legend('True Physics', 'EKF Estimate');


% =========================================================================
function [x_hat_out, Pout] = optimized_ballbot_EKF1(gyro_raw, accel_raw, enc_v, dt)
    persistent x_hat P Q R_base g
    if isempty(x_hat)
        x_hat = zeros(8,1); %intialize 8 states by zero
        P = eye(8) * 0.1;
        g = 9.81;
        Q = diag([1e-5, 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-6, 1e-6]); %This is the process noise could be tuned
        R_base = diag([5.0, 5.0, 0.05, 0.05]);% [Accelx, Accely, encoderx, encodery] this is sensor noise       
    end
    phi = x_hat(1);
    theta = x_hat(2);
    vx  = x_hat(5);
    vy = x_hat(6);
    bx  = x_hat(7);
    by = x_hat(8);
    
    omega_x_clean = gyro_raw(1) - bx;
    omega_y_clean = gyro_raw(2) - by;
    
    %predicted states
    x_pred= zeros(8,1);
    x_pred(1) = phi + omega_x_clean * dt;
    x_pred(2) = theta + omega_y_clean * dt;
    x_pred(3) = omega_x_clean;
    x_pred(4) = omega_y_clean;
    x_pred(5) = vx + (accel_raw(1) - g * sin(theta)) * dt; % Velocity predict X
    x_pred(6) = vy + (accel_raw(2) + g * sin(phi) * cos(theta)) * dt; % Velocity predict Y
    x_pred(7) = bx;
    x_pred(8) = by;
    
    F = eye(8);
    F(1,7) = -dt; 
    F(2,8) = -dt;
    F(3,3) = 0;
    F(3,7) = -1;
    F(4,4) = 0;
    F(4,8) = -1;
    F(5,2) = -g * cos(theta) * dt;
    F(6,1) = g * cos(phi) * cos(theta) * dt;
    F(6,2) = -g * sin(phi) * sin(theta) * dt;
    
    % Predicted Covariance
    P_pred = F * P * F' + Q;
    
    % Calculate actual physical acceleration from the encoders of stepper
    % motors 
    ax_actual = (enc_v(1) - vx) / dt;
    ay_actual = (enc_v(2) - vy) / dt;
    
    % If the robot is accelerating hard, the accelerometer gets noisy. 
    % We mathematically inflate the R matrix so the EKF ignores the
    % Accel. measurment and relay on the dynamics
    R_adaptive = R_base;
    accel_penalty = 1.0 + 10.0 * sqrt(ax_actual^2 + ay_actual^2); 
    R_adaptive(1,1) = R_adaptive(1,1) * accel_penalty;
    R_adaptive(2,2) = R_adaptive(2,2) * accel_penalty;
    
    z = [accel_raw(1); accel_raw(2); enc_v(1); enc_v(2)];
    
    % Expected Sensor Readings h(x)
    h_x = zeros(4,1);
    h_x(1) = g * sin(x_pred(2)) + ax_actual;               % Accel X expects
    h_x(2) = -g * sin(x_pred(1)) * cos(x_pred(2)) + ay_actual; % Accel Y expects
    h_x(3) = x_pred(5);                                    % Enc X expects
    h_x(4) = x_pred(6);                                    % Enc Y expects
    
    % Optimized Jacobian H
    H = zeros(4, 8);
    H(1,2) = g * cos(x_pred(2));
    H(2,1) = -g * cos(x_pred(1)) * cos(x_pred(2));
    H(2,2) = g * sin(x_pred(1)) * sin(x_pred(2));
    H(3,5) = 1;
 %   H(1,4) = -1/dt;
  %  H(2,5) = -1/dt;

    H(4,6) = 1;
    
    % Kalman Gain Calculation
    S = H * P_pred * H' + R_adaptive;
    K = P_pred * H' / S; 
    
    x_hat = x_pred + K * (z - h_x);
    P = (eye(8) - K * H) * P_pred;

    x_hat_out = x_hat;
    Pout = P; 
end

