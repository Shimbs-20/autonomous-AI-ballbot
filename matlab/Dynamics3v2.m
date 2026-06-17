clear; clc; close all;
disp('Starting Euler Math Engine... (Linearizing dynamics)');

syms Ix Iy phi theta psi real           
syms dIx dIy dphi dtheta dpsi real 
%phi (Roll): Tilting left/right. theta (Pitch): Tilting forward/backward.
% psi (Yaw): Spinning around the vertical axis.
chi  = [Ix; Iy; phi; theta; psi];
dchi = [dIx; dIy; dphi; dtheta; dpsi];

Mk = 3.45; Mb = 19; rk = 0.35/2; rw = 0.125/2; l = 0.336; g = 9.81;      
Jk = (2/3) * Mk * rk^2;
Jw = 0.00018883; 
Jbx = 0.6607;
Jby = 0.6370;
Jbz = 0.1744;              
J_body_COM = diag([Jbx, Jby, Jbz]);

Rx = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta);
      0 1 0;
      -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0;
      sin(psi) cos(psi) 0;
      0 0 1];
R_b2i = Rz * Ry * Rx;

T_euler = [1,  0,         -sin(theta);
           0,  cos(phi),   sin(phi)*cos(theta);
           0, -sin(phi),   cos(phi)*cos(theta)];
omega_body = T_euler * [dphi; dtheta; dpsi]; 
% Omniwheel Geometry (120 degrees apart)
skew_r = [0 -l 0; l 0 0; 0 0 0]; 
J_b_origin = J_body_COM - Mb * (skew_r * skew_r);
alpha = deg2rad(45);  
g3 = 0; g1 = deg2rad(120); g2 = deg2rad(240);
% mapping the 3D angular velocity of the ball to the angular velocity of the three motors
H_geom = (rk/rw) * [ -cos(alpha)*cos(g1), -cos(alpha)*sin(g1), sin(alpha);
                    -cos(alpha)*cos(g2), -cos(alpha)*sin(g2), sin(alpha);
                    -cos(alpha)*cos(g3), -cos(alpha)*sin(g3), sin(alpha) ];
% Kinetic & Potential Energy
v_ball_sq = dIx^2 + dIy^2;
T_ball = 0.5 * (Mk + Jk/rk^2) * v_ball_sq;
v_ball_vec = [dIx; dIy; 0];
v_com_world = v_ball_vec + R_b2i * cross(omega_body, [0;0;l]);

% T_body = 0.5 * Mb * (v_com_world.' * v_com_world) + 0.5 * omega_body.' * J_b_origin * omega_body;

T_body = 0.5 * Mb * (v_com_world.' * v_com_world) + 0.5 * omega_body.' * J_body_COM * omega_body;

% theta_dot = H_geom * omega_body + (1/rk) * H_geom * v_ball_b;
v_ball_b = R_b2i.' * v_ball_vec;
omega_ball_b = [ v_ball_b(2)/rk; 
                -v_ball_b(1)/rk; 
                 omega_body(3) ]; % Assumes yaw locking or relative slip

theta_dot = H_geom * omega_body + H_geom * omega_ball_b;

T_wheels = 0.5 * Jw * (theta_dot.' * theta_dot);

pos_com_world = [Ix; Iy; rk] + R_b2i * [0; 0; l];
V_body = Mb * g * pos_com_world(3);
T_total = T_ball + T_body + T_wheels;

% Lagrangian Derivations
M_mat = hessian(T_total, dchi);
G_vec = jacobian(V_body, chi).';
B_u   = jacobian(theta_dot, dchi).';

% Linearize around upright equilibrium (all zeros)
x0_eq  = zeros(5,1);
dx0_eq = zeros(5,1);
dG_dx = jacobian(G_vec, chi);

M_eq  = double(subs(M_mat, [chi; dchi], [x0_eq; dx0_eq]));
B_eq  = double(subs(B_u,   [chi; dchi], [x0_eq; dx0_eq]));
dG_eq = double(subs(dG_dx, [chi; dchi], [x0_eq; dx0_eq]));

A_bot = -M_eq \ dG_eq;
B_bot = M_eq \ B_eq;
A_sys = [zeros(5,5),  eye(5);
         A_bot,       zeros(5,5)];
B_sys = [zeros(5,3);
         B_bot];
%for the open loop system matrix
lambda = eig(A_sys);

% 2. LQI (Inner Balance + Outer Velocity)
disp('Designing LQI Velocity + Balance Controller');
A_bal = A_sys(3:10, 3:10); 
B_bal = B_sys(3:10, :);

% Cv maps the states we want to track with integrals. 
% State vector is [phi, theta, psi, Vx, Vy, dphi, dtheta, dpsi]
% We want to track Vx (Index 4) and Vy (Index 5)
Cv = [0, 0, 0, 1, 0, 0, 0, 0;  % Extracts Vx
      0, 0, 0, 0, 1, 0, 0, 0]; % Extracts Vy

% Augment the A and B matrices with 2 new Integral States
A_aug = [A_bal, zeros(8, 2);
         Cv,    zeros(2, 2)];
B_aug = [B_bal;
         zeros(2, 3)];

% Tuning Weights for the 10-state LQI Matrix
% [phi, theta, psi, Vx, Vy, dphi, dtheta, dpsi, INT_Vx, INT_Vy]
%1. Tune the R Matrix (The Control Theory Fix)In your MATLAB script,
% the R matrix represents the cost of using the motors. Right now,
% it is likely set too low. If we increase the values in the R matrix,
% you are mathematically telling the controller:Using motor voltage is very expensive.
% Only use it for real falls, not tiny sensor vibrations.This will instantly smooth out the chattering
Q_aug = diag([1000, 1000, 500, 10, 10, 50, 50, 10, 300, 300]); 
R_aug = diag([200, 200, 200]);         

K_lqi = lqr(A_aug, B_aug, Q_aug, R_aug);

% Eigenvalues of the augmented closed-loop system you ensure that your
% LQI design has successfully moved all the unstable open-loop poles into the stable left-half plane
lambda_cl = eig(A_aug - B_aug * K_lqi);

% DISCRETE-TIME CONVERSION (FOR THE STM32 MICROCONTROLLER)
disp('Discretizing the Augmented System for STM32...');
Ts = 0.005; % 5ms timer loop (200Hz)

% 1. Create continuous model using your 10-state Augmented matrices
sys_continuous = ss(A_aug, B_aug, eye(10), zeros(10,3));

% 2. Discretize using Zero-Order Hold
sys_discrete = c2d(sys_continuous, Ts, 'zoh');

% Extract the discrete matrices (You can document these in your paper!)
A_aug_d = sys_discrete.A;
B_aug_d = sys_discrete.B;

% 3. Calculate the Digital Controller Gain
K_discrete = dlqr(A_aug_d, B_aug_d, Q_aug, R_aug);

% 4. Print for the STM32
disp('COPY THIS K_discrete MATRIX INTO YOUR STM32 C++ CODE:');
disp(K_discrete);

% -------------------------------------------------------------------------
% 3. SIMULATION LOOP (Velocity Flight Plan)
% -------------------------------------------------------------------------
disp('Running 3D Physics Simulation...');
dt = 0.005; % 200Hz
t_sim = 0:dt:10; 
x_full = zeros(10,1);

% Initialize the integral memory to zero
int_err = [0; 0]; % [Integral_Vx_Error; Integral_Vy_Error]

history = zeros(length(t_sim), 10);
cmd_history = zeros(length(t_sim), 3); % [Cmd_Vx, Cmd_Vy, Cmd_Yaw]
u_history = zeros(length(t_sim), 3);

smooth_cmd_vx = 0;
smooth_cmd_vy = 0;
smooth_cmd_yaw = 0;
filter_alpha = 0.02; % How fast the robot accelerates (lower = smoother)
for i = 1:length(t_sim)
    current_time = t_sim(i);

    % --- THE VELOCITY FLIGHT PLAN ---
    if current_time < 2.0
        % Phase 1: Drive forward at 0.5 m/s
        cmd_vx = 1; cmd_vy = 0; cmd_yaw = 0;
    elseif current_time < 4.0
        % Phase 2: Strafe sideways at 0.5 m/s
        cmd_vx = 1; cmd_vy = 1; cmd_yaw = 0;
    elseif current_time < 6.0
        % Phase 3: Spin in place while holding position
        cmd_vx = 0.75; cmd_vy = 0.5; cmd_yaw = deg2rad(25);
    else
        % Phase 4: Full Stop
        cmd_vx = 0; cmd_vy = 0; cmd_yaw = 0;
    end
% ---> FILTER MATH <---
    smooth_cmd_vx = smooth_cmd_vx * (1 - filter_alpha) + cmd_vx * filter_alpha;
    smooth_cmd_vy = smooth_cmd_vy * (1 - filter_alpha) + cmd_vy * filter_alpha;
    smooth_cmd_yaw = smooth_cmd_yaw * (1 - filter_alpha) + cmd_yaw * filter_alpha;

    x_inner = x_full(3:10); 
    % 1. Calculate Velocity Error and Integrate
    err_vx = x_inner(4) - smooth_cmd_vx;
    err_vy = x_inner(5) - smooth_cmd_vy;
    int_err = int_err + [err_vx; err_vy] * dt;

% 2. Set Reference for non-integral states
    x_ref_inner = zeros(8,1);
    x_ref_inner(3) = cmd_yaw; 
    x_ref_inner(4) = cmd_vx; 
    x_ref_inner(5) = cmd_vy;  
    % 3. Calculate Errors
    state_err = x_inner - x_ref_inner;

    % 4. Combine into Augmented 10-State Error Vector
    aug_err = [state_err; int_err];

    % 5. Apply LQI Control Matrix
    %u = -K_lqi * aug_err; 

    % 5. Apply Discrete Control Matrix (The STM32 Brain)
    u = -K_discrete * aug_err;

    u_history(i,:) = u'; % Transpose u and save it to the history log

    % 6. Apply to system dynamics
    x_dot = A_sys * x_full + B_sys * u; 
    x_full = x_full + x_dot * dt;

    history(i,:) = x_full';
    cmd_history(i,:) = [cmd_vx, cmd_vy, cmd_yaw];
end

% -------------------------------------------------------------------------
% 4. PLOT RESULTS ()
% -------------------------------------------------------------------------
figure('Name', 'LQI Velocity Control Proof', 'Position', [100, 50, 900, 800]);

% 1. Forward Velocity (Vx)
subplot(4,1,1);
plot(t_sim, history(:,6), 'r', 'LineWidth', 2); hold on;
plot(t_sim, cmd_history(:,1), 'k--', 'LineWidth', 1.5);
title('Forward Velocity (Vx) Tracking');
ylabel('m/s'); legend('Actual Vx', 'Target Vx', 'Location', 'Southeast'); grid on;

% 2. Sideways Velocity (Vy)
subplot(4,1,2);
plot(t_sim, history(:,7), 'b', 'LineWidth', 2); hold on;
plot(t_sim, cmd_history(:,2), 'k--', 'LineWidth', 1.5);
title('Sideways Velocity (Vy) Tracking');
ylabel('m/s'); legend('Actual Vy', 'Target Vy', 'Location', 'Southeast'); grid on;

subplot(4,1,3);
plot(t_sim, rad2deg(history(:,4)), 'r', 'LineWidth', 2); hold on; 
plot(t_sim, rad2deg(history(:,3)), 'b', 'LineWidth', 2);
title('Autonomous Leaning (Pitch & Roll)');
ylabel('Degrees'); legend('Pitch (Causes Vx)', 'Roll (Causes Vy)', 'Location', 'Northeast'); grid on;

% 4. Yaw Tracking
subplot(4,1,4);
plot(t_sim, rad2deg(history(:,5)), 'g', 'LineWidth', 2); hold on; 
plot(t_sim, rad2deg(cmd_history(:,3)), 'k--', 'LineWidth', 1.5);
title('Yaw (\psi) Tracking');
xlabel('Time (s)'); ylabel('Degrees'); legend('Actual Yaw', 'Target', 'Location', 'Southeast'); grid on;

figure('Name', 'Motor Torque Demands (Reality Check)', 'Position', [550, 50, 600, 400]);
plot(t_sim, u_history(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t_sim, u_history(:,2), 'b', 'LineWidth', 1.5);
plot(t_sim, u_history(:,3), 'g', 'LineWidth', 1.5);
title('Required Motor Torque over Time');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('Motor 1 (\tau_1)', 'Motor 2 (\tau_2)', 'Motor 3 (\tau_3)', 'Location', 'Best');
grid on;

% Print the maximum torque demanded so you know what motor to buy!
max_torque = max(abs(u_history(:)));
fprintf('Maximum Torque Demanded = %.2f Nm n', max_torque);

disp('Simulation Complete! Watch the 3D Animation...');

% 5. FULL 3D PHYSICS ANIMATION
figure('Name', 'Ballbot Full 3D Simulation', 'Position', [150, 150, 900, 700]);
[Xg, Yg] = meshgrid(-5:0.5:5, -5:0.5:5);
Zg = zeros(size(Xg));
surf(Xg, Yg, Zg, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
hold on; grid on; axis equal; view(45, 30);
xlabel('World X (m)'); ylabel('World Y (m)'); zlabel('World Z (m)');

[sx, sy, sz] = sphere(30);
ball_radius = rk;
ball_surf = surf(sx*ball_radius, sy*ball_radius, sz*ball_radius + ball_radius, 'FaceColor', 'b', 'EdgeColor', 'none');
body_line = plot3([0 0], [0 0], [0 0], 'r', 'LineWidth', 6);
nose_line = plot3([0 0], [0 0], [0 0], 'g', 'LineWidth', 4);
camlight; lighting gouraud;
visual_body_length = 0.4; playback_speed = 10;      

for k = 1:playback_speed:length(t_sim)
    x_pos = history(k, 1); y_pos = history(k, 2);
    phi = history(k, 3); theta = history(k, 4); psi = history(k, 5);   

    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R_matrix = Rz * Ry * Rx; 

    ball_center = [x_pos; y_pos; rk];
    ball_surf.XData = sx * ball_radius + x_pos;
    ball_surf.YData = sy * ball_radius + y_pos;

    body_vec_world = R_matrix * [0; 0; visual_body_length];
    body_top = ball_center + body_vec_world;
    nose_vec_world = R_matrix * [0.15; 0; visual_body_length];
    nose_tip = ball_center + nose_vec_world;

    set(body_line, 'XData', [ball_center(1) body_top(1)], 'YData', [ball_center(2) body_top(2)], 'ZData', [ball_center(3) body_top(3)]);
    set(nose_line, 'XData', [body_top(1) nose_tip(1)], 'YData', [body_top(2) nose_tip(2)], 'ZData', [body_top(3) nose_tip(3)]);

    xlim([x_pos - 1.5, x_pos + 1.5]); ylim([y_pos - 1.5, y_pos + 1.5]); zlim([0, visual_body_length + 0.3]);
    title(sprintf('3D Sim | Time: %.1fs | Target Vx: %.1f | Actual Vx: %.1f', t_sim(k), cmd_history(k,1), history(k,6)));
    drawnow;
end

