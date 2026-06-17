# MATLAB Design Files

All system modeling, controller design, and EKF tuning was done in MATLAB R2023b.

---

## Folder Structure

```
matlab/
├── system_model/         Nonlinear ball-bot equations of motion
│   ├── ballbot_eom.m     Derives the Lagrangian EOM symbolically
│   └── linearize.m       Linearizes about upright equilibrium
│
├── lqi_design/           LQI gain matrix K[3×10]
│   ├── lqi_design.m      Main script — sets Q, R, solves DARE
│   ├── verify_poles.m    Checks closed-loop poles
│   └── K_LQI.mat         Saved gain matrix used in firmware
│
├── ekf_design/           EKF noise matrix tuning
│   ├── ekf_tuning.m      Sweeps Q, R parameters
│   └── verify_ekf.m      Monte Carlo verification
│
└── simulation/           Full closed-loop simulation
    ├── ballbot_sim.slx   Simulink model (nonlinear plant + LQI)
    └── plot_results.m    Generates response plots
```

---

## LQI State Vector

```
x = [φ, θ, ψ, Vx, Vy, dφ, dθ, ω_z, ∫Vx, ∫Vy]ᵀ
```

| Index | State | Unit |
|-------|-------|------|
| 0 | φ (roll tilt) | rad |
| 1 | θ (pitch tilt) | rad |
| 2 | ψ (yaw) | rad |
| 3 | Vx (ball velocity) | m/s |
| 4 | Vy (ball velocity) | m/s |
| 5 | dφ (roll rate) | rad/s |
| 6 | dθ (pitch rate) | rad/s |
| 7 | ω_z (yaw rate) | rad/s |
| 8 | ∫Vx | m |
| 9 | ∫Vy | m |

---

## Physical Parameters Used

```matlab
m_body = 15.0;   % robot body mass [kg]
m_ball = 2.0;    % ball mass [kg]
Rk     = 0.175;  % ball radius [m]
Rw     = 0.0625; % wheel radius [m]
alpha  = pi/4;   % zenith angle [rad]
J_body = 1.2;    % body inertia [kg·m²]
J_wheel = 0.0032;% wheel inertia [kg·m²]
g      = 9.81;   % gravity [m/s²]
```

---

