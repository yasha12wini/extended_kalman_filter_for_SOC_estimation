SOC Estimation using EKF and 2RC Battery Model in MATLAB
This project implements a State of Charge (SOC) estimation algorithm using the **Extended Kalman Filter (EKF)** on a **2RC equivalent circuit model** of a lithium-ion battery. It also simulates the terminal voltage using a nonlinear **OCV-SOC relationship** and compares estimated values with true values.
- Simulates a battery under a variable current profile
- Models the battery as a 2RC circuit (Ohmic resistance + 2 RC branches)
- Uses a nonlinear Open Circuit Voltage (OCV) vs SOC curve
- Adds measurement noise to voltage readings
- Implements an Extended Kalman Filter (EKF) for SOC and terminal voltage estimation
- Plots SOC and voltage estimation performance

Model Details
Battery Parameters
- Battery Capacity: 1 Ah (3600 C)
- Equivalent Circuit: 2 RC branches + series resistance
- R₀: 0.015 Ω (Ohmic)
- R₁: 0.01 Ω, C₁: 2000 F (1st RC branch)
- R₂: 0.005 Ω, C₂: 5000 F (2nd RC branch)

