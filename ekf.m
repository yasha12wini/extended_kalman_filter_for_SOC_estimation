clear; clc;

% Simulation Parameters
dt = 1;                  % Time step in seconds
T = 3600;                % Total simulation time in seconds (1 hour)
N = T/dt;                % Number of time steps

% Battery Parameters
Q_batt = 3600;           % Battery capacity in Coulombs (1 Ah)
R0 = 0.015;              % Ohmic resistance in Ohms
R1 = 0.01; C1 = 2000;    % First RC branch
R2 = 0.005; C2 = 5000;   % Second RC branch

% Initial Conditions
true_SOC = zeros(1, N);
true_SOC(1) = 0.9;       % True initial SOC

V_RC1 = zeros(1, N);     % RC1 voltage memory
V_RC2 = zeros(1, N);     % RC2 voltage memory

% Current Profile (sinusoidal variation)
I = 1 + 0.5 * (sin((1:N) * 0.005));   % Amps

% Nonlinear OCV-SOC Curve
OCV_func = @(soc) 3.6 + 0.4*soc - 0.2*soc.^2 + 0.1*soc.^3;

% Voltage arrays
V_true = zeros(1, N);
V_meas = zeros(1, N);

% True SOC and terminal voltage simulation
for i = 2:N
    true_SOC(i) = true_SOC(i-1) - (I(i-1)*dt / Q_batt);
    true_SOC(i) = max(0, min(1, true_SOC(i)));  % Clamp SOC to [0,1]

    V_RC1(i) = V_RC1(i-1) + dt * (-V_RC1(i-1)/(R1*C1) + I(i-1)/C1);
    V_RC2(i) = V_RC2(i-1) + dt * (-V_RC2(i-1)/(R2*C2) + I(i-1)/C2);

    V_ocv = OCV_func(true_SOC(i));
    V_true(i) = V_ocv - V_RC1(i) - V_RC2(i) - I(i)*R0;
    V_meas(i) = V_true(i) + normrnd(0, 0.005);  % Gaussian noise
end

% Kalman Filter Initialization
x_hat = zeros(3, N);           % States: [SOC; V_RC1; V_RC2]
x_hat(:,1) = [0.8; 0; 0];      % Initial state estimate

P = 0.01 * eye(3);             % Initial error covariance matrix
Q_k = diag([1e-5, 1e-6, 1e-6]);% Process noise covariance
R_k = 1e-4;                    % Measurement noise variance

% EKF Loop
for k = 2:N
    Ik = I(k-1);

    % Prediction step
    SOC_pred  = x_hat(1,k-1) - Ik*dt/Q_batt;
    VRC1_pred = x_hat(2,k-1) + dt*(-x_hat(2,k-1)/(R1*C1) + Ik/C1);
    VRC2_pred = x_hat(3,k-1) + dt*(-x_hat(3,k-1)/(R2*C2) + Ik/C2);
    x_pred = [SOC_pred; VRC1_pred; VRC2_pred];

    % State transition matrix
    F = [1, 0, 0;
         0, 1 - dt/(R1*C1), 0;
         0, 0, 1 - dt/(R2*C2)];

    P_pred = F * P * F' + Q_k;

    % Measurement prediction
    V_ocv_pred = OCV_func(x_pred(1));
    V_pred = V_ocv_pred - x_pred(2) - x_pred(3) - Ik*R0;

    % Jacobian of OCV w.r.t SOC
    dOCV_dSOC = 0.4 - 0.4*x_pred(1) + 0.3*x_pred(1)^2;
    H = [dOCV_dSOC, -1, -1];

    % Kalman gain
    K = P_pred * H' / (H * P_pred * H' + R_k);

    % Correction step
    x_hat(:,k) = x_pred + K * (V_meas(k) - V_pred);

    % Clamp estimated SOC
    x_hat(1,k) = max(0, min(1, x_hat(1,k)));

    % Update error covariance
    P = (eye(3) - K * H) * P_pred;
end

% Plot SOC
figure;
plot(1:N, true_SOC, 'g', 'LineWidth', 2); hold on;
plot(1:N, x_hat(1,:), 'b--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('SOC');
legend('True SOC', 'Estimated SOC (EKF)');
title('SOC Estimation using EKF and 2RC Model');
grid on;

% Plot terminal voltage
V_ocv_pred = OCV_func(x_hat(1,:));
V_est = V_ocv_pred - x_hat(2,:) - x_hat(3,:) - I*R0;

figure;
plot(1:N, V_meas, 'r'); hold on;
plot(1:N, V_est, 'k--');
xlabel('Time (s)');
ylabel('Terminal Voltage (V)');
legend('Measured Voltage', 'Estimated Voltage');
title('Voltage Tracking using EKF');
grid on;
