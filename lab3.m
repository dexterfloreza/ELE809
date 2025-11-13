%% Demo 3.1 – Closed-loop Response with State Feedback + Extended Estimator
clc; clear; close all;

%% === Simulation Parameters ===
Tf = 3.0;                % total run time [s]
Fs = 150;                % sampling frequency [Hz]
Ts = 1/Fs;
t  = (0:Ts:Tf);          % time vector
Ns = length(t);

%% === Square-wave Reference: ±50° @ 0.5 Hz ===
r = (50*pi/180) * square(2*pi*0.5*t);   % radians

%% === Discrete-Time Model (from Part A) ===
Phi = [0      0       0;
       0      1.0000  0.006667;
       0.0012 0       0.9997];
Gamma = [0.00033;
         0.000022;
         0.0039];
C = [0 1 0];
D = 0;

%% === Controller Gains (from Part B) ===
K = [145 22 11.5];
Nbar = 1.06;

%% === Extended Estimator Design (from Part C) ===
A_e = [Phi Gamma;
       zeros(1,3) 1];
B_e = [Gamma; 0];
C_e = [C 0];

% Fast near-zero estimator poles (avoid multiplicity/rank issue)
L_e = place(A_e', C_e', [0.01 0.02 0.03 0.04])';

%% === Initialize States ===
x = zeros(3,1);           % actual plant states
x_hat = zeros(3,1);       % estimated states
w_hat = 0;                % estimated disturbance
x_e_hat = [x_hat; w_hat];

% Constant DC disturbance (optional – helps verify rejection)
w = 0.15;                 % add/subtract this from u(k) in plant update

% Logs
y = zeros(Ns,1);
u = zeros(Ns,1);
w_hat_log = zeros(Ns,1);
x_hat_log = zeros(Ns,3);

%% === Simulation Loop ===
for k = 1:Ns
    % Control law with disturbance compensation
    u(k) = Nbar*r(k) - K*x_hat - w_hat;

    % Plant update (simulate disturbance at actuator input)
    x = Phi*x + Gamma*(u(k) + w);
    y(k) = C*x;

    % Extended estimator update
    x_e_hat = A_e*x_e_hat + B_e*u(k) + L_e*(y(k) - C_e*x_e_hat);
    x_hat = x_e_hat(1:3);
    w_hat = x_e_hat(4);

    % Log data
    w_hat_log(k) = w_hat;
    x_hat_log(k,:) = x_hat.';
end

%% === Convert to Degrees for Plotting ===
rad2deg = 180/pi;
r_deg = r * rad2deg;
y_deg = y * rad2deg;

%% === Plot Results ===
figure;

% (a) Output vs Reference
subplot(3,1,1);
plot(t, r_deg, 'r--', 'LineWidth', 1.2); hold on;
plot(t, y_deg, 'b', 'LineWidth', 1.3);
xlabel('Time (s)'); ylabel('Angle (°)');
legend('Reference (r_k)', 'Output (y_k)', 'Location','best');
title('(a) Output vs Reference');

% (b) Estimated DC Disturbance
subplot(3,1,2);
plot(t, w_hat_log, 'k', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\hat{w}_k');
title('(b) Estimated DC Disturbance');

% (c) Estimated States and Control Signal
subplot(3,1,3);
plot(t, x_hat_log(:,1), 'LineWidth', 1.1); hold on;
plot(t, x_hat_log(:,2), 'LineWidth', 1.1);
plot(t, x_hat_log(:,3), 'LineWidth', 1.1);
plot(t, u, 'LineWidth', 1.1);
xlabel('Time (s)'); ylabel('States and Control');
legend('\hat{x}_1 (I_a)','\hat{x}_2 (\theta_m)','\hat{x}_3 (\dot{\theta}_m)','u_k','Location','best');
title('(c) State Estimates and Control Signal');
sgtitle('Demo 3.1: Closed-loop Response with Extended Estimator');

%% === Optional: Performance Metrics ===
ss_value = mean(y_deg(round(0.9*Ns):end));
peak_value = max(y_deg);
PO = (peak_value - ss_value)/abs(ss_value)*100;
fprintf('Percent Overshoot ≈ %.2f %%\n', PO);

tol = 0.01*abs(ss_value);
idx = find(abs(y_deg - ss_value) > tol, 1, 'last');
Ts_1 = t(idx);
fprintf('Settling Time (1%%) ≈ %.4f s\n', Ts_1);
