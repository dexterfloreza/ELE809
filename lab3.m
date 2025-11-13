%% Demo 3.1 – Closed-loop Response with State Feedback + Extended Estimator
clc; clear; close all;

% === Simulation Parameters ===
Tf = 3.0;                 % total time [s]
Fs = 150;                 % sampling frequency [Hz]
Ts = 1/Fs;
t  = (0:Ts:Tf);
Ns = length(t);

% === Square-wave reference ±50° at 0.5 Hz ===
r = (50*pi/180) * square(2*pi*0.5*t);   % radians

% === System Matrices (from Part A) ===
Phi = [0      0       0;
       0      1.0000  0.006667;
       0.0012 0       0.9997];
Gamma = [0.00033;
         0.000022;
         0.0039];
C = [0 1 0];
D = 0;

% === Controller Gains (from Part B) ===
K = [145 22 11.5];
Nbar = 1.06;

% === Extended Estimator Design (from Part C) ===
A_e = [Phi Gamma;
       zeros(1,3) 1];
B_e = [Gamma; 0];
C_e = [C 0];
L_e = place(A_e', C_e', [0 0 0 0])';   % all poles at origin

% === Initialize variables ===
x = zeros(3,1);           % actual states (simulated)
x_hat = zeros(3,1);       % estimated states
w_hat = 0;                % estimated disturbance
x_e_hat = [x_hat; w_hat];

y = zeros(Ns,1);          % output
u = zeros(Ns,1);          % control signal
w_hat_log = zeros(Ns,1);  % disturbance estimate log
x_hat_log = zeros(Ns,3);  % state estimate log

% === Simulation Loop ===
for k = 1:Ns
    % Control law with disturbance compensation
    u(k) = Nbar*r(k) - K*x_hat - w_hat;

    % Update true system (no noise for simplicity)
    x = Phi*x + Gamma*u(k);
    y(k) = C*x;

    % Estimator update
    x_e_hat = A_e*x_e_hat + B_e*u(k) + L_e*(y(k) - C_e*x_e_hat);
    x_hat = x_e_hat(1:3);
    w_hat = x_e_hat(4);

    % Log data
    w_hat_log(k) = w_hat;
    x_hat_log(k,:) = x_hat.';
end

% === Convert to degrees for plotting ===
rad2deg = 180/pi;
r_deg = r * rad2deg;
y_deg = y * rad2deg;

% === Plot Results ===
figure;

% (a) yk and rk versus time
subplot(3,1,1);
plot(t, r_deg, 'r--', 'LineWidth', 1.2); hold on;
plot(t, y_deg, 'b', 'LineWidth', 1.3);
xlabel('Time (s)'); ylabel('Angle (°)');
legend('Reference (r_k)', 'Output (y_k)', 'Location','best');
title('(a) Output vs Reference');

% (b) w_hat versus time
subplot(3,1,2);
plot(t, w_hat_log, 'k', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\hat{w}_k');
title('(b) Estimated DC disturbance');

% (c) x_hat1, x_hat2, x_hat3 and u_k versus time
subplot(3,1,3);
plot(t, x_hat_log(:,1), 'LineWidth', 1.1); hold on;
plot(t, x_hat_log(:,2), 'LineWidth', 1.1);
plot(t, x_hat_log(:,3), 'LineWidth', 1.1);
plot(t, u, 'LineWidth', 1.1);
xlabel('Time (s)'); ylabel('States and Control');
legend('\hat{x}_1 (I_a)','\hat{x}_2 (\theta_m)','\hat{x}_3 (\dot{\theta}_m)','u_k','Location','best');
title('(c) State Estimates and Control Signal');
sgtitle('Demo 3.1: Closed-loop Response with Extended Estimator');

