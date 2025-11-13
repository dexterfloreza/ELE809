%% Part C/D – Extended Estimator and Servo Control Implementation
clc; clear; close all;

% === Matrices from Part B ===
Phi = [0      0       0;
       0      1.0000  0.006667;
       0.0012 0       0.9997];
Gamma = [0.00033;
         0.000022;
         0.0039];
C = [0 1 0];
D = 0;
K = [145 22 11.5];        % From Part B
Nbar = 1.06;              % From Part B

% === Build augmented system ===
A_e = [Phi Gamma;
       zeros(1,3) 1];
B_e = [Gamma; 0];
C_e = [C 0];

% === Observer gain (all poles at origin) ===
L_e = place(A_e', C_e', [0 0 0 0])';

% === Initialize ===
xhat = zeros(3,1);
what = 0;
x_e_hat = [xhat; what];
r = 1;      % step reference
N = 200;    % steps
y = 0;

% === Preallocate logs ===
u_log = zeros(1,N);
y_log = zeros(1,N);

for k = 1:N
    % control law
    u = Nbar*r - K*xhat - what;
    u_log(k) = u;

    % simulate plant output (assume perfect model)
    y = C*xhat + D*u;
    y_log(k) = y;

    % estimator update
    x_e_hat = A_e*x_e_hat + B_e*u + L_e*(y - C_e*x_e_hat);
    xhat = x_e_hat(1:3);
    what = x_e_hat(4);
end

% === Plot results ===
figure;
plot(y_log, 'LineWidth', 1.5);
xlabel('Sample (k)');
ylabel('Output Position \theta(k)');
title('Step Response with Extended Estimator (Zero Steady-State Error)');
grid on;
