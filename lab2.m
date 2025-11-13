Tf   = 3.0;                 % total time [s]
Fs   = 0.5;                % sampling frequency [Hz]
Ts   = 1/Fs;
Kp   = 9.85;                 % proportional gain
Ki   = 213.06;               % integral gain
Kd   = 0.137;                % derivative gain
Beta = Kd/Kp/20;              % filter time constant
wp   = 0;                    % proportional frequency weighting factor
wd   = 0;                    % derivative frequency weighting factor
Kt   = 100;                  % gain Kt
t    = (0:Ts:Tf);            % time vector
Ns   = length(t);            % number of samples

% 50 deg square wave @ 0.25 Hz
r = (50*pi/180) * square(2*pi*0.25*t);  

% Initialize system
servo = Initialize(Fs, 1, Tf);
theta = zeros(Ns,1);         
u     = zeros(Ns,1);         
Va    = zeros(Ns,1);         
Ik    = 0;                    
Dk    = 0;                     
Ik_prev = 0;
Dk_prev = 0;
ed_prev = 0;                
alpha_prev = 0;             
ek_prev = 0;

% Run the simulation loop
for k = 1:Ns
    enc        = ReadEncoder(servo);
    theta(k)   = EtoR(enc);
    ek   = r(k) - theta(k);      % integral error
    epk  = wp*r(k) - theta(k);   % proportional error
    edk  = wd*r(k) - theta(k);   % derivative error
    Pk = Kp * epk;
    
    % Integral calculation
    Ik = Ik_prev + Ts*( Ki*ek_prev + Kt*alpha_prev );
    ek_prev = ek;
    
    % Derivative calculation
    Dk = (Beta/(Beta + Ts))*Dk_prev + (Kd/(Beta + Ts))*(edk - ed_prev);
    ed_prev = edk;
    
    % Control output
    uk = Pk + Ik + Dk;
    
    % Actuator saturation
    Va(k) = act_sat(uk);
    alpha = Va(k) - uk;          % anti-windup back-calculation signal
    
    % Apply to plant
    DtoA(servo, VtoD(Va(k)));
    
    % Log & advance states
    Dk_prev = Dk;
    Ik_prev = Ik;
    u(k) = uk;
    alpha_prev = alpha;
end

% Terminate the system
Terminate(servo);

% Convert radians to degrees for plotting
radfac = 180/pi;
ref_deg   = radfac * r;
theta_deg = radfac * theta;

% Plot the results
figure;
plot(t, ref_deg, t, theta_deg);
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Reference', 'Measured', 'Location', 'best');
title('Reference vs Measured Response');

% Compute and display performance metrics
% Final (steady-state) value = average of last 5% of samples
ss_value = mean(theta_deg(round(0.95*Ns):end));

% Maximum peak
peak_value = max(theta_deg);

% Percent overshoot
PO = (peak_value - ss_value) / ss_value * 100;
fprintf('Percent Overshoot = %.2f %%\n', PO);

% Settling time (2% criterion)
tol = 0.02 * abs(ss_value);
idx = find(abs(theta_deg - ss_value) > tol, 1, 'last');
Ts_measured = t(idx);

% Compute Ts(1%)
tol_1 = 0.01 * abs(ss_value);
idx_ts = find(abs(theta_deg - ss_value) > tol_1, 1, 'last');
Ts_1 = t(idx_ts);

fprintf('Settling Time (2%%) = %.4f s\n', Ts_measured);
fprintf('Settling Time (1%%) = %.4f s\n', Ts_1);
