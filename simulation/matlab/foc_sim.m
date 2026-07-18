%% FOC Motor Control Simulation
% H747 Elite - Field Oriented Control Simulation
% MATLAB/Octave compatible script
% Simulates PMSM motor with FOC: current loop, speed loop, SVPWM
% Step response: 0 to 1000 RPM in 50ms

clear all; close all; clc;
fprintf('=== H747 Elite FOC Simulation ===\n');
fprintf('Simulating PMSM with Field-Oriented Control\n\n');

%% Motor Parameters (from motor_datasheet.md)
R = 0.45;               % Phase resistance (ohm, line-to-line/2 for per-phase)
L = 0.12e-3;            % Phase inductance (H, per-phase)
lambda = 0.052;         % Flux linkage (V*s/rad) = Kt
J = 1.2e-5;             % Rotor inertia (kg*m^2)
B = 5.0e-6;             % Viscous damping (Nm/(rad/s))
pole_pairs = 7;         % 14 poles -> 7 pole pairs
gear_ratio = 30;        % Gearbox ratio

%% Simulation Parameters
dt = 1e-6;              % Simulation time step (1 us)
ts = 50e-6;             % Control loop time step (50 us) = 20 kHz
T_end = 0.1;            % Simulation end time (100 ms)
t = 0:dt:T_end;         % Time vector
N = length(t);

%% Control Loop Rates (matched to motor_control_design.md)
% Current loop: 20 kHz (every 50 us)
% Speed loop:   1 kHz (every 1 ms)
% Position loop: 100 Hz (every 10 ms) - not used in this sim

ctrl_step = round(ts / dt);     % Control step in simulation steps
speed_step = round(1e-3 / dt);  % Speed loop step (1 ms)

%% Target Reference
omega_ref_rpm = 1000;           % Target speed: 1000 RPM
omega_ref = omega_ref_rpm * (2*pi/60) * pole_pairs;  % Convert to elec. rad/s

fprintf('Motor Parameters:\n');
fprintf('  R = %.3f ohm, L = %.3f mH\n', R, L*1e3);
fprintf('  Lambda = %.4f V*s/rad, J = %.2e kg*m^2\n', lambda, J);
fprintf('  Pole pairs: %d\n', pole_pairs);
fprintf('  Target: %d RPM (%.1f elec. rad/s)\n\n', omega_ref_rpm, omega_ref);

%% PI Controller Gains (from motor_control_design.md)
% Current loop PI (2 kHz BW)
Kp_i = 0.85;            % Proportional gain (V/A)
Ki_i = 320;             % Integral gain (V/(A*s))
i_max = 3.0;            % Current limit (A)

% Speed loop PI (200 Hz BW)
Kp_s = 0.12 / pole_pairs;    % Proportional gain (A/(rad/s)) - per pole pair
Ki_s = 4.0;                  % Integral gain (A/rad)

%% Initialize State Variables
% Electrical states
id = 0;                 % d-axis current (A)
iq = 0;                 % q-axis current (A)
vd = 0;                 % d-axis voltage (V)
vq = 0;                 % q-axis voltage (V)
theta_e = 0;            % Electrical angle (rad)
omega_e = 0;            % Electrical speed (rad/s)
omega_m = 0;            % Mechanical speed (rad/s)

% Mechanical state
theta_m = 0;            % Mechanical angle (rad)

% PI integrators
id_int = 0;
iq_int = 0;
speed_int = 0;

% Previous errors for anti-windup
prev_id_err = 0;
prev_iq_err = 0;
prev_speed_err = 0;

% Torque load profile
T_load = 0;             % Load torque (Nm)
load_step_time = 0.04;  % Apply load at t=40ms
load_torque = 0.02;     % Load torque magnitude (Nm)

%% Logging Arrays (downsampled for memory)
log_step = 10;          % Log every 10 simulation steps
log_idx = 1;
t_log = zeros(1, floor(N/log_step));
id_log = zeros(1, floor(N/log_step));
iq_log = zeros(1, floor(N/log_step));
vd_log = zeros(1, floor(N/log_step));
vq_log = zeros(1, floor(N/log_step));
omega_e_log = zeros(1, floor(N/log_step));
omega_m_log = zeros(1, floor(N/log_step));
theta_e_log = zeros(1, floor(N/log_step));
T_load_log = zeros(1, floor(N/log_step));
ia_log = zeros(1, floor(N/log_step));
ib_log = zeros(1, floor(N/log_step));
ic_log = zeros(1, floor(N/log_step));

%% Main Simulation Loop
fprintf('Running simulation...\n');
tic;

for n = 1:N
    current_time = t(n);

    % ---- Control Loop (20 kHz) ----
    if mod(n, ctrl_step) == 0
        % Park Transform (measurement -> dq)
        % Simulate perfect measurement for now
        % In reality: ia, ib, ic -> Clarke -> Park
        id_meas = id;
        iq_meas = iq;

        % Speed reference ramp (0 to target in 50ms)
        ramp_time = 0.05;  % 50ms ramp
        if current_time < ramp_time
            omega_ref_cur = omega_ref * (current_time / ramp_time);
        else
            omega_ref_cur = omega_ref;
        end

        % ---- Speed Loop (1 kHz) ----
        if mod(n, speed_step) == 0
            speed_err = omega_ref_cur - omega_e;

            % PI with anti-windup (clamping)
            speed_int_update = speed_int + Ki_s * speed_err * 1e-3;
            iq_ref = Kp_s * speed_err + speed_int_update;

            % Clamp iq_ref
            if iq_ref > i_max
                iq_ref = i_max;
                % Anti-windup: don't integrate
            elseif iq_ref < -i_max
                iq_ref = -i_max;
            else
                speed_int = speed_int_update;
            end

            % Id reference: 0 for non-field-weakening operation
            id_ref = 0;
        end

        % ---- Current Loop (20 kHz) ----
        % d-axis current error
        id_err = id_ref - id_meas;
        id_int_update = id_int + Ki_i * id_err * ts;

        % Anti-windup for d-axis
        vd_out = Kp_i * id_err + id_int_update;
        if vd_out > 12 || vd_out < -12
            % Clamp, don't integrate
        else
            id_int = id_int_update;
        end

        % q-axis current error
        iq_err = iq_ref - iq_meas;
        iq_int_update = iq_int + Ki_i * iq_err * ts;

        % Anti-windup for q-axis
        vq_out = Kp_i * iq_err + iq_int_update;
        if vq_out > 12 || vq_out < -12
            % Clamp, don't integrate
        else
            iq_int = iq_int_update;
        end

        % Decoupling terms (for higher speed accuracy)
        % vd_dec = -omega_e * L * iq;
        % vq_dec = omega_e * (L * id + lambda);
        % vd = vd_out + vd_dec;
        % vq = vq_out + vq_dec;
        vd = vd_out;
        vq = vq_out;

        % Voltage limit (SVPWM, max amplitude = Vdc/sqrt(3))
        Vdc = 12;
        Vmax = Vdc / sqrt(3);
        v_mag = sqrt(vd^2 + vq^2);
        if v_mag > Vmax
            vd = vd / v_mag * Vmax;
            vq = vq / v_mag * Vmax;
        end
    end

    % ---- Motor Model (PMSM electrical dynamics) ----
    % Electrical equations in dq frame:
    % vd = R*id + L*did/dt - omega_e*L*iq
    % vq = R*iq + L*diq/dt + omega_e*(L*id + lambda)

    did_dt = (vd - R*id + omega_e*L*iq) / L;
    diq_dt = (vq - R*iq - omega_e*(L*id + lambda)) / L;

    id = id + did_dt * dt;
    iq = iq + diq_dt * dt;

    % ---- Mechanical Dynamics ----
    % Torque equation: T = 1.5 * pole_pairs * lambda * iq
    T_e = 1.5 * pole_pairs * lambda * iq;

    % Applied load torque
    if current_time >= load_step_time
        T_load = load_torque;
    end

    % Mechanical equation: J*domega_m/dt = T_e - T_load - B*omega_m
    domega_m_dt = (T_e - T_load - B*omega_m) / J;
    omega_m = omega_m + domega_m_dt * dt;
    theta_m = theta_m + omega_m * dt;

    % Electrical speed and angle
    omega_e = omega_m * pole_pairs;
    theta_e = theta_e + omega_e * dt;
    theta_e = mod(theta_e, 2*pi);

    % ---- Phase Currents (inverse Park + Clarke for visualization) ----
    % Inverse Park: dq -> alpha-beta
    i_alpha = id * cos(theta_e) - iq * sin(theta_e);
    i_beta  = id * sin(theta_e) + iq * cos(theta_e);

    % Inverse Clarke: alpha-beta -> abc
    ia = i_alpha;
    ib = -0.5 * i_alpha + sqrt(3)/2 * i_beta;
    ic = -0.5 * i_alpha - sqrt(3)/2 * i_beta;

    % ---- Logging ----
    if mod(n, log_step) == 0
        t_log(log_idx) = current_time;
        id_log(log_idx) = id;
        iq_log(log_idx) = iq;
        vd_log(log_idx) = vd;
        vq_log(log_idx) = vq;
        omega_e_log(log_idx) = omega_e;
        omega_m_log(log_idx) = omega_m;
        theta_e_log(log_idx) = theta_e;
        T_load_log(log_idx) = T_load;
        ia_log(log_idx) = ia;
        ib_log(log_idx) = ib;
        ic_log(log_idx) = ic;
        log_idx = log_idx + 1;
    end
end

sim_time = toc;
fprintf('Simulation completed in %.2f seconds (%.0f steps)\n\n', sim_time, N);

%% Calculate Performance Metrics
% Find steady state (after ramp + settling)
ss_start = find(t_log >= 0.06, 1);
if isempty(ss_start)
    ss_start = round(length(t_log) * 0.75);
end

omega_e_ss = omega_e_log(ss_start:end);
omega_e_avg = mean(omega_e_ss);
omega_e_ripple = (max(omega_e_ss) - min(omega_e_ss)) / 2;
omega_e_rpm = omega_e_avg * 60 / (2*pi) / pole_pairs;

rise_time_idx = find(omega_e_log >= omega_ref * 0.9, 1);
if ~isempty(rise_time_idx)
    rise_time = t_log(rise_time_idx);
else
    rise_time = NaN;
end

settle_idx = find(abs(omega_e_log - omega_ref) <= omega_ref * 0.02, 1);
if ~isempty(settle_idx)
    settle_time = t_log(settle_idx);
else
    settle_time = NaN;
end

% Overshoot
omega_peak = max(omega_e_log);
overshoot_pct = (omega_peak - omega_ref) / omega_ref * 100;

fprintf('=== Performance Results ===\n');
fprintf('Steady-state speed:     %.1f RPM (%.1f elec. rad/s)\n', ...
    omega_e_rpm, omega_e_avg);
fprintf('Speed ripple (pk-pk):   %.2f elec. rad/s\n', omega_e_ripple);
fprintf('Rise time (10-90%%):     %.1f ms\n', rise_time * 1e3);
fprintf('Settling time (2%%):     %.1f ms\n', settle_time * 1e3);
fprintf('Overshoot:              %.1f %%\n', overshoot_pct);
fprintf('Target: 0 -> 1000 RPM in 50ms\n\n');

if rise_time <= 0.05
    fprintf('PASS: Rise time meets requirement (< 50 ms)\n');
else
    fprintf('WARNING: Rise time exceeds requirement\n');
end

%% Plot Results
figure('Position', [100, 100, 1200, 900]);

% Subplot 1: Speed Response
subplot(3, 2, 1);
plot(t_log * 1e3, omega_e_log, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_log * 1e3, omega_ref * ones(size(t_log)), 'r--', 'LineWidth', 1);
plot(t_log * 1e3, omega_m_log * pole_pairs, 'g:', 'LineWidth', 1);
xlabel('Time (ms)');
ylabel('Speed (elec. rad/s)');
title('Speed Response');
legend('Electrical speed', 'Reference', 'Mechanical*pp', 'Location', 'southeast');
grid on;
xlim([0 T_end*1e3]);

% Subplot 2: Currents dq
subplot(3, 2, 2);
plot(t_log * 1e3, id_log, 'r-', 'LineWidth', 1.5);
hold on;
plot(t_log * 1e3, iq_log, 'b-', 'LineWidth', 1.5);
xlabel('Time (ms)');
ylabel('Current (A)');
title('DQ-Axis Currents');
legend('i_d', 'i_q', 'Location', 'northeast');
grid on;
xlim([0 T_end*1e3]);

% Subplot 3: Phase Voltages (dq)
subplot(3, 2, 3);
plot(t_log * 1e3, vd_log, 'r-', 'LineWidth', 1.5);
hold on;
plot(t_log * 1e3, vq_log, 'b-', 'LineWidth', 1.5);
xlabel('Time (ms)');
ylabel('Voltage (V)');
title('DQ-Axis Voltages');
legend('v_d', 'v_q', 'Location', 'northeast');
grid on;
xlim([0 T_end*1e3]);

% Subplot 4: Phase Currents (ABC)
subplot(3, 2, 4);
% Plot only a window for visibility
window_start = round(length(t_log) * 0.8);
window_end = min(window_start + 500, length(t_log));
plot(t_log(window_start:window_end) * 1e3, ...
     ia_log(window_start:window_end), 'r-', 'LineWidth', 1);
hold on;
plot(t_log(window_start:window_end) * 1e3, ...
     ib_log(window_start:window_end), 'g-', 'LineWidth', 1);
plot(t_log(window_start:window_end) * 1e3, ...
     ic_log(window_start:window_end), 'b-', 'LineWidth', 1);
xlabel('Time (ms)');
ylabel('Current (A)');
title('Phase Currents (steady state)');
legend('i_a', 'i_b', 'i_c', 'Location', 'northeast');
grid on;

% Subplot 5: Torque
subplot(3, 2, 5);
T_e_log = 1.5 * pole_pairs * lambda * iq_log;
plot(t_log * 1e3, T_e_log, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_log * 1e3, T_load_log, 'r--', 'LineWidth', 1.5);
xlabel('Time (ms)');
ylabel('Torque (Nm)');
title('Motor Torque');
legend('Electromagnetic', 'Load', 'Location', 'northeast');
grid on;
xlim([0 T_end*1e3]);

% Subplot 6: Speed in RPM
subplot(3, 2, 6);
omega_rpm_log = omega_e_log * 60 / (2*pi) / pole_pairs;
plot(t_log * 1e3, omega_rpm_log, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_log * 1e3, omega_ref_rpm * ones(size(t_log)), 'r--', 'LineWidth', 1);
xlabel('Time (ms)');
ylabel('Speed (RPM)');
title('Mechanical Speed');
legend('Actual', 'Target', 'Location', 'southeast');
grid on;
xlim([0 T_end*1e3]);

sgtitle('H747 Elite FOC Simulation - Step Response 0 to 1000 RPM');

%% Additional Analysis: FOC Vector Diagram (steady state)
figure('Position', [100, 100, 600, 500]);
quiver(0, 0, id_log(end), iq_log(end), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);
hold on;
quiver(0, 0, vd_log(end), vq_log(end), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
xlabel('d-axis');
ylabel('q-axis');
title('FOC Vector Diagram (Steady State)');
legend('Current vector', 'Voltage vector', 'Location', 'northwest');
axis equal;
grid on;
xlim([-2 2]);
ylim([-2 2]);

fprintf('\nSimulation complete. %d plots generated.\n', 2);
