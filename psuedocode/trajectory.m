
% Trajectory Ground Truth Generator
% Run this before Simulink simulation
% Generates: pos, vel, accel, gyro, mag, orientation ground truths
% Then adds realistic sensor noise to each signal

%% Configuration
dt = 0.002;              % Sample time (s) — must match Simulink solver step
t_end = 10;              % Simulation duration (s) — match Simulink stop time
t = (0:dt:t_end)';
N = length(t);

%% Ground Truth Trajectory
% Vertical: 5 m/s^2 downward acceleration (max alt ~250m at 10s, within baro range)
% Lateral: mild sinusoidal drift to exercise GPS and magnetometer paths
az_true = 5;             % Vertical acceleration (m/s^2, NED Down)
ax_drift = 2;            % Lateral drift amplitude (m/s^2)
drift_freq = 0.5;        % Lateral drift frequency (Hz)

% Acceleration ground truth [N E D] (m/s^2)
accel_true = zeros(N, 3);
accel_true(:, 1) = ax_drift * sin(2*pi*drift_freq*t);  % North oscillation
accel_true(:, 3) = -az_true;                            % Down

% Gyro ground truth [roll rate, pitch rate, yaw rate] (rad/s)
% Small yaw rotation from lateral drift
gyro_true = zeros(N, 3);
gyro_true(:, 3) = 0.1 * sin(2*pi*drift_freq*t);  % Yaw rate

% Orientation ground truth [w x y z] quaternion
orient_true = repmat([1 0 0 0], N, 1);

% Position ground truth [N E D] (m)
pos_true = zeros(N, 3);
pos_true(:, 1) = -ax_drift/(2*pi*drift_freq)^2 * sin(2*pi*drift_freq*t);  % North
pos_true(:, 3) = -0.5 * az_true * t.^2;                                    % Down

% Velocity ground truth [N E D] (m/s)
vel_true = zeros(N, 3);
vel_true(:, 1) = ax_drift/(2*pi*drift_freq) * cos(2*pi*drift_freq*t);  % North
vel_true(:, 3) = -az_true * t;                                          % Down

% Magnetometer ground truth [x y z] (uT)
mag_true = repmat([22 0.5 43], N, 1);

%% Sensor Noise Injection
% Realistic noise levels — replace with datasheet values for your hardware
accel_noise_std = 0.5;    % m/s^2 (typical MEMS accelerometer)
gyro_noise_std  = 0.01;   % rad/s (typical MEMS gyroscope)
mag_noise_std   = 1.0;    % uT (typical magnetometer)
gps_pos_noise_std = 2.0;  % m (typical GPS CEP)
gps_vel_noise_std = 0.1;  % m/s (typical GPS velocity)

accel_noisy = accel_true + accel_noise_std * randn(N, 3);
gyro_noisy  = gyro_true  + gyro_noise_std  * randn(N, 3);
mag_noisy   = mag_true   + mag_noise_std   * randn(N, 3);
pos_noisy   = pos_true   + gps_pos_noise_std * randn(N, 3);
vel_noisy   = vel_true   + gps_vel_noise_std * randn(N, 3);

%% Package as timed signals for From Workspace blocks
% Noisy signals go to Simulink (simulates real sensor input)
accel_signal  = [t, accel_noisy];
gyro_signal   = [t, gyro_noisy];
orient_signal = [t, orient_true];   % Orientation has no noise (reference)
pos_signal    = [t, pos_noisy];
vel_signal    = [t, vel_noisy];
mag_signal    = [t, mag_noisy];

% Ground truths kept in workspace for error analysis in visualize.m
% Variables: accel_true, gyro_true, pos_true, vel_true, mag_true

fprintf("Trajectory ready | dt=%.3fs | N=%d samples | duration=%.1fs\n", dt, N, t(end));
