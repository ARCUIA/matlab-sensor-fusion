%% HUD Flight Data Visualization
% Post-processes Simulink output into a single-screen mission HUD
% Supports replay mode: set REPLAY = true before running
% Requires: 'out' from Simulink + ground truths from trajectory.m

%% 0. Replay Config
if ~exist('REPLAY', 'var')
    REPLAY = false;
end
replay_speed = 1;        % 1 = real-time, 2 = 2x speed, 0.5 = half speed
replay_fps = 30;         % Target frames per second during replay

%% 1. Extract and Format Simulink Data
t          = out.pos_out.Time;
pos_data   = squeeze(out.pos_out.Data);
vel_data   = squeeze(out.vel_out.Data);
alt_data   = squeeze(out.altitude_out.Data);
aspd_data  = squeeze(out.airspeed_out.Data);
hdg_data   = squeeze(out.heading_out.Data);
euler_data = squeeze(out.euler_out.Data);
baro_data  = squeeze(out.baro_out.Data);
climb_data = squeeze(out.climbrate_out.Data);

% Ensure [N x 3] for multi-column data
if size(pos_data, 1) == 3 && size(pos_data, 2) ~= 3
    pos_data = pos_data';
end
if size(vel_data, 1) == 3 && size(vel_data, 2) ~= 3
    vel_data = vel_data';
end
if size(euler_data, 1) == 3 && size(euler_data, 2) ~= 3
    euler_data = euler_data';
end

N_out = length(t);

%% 2. Coordinate Transformation (NED to Plot Frame)
North  = pos_data(:, 1);
East   = pos_data(:, 2);
Down   = pos_data(:, 3);
X_plot = East;
Y_plot = North;
Z_plot = -Down;

%% 3. Euler Angles
roll_deg  = euler_data(:, 1);
pitch_deg = euler_data(:, 2);
yaw_deg   = euler_data(:, 3);
% Uncomment if your block outputs radians:
% roll_deg = rad2deg(roll_deg); pitch_deg = rad2deg(pitch_deg); yaw_deg = rad2deg(yaw_deg);

%% 4. Heading, Altitude, Airspeed, Climbrate
heading_deg = mod(hdg_data, 360);
altitude    = alt_data;
airspeed    = aspd_data;
climbrate   = climb_data;

%% 5. Standard Error vs Ground Truth
N_cmp   = min(N_out, size(pos_true, 1));
pos_err = pos_data(1:N_cmp, :) - pos_true(1:N_cmp, :);
vel_err = vel_data(1:N_cmp, :) - vel_true(1:N_cmp, :);

std_err_pos = std(sqrt(sum(pos_err.^2, 2)));
std_err_vel = std(sqrt(sum(vel_err.^2, 2)));

qw = orient_true(1:N_cmp,1); qx = orient_true(1:N_cmp,2);
qy = orient_true(1:N_cmp,3); qz = orient_true(1:N_cmp,4);
roll_true  = atan2d(2*(qw.*qx + qy.*qz), 1 - 2*(qx.^2 + qy.^2));
pitch_true = asind(max(-1, min(1, 2*(qw.*qy - qz.*qx))));
yaw_true   = atan2d(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));
att_err    = sqrt((roll_deg(1:N_cmp) - roll_true).^2 + ...
             (pitch_deg(1:N_cmp) - pitch_true).^2 + ...
             (yaw_deg(1:N_cmp) - yaw_true).^2);
std_err_att = std(att_err);

%% 6. Interpolate accel ground truth to Simulink time
accel_interp = interp1(linspace(0, t(end), size(accel_true,1)), accel_true, t, 'linear', 'extrap');

%% 7. Colors
c_white  = [1 1 1];
c_cyan   = [0 1 1];
c_green  = [0 1 0];
c_red    = [1 0.3 0.3];
c_yellow = [1 1 0];
c_orange = [1 0.6 0];
c_grid   = [0.3 0.3 0.3];
c_bg     = [0.05 0.05 0.05];

%% 8. Build HUD Figure
close all;
fig = figure('Name', 'Flight HUD', 'Color', 'k', ...
    'Units', 'normalized', 'Position', [0.02 0.02 0.96 0.92], ...
    'MenuBar', 'none', 'ToolBar', 'figure');

% =====================================================================
%  LEFT SIDE: 3D trajectory (top) + 3 stacked plots (bottom)
% =====================================================================

% --- 3D Trajectory (top 38%) ---
ax3d = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.03 0.56 0.60 0.40]);
set(ax3d, 'Color', c_bg, 'XColor', c_white, 'YColor', c_white, 'ZColor', c_white);
grid(ax3d, 'on'); hold(ax3d, 'on');
set(ax3d, 'GridColor', c_grid, 'GridAlpha', 0.6, 'FontSize', 7);

pad = max(20, max(max(X_plot)-min(X_plot), max(Y_plot)-min(Y_plot)) * 0.3);
[gX, gY] = meshgrid(linspace(min(X_plot)-pad, max(X_plot)+pad, 15));
gZ = zeros(size(gX));
mesh(ax3d, gX, gY, gZ, 'FaceAlpha', 0.1, 'EdgeColor', c_green, 'EdgeAlpha', 0.2);

h_traj3d = plot3(ax3d, X_plot, Y_plot, Z_plot, 'Color', c_cyan, 'LineWidth', 1.5);
plot3(ax3d, X_plot(1), Y_plot(1), Z_plot(1), 'o', ...
    'MarkerFaceColor', c_green, 'MarkerEdgeColor', c_green, 'MarkerSize', 6);
h_end3d = plot3(ax3d, X_plot(end), Y_plot(end), Z_plot(end), 'o', ...
    'MarkerFaceColor', c_red, 'MarkerEdgeColor', c_red, 'MarkerSize', 6);

xlabel(ax3d, 'East (m)', 'Color', c_white, 'FontSize', 8);
ylabel(ax3d, 'North (m)', 'Color', c_white, 'FontSize', 8);
zlabel(ax3d, 'Altitude (m)', 'Color', c_white, 'FontSize', 8);
title(ax3d, '3D TRAJECTORY', 'Color', c_white, 'FontSize', 9, 'FontWeight', 'bold');
view(ax3d, 45, 20);
xlim(ax3d, [min(X_plot)-pad max(X_plot)+pad]);
ylim(ax3d, [min(Y_plot)-pad max(Y_plot)+pad]);
zlim(ax3d, [min(Z_plot)-10 max(Z_plot)+10]);

% --- Stacked plots: evenly spaced in bottom half ---
plot_h    = 0.12;
plot_gap  = 0.04;
plot_left = 0.06;
plot_w    = 0.54;
y3 = 0.06;                          % Orientation (bottom)
y2 = y3 + plot_h + plot_gap;        % Velocity (middle)
y1 = y2 + plot_h + plot_gap;        % Acceleration (top)

% --- Acceleration ---
ax_acc = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [plot_left y1 plot_w plot_h]);
h_acc = gobjects(1,3);
h_acc(1) = plot(ax_acc, t, accel_interp(:,1), 'Color', c_red, 'LineWidth', 0.8); hold on;
h_acc(2) = plot(ax_acc, t, accel_interp(:,2), 'Color', c_green, 'LineWidth', 0.8);
h_acc(3) = plot(ax_acc, t, accel_interp(:,3), 'Color', c_cyan, 'LineWidth', 0.8);
set(ax_acc, 'Color', c_bg, 'XColor', c_white, 'YColor', c_white, 'FontSize', 7);
set(ax_acc, 'GridColor', c_grid, 'GridAlpha', 0.5, 'XTickLabel', []);
grid(ax_acc, 'on');
ylabel(ax_acc, 'm/s^2', 'Color', c_white, 'FontSize', 7);
title(ax_acc, 'ACCELERATION [N E D]', 'Color', c_white, 'FontSize', 8, 'FontWeight', 'bold');
legend(ax_acc, 'N', 'E', 'D', 'TextColor', c_white, 'FontSize', 6, ...
    'Location', 'eastoutside', 'Color', c_bg, 'EdgeColor', c_grid);

% --- Velocity ---
ax_vel = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [plot_left y2 plot_w plot_h]);
h_vel = gobjects(1,3);
h_vel(1) = plot(ax_vel, t, vel_data(:,1), 'Color', c_red, 'LineWidth', 0.8); hold on;
h_vel(2) = plot(ax_vel, t, vel_data(:,2), 'Color', c_green, 'LineWidth', 0.8);
h_vel(3) = plot(ax_vel, t, vel_data(:,3), 'Color', c_cyan, 'LineWidth', 0.8);
set(ax_vel, 'Color', c_bg, 'XColor', c_white, 'YColor', c_white, 'FontSize', 7);
set(ax_vel, 'GridColor', c_grid, 'GridAlpha', 0.5, 'XTickLabel', []);
grid(ax_vel, 'on');
ylabel(ax_vel, 'm/s', 'Color', c_white, 'FontSize', 7);
title(ax_vel, 'VELOCITY [N E D]', 'Color', c_white, 'FontSize', 8, 'FontWeight', 'bold');
legend(ax_vel, 'N', 'E', 'D', 'TextColor', c_white, 'FontSize', 6, ...
    'Location', 'eastoutside', 'Color', c_bg, 'EdgeColor', c_grid);

% --- Orientation ---
ax_ori = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [plot_left y3 plot_w plot_h]);
h_ori = gobjects(1,3);
h_ori(1) = plot(ax_ori, t, roll_deg, 'Color', c_red, 'LineWidth', 0.8); hold on;
h_ori(2) = plot(ax_ori, t, pitch_deg, 'Color', c_green, 'LineWidth', 0.8);
h_ori(3) = plot(ax_ori, t, yaw_deg, 'Color', c_cyan, 'LineWidth', 0.8);
set(ax_ori, 'Color', c_bg, 'XColor', c_white, 'YColor', c_white, 'FontSize', 7);
set(ax_ori, 'GridColor', c_grid, 'GridAlpha', 0.5);
grid(ax_ori, 'on');
xlabel(ax_ori, 'Time (s)', 'Color', c_white, 'FontSize', 7);
ylabel(ax_ori, 'deg', 'Color', c_white, 'FontSize', 7);
title(ax_ori, 'ORIENTATION [Roll Pitch Yaw]', 'Color', c_white, 'FontSize', 8, 'FontWeight', 'bold');
legend(ax_ori, 'Roll', 'Pitch', 'Yaw', 'TextColor', c_white, 'FontSize', 6, ...
    'Location', 'eastoutside', 'Color', c_bg, 'EdgeColor', c_grid);

linkaxes([ax_acc, ax_vel, ax_ori], 'x');

% Lock all axes
disableDefaultInteractivity(ax3d);
disableDefaultInteractivity(ax_acc);
disableDefaultInteractivity(ax_vel);
disableDefaultInteractivity(ax_ori);
ax3d.Toolbar.Visible = 'off';
ax_acc.Toolbar.Visible = 'off';
ax_vel.Toolbar.Visible = 'off';
ax_ori.Toolbar.Visible = 'off';

% =====================================================================
%  RIGHT SIDE: Readouts (top) + Ground Track (bottom)
% =====================================================================

readout_x = 0.66;
readout_w = 0.32;

% --- Readout Panel ---
readout_str = build_readout(altitude, airspeed, heading_deg, climbrate, ...
    baro_data, t, std_err_pos, std_err_vel, std_err_att, ...
    c_cyan, c_yellow, c_orange, N_out);

h_readout = annotation(fig, 'textbox', [readout_x 0.58 readout_w 0.39], ...
    'String', readout_str, 'Interpreter', 'tex', ...
    'FontSize', 11, 'FontName', 'Consolas', 'FontWeight', 'bold', ...
    'Color', c_white, 'BackgroundColor', c_bg, ...
    'EdgeColor', c_white, 'LineWidth', 0.5, ...
    'FitBoxToText', 'off', 'VerticalAlignment', 'top', ...
    'Margin', 10);

% --- Ground Track ---
ax_gt = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.69 0.05 0.27 0.48]);
h_gt = plot(ax_gt, East, North, 'Color', c_cyan, 'LineWidth', 1.2); hold on;
plot(ax_gt, East(1), North(1), 'o', ...
    'MarkerFaceColor', c_green, 'MarkerEdgeColor', c_green, 'MarkerSize', 5);
h_gt_end = plot(ax_gt, East(end), North(end), 'o', ...
    'MarkerFaceColor', c_red, 'MarkerEdgeColor', c_red, 'MarkerSize', 5);
set(ax_gt, 'Color', c_bg, 'XColor', c_white, 'YColor', c_white, 'FontSize', 7);
set(ax_gt, 'GridColor', c_grid, 'GridAlpha', 0.5);
grid(ax_gt, 'on');
axis(ax_gt, 'equal');
% Enforce minimum 10m axis range so small movements don't look wonky
gt_min_range = 10;
e_mid = (max(East) + min(East)) / 2;
n_mid = (max(North) + min(North)) / 2;
e_half = max(gt_min_range/2, (max(East) - min(East)) / 2 * 1.2);
n_half = max(gt_min_range/2, (max(North) - min(North)) / 2 * 1.2);
half_span = max(e_half, n_half);
xlim(ax_gt, [e_mid - half_span, e_mid + half_span]);
ylim(ax_gt, [n_mid - half_span, n_mid + half_span]);
xlabel(ax_gt, 'East (m)', 'Color', c_white, 'FontSize', 7);
ylabel(ax_gt, 'North (m)', 'Color', c_white, 'FontSize', 7);
title(ax_gt, 'GROUND TRACK', 'Color', c_white, 'FontSize', 8, 'FontWeight', 'bold');
disableDefaultInteractivity(ax_gt);
ax_gt.Toolbar.Visible = 'off';

% =====================================================================
%  REPLAY MODE — clock-based animation, runs in real-time
% =====================================================================
if REPLAY
    % Replace the TeX readout with a plain text object for speed
    delete(h_readout);
    h_txt = text(0.5, 0.5, '', 'Parent', axes('Parent', fig, 'Units', 'normalized', ...
        'Position', [readout_x 0.58 readout_w 0.39], 'Visible', 'off'), ...
        'Units', 'normalized', 'Position', [0.05 0.95], ...
        'FontSize', 10, 'FontName', 'Consolas', 'FontWeight', 'bold', ...
        'Color', c_cyan, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

    % Precompute frame sample indices (only ~100-300 frames for 10s)
    n_frames = round(t(end) * replay_fps / replay_speed);
    frame_times = linspace(0, t(end), n_frames);
    frame_idx = interp1(t, 1:N_out, frame_times, 'nearest', 'extrap');
    frame_idx = max(1, min(N_out, round(frame_idx)));

    % Use wall clock to stay in sync
    tic;
    for f = 1:length(frame_idx)
        if ~isvalid(fig); break; end

        k = frame_idx(f);

        % Update 3D trajectory
        set(h_traj3d, 'XData', X_plot(1:k), 'YData', Y_plot(1:k), 'ZData', Z_plot(1:k));
        set(h_end3d, 'XData', X_plot(k), 'YData', Y_plot(k), 'ZData', Z_plot(k));

        % Update time-series (grow lines)
        for c = 1:3
            set(h_acc(c), 'XData', t(1:k), 'YData', accel_interp(1:k, c));
            set(h_vel(c), 'XData', t(1:k), 'YData', vel_data(1:k, c));
        end
        set(h_ori(1), 'XData', t(1:k), 'YData', roll_deg(1:k));
        set(h_ori(2), 'XData', t(1:k), 'YData', pitch_deg(1:k));
        set(h_ori(3), 'XData', t(1:k), 'YData', yaw_deg(1:k));

        % Update ground track
        set(h_gt, 'XData', East(1:k), 'YData', North(1:k));
        set(h_gt_end, 'XData', East(k), 'YData', North(k));

        % Update readout (plain text, no TeX parsing)
        [max_alt_k, ap_idx] = max(altitude(1:k));
        apogee_now = ap_idx < k && k > 10;
        r = '';
        if apogee_now
            r = sprintf('MAX ALT:    %.1f m\n', max_alt_k);
        end
        r = [r sprintf(['ALT:        %.1f m\n' ...
                         'AIRSPEED:   %.1f m/s\n' ...
                         'HEADING:    %05.1f\n' ...
                         'CLIMB RATE: %.1f ft/min\n' ...
                         'BARO:       %.0f Pa\n' ...
                         '\nt = %.3f s\n' ...
                         '\nSTANDARD ERROR\n' ...
                         '  POS:  %.2f m\n' ...
                         '  VEL:  %.2f m/s\n' ...
                         '  ATT:  %.2f'], ...
            altitude(k), airspeed(k), heading_deg(k), ...
            climbrate(k), baro_data(k), t(k), ...
            std_err_pos, std_err_vel, std_err_att)];
        set(h_txt, 'String', r);

        drawnow limitrate;

        % Wait only if ahead of wall clock
        target_time = frame_times(f) / replay_speed;
        elapsed = toc;
        if elapsed < target_time
            pause(target_time - elapsed);
        end
    end
end

% =====================================================================
%  HELPER: Build readout string for a given sample index
% =====================================================================
function s = build_readout(altitude, airspeed, heading_deg, climbrate, ...
    baro_data, t, std_err_pos, std_err_vel, std_err_att, ...
    c_cyan, c_yellow, c_orange, k)

    [max_alt, apogee_idx] = max(altitude(1:k));
    apogee_reached = apogee_idx < k && k > 10;

    lines = {};
    if apogee_reached
        lines{end+1} = sprintf('\\color[rgb]{%g %g %g}MAX ALT:    %.1f m', c_yellow, max_alt);
    end
    lines{end+1} = sprintf('\\color[rgb]{%g %g %g}ALT:        %.1f m', c_cyan, altitude(k));
    lines{end+1} = sprintf('\\color[rgb]{%g %g %g}AIRSPEED:   %.1f m/s', c_cyan, airspeed(k));
    lines{end+1} = sprintf('\\color[rgb]{%g %g %g}HEADING:    %05.1f\\circ', c_cyan, heading_deg(k));
    lines{end+1} = sprintf('\\color[rgb]{%g %g %g}CLIMB RATE: %.1f ft/min', c_cyan, climbrate(k));
    lines{end+1} = sprintf('\\color[rgb]{%g %g %g}BARO:       %.0f Pa', c_cyan, baro_data(k));
    lines{end+1} = '';
    lines{end+1} = sprintf('\\color{white}t = %.3f s', t(k));
    lines{end+1} = '';
    lines{end+1} = sprintf('\\color[rgb]{%g %g %g}STANDARD ERROR', c_orange);
    lines{end+1} = sprintf('\\color{white}  POS:  %.2f m', std_err_pos);
    lines{end+1} = sprintf('\\color{white}  VEL:  %.2f m/s', std_err_vel);
    lines{end+1} = sprintf('\\color{white}  ATT:  %.2f\\circ', std_err_att);

    s = strjoin(lines, '\n');
end
