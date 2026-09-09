%% data_MJ.m
% Compact viewer for drone pose, end-effector position, attitude,
% per-propeller thrust, MOB pure force, and battery voltage.
%
% Supported inputs:
% - data_logging_debug CSV: *_debug.csv
% - data_logging CSV: *.csv, when matching columns are available
%
clear; close all; clc;
set(groot, 'defaultFigureRenderer', 'painters');

%% 0) User config
sample_hz = 20.0;  % Used only when the CSV has no t_sec column. Match crazyflies_debug.yaml logging frequency.
axis_names = {'x', 'y', 'z'};
att_names = {'roll', 'pitch', 'yaw'};

% Offline pure MOB reconstruction. Gains mirror crazyflie-firmware:
% src/modules/src/su_params.c and src/modules/src/su_wrench_observer.c
su_params_path = fullfile(getenv("HOME"), "hitl_ws", "src", "crazyswarm2", ...
    "crazyflie", "config", "su_params.yaml");
sitl_wrench_observer_path = fullfile(getenv("HOME"), "sitl_ws", "src", ...
    "mujoco_crazyflie", "flyingpen_interface", "config", "wrench_observer.yaml");
sitl_su_params_path = fullfile(getenv("HOME"), "sitl_ws", "src", ...
    "mujoco_crazyflie", "flyingpen_interface", "config", "su_params.yaml");

% Per-flight hover calibration override. Fill these from the calibration log
% for the selected flight. Leave [] to fall back to the current su_params.yaml.
manual_calib_mass_kg = [0.045];              % e.g. 0.047311
manual_calib_com_offset_body = [0.00 0.00];      % e.g. [-0.000486, 0.000093, 0.0]

yaml_mob_mass_kg = local_read_yaml_scalar(su_params_path, "mass", 0.047311);
yaml_com_offset_body = [
    local_read_yaml_scalar(su_params_path, "comOffX", 0.0), ...
    local_read_yaml_scalar(su_params_path, "comOffY", 0.0), ...
    local_read_yaml_scalar(su_params_path, "comOffZ", 0.0)];

offline_mob_mass_kg = yaml_mob_mass_kg;
offline_com_offset_body = yaml_com_offset_body;
offline_calib_source = "current su_params.yaml";
if local_is_valid_scalar(manual_calib_mass_kg)
    offline_mob_mass_kg = manual_calib_mass_kg;
    offline_calib_source = "manual override";
end
if local_is_valid_vector3(manual_calib_com_offset_body)
    offline_com_offset_body = manual_calib_com_offset_body;
    offline_calib_source = "manual override";
end

offline_mob_gravity_ms2 = local_read_yaml_scalar(sitl_wrench_observer_path, "g", 9.81);
offline_mob_Kf = 5.0;                 % [1/s] su_Kf
offline_mob_Kp = 2.9544511501;        % [1/s] su_Kp
offline_mob_use_pos_velocity = true;   % firmware uses suVelFromPosGetWorld()
offline_mob_dt_mode = "log";           % "log" uses CSV sample dt; "fixed" uses one fixed_dt step per CSV row.
offline_mob_fixed_dt = 1.0 / 100.0;    % [s] SU_WRENCH_RATE_HZ = 100

% Offline point-contact consistency / thrust-effectiveness MOB.
% These mirror the MuJoCo SITL wrench_observer.cpp variants.
offline_pc_Ktau = local_read_yaml_scalar(sitl_wrench_observer_path, "mob.Ktau_2nd_order", 30.0);
offline_pc_KpTau = local_read_yaml_scalar(sitl_wrench_observer_path, "mob.KpTau", 10.9544511501);
offline_pc_alpha = local_read_yaml_scalar(sitl_wrench_observer_path, "mob.mob_alpha_2nd_order", 1.0);
offline_pc_Ke = local_read_yaml_scalar(sitl_su_params_path, "ke", ...
    local_read_yaml_scalar(sitl_wrench_observer_path, "mob.Ke", 50.0));
offline_pc_epsilon_tau = local_read_yaml_scalar(sitl_wrench_observer_path, "mob.epsilon_tau", 1.0e-6);

% Manual eta_T tuning. These are intentionally not loaded from YAML so each
% MATLAB run can sweep/check the offline thrust-effectiveness behavior.
offline_eta_gamma = 20.0;               % adaptation gain; larger tracks faster/noisier
offline_eta_rho = 0.1;                  % regularization/leakage; larger damps eta_T motion
offline_eta_initial = 1.0;              % initial thrust effectiveness estimate
offline_inertia_diag = [2.3951e-5, 2.3951e-5, 3.2347e-5];  % [Jxx Jyy Jzz]
boom_tip_offset_body = [0.09, 0.0, 0.035];  % [m] body-frame boom tip: +x 90 mm, +z 35 mm
offline_contact_offset_body = boom_tip_offset_body;
offline_arm_xy = 0.7071067811865476 * 0.050;  % cf21bl ARM_LENGTH
offline_k_tau_motor = 0.00569278844371417;
offline_motor_dir = [-1.0, 1.0, -1.0, 1.0];  % firmware su_wrench_observer yaw torque convention
wall_true_normal_body = [-1.0, 0.0, 0.0];     % tilted wall local normal used as true contact direction
wall_true_tangent1_body = [0.0, 1.0, 0.0];    % tilted wall local tangent basis 1
wall_true_tangent2_body = [0.0, 0.0, 1.0];    % tilted wall local tangent basis 2

% End-effector offset in the drone body frame [m].
% Edit these values to match su_params.yaml / your hardware.
ee_offset_body = boom_tip_offset_body;  % [x, y, z]

defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
if ~isfolder(defaultDir)
    defaultDir = pwd;
end

%% 1) Pick CSV and read
[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging CSV");
if isequal(file, 0)
    disp("Canceled.");
    return;
end

csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

opts = detectImportOptions(csv_path, 'Delimiter', ',');
for i = 1:numel(opts.VariableTypes)
    opts.VariableTypes{i} = 'double';
end
T = readtable(csv_path, opts);

if isempty(T) || height(T) < 2
    error("CSV has too few rows.");
end

vars = string(T.Properties.VariableNames);
fprintf("[INFO] Rows: %d, Columns: %d\n", height(T), numel(vars));

if any(vars == "t_sec")
    time = local_get1(T, vars, "t_sec");
    first_valid = find(isfinite(time), 1, 'first');
    if ~isempty(first_valid)
        time = time - time(first_valid);
    end
else
    time = (0:height(T)-1).' ./ sample_hz;
end

%% 2) Load requested signals
pose_xyz = [
    local_get1(T, vars, "pose_x"), ...
    local_get1(T, vars, "pose_y"), ...
    local_get1(T, vars, "pose_z")];

pose_rpy = [
    local_get1(T, vars, "pose_roll"), ...
    local_get1(T, vars, "pose_pitch"), ...
    unwrap(local_get1(T, vars, "pose_yaw"))];

ee_xyz = local_compute_ee_position_world(pose_xyz, pose_rpy, ee_offset_body);

motor_thrust = [
    local_get1_fallback(T, vars, "f1", "motor_f1"), ...
    local_get1_fallback(T, vars, "f2", "motor_f2"), ...
    local_get1_fallback(T, vars, "f3", "motor_f3"), ...
    local_get1_fallback(T, vars, "f4", "motor_f4")];
body_torque = [
    local_get1(T, vars, "bodyTx"), ...
    local_get1(T, vars, "bodyTy"), ...
    local_get1(T, vars, "bodyTz")];

state_vel = [
    local_get1(T, vars, "stateVx"), ...
    local_get1(T, vars, "stateVy"), ...
    local_get1(T, vars, "stateVz")];
pos_vel = [
    local_get1(T, vars, "posVx"), ...
    local_get1(T, vars, "posVy"), ...
    local_get1(T, vars, "posVz")];
gyro_body_deg_s = [
    local_get1_fallback(T, vars, "gyroBody_x", "gyro_x"), ...
    local_get1_fallback(T, vars, "gyroBody_y", "gyro_y"), ...
    local_get1_fallback(T, vars, "gyroBody_z", "gyro_z")];

battery_voltage = local_get1_fallback(T, vars, "pm_vbat", "status_battery_voltage");
status_battery_voltage = local_get1(T, vars, "status_battery_voltage");
pm_vbat = local_get1(T, vars, "pm_vbat");

wall_xyz = [
    local_get1(T, vars, "wall_x"), ...
    local_get1(T, vars, "wall_y"), ...
    local_get1(T, vars, "wall_z")];
wall_quat_xyzw = [
    local_get1(T, vars, "wall_qx"), ...
    local_get1(T, vars, "wall_qy"), ...
    local_get1(T, vars, "wall_qz"), ...
    local_get1(T, vars, "wall_qw")];
wall_rpy = local_quat_xyzw_to_rpy(wall_quat_xyzw);
[wall_true_normal_world, wall_true_tangent1_world, wall_true_tangent2_world] = ...
    local_compute_wall_basis_world(wall_quat_xyzw, wall_true_normal_body, ...
    wall_true_tangent1_body, wall_true_tangent2_body);

mob_force_none = [
    local_get1_any(T, vars, ["rawMobFx", "mobForceFinal_x", "mobForceNone_x"]), ...
    local_get1_any(T, vars, ["rawMobFy", "mobForceFinal_y", "mobForceNone_y"]), ...
    local_get1_any(T, vars, ["rawMobFz", "mobForceFinal_z", "mobForceNone_z"])];
mob_torque = [
    local_get1_any(T, vars, ["rawMobTx", "mobTorque_x"]), ...
    local_get1_any(T, vars, ["rawMobTy", "mobTorque_y"]), ...
    local_get1_any(T, vars, ["rawMobTz", "mobTorque_z"])];
firmware_eta_hat = local_get1_any(T, vars, ["etaHat", "thrustEffEtaHat"]);
firmware_eta_match_force = [
    local_get1(T, vars, "thrustEffMatchFx"), ...
    local_get1(T, vars, "thrustEffMatchFy"), ...
    local_get1(T, vars, "thrustEffMatchFz")];
firmware_eta_residual = [
    local_get1(T, vars, "thrustEffEpsTx"), ...
    local_get1(T, vars, "thrustEffEpsTy"), ...
    local_get1(T, vars, "thrustEffEpsTz")];
firmware_eta_force = [
    local_get1_any(T, vars, ["contactFx", "thrustEffCorrFx"]), ...
    local_get1_any(T, vars, ["contactFy", "thrustEffCorrFy"]), ...
    local_get1_any(T, vars, ["contactFz", "thrustEffCorrFz"])];
normal_est = [
    local_get1(T, vars, "normalEst_x"), ...
    local_get1(T, vars, "normalEst_y"), ...
    local_get1(T, vars, "normalEst_z")];
contact_force_delta = firmware_eta_force - mob_force_none;

offline_mob_force_none = local_compute_offline_pure_mob( ...
    time, pose_rpy, motor_thrust, state_vel, pos_vel, ...
    offline_mob_mass_kg, offline_mob_gravity_ms2, ...
    offline_mob_Kp, offline_mob_Kf, ...
    offline_mob_use_pos_velocity, offline_mob_dt_mode, offline_mob_fixed_dt);

[~, offline_mob_torque_2nd, ~, offline_mob_eta_force, offline_eta_hat] = ...
    local_compute_offline_point_contact_eta_mob( ...
    time, pose_rpy, motor_thrust, state_vel, pos_vel, gyro_body_deg_s, ...
    offline_mob_mass_kg, offline_mob_gravity_ms2, offline_mob_Kp, offline_mob_Kf, ...
    offline_pc_KpTau, offline_pc_Ktau, offline_pc_alpha, offline_pc_Ke, ...
    offline_pc_epsilon_tau, offline_eta_gamma, offline_eta_rho, offline_eta_initial, ...
    offline_inertia_diag, offline_com_offset_body, offline_contact_offset_body, ...
    offline_arm_xy, offline_k_tau_motor, offline_motor_dir, ...
    offline_mob_use_pos_velocity, offline_mob_dt_mode, offline_mob_fixed_dt);

valid_time = isfinite(time);
time = time(valid_time);
pose_xyz = pose_xyz(valid_time, :);
pose_rpy = pose_rpy(valid_time, :);
ee_xyz = ee_xyz(valid_time, :);
motor_thrust = motor_thrust(valid_time, :);
body_torque = body_torque(valid_time, :);
state_vel = state_vel(valid_time, :);
pos_vel = pos_vel(valid_time, :);
gyro_body_deg_s = gyro_body_deg_s(valid_time, :);
battery_voltage = battery_voltage(valid_time);
status_battery_voltage = status_battery_voltage(valid_time);
pm_vbat = pm_vbat(valid_time);
wall_xyz = wall_xyz(valid_time, :);
wall_quat_xyzw = wall_quat_xyzw(valid_time, :);
wall_rpy = wall_rpy(valid_time, :);
mob_force_none = mob_force_none(valid_time, :);
mob_torque = mob_torque(valid_time, :);
firmware_eta_hat = firmware_eta_hat(valid_time);
firmware_eta_match_force = firmware_eta_match_force(valid_time, :);
firmware_eta_residual = firmware_eta_residual(valid_time, :);
firmware_eta_force = firmware_eta_force(valid_time, :);
normal_est = normal_est(valid_time, :);
contact_force_delta = contact_force_delta(valid_time, :);
offline_mob_force_none = offline_mob_force_none(valid_time, :);
offline_mob_torque_2nd = offline_mob_torque_2nd(valid_time, :);
offline_mob_eta_force = offline_mob_eta_force(valid_time, :);
offline_eta_hat = offline_eta_hat(valid_time);
wall_true_normal_world = wall_true_normal_world(valid_time, :);
wall_true_tangent1_world = wall_true_tangent1_world(valid_time, :);
wall_true_tangent2_world = wall_true_tangent2_world(valid_time, :);
wall_normal_frame_rpy = local_basis_to_rpy( ...
    wall_true_normal_world, wall_true_tangent1_world, wall_true_tangent2_world);
mob_normal_frame_rpy_online_pure = local_force_to_normal_frame_rpy( ...
    mob_force_none, wall_true_normal_world, wall_true_tangent1_world);
mob_normal_frame_rpy_offline_pure = local_force_to_normal_frame_rpy( ...
    offline_mob_force_none, wall_true_normal_world, wall_true_tangent1_world);
mob_normal_frame_rpy_offline_eta = local_force_to_normal_frame_rpy( ...
    offline_mob_eta_force, wall_true_normal_world, wall_true_tangent1_world);
mob_normal_frame_rpy_firmware_eta = local_force_to_normal_frame_rpy( ...
    firmware_eta_force, wall_true_normal_world, wall_true_tangent1_world);

fprintf("[INFO] EE offset body [m] = [%.4f %.4f %.4f]\n", ee_offset_body);
fprintf("[INFO] Offline calibration source = %s\n", char(offline_calib_source));
fprintf("[INFO] Offline calibration mass/com = %.6f kg, [%.6f %.6f %.6f] m\n", ...
    offline_mob_mass_kg, offline_com_offset_body);
local_print_availability("drone position", pose_xyz);
local_print_availability("end-effector position", ee_xyz);
local_print_availability("drone attitude", pose_rpy);
local_print_availability("per-propeller thrust", motor_thrust);
local_print_availability("input body torque", body_torque);
local_print_availability("state velocity", state_vel);
local_print_availability("position velocity", pos_vel);
local_print_availability("body gyro", gyro_body_deg_s);
local_print_availability("battery voltage", battery_voltage);
local_print_availability("tilted wall position", wall_xyz);
local_print_availability("tilted wall attitude", wall_rpy);
local_print_availability("tilted wall quaternion", wall_quat_xyzw);
local_print_availability("tilted wall true normal", wall_true_normal_world);
local_print_availability("tilted wall tangent 1", wall_true_tangent1_world);
local_print_availability("tilted wall tangent 2", wall_true_tangent2_world);
local_print_availability("tilted wall normal RPY", wall_normal_frame_rpy);
local_print_availability("online pure normal RPY", mob_normal_frame_rpy_online_pure);
local_print_availability("offline pure normal RPY", mob_normal_frame_rpy_offline_pure);
local_print_availability("offline eta_T normal RPY", mob_normal_frame_rpy_offline_eta);
local_print_availability("MOB force pure", mob_force_none);
local_print_availability("MOB torque", mob_torque);
local_print_availability("offline MOB pure", offline_mob_force_none);
local_print_availability("offline MOB torque 2nd", offline_mob_torque_2nd);
local_print_availability("offline MOB eta_T", offline_mob_eta_force);
local_print_availability("firmware eta_T", firmware_eta_hat);
local_print_availability("firmware eta_T matched force", firmware_eta_match_force);
local_print_availability("firmware eta_T residual", firmware_eta_residual);
local_print_availability("firmware eta_T corrected force", firmware_eta_force);
local_print_availability("estimated normal", normal_est);
fprintf("[INFO] Offline MOB mass %.6f kg, Kp %.6f, Kf %.3f, dt mode %s\n", ...
    offline_mob_mass_kg, offline_mob_Kp, offline_mob_Kf, char(offline_mob_dt_mode));
fprintf("[INFO] Offline point-contact MOB Ktau %.3f, KpTau %.6f, Ke %.3f, eta gamma %.3f\n", ...
    offline_pc_Ktau, offline_pc_KpTau, offline_pc_Ke, offline_eta_gamma);

%% 3) Plot: drone position
drone_position_xlim = [];              % e.g. [10 80], [] keeps auto x-limits
drone_position_ylims = {[], [], []};   % x/y/z y-limits

figure('Name', 'Drone Position', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile;
    plot(time, pose_xyz(:, i), 'LineWidth', 1.4);
    grid on;
    ylabel(sprintf('%s [m]', axis_names{i}));
    title(sprintf('Drone position %s', axis_names{i}));
    local_apply_limits(ax, drone_position_xlim, drone_position_ylims{i});
end
xlabel('time [s]');

%% 4) Plot: end-effector position
ee_position_xlim = [];              % e.g. [10 80], [] keeps auto x-limits
ee_position_ylims = {[], [], []};   % x/y/z y-limits
drone_ee_xyz_xlim = [];             % 3D x-axis limits
drone_ee_xyz_ylim = [];             % 3D y-axis limits
drone_ee_xyz_zlim = [];             % 3D z-axis limits

figure('Name', 'End-Effector Position', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile;
    plot(time, ee_xyz(:, i), 'LineWidth', 1.4);
    grid on;
    ylabel(sprintf('%s [m]', axis_names{i}));
    title(sprintf('End-effector position %s', axis_names{i}));
    local_apply_limits(ax, ee_position_xlim, ee_position_ylims{i});
end
xlabel('time [s]');

figure('Name', 'Drone and End-Effector XYZ', 'Color', 'w');
plot3(pose_xyz(:,1), pose_xyz(:,2), pose_xyz(:,3), 'LineWidth', 1.2);
hold on;
plot3(ee_xyz(:,1), ee_xyz(:,2), ee_xyz(:,3), 'LineWidth', 1.2);
if any(isfinite(wall_xyz(:)))
    plot3(wall_xyz(:,1), wall_xyz(:,2), wall_xyz(:,3), 'LineWidth', 1.2);
end
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Drone, end-effector, and tilted wall position');
local_apply_3d_limits(gca, drone_ee_xyz_xlim, drone_ee_xyz_ylim, drone_ee_xyz_zlim);
if any(isfinite(wall_xyz(:)))
    legend({'drone', 'end-effector', 'tilted wall'}, 'Location', 'best');
else
    legend({'drone', 'end-effector'}, 'Location', 'best');
end

%% 5) Plot: drone attitude
drone_attitude_xlim = [];              % e.g. [10 80], [] keeps auto x-limits
drone_attitude_ylims = {[], [], []};   % roll/pitch/yaw y-limits

figure('Name', 'Drone Attitude', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    ax = nexttile;
    plot(time, pose_rpy(:, i), 'LineWidth', 1.4);
    grid on;
    ylabel('[rad]');
    title(sprintf('Drone attitude %s', att_names{i}));
    local_apply_limits(ax, drone_attitude_xlim, drone_attitude_ylims{i});
end
xlabel('time [s]');

%% 6) Plot: per-propeller thrust
thrust_xlim = [];      % e.g. [10 80], shared by propeller/total thrust plots
thrust_ylims = {[], []};  % {per-propeller, total}

figure('Name', 'Per-Propeller Thrust', 'Color', 'w');
plot(time, motor_thrust, 'LineWidth', 1.3);
grid on;
xlabel('time [s]');
ylabel('thrust [N]');
title('Per-propeller thrust');
legend({'f1', 'f2', 'f3', 'f4'}, 'Location', 'best');
local_apply_limits(gca, thrust_xlim, thrust_ylims{1});

figure('Name', 'Total Thrust', 'Color', 'w');
plot(time, sum(motor_thrust, 2, 'omitnan'), 'k', 'LineWidth', 1.4);
grid on;
xlabel('time [s]');
ylabel('total thrust [N]');
title('Total thrust');
local_apply_limits(gca, thrust_xlim, thrust_ylims{2});

%% 7) Plot: input torque and online/offline MOB torque
torque_xlim = [0 570];                         % e.g. [10 80], shared by torque plots
input_torque_ylims = {[-0.002 0.002], [-0.006 0.006], [-0.006 0.006]};        % x/y/z input torque y-limits
mob_torque_ylims = {[-0.002 0.002], [-0.002 0.002], [-0.002 0.002]};          % x/y/z MOB overlay y-limits

if any(isfinite(body_torque(:)))
    figure('Name', 'Input Body Torque', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        plot(time, body_torque(:, i), 'LineWidth', 1.3);
        grid on;
        ylabel(sprintf('%s [N*m]', axis_names{i}));
        title(sprintf('Input body torque %s', axis_names{i}));
        local_apply_limits(ax, torque_xlim, input_torque_ylims{i});
    end
    xlabel('time [s]');
end

if any(isfinite([mob_torque(:); offline_mob_torque_2nd(:)]))
    figure('Name', 'MOB Torque Online vs Offline Variants', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        hold on;
        torque_legend_entries = {};
        if any(isfinite(mob_torque(:, i)))
            plot(time, mob_torque(:, i), 'LineWidth', 1.3);
            torque_legend_entries{end+1} = 'online pure';
        end
        if any(isfinite(offline_mob_torque_2nd(:, i)))
            plot(time, offline_mob_torque_2nd(:, i), '--', 'LineWidth', 1.2);
            torque_legend_entries{end+1} = 'offline pure';
        end
        grid on;
        ylabel(sprintf('%s [N*m]', axis_names{i}));
        title(sprintf('MOB torque %s: online vs offline variants', axis_names{i}));
        if ~isempty(torque_legend_entries)
            legend(torque_legend_entries, 'Location', 'best');
        end
        local_apply_limits(ax, torque_xlim, mob_torque_ylims{i});
    end
    xlabel('time [s]');
end

%% 8) Plot: online/offline MOB force
mob_force_xlim = [60 570];              % e.g. [10 80], [] keeps auto x-limits
mob_force_ylims = {[-0.2 0.2], [-0.2 0.2], [-0.2 0.2]};   % x/y/z force y-limits

if any(isfinite([mob_force_none(:); offline_mob_force_none(:); offline_mob_eta_force(:); firmware_eta_force(:)]))
    figure('Name', 'MOB Force Online vs Offline Variants', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        hold on;
        if any(isfinite(mob_force_none(:, i)))
            plot(time, mob_force_none(:, i), 'LineWidth', 1.3);
        end
        if any(isfinite(offline_mob_force_none(:, i)))
            plot(time, offline_mob_force_none(:, i), '--', 'LineWidth', 1.2);
        end
        if any(isfinite(offline_mob_eta_force(:, i)))
            plot(time, offline_mob_eta_force(:, i), 'k', 'LineWidth', 1.3);
        end
        if any(isfinite(firmware_eta_force(:, i)))
            plot(time, firmware_eta_force(:, i), ':', 'LineWidth', 1.5);
        end
        grid on;
        ylabel(sprintf('%s [N]', axis_names{i}));
        title(sprintf('MOB force %s: online vs offline variants', axis_names{i}));
        mob_legend_entries = {};
        if any(isfinite(mob_force_none(:, i))), mob_legend_entries{end+1} = 'online pure'; end
        if any(isfinite(offline_mob_force_none(:, i))), mob_legend_entries{end+1} = 'offline pure'; end
        if any(isfinite(offline_mob_eta_force(:, i))), mob_legend_entries{end+1} = 'offline eta\_T'; end
        if any(isfinite(firmware_eta_force(:, i))), mob_legend_entries{end+1} = 'firmware eta\_T'; end
        if ~isempty(mob_legend_entries)
            legend(mob_legend_entries, 'Location', 'best');
        end
        local_apply_limits(ax, mob_force_xlim, mob_force_ylims{i});
    end
    xlabel('time [s]');
end

%% 9) Plot: pipeline debug
pipeline_debug_xlim = [];                         % shared by all pipeline-debug plots
pipeline_force_ylims = {[], [], []};              % raw/corrected force x/y/z
pipeline_correction_delta_ylims = {[]};            % correction-delta overlay
pipeline_estimated_normal_ylims = {[-1.05 1.05]}; % estimated-normal overlay

if any(isfinite([mob_force_none(:); firmware_eta_force(:)]))
    figure('Name', 'Raw MOB vs Corrected Contact Force', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        plot(time, mob_force_none(:, i), 'LineWidth', 1.3);
        hold on;
        plot(time, firmware_eta_force(:, i), 'LineWidth', 1.3);
        grid on;
        ylabel(sprintf('%s [N]', axis_names{i}));
        title(sprintf('Raw MOB vs corrected contact force: %s', axis_names{i}));
        legend({'raw MOB', 'corrected contact'}, 'Location', 'best');
        local_apply_limits(ax, pipeline_debug_xlim, pipeline_force_ylims{i});
    end
    xlabel('time [s]');

    figure('Name', 'Contact Force Correction Delta', 'Color', 'w');
    plot(time, contact_force_delta, 'LineWidth', 1.3);
    grid on;
    xlabel('time [s]');
    ylabel('contact - raw MOB [N]');
    title('eta_T force correction delta');
    legend({'deltaFx', 'deltaFy', 'deltaFz'}, 'Location', 'best');
    local_apply_limits(gca, pipeline_debug_xlim, pipeline_correction_delta_ylims{1});
end

if any(isfinite(normal_est(:)))
    figure('Name', 'Estimated Normal Vector', 'Color', 'w');
    plot(time, normal_est, 'LineWidth', 1.3);
    grid on;
    xlabel('time [s]');
    ylabel('normal component [-]');
    title('Firmware estimated normal');
    legend({'normalX', 'normalY', 'normalZ'}, 'Location', 'best');
    local_apply_limits(gca, pipeline_debug_xlim, pipeline_estimated_normal_ylims{1});
end

%% 10) Plot: pure MOB vs eta_T-updated MOB and normalized wall-normal comparison
pure_eta_axis_names = {'x', 'y', 'z'};
pure_eta_comparison_xlim = [0 650];
pure_eta_force_ylims = {[-0.14 0.02], [-0.14 0.02], [-0.14 0.02]};
pure_eta_normalized_ylims = {[-1.05 1.05], [-1.05 1.05], [-1.05 1.05]};

mob_force_none_normalized = local_normalize_rows(mob_force_none);
firmware_eta_force_normalized = local_normalize_rows(firmware_eta_force);

if any(isfinite([mob_force_none(:); firmware_eta_force(:); wall_true_normal_world(:)]))
    figure('Name', 'Pure MOB vs Eta-T Updated MOB and Wall Normal', ...
        'Color', 'w', 'Position', [100 100 1400 800]);
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for i = 1:3
        ax_force = nexttile(2 * i - 1);
        hold on;
        force_legend_entries = {};
        if any(isfinite(mob_force_none(:, i)))
            plot(time, mob_force_none(:, i), 'LineWidth', 1.3);
            force_legend_entries{end+1} = 'pure MOB';
        end
        if any(isfinite(firmware_eta_force(:, i)))
            plot(time, firmware_eta_force(:, i), 'LineWidth', 1.3);
            force_legend_entries{end+1} = '\eta_T-updated MOB';
        end
        grid on;
        ylabel(sprintf('%s [N]', pure_eta_axis_names{i}));
        title(sprintf('Pure vs \\eta_T-updated MOB: %s', pure_eta_axis_names{i}));
        if ~isempty(force_legend_entries)
            legend(force_legend_entries, 'Location', 'best');
        end
        local_apply_limits(ax_force, pure_eta_comparison_xlim, pure_eta_force_ylims{i});

        ax_normalized = nexttile(2 * i);
        hold on;
        normalized_legend_entries = {};
        if any(isfinite(mob_force_none_normalized(:, i)))
            plot(time, mob_force_none_normalized(:, i), 'LineWidth', 1.3);
            normalized_legend_entries{end+1} = 'normalized pure MOB';
        end
        if any(isfinite(firmware_eta_force_normalized(:, i)))
            plot(time, firmware_eta_force_normalized(:, i), 'LineWidth', 1.3);
            normalized_legend_entries{end+1} = 'normalized \eta_T-updated MOB';
        end
        if any(isfinite(wall_true_normal_world(:, i)))
            plot(time, wall_true_normal_world(:, i), 'k--', 'LineWidth', 1.5);
            normalized_legend_entries{end+1} = 'wall normal';
        end
        grid on;
        ylabel(sprintf('%s [-]', pure_eta_axis_names{i}));
        title(sprintf('Normalized force vs wall normal: %s', pure_eta_axis_names{i}));
        if ~isempty(normalized_legend_entries)
            legend(normalized_legend_entries, 'Location', 'best');
        end
        local_apply_limits(ax_normalized, pure_eta_comparison_xlim, pure_eta_normalized_ylims{i});
    end
    xlabel(ax_force, 'time [s]');
    xlabel(ax_normalized, 'time [s]');
end

%% 11) Plot: thrust effectiveness online/offline eta_T overlay
eta_overlay_xlim = [];       % shared x-limit for this section
eta_overlay_ylims = {[]};    % eta_T overlay

if any(isfinite([offline_eta_hat; firmware_eta_hat]))
    figure('Name', 'Thrust Effectiveness Online Offline Eta_T Overlay', 'Color', 'w');
    hold on;
    eta_legend_entries = {};
    if any(isfinite(firmware_eta_hat))
        plot(time, firmware_eta_hat, 'LineWidth', 1.4);
        eta_legend_entries{end+1} = 'online \eta_T';
    end
    if any(isfinite(offline_eta_hat))
        plot(time, offline_eta_hat, '--', 'LineWidth', 1.4);
        eta_legend_entries{end+1} = 'offline \eta_T';
    end
    grid on;
    xlabel('time [s]');
    ylabel('\eta_T [-]');
    title('Thrust effectiveness: online vs offline');
    legend(eta_legend_entries, 'Location', 'best');
    local_apply_limits(gca, eta_overlay_xlim, eta_overlay_ylims{1});
end

%% 12) Plot: firmware thrust effectiveness debug
firmware_eta_debug_xlim = ([0 50]);        % shared with eta_T estimate
firmware_eta_force_ylims = {[-5 5], [-5 5], [-5 5]};  % corrected/matched force y-limits
firmware_eta_residual_ylims = {[], [], []};  % torque residual y-limits

if any(isfinite([firmware_eta_match_force(:); firmware_eta_force(:); firmware_eta_residual(:)]))
    figure('Name', 'Firmware Thrust Effectiveness Debug', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        hold on;
        fw_legend_entries = {};
        if any(isfinite(firmware_eta_match_force(:, i)))
            plot(time, firmware_eta_match_force(:, i), '--', 'LineWidth', 1.2);
            fw_legend_entries{end+1} = 'matched force';
        end
        if any(isfinite(firmware_eta_force(:, i)))
            plot(time, firmware_eta_force(:, i), 'LineWidth', 1.4);
            fw_legend_entries{end+1} = 'corrected force';
        end
        grid on;
        ylabel(sprintf('%s [N]', axis_names{i}));
        title(sprintf('Firmware eta\\_T force %s', axis_names{i}));
        if ~isempty(fw_legend_entries)
            legend(fw_legend_entries, 'Location', 'best');
        end
        local_apply_limits(ax, firmware_eta_debug_xlim, firmware_eta_force_ylims{i});
    end
    xlabel('time [s]');

    figure('Name', 'Firmware Thrust Effectiveness Torque Residual', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        plot(time, firmware_eta_residual(:, i), 'LineWidth', 1.3);
        grid on;
        ylabel(sprintf('%s [N*m]', axis_names{i}));
        title(sprintf('Firmware eta\\_T torque residual %s', axis_names{i}));
        local_apply_limits(ax, firmware_eta_debug_xlim, firmware_eta_residual_ylims{i});
    end
    xlabel('time [s]');
end

%% 13) Plot: tilted wall normal-frame RPY
wall_normal_rpy_xlim = ([10 550]);             % e.g. [10 80], [] keeps auto x-limits
wall_normal_rpy_ylims = {[-0.5 0.5], [-0.5 0.5], [-0.5 0.5]};              % roll/pitch/yaw y-limits

if any(isfinite([wall_normal_frame_rpy(:); mob_normal_frame_rpy_online_pure(:); ...
        mob_normal_frame_rpy_offline_pure(:); mob_normal_frame_rpy_offline_eta(:); ...
        mob_normal_frame_rpy_firmware_eta(:)]))
    figure('Name', 'Tilted Wall Normal Frame RPY', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        hold on;
        rpy_legend_entries = {};
        if any(isfinite(wall_normal_frame_rpy(:, i)))
            plot(time, wall_normal_frame_rpy(:, i), 'k', 'LineWidth', 1.6);
            rpy_legend_entries{end+1} = 'true tilted wall';
        end
        if any(isfinite(mob_normal_frame_rpy_online_pure(:, i)))
            plot(time, mob_normal_frame_rpy_online_pure(:, i), 'LineWidth', 1.2);
            rpy_legend_entries{end+1} = 'online pure';
        end
        if any(isfinite(mob_normal_frame_rpy_offline_pure(:, i)))
            plot(time, mob_normal_frame_rpy_offline_pure(:, i), '--', 'LineWidth', 1.2);
            rpy_legend_entries{end+1} = 'offline pure';
        end
        if any(isfinite(mob_normal_frame_rpy_offline_eta(:, i)))
            plot(time, mob_normal_frame_rpy_offline_eta(:, i), '-.', 'LineWidth', 1.2);
            rpy_legend_entries{end+1} = 'offline eta\_T';
        end
        if any(isfinite(mob_normal_frame_rpy_firmware_eta(:, i)))
            plot(time, mob_normal_frame_rpy_firmware_eta(:, i), ':', 'LineWidth', 1.5);
            rpy_legend_entries{end+1} = 'firmware eta\_T';
        end
        grid on;
        ylabel('[rad]');
        title(sprintf('Tilted wall normal-frame %s', att_names{i}));
        if ~isempty(rpy_legend_entries)
            legend(rpy_legend_entries, 'Location', 'best');
        end
        local_apply_limits(ax, wall_normal_rpy_xlim, wall_normal_rpy_ylims{i});
    end
    xlabel('time [s]');
end

%% 14) Plot: battery voltage
battery_xlim = [];   % e.g. [10 80], [] keeps auto x-limits
battery_ylims = {[]};   % battery-voltage plot; e.g. {[3.2 4.2]}

figure('Name', 'Battery Voltage', 'Color', 'w');
hold on;
if any(isfinite(pm_vbat))
    plot(time, pm_vbat, 'LineWidth', 1.4);
end
if any(isfinite(status_battery_voltage))
    plot(time, status_battery_voltage, 'LineWidth', 1.2);
end
if ~any(isfinite(pm_vbat)) && ~any(isfinite(status_battery_voltage))
    plot(time, battery_voltage, 'LineWidth', 1.4);
end
grid on;
xlabel('time [s]');
ylabel('voltage [V]');
title('Battery voltage');
legend_entries = {};
if any(isfinite(pm_vbat)), legend_entries{end+1} = 'pm.vbat'; end
if any(isfinite(status_battery_voltage)), legend_entries{end+1} = 'status.battery\_voltage'; end
if isempty(legend_entries), legend_entries = {'battery voltage'}; end
legend(legend_entries, 'Location', 'best');
local_apply_limits(gca, battery_xlim, battery_ylims{1});

%% 15) Plot: tilted wall pose
wall_pose_xlim = [];                         % shared by all wall time plots
wall_position_ylims = {[], [], []};          % x/y/z position y-limits
wall_attitude_ylims = {[], [], []};          % roll/pitch/yaw y-limits
wall_rpy_overlay_ylims = {[]};               % overlay attitude plot
wall_quaternion_ylims = {[]};                % quaternion plot
ee_to_wall_x_distance_ylims = {[]};          % EE-wall x-distance plot

if any(isfinite(wall_xyz(:)))
    if ~exist('axis_names', 'var'), axis_names = {'x', 'y', 'z'}; end
    if ~exist('att_names', 'var'), att_names = {'roll', 'pitch', 'yaw'}; end

    figure('Name', 'Tilted Wall Position', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        plot(time, wall_xyz(:, i), 'LineWidth', 1.4);
        grid on;
        ylabel(sprintf('%s [m]', axis_names{i}));
        title(sprintf('Tilted wall position %s', axis_names{i}));
        local_apply_limits(ax, wall_pose_xlim, wall_position_ylims{i});
    end
    xlabel('time [s]');

    figure('Name', 'Tilted Wall Attitude', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ax = nexttile;
        plot(time, wall_rpy(:, i), 'LineWidth', 1.4);
        grid on;
        ylabel('[rad]');
        title(sprintf('Tilted wall attitude %s', att_names{i}));
        local_apply_limits(ax, wall_pose_xlim, wall_attitude_ylims{i});
    end
    xlabel('time [s]');

    figure('Name', 'Tilted Wall RPY Overlay', 'Color', 'w');
    plot(time, wall_rpy, 'LineWidth', 1.4);
    grid on;
    xlabel('time [s]');
    ylabel('attitude [rad]');
    title('Tilted wall roll, pitch, yaw');
    legend({'roll', 'pitch', 'yaw'}, 'Location', 'best');
    local_apply_limits(gca, wall_pose_xlim, wall_rpy_overlay_ylims{1});

    figure('Name', 'Tilted Wall Quaternion', 'Color', 'w');
    plot(time, wall_quat_xyzw, 'LineWidth', 1.3);
    grid on;
    xlabel('time [s]');
    ylabel('quaternion [-]');
    title('Tilted wall orientation quaternion');
    legend({'qx', 'qy', 'qz', 'qw'}, 'Location', 'best');
    local_apply_limits(gca, wall_pose_xlim, wall_quaternion_ylims{1});

    figure('Name', 'End-Effector to Wall X Distance', 'Color', 'w');
    plot(time, ee_xyz(:, 1) - wall_xyz(:, 1), 'LineWidth', 1.4);
    grid on;
    xlabel('time [s]');
    ylabel('x distance [m]');
    title('End-effector x distance from tilted wall');
    local_apply_limits(gca, wall_pose_xlim, ee_to_wall_x_distance_ylims{1});
end

%% 16) Plot: drone and tilted wall roll/pitch
drone_wall_rp_xlim = [];            % shared by roll/pitch comparison plots
drone_wall_rp_ylims = {[], []};     % {roll, pitch}

if any(isfinite(wall_rpy(:)))
    figure('Name', 'Drone vs Tilted Wall Roll Pitch', 'Color', 'w');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax = nexttile;
    plot(time, pose_rpy(:, 1), 'LineWidth', 1.5);
    hold on;
    plot(time, wall_rpy(:, 1), 'LineWidth', 1.5);
    grid on;
    ylabel('roll [rad]');
    title('Drone roll and tilted wall roll');
    legend({'drone roll', 'tilted wall roll'}, 'Location', 'best');
    local_apply_limits(ax, drone_wall_rp_xlim, drone_wall_rp_ylims{1});

    ax = nexttile;
    plot(time, pose_rpy(:, 2), 'LineWidth', 1.5);
    hold on;
    plot(time, wall_rpy(:, 2), 'LineWidth', 1.5);
    grid on;
    xlabel('time [s]');
    ylabel('pitch [rad]');
    title('Drone pitch and tilted wall pitch');
    legend({'drone pitch', 'tilted wall pitch'}, 'Location', 'best');
    local_apply_limits(ax, drone_wall_rp_xlim, drone_wall_rp_ylims{2});
end

%% Local functions
function x = local_get1(T, vars, name)
    if any(vars == name)
        x = T{:, char(name)};
    else
        x = nan(height(T), 1);
    end
end

function x = local_get1_fallback(T, vars, primary, fallback)
    x = local_get1(T, vars, primary);
    missing = ~isfinite(x);
    if any(missing)
        y = local_get1(T, vars, fallback);
        x(missing) = y(missing);
    end
end

function x = local_get1_any(T, vars, names)
    x = nan(height(T), 1);
    for name = names
        missing = ~isfinite(x);
        if ~any(missing)
            return;
        end
        y = local_get1(T, vars, name);
        x(missing) = y(missing);
    end
end

function ee_pos = local_compute_ee_position_world(pose_xyz, pose_rpy, ee_offset_body)
    n = size(pose_xyz, 1);
    ee_pos = nan(n, 3);
    for k = 1:n
        if any(~isfinite(pose_xyz(k, :))) || any(~isfinite(pose_rpy(k, :)))
            continue;
        end
        R = local_rpy_to_rotmat(pose_rpy(k, :));
        ee_pos(k, :) = pose_xyz(k, :) + (R * ee_offset_body(:)).';
    end
end

function R = local_rpy_to_rotmat(rpy)
    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);

    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);

    R = [
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
        -sp,   cp*sr,            cp*cr];
end

function rpy = local_quat_xyzw_to_rpy(q_xyzw)
    n = size(q_xyzw, 1);
    rpy = nan(n, 3);
    for k = 1:n
        q = q_xyzw(k, :);
        if any(~isfinite(q))
            continue;
        end
        qn = norm(q);
        if qn < 1.0e-9
            continue;
        end
        q = q ./ qn;
        x = q(1); y = q(2); z = q(3); w = q(4);

        sinr_cosp = 2.0 * (w*x + y*z);
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
        roll = atan2(sinr_cosp, cosr_cosp);

        sinp = 2.0 * (w*y - z*x);
        if abs(sinp) >= 1.0
            pitch = sign(sinp) * pi / 2.0;
        else
            pitch = asin(sinp);
        end

        siny_cosp = 2.0 * (w*z + x*y);
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
        yaw = atan2(siny_cosp, cosy_cosp);

        rpy(k, :) = [roll, pitch, yaw];
    end
    rpy(:, 3) = unwrap(rpy(:, 3));
end

function [normal_world_all, tangent1_world_all, tangent2_world_all] = ...
    local_compute_wall_basis_world(q_xyzw, normal_body, tangent1_body, tangent2_body)
    n = size(q_xyzw, 1);
    normal_world_all = nan(n, 3);
    tangent1_world_all = nan(n, 3);
    tangent2_world_all = nan(n, 3);
    normal_body = normal_body(:);
    tangent1_body = tangent1_body(:);
    tangent2_body = tangent2_body(:);
    for k = 1:n
        q = q_xyzw(k, :);
        if any(~isfinite(q))
            continue;
        end
        qn = norm(q);
        if qn < 1.0e-9
            continue;
        end
        q = q ./ qn;
        R = local_quat_xyzw_to_rotmat(q);
        normal_world = local_normalize_row((R * normal_body).');
        tangent1_world = local_normalize_row((R * tangent1_body).');
        tangent2_world = local_normalize_row((R * tangent2_body).');
        if any(~isfinite([normal_world, tangent1_world, tangent2_world]))
            continue;
        end

        tangent1_world = tangent1_world - dot(tangent1_world, normal_world) * normal_world;
        if norm(tangent1_world) < 1.0e-9
            tangent1_world = cross(tangent2_world, normal_world);
        end
        if norm(tangent1_world) < 1.0e-9
            tangent1_world = local_pick_perpendicular_axis(normal_world);
        else
            tangent1_world = tangent1_world ./ norm(tangent1_world);
        end

        tangent2_world = cross(normal_world, tangent1_world);
        if norm(tangent2_world) < 1.0e-9
            continue;
        end
        tangent2_world = tangent2_world ./ norm(tangent2_world);
        tangent1_world = cross(tangent2_world, normal_world);
        tangent1_world = tangent1_world ./ norm(tangent1_world);

        normal_world_all(k, :) = normal_world;
        tangent1_world_all(k, :) = tangent1_world;
        tangent2_world_all(k, :) = tangent2_world;
    end
    tangent1_world_all = local_enforce_sign_continuity(tangent1_world_all);
    for k = 1:n
        normal_world = normal_world_all(k, :);
        tangent1_world = tangent1_world_all(k, :);
        if any(~isfinite([normal_world, tangent1_world]))
            continue;
        end
        tangent2_world = cross(normal_world, tangent1_world);
        if norm(tangent2_world) < 1.0e-9
            continue;
        end
        tangent2_world_all(k, :) = tangent2_world ./ norm(tangent2_world);
        tangent1_world_all(k, :) = cross(tangent2_world_all(k, :), normal_world);
        tangent1_world_all(k, :) = tangent1_world_all(k, :) ./ norm(tangent1_world_all(k, :));
    end
end

function R = local_quat_xyzw_to_rotmat(q)
    x = q(1); y = q(2); z = q(3); w = q(4);
    R = [
        1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w);
        2.0 * (x*y + z*w),       1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w);
        2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x*x + y*y)];
end

function normals = local_enforce_sign_continuity(normals)
    for k = 2:size(normals, 1)
        prev = normals(k - 1, :);
        curr = normals(k, :);
        if all(isfinite(prev)) && all(isfinite(curr)) && dot(prev, curr) < 0
            normals(k, :) = -curr;
        end
    end
end

function row = local_normalize_row(row)
    row_norm = norm(row);
    if row_norm > 1.0e-9 && all(isfinite(row))
        row = row ./ row_norm;
    else
        row = [nan, nan, nan];
    end
end

function normalized = local_normalize_rows(vectors)
    normalized = nan(size(vectors));
    row_norms = sqrt(sum(vectors.^2, 2));
    valid = all(isfinite(vectors), 2) & row_norms > 1.0e-9;
    normalized(valid, :) = vectors(valid, :) ./ row_norms(valid);
end

function rpy = local_basis_to_rpy(x_axis_world, y_axis_world, z_axis_world)
    n = size(x_axis_world, 1);
    rpy = nan(n, 3);
    for k = 1:n
        x_axis = x_axis_world(k, :);
        y_axis = y_axis_world(k, :);
        z_axis = z_axis_world(k, :);
        if any(~isfinite([x_axis, y_axis, z_axis]))
            continue;
        end
        R = [x_axis(:), y_axis(:), z_axis(:)];
        rpy(k, :) = local_rotmat_to_rpy(R);
    end
    rpy(:, 3) = unwrap(rpy(:, 3));
end

function rpy = local_force_to_normal_frame_rpy(force_world, ref_normal_world, ref_tangent1_world)
    n = size(force_world, 1);
    x_axis_world = nan(n, 3);
    y_axis_world = nan(n, 3);
    z_axis_world = nan(n, 3);

    for k = 1:n
        force = force_world(k, :);
        ref_normal = ref_normal_world(k, :);
        ref_tangent1 = ref_tangent1_world(k, :);
        if any(~isfinite([force, ref_normal, ref_tangent1])) || norm(force) < 1.0e-9
            continue;
        end

        x_axis = force ./ norm(force);
        if dot(x_axis, ref_normal) < 0.0
            x_axis = -x_axis;
        end

        y_axis = ref_tangent1 - dot(ref_tangent1, x_axis) * x_axis;
        if norm(y_axis) < 1.0e-9
            y_axis = local_pick_perpendicular_axis(x_axis);
        else
            y_axis = y_axis ./ norm(y_axis);
        end

        z_axis = cross(x_axis, y_axis);
        if norm(z_axis) < 1.0e-9
            continue;
        end
        z_axis = z_axis ./ norm(z_axis);
        y_axis = cross(z_axis, x_axis);
        y_axis = y_axis ./ norm(y_axis);

        x_axis_world(k, :) = x_axis;
        y_axis_world(k, :) = y_axis;
        z_axis_world(k, :) = z_axis;
    end

    rpy = local_basis_to_rpy(x_axis_world, y_axis_world, z_axis_world);
end

function axis = local_pick_perpendicular_axis(x_axis)
    if abs(x_axis(1)) < 0.9
        seed = [1.0, 0.0, 0.0];
    else
        seed = [0.0, 1.0, 0.0];
    end
    axis = seed - dot(seed, x_axis) * x_axis;
    axis = axis ./ norm(axis);
end

function rpy = local_rotmat_to_rpy(R)
    pitch = asin(max(-1.0, min(1.0, -R(3, 1))));
    roll = atan2(R(3, 2), R(3, 3));
    yaw = atan2(R(2, 1), R(1, 1));
    rpy = [roll, pitch, yaw];
end

function f_hat_world = local_compute_offline_pure_mob(time, pose_rpy, motor_thrust, ...
    state_vel, pos_vel, mass_kg, gravity_ms2, Kp, Kf, use_pos_velocity, dt_mode, fixed_dt)

    n = numel(time);
    f_hat_world = nan(n, 3);
    p_hat_world = zeros(1, 3);
    f_hat_prev_world = zeros(1, 3);
    dt_last_valid = fixed_dt;

    for k = 1:n
        if strcmpi(char(dt_mode), 'fixed')
            dt = fixed_dt;
        elseif k == 1
            dt = dt_last_valid;
        else
            dt = time(k) - time(k - 1);
            if isfinite(dt) && dt > 0
                dt_last_valid = dt;
            else
                dt = dt_last_valid;
            end
        end

        v_world = state_vel(k, :);
        if use_pos_velocity && all(isfinite(pos_vel(k, :)))
            v_world = pos_vel(k, :);
        end

        if ~(isfinite(dt) && dt > 0) || ...
                any(~isfinite(pose_rpy(k, :))) || ...
                any(~isfinite(motor_thrust(k, :))) || ...
                any(~isfinite(v_world))
            if k > 1
                f_hat_world(k, :) = f_hat_world(k - 1, :);
            end
            continue;
        end

        body_force = [0.0, 0.0, sum(motor_thrust(k, :))];
        R = local_rpy_to_rotmat(pose_rpy(k, :));
        world_force = (R * body_force(:)).';

        p_world = mass_kg * v_world;
        p_err_world = p_world - p_hat_world;
        gravity_world = [0.0, 0.0, -mass_kg * gravity_ms2];

        p_hat_dot_world = gravity_world + world_force + f_hat_prev_world + Kp * p_err_world;
        f_hat_dot_world = Kf * p_err_world;

        p_hat_world = p_hat_world + dt * p_hat_dot_world;
        f_hat_prev_world = f_hat_prev_world + dt * f_hat_dot_world;

        p_hat_world = local_sanitize_row(p_hat_world);
        f_hat_prev_world = local_sanitize_row(f_hat_prev_world);
        f_hat_world(k, :) = f_hat_prev_world;
    end
end

function [force_2nd, torque_2nd, force_kep, force_eta, eta_hat] = ...
    local_compute_offline_point_contact_eta_mob(time, pose_rpy, motor_thrust, ...
    state_vel, pos_vel, gyro_body_deg_s, mass_kg, gravity_ms2, Kp, Kf, ...
    KpTau, Ktau, mob_alpha, Ke, epsilon_tau, eta_gamma, eta_rho, eta_initial, ...
    inertia_diag, com_offset_body, contact_offset_body, arm_xy, k_tau_motor, motor_dir, ...
    use_pos_velocity, dt_mode, fixed_dt)

    n = numel(time);
    force_2nd = nan(n, 3);
    torque_2nd = nan(n, 3);
    force_kep = nan(n, 3);
    force_eta = nan(n, 3);
    eta_hat = nan(n, 1);

    obs2.p_lin_hat = zeros(1, 3);
    obs2.p_ang_hat = zeros(1, 3);
    obs2.force_hat = zeros(1, 3);
    obs2.torque_hat = zeros(1, 3);
    obs2.force_out = zeros(1, 3);
    obs2.torque_out = zeros(1, 3);

    con.p_lin_hat_base = zeros(1, 3);
    con.p_lin_hat_con = zeros(1, 3);
    con.p_ang_hat = zeros(1, 3);
    con.force_hat_base = zeros(1, 3);
    con.force_hat_con = zeros(1, 3);
    con.torque_hat = zeros(1, 3);
    con.force_out = zeros(1, 3);
    con.torque_out = zeros(1, 3);

    matched_force.signal = zeros(1, 3);
    matched_force.signal_dot = zeros(1, 3);
    matched_force.output = zeros(1, 3);
    matched_torque.signal = zeros(1, 3);
    matched_torque.signal_dot = zeros(1, 3);
    matched_torque.output = zeros(1, 3);

    eta = max(1.0e-6, eta_initial);
    dt_last_valid = fixed_dt;

    for k = 1:n
        if strcmpi(char(dt_mode), 'fixed')
            dt = fixed_dt;
        elseif k == 1
            dt = dt_last_valid;
        else
            dt = time(k) - time(k - 1);
            if isfinite(dt) && dt > 0
                dt_last_valid = dt;
            else
                dt = dt_last_valid;
            end
        end

        v_world = state_vel(k, :);
        if use_pos_velocity && all(isfinite(pos_vel(k, :)))
            v_world = pos_vel(k, :);
        end

        if ~(isfinite(dt) && dt > 0) || ...
                any(~isfinite(pose_rpy(k, :))) || ...
                any(~isfinite(motor_thrust(k, :))) || ...
                any(~isfinite(v_world)) || ...
                any(~isfinite(gyro_body_deg_s(k, :)))
            if k > 1
                force_2nd(k, :) = force_2nd(k - 1, :);
                torque_2nd(k, :) = torque_2nd(k - 1, :);
                force_kep(k, :) = force_kep(k - 1, :);
                force_eta(k, :) = force_eta(k - 1, :);
                eta_hat(k) = eta_hat(k - 1);
            end
            continue;
        end

        R = local_rpy_to_rotmat(pose_rpy(k, :));
        thrust_row = motor_thrust(k, :);
        f1 = thrust_row(1); f2 = thrust_row(2); f3 = thrust_row(3); f4 = thrust_row(4);

        body_force = [0.0, 0.0, f1 + f2 + f3 + f4];
        world_force = (R * body_force(:)).';
        body_torque = [
            arm_xy * ((f3 + f4) - (f1 + f2)), ...
            arm_xy * ((f2 + f3) - (f1 + f4)), ...
            k_tau_motor * (motor_dir(1) * f1 + motor_dir(2) * f2 + ...
                           motor_dir(3) * f3 + motor_dir(4) * f4)];
        body_torque = body_torque + cross(com_offset_body, body_force);

        omega_body = deg2rad(gyro_body_deg_s(k, :));
        p_lin_world = mass_kg * v_world;
        p_ang_body = inertia_diag .* omega_body;
        cori_body = cross(omega_body, p_ang_body);
        grav_world = [0.0, 0.0, mass_kg * gravity_ms2];
        contact_offset_world = (R * contact_offset_body(:)).';

        [matched_force, matched_force_input_world] = local_observer_matched_signal( ...
            matched_force, world_force, dt, Kf, Kp, mob_alpha);
        [matched_torque, matched_torque_input_body] = local_observer_matched_signal( ...
            matched_torque, body_torque, dt, Ktau, KpTau, mob_alpha);
        matched_torque_input_world = (R * matched_torque_input_body(:)).';

        obs2 = local_run_observer_variant(obs2, p_lin_world, p_ang_body, ...
            world_force, body_torque, grav_world, cori_body, dt, Kp, KpTau, Kf, Ktau, mob_alpha);
        con = local_run_consistency_observer(con, p_lin_world, p_ang_body, ...
            world_force, body_torque, grav_world, cori_body, contact_offset_world, R, ...
            dt, Kp, KpTau, Kf, Ktau, mob_alpha, Ke, epsilon_tau);

        drone_force_2nd = obs2.force_out;
        drone_torque_world_2nd = (R * obs2.torque_out(:)).';
        [eta, drone_force_eta] = local_run_eta_t_correction( ...
            eta, drone_force_2nd, drone_torque_world_2nd, ...
            matched_force_input_world, matched_torque_input_world, contact_offset_world, ...
            dt, eta_gamma, eta_rho);

        force_2nd(k, :) = drone_force_2nd;
        torque_2nd(k, :) = drone_torque_world_2nd;
        force_kep(k, :) = con.force_out;
        force_eta(k, :) = drone_force_eta;
        eta_hat(k) = eta;
    end
end

function state = local_run_observer_variant(state, p_lin_world, p_ang_body, ...
    u_lin_world, u_tau_body, grav_world, cori_body, dt, Kp, KpTau, Kf, Ktau, mob_alpha)

    p_lin_residual = p_lin_world - state.p_lin_hat;
    p_ang_residual = p_ang_body - state.p_ang_hat;

    state.force_hat = state.force_hat + dt * Kf * p_lin_residual;
    state.torque_hat = state.torque_hat + dt * Ktau * p_ang_residual;
    state.p_lin_hat = state.p_lin_hat + dt * ...
        (u_lin_world - grav_world + state.force_hat + Kp * p_lin_residual);
    state.p_ang_hat = state.p_ang_hat + dt * ...
        (u_tau_body - cori_body + state.torque_hat + KpTau * p_ang_residual);

    state.force_hat = local_sanitize_row(state.force_hat);
    state.torque_hat = local_sanitize_row(state.torque_hat);
    state.p_lin_hat = local_sanitize_row(state.p_lin_hat);
    state.p_ang_hat = local_sanitize_row(state.p_ang_hat);
    state.force_out = local_lpf1_row(state.force_out, state.force_hat, mob_alpha);
    state.torque_out = local_lpf1_row(state.torque_out, state.torque_hat, mob_alpha);
end

function state = local_run_consistency_observer(state, p_lin_world, p_ang_body, ...
    u_lin_world, u_tau_body, grav_world, cori_body, contact_offset_world, R, ...
    dt, Kp, KpTau, Kf, Ktau, mob_alpha, Ke, epsilon_tau)

    p_ang_residual = p_ang_body - state.p_ang_hat;
    state.torque_hat = state.torque_hat + dt * Ktau * p_ang_residual;
    tau_hat_world = (R * state.torque_hat(:)).';

    p_lin_residual_base = p_lin_world - state.p_lin_hat_base;
    state.force_hat_base = state.force_hat_base + dt * Kf * p_lin_residual_base;

    p_lin_residual_con = p_lin_world - state.p_lin_hat_con;
    r_cross_f = cross(contact_offset_world, state.force_hat_con);
    e_tau_world = tau_hat_world - r_cross_f;
    consistency_residual = cross(e_tau_world, contact_offset_world);
    rho_tau = norm(e_tau_world) / ...
        (norm(contact_offset_world) * norm(state.force_hat_con) + epsilon_tau);
    if ~isfinite(rho_tau)
        rho_tau = 0.0;
    end

    state.force_hat_con = state.force_hat_con + dt * ...
        (Kf * p_lin_residual_con + Ke * consistency_residual);
    state.p_ang_hat = state.p_ang_hat + dt * ...
        (u_tau_body - cori_body + state.torque_hat + KpTau * p_ang_residual);
    state.p_lin_hat_base = state.p_lin_hat_base + dt * ...
        (u_lin_world - grav_world + state.force_hat_base + Kp * p_lin_residual_base);
    state.p_lin_hat_con = state.p_lin_hat_con + dt * ...
        (u_lin_world - grav_world + state.force_hat_con + Kp * p_lin_residual_con);

    state.force_hat_base = local_sanitize_row(state.force_hat_base);
    state.force_hat_con = local_sanitize_row(state.force_hat_con);
    state.torque_hat = local_sanitize_row(state.torque_hat);
    state.p_lin_hat_base = local_sanitize_row(state.p_lin_hat_base);
    state.p_lin_hat_con = local_sanitize_row(state.p_lin_hat_con);
    state.p_ang_hat = local_sanitize_row(state.p_ang_hat);
    state.force_out = local_lpf1_row(state.force_out, state.force_hat_con, mob_alpha);
    state.torque_out = local_lpf1_row(state.torque_out, state.torque_hat, mob_alpha);
end

function [state, output] = local_observer_matched_signal(state, input_signal, dt, stiffness_gain, damping_gain, mob_alpha)
    signal_ddot = max(0.0, stiffness_gain) * (input_signal - state.signal) - ...
        max(0.0, damping_gain) * state.signal_dot;
    state.signal_dot = state.signal_dot + dt * signal_ddot;
    state.signal = state.signal + dt * state.signal_dot;
    state.output = local_lpf1_row(state.output, state.signal, mob_alpha);
    output = state.output;
end

function [eta, corrected_force_world] = local_run_eta_t_correction(eta, ...
    force_world_raw, torque_world_raw, matched_force_input_world, matched_torque_input_world, ...
    contact_offset_world, dt, gamma, rho_eta)

    y_eta = cross(contact_offset_world, matched_force_input_world) - matched_torque_input_world;
    eps_tau = cross(contact_offset_world, force_world_raw) - torque_world_raw;
    eps_eta = eps_tau - y_eta * (eta - 1.0);
    denominator = local_sanitize_positive(rho_eta, 0.01) + dot(y_eta, y_eta);
    eta = eta + dt * max(0.0, gamma) * dot(y_eta, eps_eta) / denominator;
    eta = max(1.0e-6, eta);

    corrected_force_world = force_world_raw - (eta - 1.0) * matched_force_input_world;
    corrected_force_world = local_sanitize_row(corrected_force_world);
end

function y = local_lpf1_row(y_prev, x, alpha)
    y = y_prev + alpha * (x - y_prev);
    y = local_sanitize_row(y);
end

function value = local_sanitize_positive(value, fallback)
    if ~(isfinite(value) && value > 1.0e-9)
        value = fallback;
    end
end

function row = local_sanitize_row(row)
    row(~isfinite(row)) = 0.0;
end

function value = local_read_yaml_scalar(path, key, fallback)
    value = fallback;
    if ~isfile(path)
        return;
    end

    txt = fileread(path);
    pattern = ['(?m)^\s*', regexptranslate('escape', char(key)), ...
        '\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)'];
    token = regexp(txt, pattern, 'tokens', 'once');
    if ~isempty(token)
        parsed = str2double(token{1});
        if isfinite(parsed)
            value = parsed;
        end
    end
end

function local_print_availability(label, x)
    ratio = nnz(isfinite(x)) / numel(x);
    fprintf("[INFO] %-24s finite %.1f %%\n", label, 100.0 * ratio);
end

function local_apply_limits(ax, x_limits, y_limits)
    if local_is_valid_limits(x_limits)
        xlim(ax, x_limits);
    end
    if local_is_valid_limits(y_limits)
        ylim(ax, y_limits);
    end
end

function local_apply_3d_limits(ax, x_limits, y_limits, z_limits)
    if local_is_valid_limits(x_limits)
        xlim(ax, x_limits);
    end
    if local_is_valid_limits(y_limits)
        ylim(ax, y_limits);
    end
    if local_is_valid_limits(z_limits)
        zlim(ax, z_limits);
    end
end

function ok = local_is_valid_limits(limits)
    ok = isnumeric(limits) && isvector(limits) && numel(limits) == 2 && ...
        all(isfinite(limits)) && limits(2) > limits(1);
end

function ok = local_is_valid_scalar(value)
    ok = isnumeric(value) && isscalar(value) && isfinite(value);
end

function ok = local_is_valid_vector3(value)
    ok = isnumeric(value) && isvector(value) && numel(value) == 3 && all(isfinite(value));
end
