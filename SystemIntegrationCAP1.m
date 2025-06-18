% Comprehensive HPV System Simulation Script with Enhanced System Integration
% Author: Derek Martinez & Tim Yip 
% Date: 9/19/2024
% Description:
% This script performs integrated mechanical, electrical, and safety analyses
% for the Human Powered Vehicle (HPV) design. The analyses are interconnected
% to reflect system-level interactions, providing comprehensive simulation results,
% detailed calculations, and enhanced visualizations.

%% ----------------------- Initialization ----------------------- %%
clear; clc; close all;

%% ----------------------- Configuration Parameters ----------------------- %%
% Mechanical Parameters
theta_H = 60;        % Head angle relative to the horizontal (degrees)
theta_PH = 24.55;    % Pedal angle relative to the head axis (degrees)
theta_CH = 24.55;    % Crank angle relative to the head axis (degrees)

Q = 12.35;           % Tread (inches)
L_C = 10.8;          % Crank length (inches)
T = 2.26;            % Trail of the front wheel (inches)
r_F = 13;            % Front wheel radius (inches)

M_FX = 7.38;         % Steering moment due to pedal force in x-direction (ft·lb)
M_FY = 7.38;         % Steering moment due to pedal force in y-direction (ft·lb)
M_FZ = 7.38;         % Steering moment due to pedal force in z-direction (ft·lb)

wheelbase = 46.57;       % Wheelbase (inches)
tire_width = 2;          % Tire width (inches)
current_clearance = 2.9; % Current distance from frame (inches)
desired_clearance = 7.5; % Desired distance from frame (inches)

% Electrical Parameters
battery_capacity_Wh = 500;         % Battery Capacity in Watt-hours
initial_voltage_V = 48;            % Initial Battery Voltage (V)
final_voltage_V = 42;              % Final Battery Voltage after discharge (V)
peukert_exponent = 1.1;            % Peukert's exponent for the battery

motor_power_output_W = 500;        % Motor Power Output (W)
motor_efficiency = 0.85;           % Motor Efficiency (decimal)

human_power_W = 150;               % Human Power Contribution (W)
system_efficiency = 0.9;           % Overall System Efficiency (decimal)
drivetrain_efficiency = 0.95;      % Drivetrain Efficiency (decimal)

drag_coefficient = 0.5;            % Drag Coefficient (dimensionless)
frontal_area_m2 = 0.5;             % Frontal Area (m^2)
air_density_kg_m3 = 1.225;         % Air Density at sea level (kg/m^3)
rolling_resistance_coeff = 0.005;  % Rolling Resistance Coefficient (dimensionless)
gravity_m_s2 = 9.81;               % Gravitational Acceleration (m/s^2)

% Safety Parameters
vehicle_weight_lbs = 600;          % Vehicle weight in pounds (updated)
driver_weight_lbs = 175;           % Driver weight in pounds
stopping_distance = 6;             % Desired stopping distance (m)
mu = 0.7;                          % Coefficient of friction (dry conditions)
front_brake_ratio = 0.70;          % Front brake force ratio
rear_brake_ratio = 0.30;           % Rear brake force ratio

% Plotting Parameters
num_points = 1000;                 % Increased number of points for smoother plots

%% ----------------------- Unit Conversions ----------------------- %%
% Mechanical Conversions
theta_H_rad = deg2rad(theta_H);
theta_PH_rad = deg2rad(theta_PH);
theta_CH_rad = deg2rad(theta_CH);

Q_ft = Q / 12;
L_C_ft = L_C / 12;
T_ft = T / 12;
r_F_ft = r_F / 12;
wheelbase_ft = wheelbase / 12;
tire_width_ft = tire_width / 12;

% Safety Conversions
lbs_to_kg = 0.45359237;
vehicle_mass_kg = vehicle_weight_lbs * lbs_to_kg;
driver_mass_kg = driver_weight_lbs * lbs_to_kg;
total_mass = vehicle_mass_kg + driver_mass_kg;  % in kg

%% ----------------------- Mechanical Analysis ----------------------- %%
% Force Calculations
d_F_in = (r_F - T * tan(theta_H_rad)) * cos(theta_H_rad);
d_F_ft = d_F_in / 12;
L_C_sin_ft = (L_C * sin(theta_CH_rad)) / 12;

F_X = M_FX / ((Q_ft / 2) * sind(theta_PH));
F_Z = M_FZ / ((Q_ft / 2) * cosd(theta_PH));
F_Y = M_FY / (d_F_ft + L_C_sin_ft);

total_force_calculated = F_X + F_Y + F_Z;

% Turn Radius Calculations
turn_radius_30_in = (wheelbase_ft / sind(30) + (tire_width_ft / 2)) * 12;

theta_current_deg = rad2deg(atan(current_clearance / r_F));
turn_radius_current_in = (wheelbase_ft / sind(theta_current_deg) + (tire_width_ft / 2)) * 12;

theta_desired_deg = rad2deg(atan(desired_clearance / r_F));
turn_radius_desired_in = (wheelbase_ft / sind(theta_desired_deg) + (tire_width_ft / 2)) * 12;

% Stress Analysis (Simplified)
force = M_FY * 12; % Convert ft·lb to in·lb
beam_length = wheelbase; % in inches
moment_of_inertia = 100; % in^4 (example value)
c = 2; % distance from neutral axis to outer fiber (inches)
sigma = (force * c) / moment_of_inertia; % Bending stress in psi

%% ----------------------- Electrical Analysis ----------------------- %%
% Effective Vehicle Mass for Electrical Analysis
vehicle_weight_kg = vehicle_mass_kg + driver_mass_kg;  % Total weight in kg

% Battery Capacity and Runtime Estimation
average_power_W = system_efficiency * motor_power_output_W;
runtime_hours = battery_capacity_Wh / average_power_W;

% Motor Power Requirement and Efficiency
input_power_W = motor_power_output_W / motor_efficiency;

% Electrical Assistance Calculation
total_power_output_W = human_power_W + motor_power_output_W;
effective_power_W = system_efficiency * total_power_output_W;

% Speed and Range Estimation
power_required = @(v) 0.5 * drag_coefficient * frontal_area_m2 * air_density_kg_m3 .* v.^3 + rolling_resistance_coeff * vehicle_weight_kg * gravity_m_s2 .* v;

v_lower = 0.1; % small positive number
v_upper = 50; % reasonable upper limit for speed in m/s
speed_eq = @(v) effective_power_W - power_required(v);
[max_speed_m_s, fval, exitflag] = fzero(speed_eq, [v_lower, v_upper]);

if exitflag <= 0
    warning('fzero did not converge. The solution may not be accurate.');
end

max_speed_kmh = max_speed_m_s * 3.6;

% Effective Battery Capacity (Peukert-adjusted)
effective_capacity_Wh = battery_capacity_Wh / (average_power_W)^(peukert_exponent - 1);

%% ----------------------- Safety Analysis ----------------------- %%
% Deceleration Calculation
speed_ms = max_speed_m_s; % Use maximum speed from electrical analysis
speed_kmh = speed_ms * 3.6;
deceleration = speed_ms^2 / (2 * stopping_distance);

% Total Braking Force Required
braking_force_total = total_mass * deceleration;

% Maximum Friction Force Available
friction_force_max = mu * total_mass * gravity_m_s2;

% Brake Force Distribution
front_braking_force_total = front_brake_ratio * braking_force_total;
front_braking_force_per_wheel = front_braking_force_total / 2; % Assuming two front brakes
rear_braking_force_total = rear_brake_ratio * braking_force_total;
rear_braking_force_per_wheel = rear_braking_force_total / 2;   % Assuming two rear brakes

% Kinetic Energy Calculation
kinetic_energy_J = 0.5 * total_mass * speed_ms^2;
kinetic_energy_Wh = kinetic_energy_J / 3600;

% Regenerative Braking Potential
regen_efficiency = 0.60;
regen_energy_Wh = regen_efficiency * kinetic_energy_Wh;

% Update Effective Battery Capacity
updated_battery_capacity_Wh = effective_capacity_Wh + regen_energy_Wh;
if updated_battery_capacity_Wh > battery_capacity_Wh
    updated_battery_capacity_Wh = battery_capacity_Wh;
end

%% ----------------------- System Integration Analysis ----------------------- %%
% Integration of Mechanical and Electrical Systems
% Mechanical forces impact power consumption due to increased resistance
% Let's define a mechanical resistance factor based on total mechanical force

mechanical_force_reference = total_force_calculated; % Use initial total force as reference
mechanical_force_range = linspace(total_force_calculated * 0.8, total_force_calculated * 1.2, num_points);
mechanical_resistance_factors = 1 + (mechanical_force_range - mechanical_force_reference) / mechanical_force_reference;

% Ensure mechanical_resistance_factors are positive
mechanical_resistance_factors(mechanical_resistance_factors <= 0) = eps;

adjusted_speeds_m_s = zeros(size(mechanical_resistance_factors));

for i = 1:length(mechanical_resistance_factors)
    mr_factor = mechanical_resistance_factors(i);
    adj_power_req = @(v) mr_factor * power_required(v);
    adj_speed_eq = @(v) effective_power_W - adj_power_req(v);
    v_lower = 0.1;
    v_upper = 50;
    [adj_speed_m_s, ~, exitflag] = fzero(adj_speed_eq, [v_lower, v_upper]);
    adjusted_speeds_m_s(i) = adj_speed_m_s;
end

% Recalculate adjusted parameters based on adjusted speeds
adjusted_max_speed_m_s = adjusted_speeds_m_s(end);
adjusted_max_speed_kmh = adjusted_max_speed_m_s * 3.6;
adjusted_decelerations = adjusted_speeds_m_s.^2 / (2 * stopping_distance);
adjusted_braking_forces_N = total_mass .* adjusted_decelerations;

% Adjusted Kinetic Energy and Regenerative Energy
adjusted_kinetic_energies_J = 0.5 * total_mass .* adjusted_speeds_m_s.^2;
adjusted_kinetic_energies_Wh = adjusted_kinetic_energies_J / 3600;
adjusted_regen_energies_Wh = regen_efficiency * adjusted_kinetic_energies_Wh;

% Update Effective Battery Capacity with adjusted regenerative energy
adjusted_effective_battery_capacities_Wh = effective_capacity_Wh + adjusted_regen_energies_Wh;
adjusted_effective_battery_capacities_Wh(adjusted_effective_battery_capacities_Wh > battery_capacity_Wh) = battery_capacity_Wh;

%% ----------------------- Display Results ----------------------- %%
% Mechanical Results
fprintf('======= Mechanical Analysis Results =======\n');
fprintf('Calculated F_X: %.2f lb\n', F_X);
fprintf('Calculated F_Y: %.2f lb\n', F_Y);
fprintf('Calculated F_Z: %.2f lb\n', F_Z);
fprintf('Total Force: %.2f lb\n', total_force_calculated);
fprintf('Turn Radius at 30 degrees: %.2f inches\n', turn_radius_30_in);
fprintf('Turn Radius with Current Clearance (%.2f degrees): %.2f inches\n', theta_current_deg, turn_radius_current_in);
fprintf('Turn Radius with Desired Clearance (%.2f degrees): %.2f inches\n', theta_desired_deg, turn_radius_desired_in);
fprintf('Calculated Bending Stress: %.2f psi\n\n', sigma);

% Electrical Results
fprintf('======= Electrical Analysis Results =======\n');
fprintf('Battery Capacity: %.2f Wh\n', battery_capacity_Wh);
fprintf('Effective Battery Capacity (Peukert-adjusted): %.2f Wh\n', effective_capacity_Wh);
fprintf('Average Power Consumption: %.2f W\n', average_power_W);
fprintf('Estimated Runtime: %.2f hours\n', runtime_hours);
fprintf('Motor Power Output: %.2f W\n', motor_power_output_W);
fprintf('Motor Efficiency: %.2f%%\n', motor_efficiency * 100);
fprintf('Input Power Required: %.2f W\n', input_power_W);
fprintf('Human Power Contribution: %.2f W\n', human_power_W);
fprintf('Total Power Output: %.2f W\n', total_power_output_W);
fprintf('Effective Power after System Efficiency: %.2f W\n', effective_power_W);
fprintf('Estimated Maximum Speed: %.2f m/s (%.2f km/h)\n\n', max_speed_m_s, max_speed_kmh);

% Safety Results
fprintf('======= Safety Analysis Results =======\n');
fprintf('Total mass (kg): %.2f kg\n', total_mass);
fprintf('Original Speed: %.2f km/h (%.2f m/s)\n', speed_kmh, speed_ms);
fprintf('Desired stopping distance: %.2f meters\n', stopping_distance);
fprintf('Deceleration required: %.3f m/s^2\n', deceleration);
fprintf('Total braking force required: %.2f N\n', braking_force_total);
fprintf('Maximum friction force available: %.2f N\n\n', friction_force_max);

% Adjusted Results
fprintf('======= Adjusted Results Due to System Integration =======\n');
fprintf('Mechanical Resistance Factor Range: %.2f to %.2f\n', min(mechanical_resistance_factors), max(mechanical_resistance_factors));
fprintf('Adjusted Maximum Speed Range: %.2f to %.2f m/s (%.2f to %.2f km/h)\n', min(adjusted_speeds_m_s), max(adjusted_speeds_m_s), min(adjusted_speeds_m_s)*3.6, max(adjusted_speeds_m_s)*3.6);
fprintf('Adjusted Deceleration Range: %.3f to %.3f m/s^2\n', min(adjusted_decelerations), max(adjusted_decelerations));
fprintf('Adjusted Total Braking Force Range: %.2f to %.2f N\n', min(adjusted_braking_forces_N), max(adjusted_braking_forces_N));
fprintf('Adjusted Kinetic Energy Range: %.2f to %.2f J (%.2f to %.2f Wh)\n', min(adjusted_kinetic_energies_J), max(adjusted_kinetic_energies_J), min(adjusted_kinetic_energies_Wh), max(adjusted_kinetic_energies_Wh));
fprintf('Adjusted Regenerative Energy Recovered Range: %.2f to %.2f Wh\n', min(adjusted_regen_energies_Wh), max(adjusted_regen_energies_Wh));
fprintf('Adjusted Effective Battery Capacity Range: %.2f to %.2f Wh\n', min(adjusted_effective_battery_capacities_Wh), max(adjusted_effective_battery_capacities_Wh));
fprintf('Note: Battery capacity cannot exceed nominal capacity of %.2f Wh.\n\n', battery_capacity_Wh);

% Visualizations
%% ----------------------- Enhanced Figures and Detailed Calculations ----------------------- %%

% 1. Power Required vs. Speed with Varying Mechanical Resistance (Enhanced)
speed_range_m_s = linspace(0.1, max(max_speed_m_s, adjusted_max_speed_m_s) * 1.2, num_points);
original_power_required_W = power_required(speed_range_m_s);

% Generate power required curves for multiple mechanical resistance factors
figure;
hold on;
num_curves = 20; % Increased number of curves for smoother representation
indices = round(linspace(1, length(mechanical_resistance_factors), num_curves));
colors = parula(num_curves); % Use 'parula' colormap for better visual differentiation

for idx = 1:num_curves
    mr_factor = mechanical_resistance_factors(indices(idx));
    adjusted_power_W = mr_factor * power_required(speed_range_m_s);
    plot(speed_range_m_s * 3.6, adjusted_power_W, 'Color', colors(idx, :), 'LineWidth', 1.5);
end

% Plot the original power required curve
h_original = plot(speed_range_m_s * 3.6, original_power_required_W, 'k-', 'LineWidth', 2);
set(h_original, 'DisplayName', 'Original Power Required');

% Add effective power line
h_eff_power = yline(effective_power_W, 'r--', 'LineWidth', 2);
% Manually set the legend entry for yline
set(get(get(h_eff_power, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_eff_power, 'DisplayName', 'Effective Power Output');

xlabel('Speed (km/h)', 'FontSize', 12);
ylabel('Power Required (W)', 'FontSize', 12);
title('Power Required vs. Speed with Varying Mechanical Resistance', 'FontSize', 14);

% Adjust colorbar to display mechanical resistance factors
colormap(parula);
cb = colorbar;
cb.Ticks = linspace(0, 1, 5);
cb.TickLabels = arrayfun(@(x) sprintf('%.2f', x), linspace(min(mechanical_resistance_factors), max(mechanical_resistance_factors), 5), 'UniformOutput', false);
cb.Label.String = 'Mechanical Resistance Factor (MRF)';
cb.Label.FontSize = 12;

% Add legend
legend([h_original, h_eff_power], 'Location', 'northwest');
grid on;
set(gca, 'FontSize', 12);
hold off;


% 2. Mechanical Resistance Factor Impact on Speed (Enhanced)
figure;
h_adj_speed = plot(mechanical_resistance_factors, adjusted_speeds_m_s * 3.6, 'g-', 'LineWidth', 2, 'DisplayName', 'Adjusted Speed');
xlabel('Mechanical Resistance Factor', 'FontSize', 12);
ylabel('Adjusted Maximum Speed (km/h)', 'FontSize', 12);
title('Impact of Mechanical Resistance on Maximum Speed', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);
hold on;

% Add annotations for key points
[max_speed, max_idx] = max(adjusted_speeds_m_s * 3.6);
[min_speed, min_idx] = min(adjusted_speeds_m_s * 3.6);

% Plot markers at max and min speeds
h_max_speed = plot(mechanical_resistance_factors(max_idx), max_speed, 'bo', 'MarkerSize', 8, 'DisplayName', 'Max Speed');
h_min_speed = plot(mechanical_resistance_factors(min_idx), min_speed, 'ro', 'MarkerSize', 8, 'DisplayName', 'Min Speed');

text(mechanical_resistance_factors(max_idx), max_speed, sprintf(' Max Speed: %.2f km/h', max_speed), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 12);
text(mechanical_resistance_factors(min_idx), min_speed, sprintf('Min Speed: %.2f km/h', min_speed), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 12);

% Define acceptable speed threshold
acceptable_speed = 25; % in km/h
h_yline = yline(acceptable_speed, 'm--', 'LineWidth', 2, 'DisplayName', 'Acceptable Speed Threshold');

% Shade regions
below_acceptable = adjusted_speeds_m_s * 3.6 < acceptable_speed;
h_area = area(mechanical_resistance_factors(below_acceptable), adjusted_speeds_m_s(below_acceptable) * 3.6, ...
    'FaceColor', [1.0 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
% Include area in legend
set(get(get(h_area, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_area, 'DisplayName', 'Below Acceptable Speed');

legend([h_adj_speed, h_max_speed, h_min_speed, h_yline, h_area], 'Location', 'northeast');
hold off;

% 3. Regenerative Energy vs. Mechanical Resistance Factor (Enhanced with Annotations)
figure;
h_area = area(mechanical_resistance_factors, adjusted_regen_energies_Wh, ...
    'FaceColor', [0.7 0.9 0.7], 'EdgeColor', 'g', 'FaceAlpha', 0.5);
% Include area in legend
set(get(get(h_area, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_area, 'DisplayName', 'Regenerative Energy');

xlabel('Mechanical Resistance Factor', 'FontSize', 12);
ylabel('Regenerative Energy Recovered (Wh)', 'FontSize', 12);
title('Regenerative Energy vs. Mechanical Resistance Factor', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);
hold on;

% Find maximum and minimum regenerative energy
[max_regen_energy, idx_max] = max(adjusted_regen_energies_Wh);
[min_regen_energy, idx_min] = min(adjusted_regen_energies_Wh);

% Plot markers at these points
h_max = plot(mechanical_resistance_factors(idx_max), max_regen_energy, 'ro', 'MarkerSize', 8, 'DisplayName', 'Max Regen Energy');
h_min = plot(mechanical_resistance_factors(idx_min), min_regen_energy, 'bo', 'MarkerSize', 8, 'DisplayName', 'Min Regen Energy');

% Annotate these points
text(mechanical_resistance_factors(idx_max), max_regen_energy, sprintf(' Max: %.2f Wh', max_regen_energy), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 12);
text(mechanical_resistance_factors(idx_min), min_regen_energy, sprintf('Min: %.2f Wh', min_regen_energy), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 12);

legend([h_area, h_max, h_min], 'Location', 'northwest');
hold off;

% 4. Adjusted Battery Capacity vs. Mechanical Resistance Factor (Enhanced Comparison Plot)
figure;
h_adj_battery = plot(mechanical_resistance_factors, adjusted_effective_battery_capacities_Wh, 'c-', 'LineWidth', 2, 'DisplayName', 'Adjusted Battery Capacity');
hold on;
h_nominal_battery = yline(battery_capacity_Wh, 'r--', 'LineWidth', 2, 'DisplayName', 'Nominal Battery Capacity');

% Highlight where adjusted capacity reaches nominal capacity
full_capacity_indices = adjusted_effective_battery_capacities_Wh >= battery_capacity_Wh * 0.99;
h_fill = area(mechanical_resistance_factors(full_capacity_indices), adjusted_effective_battery_capacities_Wh(full_capacity_indices), ...
    'FaceColor', [0.9 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
% Include area in legend
set(get(get(h_fill, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_fill, 'DisplayName', 'Capacity Limit Reached');

% Annotations
[max_adjusted_capacity, idx_max] = max(adjusted_effective_battery_capacities_Wh);
plot(mechanical_resistance_factors(idx_max), max_adjusted_capacity, 'ko', 'MarkerSize', 8, 'DisplayName', 'Max Adjusted Capacity');
text(mechanical_resistance_factors(idx_max), max_adjusted_capacity, sprintf(' Max Adjusted Capacity: %.2f Wh', max_adjusted_capacity), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 12);

xlabel('Mechanical Resistance Factor', 'FontSize', 12);
ylabel('Battery Capacity (Wh)', 'FontSize', 12);
title('Adjusted vs. Nominal Battery Capacity', 'FontSize', 14);
legend([h_adj_battery, h_nominal_battery, h_fill], 'Location', 'best');
grid on;
set(gca, 'FontSize', 12);
hold off;


% 5. Adjusted Braking Force Required vs. Mechanical Resistance Factor (Enhanced with Detailed Annotations)
figure;
h_braking_force = plot(mechanical_resistance_factors, adjusted_braking_forces_N, 'k-', 'LineWidth', 2, 'DisplayName', 'Braking Force Required');
hold on;
h_friction_force = yline(friction_force_max, 'r--', 'LineWidth', 2, 'DisplayName', 'Max Friction Force');

% Shade unsafe region where braking force exceeds friction force
unsafe_indices = adjusted_braking_forces_N > friction_force_max;
x_fill = [mechanical_resistance_factors(unsafe_indices)'; flipud(mechanical_resistance_factors(unsafe_indices)')];
y_fill = [adjusted_braking_forces_N(unsafe_indices)'; friction_force_max * ones(sum(unsafe_indices), 1)];

h_unsafe_fill = fill(x_fill, y_fill, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% Include fill in legend
set(get(get(h_unsafe_fill, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_unsafe_fill, 'DisplayName', 'Unsafe Braking Zone');

% Annotations
exceed_indices = find(unsafe_indices, 1);
if ~isempty(exceed_indices)
    exceed_mrf = mechanical_resistance_factors(exceed_indices);
    exceed_force = adjusted_braking_forces_N(exceed_indices);
    h_exceed = plot(exceed_mrf, exceed_force, 'ro', 'MarkerSize', 8, 'DisplayName', 'Exceeds Friction Force');
    text(exceed_mrf, exceed_force, sprintf(' Exceeds at MRF: %.2f', exceed_mrf), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 12);
else
    h_exceed = []; % If not exceeding, set as empty
end

xlabel('Mechanical Resistance Factor', 'FontSize', 12);
ylabel('Braking Force (N)', 'FontSize', 12);
title('Braking Force vs. Mechanical Resistance Factor', 'FontSize', 14);
legend_handles = [h_braking_force, h_friction_force, h_unsafe_fill];
if ~isempty(h_exceed)
    legend_handles = [legend_handles, h_exceed];
end
legend(legend_handles, 'Location', 'northwest');
grid on;
set(gca, 'FontSize', 12);
hold off;

% 6. Bending Stress vs. Mechanical Resistance Factor (Enhanced with Material Limits)
adjusted_sigmas = sigma * mechanical_resistance_factors;

figure;
h_bending_stress = plot(mechanical_resistance_factors, adjusted_sigmas, 'b-', 'LineWidth', 2, 'DisplayName', 'Bending Stress');
hold on;
allowable_stress_psi = 25000;  % Example allowable stress
yield_stress_psi = 30000;      % Example yield stress
ultimate_stress_psi = 45000;   % Example ultimate tensile stress

h_allowable = yline(allowable_stress_psi, 'g--', 'LineWidth', 2, 'DisplayName', 'Allowable Stress');
h_yield = yline(yield_stress_psi, 'y--', 'LineWidth', 2, 'DisplayName', 'Yield Stress');
h_ultimate = yline(ultimate_stress_psi, 'r--', 'LineWidth', 2, 'DisplayName', 'Ultimate Stress');

% Highlight regions
unsafe_indices = adjusted_sigmas > allowable_stress_psi;
x_fill = [mechanical_resistance_factors(unsafe_indices)'; flipud(mechanical_resistance_factors(unsafe_indices)')];
y_fill = [adjusted_sigmas(unsafe_indices)'; allowable_stress_psi * ones(sum(unsafe_indices), 1)];

h_unsafe_fill = fill(x_fill, y_fill, 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% Include fill in legend
set(get(get(h_unsafe_fill, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_unsafe_fill, 'DisplayName', 'Exceeds Allowable Stress');

xlabel('Mechanical Resistance Factor', 'FontSize', 12);
ylabel('Bending Stress (psi)', 'FontSize', 12);
title('Bending Stress vs. Mechanical Resistance Factor', 'FontSize', 14);
legend([h_bending_stress, h_allowable, h_yield, h_ultimate, h_unsafe_fill], 'Location', 'northwest');
grid on;
set(gca, 'FontSize', 12);
hold off;

% 7. Adjusted Runtime vs. Mechanical Resistance Factor (Enhanced)
adjusted_average_powers_W = average_power_W * mechanical_resistance_factors;
adjusted_runtimes_hours = battery_capacity_Wh ./ adjusted_average_powers_W;

figure;
h_runtime = plot(mechanical_resistance_factors, adjusted_runtimes_hours, 'r-', 'LineWidth', 2, 'DisplayName', 'Adjusted Runtime');
hold on;
xlabel('Mechanical Resistance Factor', 'FontSize', 12);
ylabel('Adjusted Runtime (hours)', 'FontSize', 12);
title('Adjusted Runtime vs. Mechanical Resistance Factor', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

% Define acceptable runtime threshold
acceptable_runtime = 1.0; % in hours
h_runtime_line = yline(acceptable_runtime, 'g--', 'LineWidth', 2, 'DisplayName', 'Acceptable Runtime');

% Shade regions where runtime is below acceptable threshold
below_threshold = adjusted_runtimes_hours < acceptable_runtime;

x_fill = [mechanical_resistance_factors(below_threshold)'; flipud(mechanical_resistance_factors(below_threshold)')];
y_fill = [adjusted_runtimes_hours(below_threshold)'; acceptable_runtime * ones(sum(below_threshold), 1)];

h_fill = fill(x_fill, y_fill, 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
% Include fill in legend
set(get(get(h_fill, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_fill, 'DisplayName', 'Below Acceptable Runtime');

% Annotate critical points
[min_runtime, min_idx] = min(adjusted_runtimes_hours);
[max_runtime, max_idx] = max(adjusted_runtimes_hours);
h_min = plot(mechanical_resistance_factors(min_idx), min_runtime, 'bo', 'MarkerSize', 8, 'DisplayName', 'Minimum Runtime');
h_max = plot(mechanical_resistance_factors(max_idx), max_runtime, 'go', 'MarkerSize', 8, 'DisplayName', 'Maximum Runtime');

text(mechanical_resistance_factors(min_idx), min_runtime, sprintf('Min: %.2f h', min_runtime), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 12);
text(mechanical_resistance_factors(max_idx), max_runtime, sprintf('Max: %.2f h', max_runtime), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 12);

% Add legend with specific handles and labels
legend([h_runtime, h_runtime_line, h_fill, h_min, h_max], 'Location', 'best');
hold off;

% 8. Efficiency vs. Speed (Enhanced)
speeds_m_s = linspace(0.1, max_speed_m_s * 1.2, num_points);
powers_W = power_required(speeds_m_s);
efficiencies = (effective_power_W ./ powers_W) * 100; % As percentage

figure;
h_efficiency = plot(speeds_m_s * 3.6, efficiencies, 'm-', 'LineWidth', 2, 'DisplayName', 'Efficiency Curve');
xlabel('Speed (km/h)', 'FontSize', 12);
ylabel('Efficiency (%)', 'FontSize', 12);
title('System Efficiency vs. Speed', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);
hold on;

% Find and mark maximum efficiency point
[max_efficiency, idx_max_eff] = max(efficiencies);
max_eff_speed_kmh = speeds_m_s(idx_max_eff) * 3.6;
h_max_eff = plot(max_eff_speed_kmh, max_efficiency, 'ro', 'MarkerSize', 8, 'DisplayName', 'Max Efficiency');

text(max_eff_speed_kmh, max_efficiency, sprintf('Max Efficiency: %.2f%% at %.2f km/h', max_efficiency, max_eff_speed_kmh), ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'FontSize', 12);

% Define efficiency zones
high_efficiency = efficiencies > 80;
medium_efficiency = efficiencies <= 80 & efficiencies > 60;
low_efficiency = efficiencies <= 60;

% Shade efficiency zones
h_high_eff = area(speeds_m_s(high_efficiency) * 3.6, efficiencies(high_efficiency), 'FaceColor', [0.7 1.0 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
set(get(get(h_high_eff, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_high_eff, 'DisplayName', 'High Efficiency');

h_med_eff = area(speeds_m_s(medium_efficiency) * 3.6, efficiencies(medium_efficiency), 'FaceColor', [1.0 1.0 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
set(get(get(h_med_eff, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_med_eff, 'DisplayName', 'Medium Efficiency');

h_low_eff = area(speeds_m_s(low_efficiency) * 3.6, efficiencies(low_efficiency), 'FaceColor', [1.0 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
set(get(get(h_low_eff, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
set(h_low_eff, 'DisplayName', 'Low Efficiency');

% Add legend
legend([h_efficiency, h_max_eff, h_high_eff, h_med_eff, h_low_eff], 'Location', 'southwest');
hold off;

% 9. Total Energy Consumption over Distance (Enhanced)
distance_m = linspace(0, 10000, num_points); % Distance up to 10 km

% Define average speeds (low, average, high)
average_speeds_m_s = [adjusted_speeds_m_s(round(num_points*0.2)), mean(adjusted_speeds_m_s), adjusted_speeds_m_s(round(num_points*0.8))];
average_speeds_kmh = average_speeds_m_s * 3.6;
labels = {'Low Speed', 'Average Speed', 'High Speed'};
colors = {'b', 'g', 'r'};

figure;
hold on;
plot_handles = gobjects(length(average_speeds_m_s), 1); % Preallocate array for plot handles
total_energy_Wh_all = cell(length(average_speeds_m_s), 1);

for i = 1:length(average_speeds_m_s)
    time_s = distance_m / average_speeds_m_s(i);
    % Calculate power required at each speed
    power_W = power_required(average_speeds_m_s(i));
    total_energy_Wh = (power_W / 3600) * time_s;
    total_energy_Wh_all{i} = total_energy_Wh; % Store for annotation
    % Plot and store handle
    plot_handles(i) = plot(distance_m / 1000, total_energy_Wh, colors{i}, 'LineWidth', 2, 'DisplayName', sprintf('%s (%.1f km/h)', labels{i}, average_speeds_kmh(i)));
end

% Add battery capacity line
h_battery = yline(battery_capacity_Wh, 'm--', 'LineWidth', 2, 'DisplayName', 'Battery Capacity');

xlabel('Distance Traveled (km)', 'FontSize', 12);
ylabel('Total Energy Consumed (Wh)', 'FontSize', 12);
title('Total Energy Consumption over Distance', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

% Annotate where energy consumption meets battery capacity
for i = 1:length(average_speeds_m_s)
    total_energy_Wh = total_energy_Wh_all{i};
    idx = find(total_energy_Wh >= battery_capacity_Wh, 1);
    if ~isempty(idx)
        distance_km = distance_m(idx) / 1000;
        plot(distance_km, battery_capacity_Wh, 'ko', 'MarkerSize', 8);
        text(distance_km, battery_capacity_Wh, sprintf('Range at %.1f km/h: %.2f km', average_speeds_kmh(i), distance_km), ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10);
    end
end

% Add legend with plot handles
legend([plot_handles; h_battery], 'Location', 'northwest');
hold off;


%% ----------------------- Conclusion ----------------------- %%
% The enhanced integrated analysis provides a more detailed understanding of the interdependence of mechanical,
% electrical, and safety aspects of the HPV design. The advanced visualizations help identify critical points,
% such as regions where the bending stress exceeds allowable limits or where braking force requirements surpass
% the maximum friction force available. This allows for better design optimization and safety considerations.

fprintf('--- End of Enhanced Integrated System Simulation with Advanced Processing and Analysis ---\n');
