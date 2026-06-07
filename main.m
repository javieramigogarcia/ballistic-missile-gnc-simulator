%% GNC Launch Vehicle Simulation Framework
% Main Execution and Telemetry Analysis Script
%
% Description:
%   This script automates the execution of the Simulink GNC model,
%   extracts vehicle states, and generates professional engineering plots
%   for the trajectory tracking and thrust vector control (TVC) actuator effort.

clear; clc; close all;

fprintf('==================================================\n');
fprintf('   GNC LAUNCH VEHICLE SIMULATION FRAMEWORK v1.0   \n');
fprintf('==================================================\n');

%% 1. Run Simulink Model

model_name = 'simulink_guidance_system'; 

fprintf('[@] Initializing physics engine and loading model: %s...\n', model_name);
if ~bdIsLoaded(model_name)
    load_system(model_name);
end

fprintf('[@] Running orbital ascent simulation (0 to 231 seconds)...\n');
out = sim(model_name);
fprintf('[+] Simulation completed successfully.\n\n');

%% 2. Extract Data
t_sim = out.tout;
Y_sim = out.Y_out;
delta_c_rad = out.delta_c_sim;

% Handle matrix dimensions dynamically (robust against transpositions)
if size(Y_sim, 2) == 4
    gamma_real = Y_sim(:, 2) .* (180/pi);
else
    gamma_real = Y_sim(2, :)' .* (180/pi);
end

if size(delta_c_rad, 2) > 1
    delta_c_deg = delta_c_rad' .* (180/pi);
else
    delta_c_deg = delta_c_rad .* (180/pi);
end

%% 3. Reconstruct Ideal Guidance Command (Sine Pitch Program)
gamma_ref_ideal = zeros(length(t_sim), 1);
for i = 1:length(t_sim)
    t = t_sim(i);
    if t <= 10
        gamma_ref_ideal(i) = 90;
    elseif t > 231
        gamma_ref_ideal(i) = 0;
    else
        t_norm = (t - 10) / (231 - 10);
        gamma_ref_ideal(i) = 90 * (1 - sin((pi/2) * t_norm));
    end
end

%% 4. Generate Plot 1: Trajectory Tracking
fprintf('[@] Generating Pitch Angle Tracking visualization...\n');
figure('Name', 'GNC Trajectory Tracking', 'Color', [1 1 1]);
plot(t_sim, gamma_ref_ideal, 'r--', 'LineWidth', 2); hold on;
plot(t_sim, gamma_real, 'b', 'LineWidth', 1.5);
xlabel('Time [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Pitch Angle \gamma [deg]', 'FontSize', 11, 'FontWeight', 'bold');
title('GNC: Optimal Trajectory Tracking (Sine Pitch Program)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Guidance Command (Reference)', 'Actual Missile Flight', 'Location', 'northeast');
grid on;
set(gca, 'GridAlpha', 0.15);

%% 5. Generate Plot 2: TVC Actuator Effort
fprintf('[@] Generating Actuator Telemetry visualization...\n');
figure('Name', 'Actuator Telemetry', 'Color', [1 1 1]);
plot(t_sim, delta_c_deg, 'g', 'LineWidth', 1.5); hold on;
yline(10, 'r--', 'Upper Limit (+10 deg)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'right');
yline(-10, 'r--', 'Lower Limit (-10 deg)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'right');
xlabel('Time [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Nozzle Deflection \delta_c [deg]', 'FontSize', 11, 'FontWeight', 'bold');
title('Actuator Telemetry: Thrust Vector Control Effort', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-15 15]);
grid on;
set(gca, 'GridAlpha', 0.15);

fprintf('\n====================================================\n');
fprintf('   ALL PLOTS RENDERED - SYSTEM ANALYSIS READY      \n');
fprintf('====================================================\n');

%% 6. Generate Plot 3: Trajectory Profile (Altitude vs. Downrange)
fprintf('[@] Generating Trajectory Profile (Altitude vs Range)...\\n');

Rt = 6378140; % Earth mean radius [m]

% Extract data respecting your state vector [V, gamma, z, psi]
if size(Y_sim, 2) >= 4
    H_km = Y_sim(:, 3) / 1000;              % Index 3 is Altitude (z)
    psi_rad = Y_sim(:, 4);                  % Index 4 is the central angle (psi)
else
    H_km = Y_sim(3, :)' / 1000;
    psi_rad = Y_sim(4, :)';
end

% Convert spherical angle to actual horizontal distance (Arc Length) in km
X_km = (psi_rad .* Rt) / 1000;

figure('Name', 'Flight Profile', 'Color', [1 1 1]);
plot(X_km, H_km, 'b', 'LineWidth', 2); hold on;

% Karman Line at 100 km (Edge of Space)
yline(100, 'k--', 'Karman Line (Edge of Space)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');

xlabel('Downrange Distance X [km]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Altitude H [km]', 'FontSize', 11, 'FontWeight', 'bold');
title('Flight Profile: Altitude vs. Downrange', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.15);