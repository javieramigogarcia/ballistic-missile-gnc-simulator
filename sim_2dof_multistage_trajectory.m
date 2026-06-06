clear; clc; close all
addpath('D:\ING AEROESPACIAL\CUARTO\Misiles\Master_Misiles\functions');

%% 1. INITIAL CONDITIONS
z0 = 0;        % Initial altitude [m]
V0 = 0;        % Initial velocity [m/s]
gamma0 = pi/2; % Initial flight path angle (vertical climb) [rad]
psi0 = 0;      % Initial downrange angle [rad]

% Define the initial state vector Y0
Y0 = [V0; gamma0; z0; psi0];

% Call Stage 1 parameters
[m_t0, tb_1, mass_flow1, T1] = f_stage1();

% Call Gravity Turn parameters
[t_GT, theta_GT] = f_gravity_turn();

% Call Stage 2 parameters
[T2, mass_flow2, m0_phase3, tb_2] = f_stage2();

% Time span for phase 1 (Vertical Ascent)
tspan_1 = [0 t_GT];

% Time span for phase 2 (Gravity Turn to Burnout)
tspan_2 = [t_GT, tb_1];

% Time span for phase 3 (Stage 2 flight)
tspan_3 = [tb_1, tb_1 + tb_2];

%% 2. RUN INTEGRATION
% --- PHASE 1: Vertical Ascent ---
[t_phase1, Y_phase1] = ode45(@f_derivatives, tspan_1, Y0);

% --- THE PITCH KICK ---
% Extract the exact state at t_GT (last row) and transpose to column vector
Y0_phase2 = Y_phase1(end, :)'; 

% Subtract the kick angle to initialize the turn
Y0_phase2(2) = Y0_phase2(2) - theta_GT;



% --- PHASE 2: Gravity Turn ---
% Set integration options including the event function
options = odeset('Events', @(t,Y) f_event_burnout(t, Y, tb_1));  

% Call ode45 starting from the modified state
[t_phase2, Y_phase2] = ode45(@f_derivatives, tspan_2, Y0_phase2, options);



% --- PHASE 3: Stage 2 Flight ---
% Extract the exact state at tb_1 (last row) and transpose to column vector
Y0_phase3 = Y_phase2(end, :)';

% Set integration options including the event function
options = odeset('Events', @(t,Y) f_event_burnout(t, Y, tb_1 + tb_2));  

% Call ode45 starting from the modified state
[t_phase3, Y_phase3] = ode45(@f_derivatives, tspan_3, Y0_phase3, options);

% --- ASSEMBLE TRAJECTORY ---
t_stage2 = [t_phase1; t_phase2; t_phase3];
Y_stage2 = [Y_phase1; Y_phase2; Y_phase3];

z_out = Y_stage2(:, 3);
gamma_out = Y_stage2(:, 2) .* (180/pi); % Convert to degrees for plotting
psi_out = Y_stage2(:, 4);
Rt = 6378140; 
x_out = psi_out .* Rt; % Calculate downrange distance in meters


%% 3. PLOT RESULTS
figure;
plot(x_out/1000, z_out/1000);
xlabel('Downrange Distance [km]');
ylabel('Altitude [km]');
title('2-Stage Ballistic Missile Trajectory');
grid on;
axis equal;

figure;
plot(t_stage2, gamma_out);
xlabel('Time [s]');
ylabel('Flight Path Angle [deg]');
title('Flight Path Angle vs Time');
grid on;