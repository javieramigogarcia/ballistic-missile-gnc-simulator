% --- practica_modulo2.m ---
clear; clc; close all
addpath('D:\ING AEROESPACIAL\CUARTO\Misiles\Master_Misiles\funciones');

% The mission for this exercise is to simulate the Stage 1 Gravity Turn.

%% 1. INITIAL CONDITIONS
z0 = 0;        % Initial altitude [m]
V0 = 0;        % Initial velocity [m/s]
gamma0 = pi/2; % Initial flight path angle (vertical climb) [rad]
psi0 = 0;      % Initial downrange angle [rad]

% Define the initial state vector Y0
Y0 = [V0; gamma0; z0; psi0];

% Call Stage 1 parameters
[m_t0, m_p1, tb_1, Isp_1] = f_stage1();

% Call Gravity Turn parameters
[t_GT, theta_GT] = f_gravity_turn();

% Time span for phase 1 (Vertical Ascent)
tspan_1 = [0 t_GT];

% Time span for phase 2 (Gravity Turn to Burnout)
tspan_2 = [t_GT, tb_1];

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

% --- ASSEMBLE TRAJECTORY ---
t_stage1 = [t_phase1; t_phase2];
Y_stage1 = [Y_phase1; Y_phase2];

z_out = Y_stage1(:, 3);
gamma_out = Y_stage1(:, 2) .* (180/pi); % Convert to degrees for plotting
psi_out = Y_stage1(:, 4);
Rt = 6378140; 
x_out = psi_out .* Rt; % Calculate downrange distance in meters

%% 3. PLOT RESULTS
figure;
plot(x_out/1000, z_out/1000);
xlabel('Downrange Distance [km]');
ylabel('Altitude [km]');
title('Stage 1 Gravity Turn Trajectory');
grid on;
axis equal;

figure;
plot(t_stage1, gamma_out);
xlabel('Time [s]');
ylabel('Flight Path Angle [deg]');
title('Flight Path Angle vs Time');
grid on;

%% LOCAL FUNCTIONS
function dYdt = f_derivatives(t, Y)
    % Obtain current State Vector values
    V = Y(1);
    gamma = Y(2);
    z = Y(3);
    psi = Y(4);

    Rt = 6378140;  % Earth mean radius [m]

    % Call parameters
    [m_t0, m_p1, tb_1, Isp_1] = f_stage1();
    [t_GT, ~] = f_gravity_turn();

    % Define the mass flow rate and instantaneous mass
    m_dot1 = m_p1 / tb_1; 
    m = m_t0 - m_dot1 * t;

    % Calculate the Thrust
    T = Isp_1 * 9.80665 * m_dot1; 

    % Calls external function of gravity
    g = f_gravity(z); 
    
    % Calls internal function of aerodynamic drag
    D = f_aerodynamic_drag(Y);
    
    if t <= t_GT
        % PHASE 1: Strictly vertical (Avoids g/V division by zero)
        dV_dt = (T - D)/m - g * sin(gamma);  
        dgamma_dt = 0;             
        dz_dt = V * sin(gamma);   
        dpsi_dt = 0;               
    else
        % PHASE 2: Gravity Turn Kinematics
        dV_dt = (T - D)/m - g * sin(gamma);  
        
        % Flight path angle derivative (Gravity vs Earth Curvature)
        dgamma_dt = -(g / V) * cos(gamma) + (V / (Rt + z)) * cos(gamma);  
        
        dz_dt = V * sin(gamma);   
        
        % Downrange angle derivative
        dpsi_dt = (V * cos(gamma)) / (Rt + z);               
    end

    % Reassemblies derivates in a column vector
    dYdt = [dV_dt; dgamma_dt; dz_dt; dpsi_dt];
end

function D = f_aerodynamic_drag(Y)
    V = Y(1); 
    z = Y(3); 
    S = 2;         
    Cd = 0.5;      
    [rho, ~, ~] = f_atmosphere_isa(z);
    D = 0.5 * rho * Cd * S * (V^2); 
end

function [value, isterminal, direction] = f_event_burnout(t, Y, tb)
    value = t - tb;      
    isterminal = 1;      
    direction = 0;       
end

function [m_t0, m_p1, tb_1, Isp_1] = f_stage1()
    m_t0 = 110500; 
    m_p1 = 65000;  
    tb_1 = 121;    
    Isp_1 = 280;   
end

function [t_GT, theta_GT] = f_gravity_turn()
    t_GT = 10;                  % Time to gravity turn [s]
    theta_GT = 0.5 * (pi / 180);  % Pitch angle [rad]
end