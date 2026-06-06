% --- practica_modulo2.m ---
clear; clc; close all
addpath('D:\ING AEROESPACIAL\CUARTO\Misiles\Master_Misiles\funciones');

% The mission for this exercise is to simulate vertical ascension of Stage 1.

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

% Time span for phase 1
tspan_1 = [0 t_GT];

% Time span for phase 1
tspan_2 = [t_GT, tb_1];

%% 2. RUN INTEGRATION
% Call ode45, stopping at the start of the gravity turn
[t_phase1, Y_phase1] = ode45(@f_derivatives, tspan_1, Y0);

% Extract vector values of the simulation at each instant
Y_phase1(end, :);

% Creation of Y0_phase2
Y0_phase2 = Y_phase1; % Initialize Y0 for phase 2
Y0_phase2(2) = Y_phase1(2) - theta_GT;

% Set integration options including the event function
% Note: passing tb_1 using an anonymous function @(t, Y)
options = odeset('Events', @(t,Y) f_event_burnout(t, Y, tb_1));  

% Call ode45, stopping at the start of the gravity turn
[t_phase2, Y_phase2] = ode45(@f_derivatives, tspan_2, Y0_phase2, options);

% Extract vector values of the simulation at each instant
V_out1 = Y_phase1(:, 1);
z_out1 = Y_phase1(:, 3);

V_out2 = Y_phase2(:, 1);
z_out2 = Y_phase2(:, 3);

% Combine outputs for plotting
t_stage1 = [t_phase1; t_phase2];
z_out = [z_out1; z_out2];
V_out = [V_out1; V_out2];
%% 3. PLOT RESULTS
figure;
plot(t_stage1, z_out);
xlabel('Time [s]');
ylabel('Altitude [m]');
title('Stage 1 Ascension');
grid on;

%% LOCAL FUNCTIONS
function dYdt = f_derivatives(t, Y)
    % Obtain current State Vector values
    V = Y(1);
    gamma = Y(2);
    z = Y(3);
    psi = Y(4);

    Rt = 6371000;  % Earth radius [m]

    % Call Stage 1 parameters
    [m_t0, m_p1, tb_1, Isp_1] = f_stage1();

    % Call Gravity Turn parameters
    [t_GT, theta_GT] = f_gravity_turn();

    % Define the mass flow rate
    m_dot1 = m_p1 / tb_1; % [kg/s]

    % Define the mass for every instant
    m = m_t0 - m_dot1 * t;

    % Calculate the Thrust
    T = Isp_1 * 9.80665 * m_dot1; % [N]

    % Calls external function of gravity
    g = f_gravity(z); 
    
    % Calls internal function of aerodynamic drag
    D = f_aerodynamic_drag(Y);
    
    if t <= t_GT
        % Differential Equation 
        % Note: Standard format for V derivative isolates D/m and g*sin(gamma)
        dV_dt = (T - D)/m - g * sin(gamma);  
        dgamma_dt = 0;             
        dz_dt = V * sin(gamma);   
        dpsi_dt = 0;               
    
        % Reassemblies derivates in a column vector
        dYdt = [dV_dt; dgamma_dt; dz_dt; dpsi_dt];
    else
        % Differential Equation 
        % Note: Standard format for V derivative isolates D/m and g*sin(gamma)
        dV_dt = (T - D)/m - g * sin(gamma);  
        dgamma_dt = sin(gamma)*((g / V) - (V / (Rt + z)));             
        dz_dt = V * sin(gamma);   
        dpsi_dt = 0;               
    
        % Reassemblies derivates in a column vector
        dYdt = [dV_dt; dgamma_dt; dz_dt; dpsi_dt];


    end
end

function D = f_aerodynamic_drag(Y)
    % Obtain current State Vector values
    V = Y(1); % It is needed velocity to calculate aerodynamic drag force
    z = Y(3); % It is needed altitude to calculate air density

    % Drag Parameters
    S = 2;         % Area [m^2]
    Cd = 0.5;      % Drag coefficient

    % Call atmosphere function to get density
    [rho, ~, ~] = f_atmosphere_isa(z);

    % Calculate aerodynamic drag force
    D = 0.5 * rho * Cd * S * (V^2); 
end

function [value, isterminal, direction] = f_event_burnout(t, Y, tb)
    % Evaluates if integration time has reached burn time
    value = t - tb; % Stop condition     
    isterminal = 1; % Stop activated     
    direction = 0;  % Direction does not care to activate the stop     
end

function [m_t0, m_p1, tb_1, Isp_1] = f_stage1()
    % Define the Stage 1 parameters
    m_t0 = 110500; % Mass of the vehicle at the initial instant [kg]
    m_p1 = 65000;  % Propellant mass at the initial instant [kg]
    tb_1 = 121;    % Burn time for Stage 1 [s]
    Isp_1 = 280;   % Specific impulse for Stage 1 [s]
end

function [t_GT, tetha_GT] = f_gravity_turn()
    t_GT = 10;                  % Time to gravity turn [s]
    theta_GT = 0.5 * pi / 180;  % Pitch angle [rad]
end