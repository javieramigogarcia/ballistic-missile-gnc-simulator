% --- practica_modulo1.m ---
clear; clc; close all

addpath('D:\ING AEROESPACIAL\CUARTO\Misiles\Master_Misiles\funciones'); 

% 1. INITIAL CONDITIONS
z0 = 10000;    % Initial altitude [m]
V0 = 0;        % Initial velocity [m/s]
gamma0 = -pi/2;% Initial flight path angle (vertical drop) [rad]
psi0 = 0;      % Initial downrange angle [rad]

% Define the initial state vector Y0
Y0 = [V0; gamma0; z0; psi0];

% Time span for the simulation
tspan = [0 60];

% 2. RUN INTEGRATION
% Calls ode45 giving the derivates, the time span and initial state
[t, Y] = ode45(@f_derivatives, tspan, Y0);

% Y vector values at the end of the simulation
V_out = Y(:, 1);
z_out = Y(:, 3);

% 3. PLOT RESULTS
figure;
plot(t, z_out);
xlabel('Time [s]');
ylabel('Altitude [m]');
title('Free Fall Simulation');
grid on;


% LOCAL FUNCTIONS

function dYdt = f_derivatives(t, Y)
    % Obtain current state vector variables
    V = Y(1);
    gamma = Y(2);
    z = Y(3);
    psi = Y(4);

    % Calls external function of gravity
    g = f_gravity(z); 

    % Differential Equation (simplified for free fall)
    dV_dt = -g * sin(gamma);  
    dgamma_dt = 0;             
    dz_dt = V * sin(gamma);   
    dpsi_dt = 0;               

    % Reassemblies derivates in a column vector
    dYdt = [dV_dt; dgamma_dt; dz_dt; dpsi_dt];
end