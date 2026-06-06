function [T2, mass_flow2, m0_phase3, tb_2] = f_stage2()
    % Parameters for Stage 2
    M02 = 35000;    % Total mass Stage 2 [kg]
    Mp2 = 32000;    % Propellant mass Stage 2 [kg]
    tb_2 = 110;     % Burn time Stage 2 [s]
    Isp_2 = 310;    % Specific impulse Stage 2 [s]
    payload = 4500; % Payload mass [kg]
    g0 = 9.80665;   % Standard gravity [m/s^2]

    % Outputs
    m0_phase3 = M02 + payload;       % Initial mass after staging (39,500 kg)
    mass_flow2 = Mp2 / tb_2;         % Mass flow rate [kg/s]
    T2 = mass_flow2 * Isp_2 * g0;    % Constant Thrust Stage 2 [N]
end