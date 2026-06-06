function [m_t0, tb_1, mass_flow1, T1] = f_stage1()
    % Parameters for Stage 1
    m_t0 = 110500;  % Total initial mass Stage 1 [kg]
    m_p1 = 65000;   % Propellant mass Stage 1 [kg]
    tb_1 = 121;     % Burn time Stage 1 [s]
    Isp_1 = 280;    % Specific impulse Stage 1 [s]
    g0 = 9.80665;   % Standard gravity [m/s^2]

    % Outputs
    mass_flow1 = m_p1 / tb_1;         % Mass flow rate [kg/s]
    T1 = mass_flow1 * Isp_1 * g0;     % Constant Thrust Stage 1 [N]
end