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
