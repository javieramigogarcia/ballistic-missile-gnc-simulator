function D = f_aerodynamic_drag(Y)
    V = Y(1); 
    z = Y(3); 
    S = 2;         
    Cd = 0.5;      
    [rho, ~, ~] = f_atmosphere_isa(z);
    D = 0.5 * rho * Cd * S * (V^2); 
end