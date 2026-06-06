function [g] = f_gravity(z)

    % Earth's physical constants
    mu = 3.9857e14;   % Earth's standard gravitational parameter [m^3/s^2]
    Rt = 6378140;     % Earth's mean radius [m]
    
    % Spherical gravity equation
    g = mu ./ (Rt + z).^2;
end