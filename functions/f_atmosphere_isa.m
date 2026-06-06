function [rho, T, a] = f_atmosphere_isa(z)
    % F_ATMOSPHERE_ISA Calculates ISA properties up to 30 km.
    % Inputs:
    %   z - Altitude [m]
    % Outputs:
    %   rho - Density [kg/m^3]
    %   T   - Temperature [K]
    %   a   - Speed of sound [m/s]
    
    if z <= 11000
        % Troposphere
        T = 288.5 .* (1 - z ./ 44338);
        rho = 1.225 .* (1 - z ./ 44338).^4.25;
    else
        % Stratosphere
        T = 216.7;
        rho = 0.365 .* exp(-(z - 11000) ./ 6350);
    end
    
    % Speed of sound
    a = sqrt(1.4 .* 287 .* T);
end