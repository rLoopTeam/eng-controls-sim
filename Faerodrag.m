% Input: velocity (m/s), brakegap (mm), hover height (mm)
% Output: Drag force (N)
function F = Faerodrag(v,rho)
    %%%% Aero Drag parameters %%%%
    Cd = 1.1849;        % Drag coeff
    Ap = 1.14;          % Projectet drag area

    %%%% Compute aerodrag %%%%
    F =  rho * Cd * Ap * v^2 / 2;
end