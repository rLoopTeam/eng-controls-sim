% Lift force for 1 hover-engine

% Input: velocity (m/s), brakegap (mm), hover height (mm)
% Output: Hover lift force (N)
function F = Fhoverlift(v,z,RPM)
    %%%% Do you even 'lift', bro? %%%%
    F = 1142 * exp(-99.144*10^(-3)*z) * atan(0.089501*(v + 0.00932005*RPM));
    
end