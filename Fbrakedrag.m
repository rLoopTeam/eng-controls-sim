% Input: velocity (m/s), brakegap (mm), hover height (mm)
% Output: Total brake drag force (N)
function F = Fbrakedrag(v,bg)
    F = (5632 * exp(-202*bg*10^-3) * (-exp(-0.3*v) + 1) * (1.5*exp(-0.02*v) + 1) );

end