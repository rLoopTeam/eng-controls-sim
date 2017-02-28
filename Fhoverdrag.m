% Input: velocity (m/s) hover height (mm)
% Output: Total drag force (N)
function F = Fhoverdrag(v,z)
    
    %%%% Hover Drag parameters %%%%
    N_he = 8;           % No. of hover engines
    
    F = 0.5 * N_he * (z*10^(-3)*(-14166.667)+235) * (-exp(-0.16*v) + 1) * (1.6*exp(-0.02*v) + 1);
%     F = 0.25*N_he*Fhoverlift(v,z,0);     % Total hover drag estimated as 1/4th hover lift at 0 rpm
    
    if F < 0
        F = 0;
    end
    
%    F = 150 * (-exp(-0.16*v) + 1) * (1.6*exp(-0.02*v) + 1);
%    F = 0;

end