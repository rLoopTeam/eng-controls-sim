% Input: velocity (m/s), brakegap (mm)
% Output: Brake lift force for one brakepad due to magnetic repulsion between brakes and eddy currents (N)
function F = Fbrakelift(v,bg)
    % lift force due to eddy currents (N)
    F_eddylift = 3265.1 * exp(-209.4*bg*10^-3) * log(v + 1) - 2636.7*exp(-207*bg*10^-3) * (v + 0.6) * exp(-0.16*v);
    
    % lift force due to magnetic repulsion between brakes (N)
    F_maglift = 1.8643*10^6*(bg*10^-3)^2 - 55.644*10^3*(bg*10^-3) + 531.43;
    
    % Net brake lift (N)
    F = F_eddylift + F_maglift;
end
