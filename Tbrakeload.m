% Input: velocity (m/s), brakegap (mm)
% Output: Load torque on brake stepper motors due to magnetic lift and drag forces (N*m)
function T = Tbrakeload(xdot,xddot,bg)
    % lift force due to eddy currents (N)
    F_eddylift = 3265.1 * exp(-209.4*bg*10^-3) * log(xdot + 1) - 2636.7*exp(-207*bg*10^-3) * (xdot + 0.6) * exp(-0.16*xdot);
    % lift force due to magnetic repulsion between brakes (N)
    F_maglift = 1.8643*10^6*(bg*10^-3)^2 - 55.644*10^3*(bg*10^-3) + 531.43;
    
    % Net brake lift (N)
    Fbrakelift_net = F_eddylift + F_maglift;
    
    % lead pitch on W1003WF-24P-C3Z4 ball screw (m)
    P_ballscrew = 0.004;
    % Mass of brake block (kg)
    m_block = 11.5;
    % Coeff of friction - max estimation
    mu = 0.2;
    
    % Inertial Force due to mass of the brake block (N)
    Finertial_brake = m_block * xddot;
    % brake load (N)
    Fload_brakes = Fbrakelift_net*sin(17*pi()/180) - (0.5*Fdrag.brake(xdot,bg) + Finertial_brake) * cos(17*pi()/180);
    % Normal force on ballscrew
    N = Fbrakelift_net * cos(17*pi()/180) + (0.5*Fdrag.brake(xdot,bg) + Finertial_brake) * sin(17*pi()/180);
    % Frictional force due to brake block (N)
    Ffric = mu * N;
    
    % Net load torque on brake stepper motor (N*m)
    T = 0.5 * P_ballscrew * (Fload_brakes + Ffric)/pi();
end
