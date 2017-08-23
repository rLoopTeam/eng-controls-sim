% Input: velocity (m/s), brakegap (mm)
% Output: Brake lift force for one brakepad (N)
function T = Tbrakeload(v,bg)
    % lead pitch on W1003WF-24P-C3Z4 ball screw (m)
    P_ballscrew = 0.004;
    
    % Net brake lift (N)
    Fload_brakes = Fbrakelift(v,bg)*sin(17*pi()/180) - 0.5*Fdrag.brake(v,bg)*cos(17*pi()/180);
    
    % Net load torque on brake stepper motor (N*m)
    T = 0.5*P_ballscrew*Fload_brakes/pi();
end
