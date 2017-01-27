%%%% Input %%%%
% brake gap IC at beginning of request
% brake gap target
% deltat - time elapsed since last brake gap request
% Fstepperload - stepper motor load

%%%% Output %%%%
% brake gap

function omega0 = omega0Calc(brakegap0, brakegapSet)
    % brakegapIC = 25;
    % brakegapSet = 2.5;
    deltab = brakegapSet - brakegap0;
    
    omega0 = real(-4.097062071*log(abs(deltab)) + 10.7849);             % Natural frequency approximation to produce a settling time of 1.5s for zeta = 1
%     zeta = 1;
%     
%     % Stepper motor response dynamics for rotional kinematics
%     % % Stepper motor response dynamics transfer function for initial conditions
%     G = tf([omega0^2],[1 2*zeta*omega0 omega0^2]);
    % % Step response for transfer function
    % [y,t] = step(G);
%     stepinfo(G)
% 
%     [y] = lsim(sys_SM, u, t);
    
end
