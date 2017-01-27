% ts = 0.427;
% omega0 = -log(0.01)/(zeta*ts);      % Natural frequency, given settling time for 2nd order system
% num = [omega0^2];
% den = [1 2*zeta*omega0 omega0^2];
% G = tf(num,den);
% stepinfo(G)
% omega0
% 
% ts = 0.655;
% omega0 = -log(0.01)/(zeta*ts);      % Natural frequency, given settling time for 2nd order system
% num = [omega0^2];
% den = [1 2*zeta*omega0 omega0^2];
% G = tf(num,den);
% stepinfo(G)
% omega0

%%%% Input %%%%
% brake gap IC at beginning of request
% brake gap target
% deltat - time elapsed since last brake gap request
% Fstepperload - stepper motor load

%%%% Output %%%%
% brake gap

function ts = tsCalc(brakegap0, brakegapSet)
    % brakegapIC = 25;
    % brakegapSet = 2.5;
    deltab = brakegapSet - brakegap0;
    
    ts = real(0.3164934337*log(abs(deltab)) + 0.54);             % Natural frequency approximation to produce a settling time of 1.5s for zeta = 1
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
