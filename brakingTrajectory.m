% brakingTrajectory.m generates arrays for desired target velocity,
% 'velocitySet', a distance, 'distanceSet'
% for GainScheduledPIDBrakingSystem.mdl
%
% Notes: 
% Need to acquire experimental data from stepper motors for modeling
% actual response dynamics, such as damping, phase lag, and time delay.
% Trajectory calcs do not account for aero-drag, hover engine drag.
% References Fdrag.m

%% Physical parameters
mpod = 320;         % Mass of pod (kg)
%td = 1;             % Time delay due to stepper motor lag (needs to be experimentally calculated) (s)

%% INPUTS
% Initial conditions for controlled braking period
xdot0 = 120;        % Initial velocity at beginning of controlled braking period (m/s)
xdotf = 50;         % Final velocity at end of controlled braking period (m/s)
%deltaxset = 480;    % Distance to target point at beginning of controlled braking period (m)
%deltaxcoast = 580;  % Coasting distance (~580m) prior to controlled braking (m)
brakegapIC = 25;    % Brake Gap at beginning of controlled braking period (mm)
brakegapNom = 4;	% Nominal Brake Gap for controlled braking period (mm)
brakegapspeed = 6;  % Max speed at which brake gap can be operated (mm/s)

%%
% Generate array that corresponds distance traveled since beginning of
% controlled braking period, x, to velocity setpoint based on 1D
% kinematics, xdotref

%% Generate setpoint velocity array for brake engagement period
% Notes: this model is innaccurate as it assumes stepper motor reaches max speed instantaneously.
% Empirical data required for modeling actual stepper motor response dynamics.

% create array for velocity setpoint lookup table
dt = 0.05;             % time step (s)
distanceSet = [];      % Initialize array for distance traveled since beginning of controlled braking period
velocitySet = [];      % Initialize array for velocity setpoint LUT

x = 0;                  % Initialize position (m)
v = xdot0;              % Initialize velocity (m/s)
t = 0;                  % Initialize time (s)
brakegap = brakegapIC;  % Initialize brake gap (mm)
ct = 0;                 % Initialize while loop counter
while v > xdotf
    ct = ct + 1;
    if brakegap > brakegapNom
        brakegap = brakegap - brakegapspeed*dt;
    else
        brakegap = brakegapNom;
    end
        a = -Fdrag(brakegap,v)/mpod;    % acceleration calc computed from A34 halbach data (m/s^2)
        v = v + a*dt;                   % Velocity calc (m/s)
        x = x + v*dt + 0.5*a*dt^2;      % Position calc (m)
        distanceSet(ct) = x;            % Add distance data entry to array
        velocitySet(ct) = v;            % Add velocity data entry to array
end
plot(distanceSet,velocitySet)
title('Deceleration trajectory')
xlabel('Distance (m)')
ylabel('Velocity (m/s)')
