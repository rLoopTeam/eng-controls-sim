%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% filename: brakingTrajectory.m 
% author: dr. briefs
% date: 2017/8/5
%
% purpose:
% - generates graphs for estimating trajectory profile
% - generates csv files for predefining desired setpoint velocity vs. track
% position, 'velocitySet' & 'distanceSet' see PIDTuner.m
%
% Dependencies:
% simParameters.m
% Fdrag.m
% brakeactuator.m
%
% Note: Fload_brakes neglects magnetic load due to force of brakes acting on eachother.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Clear workspace and Generate Simulation Constraints
clear
simParameters

formatSpec = 'Generating case no. %0.f...\n\n';
str = sprintf(formatSpec, caseno);
fprintf(str,'\n')

%% Generate multiple trajectory scenarios: two for +/- max relative error, and one for nominal case.
% Fdrag_actual = Fdrag_approx/(1 - eta)
      
%% Simulation initial conditions
n = 1;
t = [0];                    % Initialize time array (s)
x = [0];                    % Initialize distance array (m)
xdot = [0];                 % Initialize velocity array (m/s)
xddot = [0];                % Initialize acceleration array (m/s^2)
brakegap = [25];            % Initialize brake Gap (mm)

t0 = t(n);                      % Store time Stamp
x0 = x(n);                      % pod distance at beginning of push phase (m/s)
xdot0 = xdot(n);                % pod velocity at beginning of push phase (m/s)

%     Fdrag_aero = [Fdrag.aero(xdot(1),rho) / (1 - eta)];              % Initialize array
%     Fdrag_hover = [Fdrag.hover(xdot(1),z_nom*10^3) / (1 - eta)];     % Initialize array
%     Fdrag_brake = [Fdrag.brake(xdot(1),brakegap(1)) / (1 - eta)];    % Initialize array
%     Fthrust = [gForce_pusher*g];
%     Flimprop = [0];

%% Phase 1: Generate Trajectory profile for Pusher Phase
while t(n) < deltat_pusher      % pusher phase constrained by time
% while xdot(n) < vpod_max %       % pusher phase constrained by velocity
% while x(n) < deltax_pusher      % pusher phase constrained by distance
    n = n + 1;

    % Compute Forces
    Fdrag_aero(n) = Fdrag.aero(xdot(n-1),rho) / (1 - eta_aerodrag);
    Fdrag_hover(n) = Fdrag.hover(xdot(n-1),z_nom) / (1 - eta_hoverdrag);
    Fdrag_brake(n) = Fdrag.brake(xdot(n-1),brakegap(n-1)) / (1 - eta_brakedrag);
    Fdrag_ski(n) = Fdrag.ski(xdot(n-1),z_nom) / (1 - eta_skidrag);

    Fdrag_net = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) + Fdrag_ski(n);

    Fthrust(n) = mpod*gForce_pusher*g;

    % Compute kinematics
    xddot(n) = (Fthrust(n) - Fdrag_net)/mpod;
    xdot(n) = xdot(n-1) + xddot(n)*dt;
    x(n) = x(n-1) + xdot(n-1)*dt + 0.5*xddot(n)*dt^2;
    t(n) = t(n-1) + dt;

    brakegap(n) = brakegap(n-1);

    % Compute load along brake actuator lead screw
    Fload_brakes(n) = Fbrakelift(xdot(n),brakegap(n))*sin(17*pi()/180) - Fdrag_brake(n)*cos(17*pi()/180)/2;

%     % If max pusher distance reached, exit pusher phase
%     if x(n) >= deltax_pusher_max
%         break;
%     end
end

% Mark phase 1 final conditions
t1 = t(n);                              % Store time Stamp
x1 = x(n);                              % pod distance at beginning of cruising phase (m/s)
xdot1 = xdot(n);                        % pod velocity at beginning of cruising phase (m/s)

%% Phase 2: Generate Trajectory profile for Cruising Phase - constrained by time
while t(n) < (t1 + deltat_cruising)     % Cruising phase constrained by time
    n = n + 1;

    % Compute Forces
    Fdrag_aero(n) = Fdrag.aero(xdot(n-1),rho) / (1 - eta_aerodrag);
    Fdrag_hover(n) = Fdrag.hover(xdot(n-1),z_nom) / (1 - eta_hoverdrag);
    Fdrag_brake(n) = Fdrag.brake(xdot(n-1),brakegap(n-1)) / (1 - eta_brakedrag);
    Fdrag_ski(n) = Fdrag.ski(xdot(n-1),z_nom) / (1 - eta_skidrag);

    Fdrag_net = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) + Fdrag_ski(n);
    Fthrust(n) = 0;

    % Compute kinematics
    xddot(n) = (Fthrust(n) - Fdrag_net)/mpod;
    xdot(n) = xdot(n-1) + xddot(n)*dt;
    x(n) = x(n-1) + xdot(n-1)*dt + 0.5*xddot(n)*dt^2;
    t(n) = t(n-1) + dt;

    brakegap(n) = brakegap(n-1);

    % Compute load along brake actuator lead screw
    Fload_brakes(n) = Fbrakelift(xdot(n),brakegap(n))*sin(17*pi()/180) - Fdrag_brake(n)*cos(17*pi()/180)/2;
end

% Mark phase 2 final conditions
n2 = n;
t2 = t(n);           % Store time Stamp
x2 = x(n);           % pod distance at beginning of cruising phase (m/s)
xdot2 = xdot(n);     % pod velocity at beginning of cruising phase (m/s)
b0 = brakegap(n);    % store initial brakegap position

% %% Generate Trajectory profile for aux propulsion using LIM Phase
%     tLIM = t(n);                              % Store time Stamp
%     xLIM = x(n);                              % pod distance at beginning of cruising phase (m/s)
%     xdot1 = xdot(n);                        % pod velocity at beginning of cruising phase (m/s)
%     while t(n) < (tLIM + deltat_LIM)     % Cruising phase constrained by time
%         n = n + 1;
%         % Compute drag forces
% %         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho)/ (1 - eta);
% %         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom)/ (1 - eta);
% %         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1))/ (1 - eta);
%         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
%         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom);
%         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
%         Flimprop(n) = Flimthrust(xdot(n-1));
%         
%         Fdrag_net = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
%         Fthrust(n) = Flimprop(n);     % Auxiliary thrust using LIM
%         
%         % Compute kinematics
%         xddot(n) = (Fthrust(n) - Fdrag_net)/mpod;
%         xdot(n) = xdot(n-1) + xddot(n)*dt;
%         x(n) = x(n-1) + xdot(n)*dt;
%         brakegap(n) = brakegap(n-1);
%         t(n) = t(n-1) + dt;
%         
%     end

%% Phase 3: Generate Deceleration Trajectory for Ideal Brake Deployment - constrained by final velocity
% Deploy brakes to a "nominal" distance, prior to running PID
% controlled braking, such that the nominal brake gap alone should
% theoretically yield a perfect stop at target distance, where PID could 
% still make course corrections after initial settling time, if need be.

% Determine nominal brake gap value, such that pod reaches target distance at desired final velocity
% Empirical data needed for validating brake actuator response dynamics, see brakeactuator.m.
fprintf('Calculating optimal nominal brake gap...\n')
% for brakegapNom = 15:-0.5:2.5;    % Determine optimal brakegapNom

    i = 0;
%     while xdot(n) - xdotf > 0.01
    while xdot(n) > xdotf
        n = n + 1;      % counter for simulation time step
        i = i + 1;      % counter for brakeactuator array

        % Compute Forces
        Fdrag_aero(n) = Fdrag.aero(xdot(n-1),rho) / (1 - eta_aerodrag);
        Fdrag_hover(n) = Fdrag.hover(xdot(n-1),z_nom) / (1 - eta_hoverdrag);
        Fdrag_brake(n) = Fdrag.brake(xdot(n-1),brakegap(n-1)) / (1 - eta_brakedrag);
        Fdrag_ski(n) = Fdrag.ski(xdot(n-1),z_nom) / (1 - eta_skidrag);

        Fdrag_net = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) + Fdrag_ski(n);

        Fthrust(n) = 0;

        % Compute kinematics
        xddot(n) = (Fthrust(n) - Fdrag_net)/mpod;
        xdot(n) = xdot(n-1) + xddot(n)*dt;
        x(n) = x(n-1) + xdot(n-1)*dt + 0.5*xddot(n)*dt^2;
        t(n) = t(n-1) + dt;

        % Calculate current brake gap distance
        [b,ts] = brakeactuator(b0,brakegapNom, t(n) - t2 );
        brakegap(n) = b;
        %% Phase 4: Generate Controlled Braking Trajectory - constrained by initial brake deployment settling time (~4.8s)
        if (t(n) - t2) - ts < 0.01 % brake actuator settling time reached. PID controller engaged.
            % Mark phase 3 final conditions
            n3 = n;
            x3 = x(n);
            xdot3 = xdot(n);
            t3 = t(n);
        end

        % Constrain brakegap to limit switches
        if brakegap(n) > 25
            brakegap(n) = 25;
        end
        if brakegap(n) < 2.5
            brakegap(n) = 2.5;
        end

        % Compute load along brake actuator lead screw
        Fload_brakes(n) = Fbrakelift(xdot(n),brakegap(n))*sin(17*pi()/180) - Fdrag_brake(n)*cos(17*pi()/180)/2;
    end

%     % if total distance traveled is within 0.05m of target, break for loop
%     if abs(x(end) - xf) < 0.01 %&& xdotf - xdot(end) < 0.01
%         break
%     else	% else, reset arrays and try again
%         n = n2;
% 
%         % reset force profile
%         Fdrag_aero = Fdrag_aero(1:n2);
%         Fdrag_hover = Fdrag_hover(1:n2);
%         Fdrag_brake = Fdrag_brake(1:n2);            
%         Fdrag_ski = Fdrag_ski(1:n2);
%         Fthrust = Fthrust(1:n2);
% 
%         % reset kinematics
%         xddot = xddot(1:n2);
%         xdot = xdot(1:n2);
%         x = x(1:n2);
%         t = t(1:n2);
% 
%         % reset brakegap profile
%         brakegap = brakegap(1:n2);
% 
%         % Compute load along brake actuator lead screw
%         Fload_brakes = Fload_brakes(1:n2);
%     end
% end

%% Save Distance and Velocity setpoints to be used for PID controlled braking (see 'GainScheduledPIDBrakingSystem.m')
velocitySet = xdot(n2:end);
distanceSet = x(n2:end);

%% Time-dependent Trajectory Graphs
fprintf('Generating plots...\n')

figure(1)
subplot(411)
hold on
plot(t,x)
plot([0 t(length(t))],[xf xf])
axis([0 1.2*t(length(t)) 0 1.2*xf])

grid on
grid minor    
title(['Trajectory case no. ' num2str(caseno) ': ' num2str(mpod) 'kg pod mass | ' num2str(P/1000,4) 'kPa @' num2str(T,4) 'K | ' num2str(gForce_pusher,2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(xdot1,4) 'm/s max velocity | ' num2str(deltat_cruising,2) 's cruising | ' num2str(brakegapNom) 'mm nominal brakegap'])
ylabel('Distance (m)')
legend('Pod travel','Target distance');

subplot(412)
plot(t,xdot)
axis([0 1.2*t(length(t)) 0 1.2*xdot1])
grid on
grid minor
ylabel('Velocity (m/s)')
legend('Velocity profile');

subplot(413)
hold on
plot(t,xddot/g)
plot(t,Fthrust/(mpod*g))
%     plot(t,Flimprop/(mpod*g))
plot(t,-Fdrag_aero/(mpod*g))
plot(t,-Fdrag_hover/(mpod*g))
plot(t,-Fdrag_brake/(mpod*g))
plot(t,-Fdrag_ski/(mpod*g))
%     axis([0 1.25*t(length(t)) -1.5 1.5])
axis([0 1.2*t(length(t)) -2.1 2.1])
grid on
grid minor
ylabel('Acceleration (gs)')
%     legend('Total acceleration','Pusher','LIM prop','Aerodynamic drag','Hover drag','Braking drag');
legend('Total acceleration','Pusher','Aerodynamic drag','Hover drag','Braking drag','Ski drag');

subplot(414)
plot(t,brakegap)
axis([0 1.2*t(length(t)) 0 30])
grid on
grid minor
ylabel('Brake gap (mm)')
xlabel('time (s)')
legend('Braking profile');

%% Distance-dependent Trajectory Graphs
figure(2)
subplot(311)
hold on
plot(x,xdot)
%     plot(xf + deltax_dangerzone - xdotOverride*4.8,xdotOverride)
%     plot(x,v_override)
plot([x1 x1],[0 1.1*xdot2])
plot([x2 x2],[0 1.1*xdot2])
% plot([x3 x3],[0 1.1*xdot2])
plot([xf xf],[0 1.1*xdot2])
plot([xf+deltax_dangerzone xf+deltax_dangerzone],[0 1.1*xdot2],'r')
axis([0 1.2*xf 0 1.1*xdot2])
grid on
grid minor
title(['Trajectory case no. ' num2str(caseno) ': ' num2str(mpod) 'kg pod mass | ' num2str(P/1000,4) 'kPa @' num2str(T,4) 'K | ' num2str(gForce_pusher,2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(xdot1,4) 'm/s max velocity | ' num2str(deltat_cruising,2) 's cruising | ' num2str(brakegapNom) 'mm nominal brakegap'])
%     formatSpec = 'PID Override Trigger for deltax_{dangerzone} = %0.fm';
%formatSpec = 'PID Override Trigger';
%str = sprintf(formatSpec,deltax_dangerzone);
% legend('rPod Trajectory','Pusher Jettisoned','Braking Engaged','PID Controller Engaged','Target Distance','Danger Zone');
legend('rPod Trajectory','Pusher Jettisoned','Braking Engaged','Target Distance','Danger Zone');
ylabel('Velocity (m/s)')

subplot(312)
plot(x,brakegap)
axis([0 1.2*xf 0 30])
grid on
grid minor
ylabel('Brake gap (mm)')
xlabel('Distance (m)')
legend('Braking profile');

subplot(313)
plot(x,Fload_brakes)
axis([0 1.2*xf 1.1*min(Fload_brakes) 1.1*max(Fload_brakes)])
grid on
grid minor
ylabel('Brake load (N)')
xlabel('Distance (m)')
legend('load along leadscrew');

%% Output Trajectory data to csv
fprintf('Saving trajectory data to csv...\n')

header = {'t', 'x', 'xdot', 'xddot', 'drag_aero', 'drag_hover', 'drag_brake', 'drag_ski', 'brakegap', 'Fload_brakes'};

data = table(t', x', xdot', xddot', -Fdrag_aero'/(mpod*g), -Fdrag_hover'/(mpod*g), -Fdrag_brake'/(mpod*g), -Fdrag_ski'/(mpod*g), brakegap', Fload_brakes');
data.Properties.VariableNames = header;

formatSpec = 'Trajectory_case_no_%0.f.csv';
filename = sprintf(formatSpec,caseno);
writetable(data,filename,'Delimiter',',')

% %% Output Trajectory to csv
% % header = {'time (s)', 'Distance (m)', 'Velocity (m/s)', 'Acceleration (m/s^2)', 'Aerodrag (gs)', 'Hoverdrag (gs)', 'Brakedrag (gs)', 'brakegap (mm)'};
% data = [t; x; xdot; xddot; -Fdrag_aero/(mpod*g); -Fdrag_hover/(mpod*g); -Fdrag_brake/(mpod*g); -Fdrag_ski/(mpod*g); brakegap; Fload_brakes]';
% % data = [t; x; xdot; xddot; Fthrust/(mpod*g); -Fdrag_aero/(mpod*g); -Fdrag_hover/(mpod*g); -Fdrag_brake/(mpod*g); brakegap; caseno*ones(size(x))]';
% 
% formatSpec = 'Trajectory_case_no_%0.f.csv';
% filename = sprintf(formatSpec,caseno);
% writetable(data,filename,'Delimiter',',')
% 
% %% Output Trajectory Header file to csv
% formatSpec = 'Trajectory Header for case no. %0.f.csv';
% filename = sprintf(formatSpec,caseno);
% fid = fopen(filename,'w');
% fprintf(fid, 'time (s)\tDistance (m)\tVelocity (m/s)\tTotal Acceleration (m/s^2)\tPropulsion (gs)\tAerodrag (gs)\tHoverdrag (gs)\tBrakedrag (gs)\tSkidrag (gs)\tbrakegap (mm)\tbrakeload (N)');
% % fprintf(fid, 'time (s)\tDistance (m)\tVelocity (m/s)\tTotal Acceleration (m/s^2)\tPropulsion (gs)\tAerodrag (gs)\tHoverdrag (gs)\tBrakedrag (gs)\tbrakegap (mm)\tCase No.');
% fclose(fid);

%% Output Simulation Parameters to csv
fprintf('Saving simulation parameters to csv...\n')

parameternames = {   'mpod', 
                    'dt', 
                    'xf', 
                    'xdotf', 
                    'gForce_pusher',
%                     'deltax_pusher'
                    'deltat_pusher',
%                     'vpod_max',
                    'deltat_cruising',
%                     'gForce_brakedrag',
                    'brakegapNom',
                    'deltax_dangerzone',
                    'z_nom',
                    'Ppsi',
                    'P',
                    'T',
                    'R',
                    'rho'
                    'eta_aerodrag',
                    'eta_hoverdrag',
                    'eta_brakedrag',
                    'eta_skidrag'
                    }

value = [   mpod,
            dt,
            xf,
            xdotf,
            gForce_pusher,
%             deltax_pusher,
            deltat_pusher,
%             vpod_max,
            deltat_cruising,
%             gForce_brakedrag,
            brakegapNom,
            deltax_dangerzone,
            z_nom,
            Ppsi,
            P,
            T,
            R,
            rho,
            eta_aerodrag,
            eta_hoverdrag,
            eta_brakedrag,
            eta_skidrag
            ]

description = { 'Total pod mass (kg)',
                'time step (s)',
                'Target distance (m)',
                'Target final velocity at xf (m/s)',
                'Pusher acceleration (gs)',
%                 'Desired max push distance (max: 487.68m or 1600ft) (m)'
%                 'Max push distance (max: 487.68m or 1600ft) (m)',
                'Push time (s)',
%                 'Constraint on max velocity (m/s)',
                'Cruising time between pusher and deceleration phase (minimum 2s required) (s)',
%                 'Constraint on max braking force (gs)',
                'Nominal brake gap during controlled braking phase (mm)',
                'Distance between final target and end of track (DANGER ZONE!!!) (m)'
                'Nominal hover height (m)',
                'Atmospheric air pressure inside SpaceX Hyperloop test tube (Psi)',
                'Atmospheric air pressure inside SpaceX Hyperloop test tube (Pa)',
                'Atmospheric air temperature inside SpaceX Hyperloop test tube (K)',
                'Ideal gas constant (J/(kg*K)',
                'Air density inside SpaceX Hyperloop test tube (kg/m^3)',
                'Estimated aerodynamic drag relative error',
                'Estimated hover-engine drag relative error',
                'Estimated brake drag relative error',
                'Estimated ski drag relative error'
                }

data = table(parameternames, value, description);

formatSpec = 'SimParameters_case_no_%0.f.csv';
filename = sprintf(formatSpec,caseno);
writetable(data,filename,'Delimiter',',')

