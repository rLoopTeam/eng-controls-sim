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
% Fbrake_lift.m
% brakeactuator.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Clear workspace and Generate Simulation Constraints
% clear
% simParameters

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
gForce_pusher = [0];

t0 = t(n);                      % Store time Stamp
x0 = x(n);                      % pod distance at beginning of push phase (m/s)
xdot0 = xdot(n);                % pod velocity at beginning of push phase (m/s)

Fdrag_aero = [0];              % Initialize array
Fdrag_hover = [0];     % Initialize array
Fdrag_brake = [0];    % Initialize array
Fdrag_ski = [0];
Fthrust = [0];
Fdrag_net = [0];
% Flimprop = [0];
Tload_brakes = [0];

%% Phase 1: Generate Trajectory profile for Pusher Phase
while t(n) < deltat_pusher      % pusher phase constrained by time
% while xdot(n) < vpod_max %       % pusher phase constrained by velocity
% while x(n) < deltax_pusher      % pusher phase constrained by distance
    n = n + 1;

    % Compute Forces
    Fdrag_aero(n) = Fdrag.aero(xdot(n-1),rho) / (1 - eta_aerodrag);
    Fdrag_brake(n) = Fdrag.brake(xdot(n-1),brakegap(n-1)) / (1 - eta_brakedrag);
    Fdrag_hover(n) = Fdrag.hover(xdot(n-1),z_nom) / (1 - eta_hoverdrag);
    Fdrag_ski(n) = Fdrag.ski(xdot(n-1),z_nom) / (1 - eta_skidrag);
    
    Fdrag_net(n) = Fdrag_aero(n) + hover_option*Fdrag_hover(n) + ski_option*Fdrag_ski(n) + Fdrag_brake(n);
    
    % Compute jerk (time derivative of acceleration)
    xdddot_jerk = gForce_pusher_max/deltat_jerk;
    
    % Compute gForce at time, t
    gForce_pusher(n) = gForce_pusher(n-1) + xdddot_jerk*dt;
    if gForce_pusher(n) >= gForce_pusher_max
        gForce_pusher(n) = gForce_pusher_max;
    end
    
    Fthrust(n) = mpod*gForce_pusher(end)*g;

    % Compute kinematics
    xddot(n) = (Fthrust(n) - Fdrag_net(n))/mpod;
    xdot(n) = xdot(n-1) + xddot(n)*dt;
    x(n) = x(n-1) + xdot(n-1)*dt + 0.5*xddot(n)*dt^2;
    t(n) = t(n-1) + dt;

    brakegap(n) = brakegap(n-1);

    % Compute load along brake actuator lead screw
    Tload_brakes(n) = Tbrakeload(xdot(n),xddot(n),brakegap(n)) / (1 - eta_brakedrag);

    % If max pusher distance reached, exit pusher phase
    if x(n) >= deltax_pusher_max
        break;
    end
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
    Fdrag_brake(n) = Fdrag.brake(xdot(n-1),brakegap(n-1)) / (1 - eta_brakedrag);
    Fdrag_hover(n) = Fdrag.hover(xdot(n-1),z_nom) / (1 - eta_hoverdrag);
    Fdrag_ski(n) = Fdrag.ski(xdot(n-1),z_nom) / (1 - eta_skidrag);
    
    Fdrag_net(n) = Fdrag_aero(n) + hover_option*Fdrag_hover(n) + ski_option*Fdrag_ski(n) + Fdrag_brake(n);

    Fthrust(n) = 0;

    % Compute kinematics
    xddot(n) = (Fthrust(n) - Fdrag_net(n))/mpod;
    xdot(n) = xdot(n-1) + xddot(n)*dt;
    x(n) = x(n-1) + xdot(n-1)*dt + 0.5*xddot(n)*dt^2;
    t(n) = t(n-1) + dt;

    brakegap(n) = brakegap(n-1);

    % Compute load along brake actuator lead screw
    Tload_brakes(n) = Tbrakeload(xdot(n),xddot(n),brakegap(n)) / (1 - eta_brakedrag);
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
%         Fdrag_net(n) = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
%         Fthrust(n) = Flimprop(n);     % Auxiliary thrust using LIM
%         
%         % Compute kinematics
%         xddot(n) = (Fthrust(n) - Fdrag_net(n))/mpod;
%         xdot(n) = xdot(n-1) + xddot(n)*dt;
%         x(n) = x(n-1) + xdot(n)*dt;
%         brakegap(n) = brakegap(n-1);
%         t(n) = t(n-1) + dt;
%         
%     end

%% Phase 3: Generate Deceleration Trajectory for Initial Brake Deployment - constrained by final velocity
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
    Fdrag_brake(n) = Fdrag.brake(xdot(n-1),brakegap(n-1)) / (1 - eta_brakedrag);
    Fdrag_hover(n) = Fdrag.hover(xdot(n-1),z_nom) / (1 - eta_hoverdrag);
    Fdrag_ski(n) = Fdrag.ski(xdot(n-1),z_nom) / (1 - eta_skidrag);
    
    Fdrag_net(n) = Fdrag_aero(n) + hover_option*Fdrag_hover(n) + ski_option*Fdrag_ski(n) + Fdrag_brake(n);

    Fthrust(n) = 0;

    % Compute kinematics
    xddot(n) = (Fthrust(n) - Fdrag_net(n))/mpod;
    xdot(n) = xdot(n-1) + xddot(n)*dt;
    x(n) = x(n-1) + xdot(n-1)*dt + 0.5*xddot(n)*dt^2;
    t(n) = t(n-1) + dt;

    % Calculate current brake gap distance
    if instant_braking == true
        ts = 0; % settling time is 0 for instant braking
        brakegap(n) = brakegapNom;
    else
        [b,ts] = brakeactuator(b0,brakegapNom, t(n) - t2 );
        brakegap(n) = b;

    end

    %% Phase 4: Generate Controlled Braking Trajectory - constrained by initial brake deployment settling time (~4.8s)
    if (t(n) - t2) >= ts % brake actuator settling time reached. PID controller engaged.
        break
    end

    % Constrain brakegap to limit switches
    if brakegap(n) > 25
        brakegap(n) = 25;
    end
    if brakegap(n) < 2.5
        brakegap(n) = 2.5;
    end

    % Compute load along brake actuator lead screw
    Tload_brakes(n) = Tbrakeload(xdot(n),xddot(n),brakegap(n)) / (1 - eta_brakedrag);
end

% Mark phase 3 final conditions
n3 = n;
x3 = x(n);
xdot3 = xdot(n);
t3 = t(n);

%% Phase 4: Generate Remaining Deceleration Trajectory - constrained by final velocity
% Note: for PID controlled braking, brakes will adjust during this phase to reach target distance
while xdot(n) > xdotf
    n = n + 1;      % counter for simulation time step

    % Compute Forces
    Fdrag_aero(n) = Fdrag.aero(xdot(n-1),rho) / (1 - eta_aerodrag);
    Fdrag_brake(n) = Fdrag.brake(xdot(n-1),brakegap(n-1)) / (1 - eta_brakedrag);
    Fdrag_hover(n) = Fdrag.hover(xdot(n-1),z_nom) / (1 - eta_hoverdrag);
    Fdrag_ski(n) = Fdrag.ski(xdot(n-1),z_nom) / (1 - eta_skidrag);
    
    Fdrag_net(n) = Fdrag_aero(n) + hover_option*Fdrag_hover(n) + ski_option*Fdrag_ski(n) + Fdrag_brake(n);

    Fthrust(n) = 0;

    % Compute kinematics
    xddot(n) = (Fthrust(n) - Fdrag_net(n))/mpod;
    xdot(n) = xdot(n-1) + xddot(n)*dt;
    x(n) = x(n-1) + xdot(n-1)*dt + 0.5*xddot(n)*dt^2;
    t(n) = t(n-1) + dt;

    brakegap(n) = brakegap(n-1);

    % Compute load along brake actuator lead screw
    Tload_brakes(n) = Tbrakeload(xdot(n),xddot(n),brakegap(n)) / (1 - eta_brakedrag);
end

%% Save Distance and Velocity setpoints to be used for PID controlled braking (see 'GainScheduledPIDBrakingSystem.m')
velocitySet = xdot(n3:end);
distanceSet = x(n3:end);

%% Time-dependent Trajectory Graphs
fprintf('Generating plots...\n')

figure(1)
subplot(411)
if  hover_option == false && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity'])
elseif hover_option == true && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag'])
elseif hover_option == false && ski_option == true
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
else
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
end
hold on
plot(t,x,'b')
plot([0 t(end)],[xf xf],'m')
plot([0 t(end)],[(xf+deltax_dangerzone) (xf+deltax_dangerzone)],'r')
axis([0 1.2*t(end) 0 1.2*(xf+deltax_dangerzone)])

grid on
grid minor    
ylabel('Distance (m)')
legend('Pod Travel','Target Distance','Danger Zone');

subplot(412)
plot(t,xdot,'b')
axis([0 1.2*t(end) 0 1.2*xdot1])
grid on
grid minor
ylabel('Velocity (m/s)')
legend('Velocity profile');

subplot(413)
hold on
plot(t,xddot/g,'b')
plot(t,Fthrust/(mpod*g),'g')
%     plot(t,Flimprop/(mpod*g))
plot(t,-Fdrag_aero/(mpod*g),'y')
plot(t,-Fdrag_brake/(mpod*g),'r')
if hover_option == true
    plot(t,-Fdrag_hover/(mpod*g),'m')
end
if ski_option == true
    plot(t,-Fdrag_ski/(mpod*g),'c')
end
axis([0 1.2*t(length(t)) -1.1*max(Fdrag_net/(mpod*g)) 1.1*max(Fthrust/(mpod*g))])
grid on
grid minor
ylabel('Acceleration (gs)')
%     legend('Total acceleration','Pusher','LIM prop','Aerodynamic drag','Hover drag','Braking drag');
if  hover_option == false && ski_option == false
    legend('Total acceleration','Pusher','Aerodynamic drag','Braking drag');
elseif hover_option == true && ski_option == false
    legend('Total acceleration','Pusher','Aerodynamic drag','Braking drag','Hover drag');
elseif hover_option == false && ski_option == true
    legend('Total acceleration','Pusher','Aerodynamic drag','Braking drag','Ski drag');
else
    legend('Total acceleration','Pusher','Aerodynamic drag','Braking drag','Hover drag','Ski drag');
end


subplot(414)
hold on
plot(t,brakegap,'b')
plot(0,brakegap,'r')
axis([0 1.2*t(length(t)) 0 30])
grid on
grid minor
ylabel('Brake gap (mm)')
xlabel('time (s)')
legend('Braking profile');

%% Distance-dependent Trajectory Graphs
figure(2)
subplot(311)
if  hover_option == false && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity'])
elseif hover_option == true && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag'])
elseif hover_option == false && ski_option == true
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
else
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
end
hold on
plot(x,xdot,'b')
plot([x1 x1],[0 1.1*max(xdot)],'g')
plot([x2 x2],[0 1.1*max(xdot)],'c')
if PIDcontroller == true
    plot([x3 x3],[0 1.1*max(xdot)],'y')
end
plot([xf xf],[0 1.1*max(xdot)],'m')
plot([(xf+deltax_dangerzone) (xf+deltax_dangerzone)],[0 1.1*max(xdot)],'r')
axis([0 1.2*(xf+deltax_dangerzone) 0 1.1*max(xdot)])
grid on
grid minor
ylabel('Velocity (m/s)')
if PIDcontroller == true
    legend('rPod Trajectory','Pusher Jettisoned','Braking Engaged','PID Controller Engaged','Target Distance','Danger Zone');
else
    legend('rPod Trajectory','Pusher Jettisoned','Braking Engaged','Target Distance','Danger Zone');
end

subplot(312)
hold on
plot(x,brakegap,'b')
axis([0 1.2*(xf+deltax_dangerzone) 0 30])
grid on
grid minor
ylabel('Brake gap (mm)')
xlabel('Distance (m)')
% legend('Braking profile');

subplot(313)
hold on
plot(x,Tload_brakes,'b')
plot([0 x(end)], [-3 -3],'r')
plot([0 x(end)], [3 3],'r')
axis([0 1.2*(xf+deltax_dangerzone) -3.5 3.5])
% axis([0 1.2*(xf+deltax_dangerzone) 1.1*min(Tload_brakes) 1.1*max(Tload_brakes)])
grid on
grid minor
ylabel('Brake load (N*m)')
xlabel('Distance (m)')
legend('torque load on brake stepper motor','holding torque limits on brake stepper motor');

%% Velocity vs Time Graph
figure(3)
if  hover_option == false && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity'])
elseif hover_option == true && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag'])
elseif hover_option == false && ski_option == true
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
else
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
end
hold on
plot(t,x,'b')
plot([0 t(end)],[xf xf],'m')
plot([0 t(end)],[(xf+deltax_dangerzone) (xf+deltax_dangerzone)],'r')
axis([0 1.2*t(end) 0 1.2*(xf+deltax_dangerzone)])

grid on
grid minor    
ylabel('Distance (m)')
xlabel('time (s)')
legend('Pod Travel','Target Distance','Danger Zone');

%% Velocity vs Distance Graph
figure(4)
if  hover_option == false && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity'])
elseif hover_option == true && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag'])
elseif hover_option == false && ski_option == true
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
else
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
end
hold on
plot(x,xdot,'b')
plot([x1 x1],[0 1.1*max(xdot)],'g')
plot([x2 x2],[0 1.1*max(xdot)],'c')
if PIDcontroller == true
    plot([x3 x3],[0 1.1*max(xdot)],'y')
end
plot([xf xf],[0 1.1*max(xdot)],'m')
plot([(xf+deltax_dangerzone) (xf+deltax_dangerzone)],[0 1.1*max(xdot)],'r')
axis([0 1.2*(xf+deltax_dangerzone) 0 1.1*max(xdot)])
grid on
grid minor
ylabel('Velocity (m/s)')
xlabel('Distance (m)')
if PIDcontroller == true
    legend('rPod Trajectory','Pusher Jettisoned','Braking Engaged','PID Controller Engaged','Target Distance','Danger Zone');
else
    legend('rPod Trajectory','Pusher Jettisoned','Braking Engaged','Target Distance','Danger Zone');
end

%% Drag Contribution vs. Time Graphs
figure(5)
if  hover_option == false && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity'])
elseif hover_option == true && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag'])
elseif hover_option == false && ski_option == true
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
else
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
end
hold on
plot(t,100*Fdrag_aero./Fdrag_net,'b')
plot(t,100*Fdrag_brake./Fdrag_net,'r')
if hover_option == true
    plot(t,100*Fdrag_hover./Fdrag_net,'m')
end
if ski_option == true
    plot(t,100*Fdrag_ski./Fdrag_net,'c')
end
xlabel('time (s)')
ylabel('drag contribution (%)')
legend('aerodrag','brakedrag','hoverdrag','skidrag')

%% Drag Contribution vs. Velocity Graphs
figure(6)
if  hover_option == false && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity'])
elseif hover_option == true && ski_option == false
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag'])
elseif hover_option == false && ski_option == true
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
else
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(gForce_pusher(end),2) 'g acceleration for ' num2str(t1,3) 's ' num2str(x1,4) 'm | ' num2str(max(x),4) 'm pod travel | ' num2str(max(xdot),4) 'm/s max velocity | ' num2str(max(Fdrag_hover),4) 'N max hover drag | ' num2str(max(Fdrag_ski),4) 'N max ski drag'])
end
hold on
plot(xdot,100*Fdrag_aero./Fdrag_net,'b')
plot(xdot,100*Fdrag_brake./Fdrag_net,'r')
if hover_option == true
    plot(xdot,100*Fdrag_hover./Fdrag_net,'m')
end
if ski_option == true
    plot(xdot,100*Fdrag_ski./Fdrag_net,'c')
end
xlabel('velocity (m/s)')
ylabel('drag contribution (%)')
legend('aerodrag','brakedrag','hoverdrag','skidrag')

%% Output Trajectory data to csv
fprintf('Saving trajectory data to csv...\n')

header = {'t', 'x', 'xdot', 'xddot', 'drag_aero', 'drag_hover', 'drag_ski', 'drag_brake', 'aerodrag_contribution', 'hoverdrag_contribution', 'skidrag_contribution', 'brakedrag_contribution', 'brakegap', 'Tload_brakes'};
data = table(t', x', xdot', xddot', -Fdrag_aero'/(mpod*g), -hover_option*Fdrag_hover'/(mpod*g), -ski_option*Fdrag_ski'/(mpod*g), -Fdrag_brake'/(mpod*g), 100*Fdrag_aero'./Fdrag_net', 100*hover_option*Fdrag_hover'./Fdrag_net', 100*ski_option*Fdrag_ski'./Fdrag_net', 100*Fdrag_brake'./Fdrag_net', brakegap', Tload_brakes');

data.Properties.VariableNames = header;

formatSpec = 'Trajectory_case_no_%0.f.csv';
filename = sprintf(formatSpec,caseno);
writetable(data,filename,'Delimiter',',')

header_avionics = {'ACCEL_X'};
data_avionics = table(-int32(512*xddot'));
data_avionics.Properties.VariableNames = header_avionics;
formatSpec_avionics = 'Trajectory_case_no_%0.f_avionics.csv';
filename_avionics = sprintf(formatSpec_avionics ,caseno);
writetable(data_avionics,filename_avionics,'Delimiter',',')

% %% Output Trajectory to csv
% % header = {'time (s)', 'Distance (m)', 'Velocity (m/s)', 'Acceleration (m/s^2)', 'Aerodrag (gs)', 'Hoverdrag (gs)', 'Brakedrag (gs)', 'brakegap (mm)'};
% data = [t; x; xdot; xddot; -Fdrag_aero/(mpod*g); -Fdrag_hover/(mpod*g); -Fdrag_brake/(mpod*g); -Fdrag_ski/(mpod*g); brakegap; Tload_brakes]';
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

parameternames = {	'mpod', 
                    'dt', 
                    'xf', 
                    'xdotf', 
                    'gForce_pusher(end)',
%                     'deltax_pusher'
                    'deltat_pusher',
%                     'vpod_max',
                    'deltat_cruising',
%                     'gForce_brakedrag',
                    'brakegapNom',
                    'deltax_dangerzone',
                    'z_nom',
                    'hover_option',
                    'ski_option',
                    'instant_braking',
                    'PIDcontroller',
                    'Ppsi',
                    'P',
                    'T',
                    'R',
                    'rho'
                    'eta_aerodrag',
                    'eta_hoverdrag',
                    'eta_brakedrag',
                    'eta_skidrag',
                    }

value = [   mpod,
            dt,
            xf,
            xdotf,
            gForce_pusher(end),
%             deltax_pusher,
            deltat_pusher,
%             vpod_max,
            deltat_cruising,
%             gForce_brakedrag,
            brakegapNom,
            deltax_dangerzone,
            z_nom,
            hover_option,
            ski_option,
            instant_braking,
            PIDcontroller,
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
                'Distance between final target and end of track (DANGER ZONE!!!) (m)',
                'Nominal hover height (m)',
                'Enables/disables hover-engines',
                'Enables/disables skis',
                'true = brakes reach nominal brakegap instantaneously',
                'true = brakes actuators use PID controller to adjust trajectory',
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

%%