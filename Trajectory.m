% brakingTrajectory.m generates arrays for desired target velocity,
% 'velocitySet', a distance, 'distanceSet'
% for GainScheduledPIDBrakingSystem.mdl
%
% Notes: 
% Need to acquire experimental data from stepper motors for modeling
% actual response dynamics, such as damping, phase lag, and time delay.
% Trajectory calcs do not account for aero-drag, hover engine drag.
% References Fdrag.m

%% Clear workspace and Generate Simulation Constraints
clear
simParameters

formatSpec = 'Running case no. %0.f...\n\n';
str = sprintf(formatSpec, caseno);
fprintf(str)

%% Initiaize crash vs survival counter
crash_counter = 0;
survival_counter = 0;

for vpod_max = 120:-0.1:50    % Loop through v_max, such that desired sim constraints are satisfied
% %% Compute multiple trajectory cases for -50% to +50% relative error, where relative = (actual value - approximate value)/(actual value)
% Fdrag_actual = Fdrag_approx/(1 - eta)
% for eta = 0.0:0.01:0.0
      
    %% Simulation initial conditions
    n = 1;
    t = [0];                    % Initialize time array (s)
    x = [0];                    % Initialize distance array (m)
    xdot = [0];                 % Initialize velocity array (m/s)
    xddot = [0];                % Initialize acceleration array (m/s^2)
    brakegap = [25];            % Initialize brake Gap (mm)

    Fdrag_aero = [Faerodrag(xdot(1),rho)];              % Initialize array
    Fdrag_hover = [Fhoverdrag(xdot(1),z_nom*10^3)];          % Initialize array
    Fdrag_brake = [Fbrakedrag(xdot(1),brakegap(1))];    % Initialize array
    Fthrust = [gForce_pusher*g];
    
    %% Generate Trajectory profile for Pusher Phase - Constrained by max velocity
    t0 = t(n);                              % Store time Stamp
    x0 = x(n);                              % pod distance at beginning of push phase (m/s)
    xdot0 = xdot(n);                        % pod velocity at beginning of push phase (m/s)
    while xdot < vpod_max %                 % pusher phase constrained by velocity
%     while x(n) < deltax_pusher              % pusher phase constrained by distance
        n = n + 1;
        % Compute drag forces
%         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho)/ (1 - eta);
%         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3)/ (1 - eta);
%         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1))/ (1 - eta);
        Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
        Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3);
        Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
        
        Fdrag = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
        Fthrust(n) = mpod*gForce_pusher*g;
        
        % Compute kinematics
        xddot(n) = (Fthrust(n) - Fdrag)/mpod;
        xdot(n) = xdot(n-1) + xddot(n)*dt;
        x(n) = x(n-1) + xdot(n)*dt;
        brakegap(n) = brakegap(n-1);
        t(n) = t(n-1) + dt;
        
        % If max pusher distance reached, exit pusher phase
        if x(n) >= deltax_pusher_max
            break;
        end
        
    end
    
%     %% Generate Trajectory profile for Pusher Phase - Constrained by distance pushed
%     t0 = t(n);                              % Store time Stamp
%     x0 = x(n);                              % pod distance at beginning of push phase (m/s)
%     xdot0 = xdot(n);                        % pod velocity at beginning of push phase (m/s)
%     % while xdot < vpod_max %                 % pusher phase constrained by velocity
%     while x(n) < deltax_pusher              % pusher phase constrained by distance
%         n = n + 1;
%         % Compute drag forces
% %         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho)/ (1 - eta);
% %         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3)/ (1 - eta);
% %         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1))/ (1 - eta);
%         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
%         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3);
%         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
%         
%         Fdrag = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
%         Fthrust(n) = mpod*gForce_pusher*g;
%         
%         % Compute kinematics
%         xddot(n) = (Fthrust(n) - Fdrag)/mpod;
%         xdot(n) = xdot(n-1) + xddot(n)*dt;
%         x(n) = x(n-1) + xdot(n)*dt;
%         brakegap(n) = brakegap(n-1);
%         t(n) = t(n-1) + dt;
%     end
%     

%% Generate Trajectory profile for Cruising Phase
    t1 = t(n);                              % Store time Stamp
    x1 = x(n);                              % pod distance at beginning of cruising phase (m/s)
    xdot1 = xdot(n);                        % pod velocity at beginning of cruising phase (m/s)
    while t(n) < (t1 + deltat_cruising)     % Cruising phase constrained by time
        n = n + 1;
        % Compute drag forces
%         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho)/ (1 - eta);
%         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3)/ (1 - eta);
%         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1))/ (1 - eta);
        Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
        Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3);
        Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
        
        Fdrag = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
        Fthrust(n) = 0;
        
        % Compute kinematics
        xddot(n) = (Fthrust(n) - Fdrag)/mpod;
        xdot(n) = xdot(n-1) + xddot(n)*dt;
        x(n) = x(n-1) + xdot(n)*dt;
        brakegap(n) = brakegap(n-1);
        t(n) = t(n-1) + dt;
        
    end

    %% Generate Deceleration Trajectory for brake deployment - constrained by total track distance
    % Notes: this model generates a brake gap table for constrained brakedrag force
    % Empirical data required for validating model for stepper motor response dynamics.
    
    % Record trajectory instances at beginning of deceleration period
%     Fdrag_aero(n) = Faerodrag(xdot(n-1),rho)/ (1 - eta);
%     Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3)/ (1 - eta);
%     Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1))/ (1 - eta);
    Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
    Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3);
    Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
    
    Fdrag = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
    Fthrust(n) = 0;
    
    % Compute kinematics
    xddot2 = (Fthrust(n) - Fdrag) / mpod;
    xdot2 = xdot(n-1) + xddot(n)*dt;
    x2 = x(n-1) + xdot(n)*dt;
    brakegap2 = brakegap(n);                   % Store brakegap at beginning of deceleration period
    t2 = t(n-1) + dt;
    
    % Keep adjusting deltat and recomputing decel trajectory until final velocity is reached below final track distance
    ts = 4.8;     % Settling time for full brake deployment
%     for ts = 4.8:0.1:10     % Settling time for full brake deployment
    for brakegapNom = 25:-0.5:2.5;  % Adjust nominal brake gap until pod reaches target at desired final velocity
    
        % Initialize dummy arrays
%         Fdrag_aeroDecel = [Faerodrag(xdot(n-1),rho)] / (1 - eta);
%         Fdrag_hoverDecel = [Fhoverdrag(xdot(n-1),z_nom*10^3)] / (1 - eta);
%         Fdrag_brakeDecel = [Fbrakedrag(xdot(n-1),brakegap(n-1))] / (1 - eta);
        Fdrag_aeroDecel = [Faerodrag(xdot(n-1),rho)];
        Fdrag_hoverDecel = [Fhoverdrag(xdot(n-1),z_nom*10^3)];
        Fdrag_brakeDecel = [Fbrakedrag(xdot(n-1),brakegap(n-1))];
        
        % Compute thrust forces
        FthrustDecel = [0];
        
        xddotDecel = [xddot2];
        xdotDecel = [xdot2];
        xDecel = [x2];
        tDecel = [t2];
        bDecel = [brakegap2];
        
        % Deploy brake gap in 1mm increments
%         brakegapSet = brakegap2 - 1;      % Set initial brake gap target to 24mm
        brakegapSet = brakegapNom;      % Set initial brake gap target to 24mm
        
        % Pre-compute brake profile based on stepper motor response dynamics
        zeta = 1.0;                         % Damping coefficient for stepper motor response dynamics
%         ts = 4.8;
%         wn = wnCalc(ts,zeta);             % Natural frequency, given settling time and zeta for 2nd order system
        wn = -log(0.003)/(ts*zeta);         % Natural frequency, given settling time and zeta for 2nd order system
        num = [wn^2];
        den = [1 2*zeta*wn wn^2];
        G = tf(num,den);                    % Transfer function for stepper motor response dynamics (rad/s)
        
        % Pre-compute brake gap step response
        [y,tstepper] = step(G, [0:dt:ts] );             % Step response function
        dt = tstepper(2) - tstepper(1);     % Redefine time step

        i = 1;                  % Initialize counter
        j = 0;                  % Initialize counter
        b0 = brakegap2;
        while xdotDecel(i) > xdotf

            i = i + 1;
            j = j + 1;

            if( j > length(tstepper) )
                j = 1;
                b0 = bDecel(i-1);
                brakegapSet = bDecel(i-1) - 1;                % Deploy brake gap in 1mm increments
                if brakegapSet <= brakegapNom
                    brakegapSet = brakegapNom;
                end
            end

            bDecel(i) = b0 + (brakegapSet - b0) * y(j);     % Compute brakegap for step input signal
            if bDecel(i) < 2.5
                bDecel(i) = 2.5;
            end

            % Compute drag forces
%             Fdrag_aeroDecel(i) = Faerodrag(xdotDecel(i-1),rho) / (1 - eta);
%             Fdrag_hoverDecel(i) = Fhoverdrag(xdotDecel(i-1),z_nom*10^3) / (1 - eta);
%             Fdrag_brakeDecel(i) = Fbrakedrag(xdotDecel(i-1),bDecel(i-1)) / (1 - eta);
            Fdrag_aeroDecel(i) = Faerodrag(xdotDecel(i-1),rho);
            Fdrag_hoverDecel(i) = Fhoverdrag(xdotDecel(i-1),z_nom*10^3);
            Fdrag_brakeDecel(i) = Fbrakedrag(xdotDecel(i-1),bDecel(i-1));
            
            Fdrag = Fdrag_aeroDecel(i) + Fdrag_hoverDecel(i) + Fdrag_brakeDecel(i);
            
            % Compute thrust forces
            FthrustDecel(i) = 0;
            
            % Compute kinematics
            xddotDecel(i) = (FthrustDecel(i) - Fdrag)/mpod;
            xdotDecel(i) = xdotDecel(i-1) + xddotDecel(i)*dt;
            xDecel(i) = xDecel(i-1) + xdotDecel(i)*dt;
            tDecel(i) = tDecel(i-1) + dt;
            
        end
        
        % If final velocity met and target reached within 10m, break loop
        if xdotDecel(i) <= xdotf && xDecel(i) - xf < 10
            break;
        % If final velocity met, but target undershot, then drag forces are too high, break loop
        elseif xdotDecel(i) <= xdotf && xDecel(i) < xf
            break;
        end    
    
    end
    
%     % If brake actuator settling time was to long
%     if (xdotDecel(i) < xdotf && xDecel(i) > xf + deltax_dangerzone)
%         formatSpec = 'Pod crashed at %0.2f m/s \n';
%         str = sprintf(formatSpec, xdot(n));
%         fprintf(str)
%         
%         crash_counter = crash_counter + 1;
%     elseif (xdotDecel(i) < xdotf && xDecel(i) >= xf)
%         formatSpec = 'Pod survived, but overshot target by %0.2f \n';
%         str = sprintf(formatSpec, xDecel(i) - xf);
%         fprintf(str)
%         
%         survival_counter = survival_counter + 1;
%     elseif (xdotDecel(i) <= xdotf && xDecel(i) < xf)
%         formatSpec = 'Pod survived, but undershot target by %0.2f \n';
%         str = sprintf(formatSpec, xf - xDecel(i));
%         fprintf(str)
%         
%         survival_counter = survival_counter + 1;
%     end
    
    % step(G)
    
    % Compute drag forces
    Fdrag_aero = [Fdrag_aero, Fdrag_aeroDecel];
    Fdrag_hover = [Fdrag_hover, Fdrag_hoverDecel];
    Fdrag_brake = [Fdrag_brake, Fdrag_brakeDecel];
    
    % Compute thrust forces
    Fthrust = [Fthrust, FthrustDecel];
    
    % Compute kinematics
    xddot = [xddot, xddotDecel];
    xdot = [xdot, xdotDecel];
    x = [x, xDecel];
    brakegap = [brakegap, bDecel];
    t = [t, tDecel];
    

%     %% Generate Trajectory profile for Brake Engagement to Nominal Brake Gap Phase
% 
%     % Compute brakegap setpoint constrained within nominal brake gap and max brakedrag
%     % brakegapSet = brakegapCalc(gForce_brakedrag*g*mpod - Fhoverdrag(xdot(n-1),z_nom*10^3), xdot(n-1));   % Nominal Brake Gap constrained at a max brake force minus hoverdrag force to avoid overpitching
%     % if brakegapSet < brakegapNom
% %         brakegapSet = brakegapNom;
%         brakegapSet = 2.5;
%     % end
% 
%     % Calculate settling time to reach nominal brakegap
%     % Extract from table or curve fitting equation (Data needed)
% 
%     % Transfer function for stepper motor response dynamics during 25mm to nomimal brakegap engagement phase
%     zeta = 1.0;                                          % Damping coefficient for stepper motor response dynamics
% %     wn = omega0Calc(brakegap(n),brakegapSet);        % Natural frequency, given settling time for 2nd order system
%     wn = 1.2165;        % Natural frequency, given settling time for 2nd order system
%     num = [wn^2];
%     den = [1 2*zeta*wn wn^2];
%     G = tf(num,den);                                    % Transfer function for stepper motor response dynamics (rad/s)
% %     ts = tsCalc(brakegap(n),brakegapSet)
%     ts = 4.8;
% 
%     [y,tstepper] = step(G,ts);                           % Step response function
%     dt = tstepper(2) - tstepper(1);                      % Redefine time step
%     
%     brakegap0 = brakegap(n);    % Store brakegap at beginning of deceleration period
%     j = 0;                      % Initialize while loop counter
%     while j < length(tstepper)      % Run until stepper motor settling time is reached
%         n = n + 1;
%         j = j + 1;
%         % Compute brakegap for step input signal
%     %     brakegap(n) = brakegap0 - abs(brakegapSet - brakegap0) * y(j);
%         brakegap(n) = brakegap0 + (brakegapSet - brakegap0) * y(j);
%         % Compute drag forces
% %         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho)/ (1 - eta);
% %         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3)/ (1 - eta);
% %         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1))/ (1 - eta);
%         Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
%         Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3);
%         Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
%         
%         Fdrag = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
%         Fthrust(n) = 0;
%         
%         % Compute kinematics
%         xddot(n) = (Fthrust(n) - Fdrag) / mpod;
%         xdot(n) = xdot(n-1) + xddot(n)*dt;
%         x(n) = x(n-1) + xdot(n)*dt;
%         t(n) = t(n-1) + dt;
%     end
%     
%     %% Generate Deceleration Trajectory - Incremental brake deployment - constrained by total track distance
%     % Notes: this model generates a brake gap table for constrained brakedrag force
%     % Empirical data required for validating model for stepper motor response dynamics.
% 
%     % Record trajectory instances at beginning of deceleration period
% %     Fdrag_aero(n) = Faerodrag(xdot(n-1),rho)/ (1 - eta);
% %     Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3)/ (1 - eta);
% %     Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1))/ (1 - eta);
%     Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
%     Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom*10^3);
%     Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
% 
%     Fdrag = Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n);
%     Fthrust(n) = 0;
% 
%     % Compute kinematics
%     xddot2 = (Fthrust(n) - Fdrag) / mpod;
%     xdot2 = xdot(n-1) + xddot(n)*dt;
%     x2 = x(n-1) + xdot(n)*dt;
%     brakegap2 = brakegap(n);                   % Store brakegap at beginning of deceleration period
%     t2 = t(n-1) + dt;
% 
%     % Pre-compute stepper motor response dynamics (transfer function) for 1mm incremental brakegap deploy
%     zeta = 1.0;                         % Damping coefficient for stepper motor response dynamics
%     ts = 830
% %     wn = omega0Calc(2,1);                 % Natural frequency, given settling time for 2nd order system
%     wn = omega0Calc(ts,zeta);             % Natural frequency, given settling time and zeta for 2nd order system
%     % wn = -log(0.01)/(zeta*ts);      % Natural frequency approx, given settling time for 2nd order system
%     num = [wn^2];
%     den = [1 2*zeta*wn wn^2];
%     G = tf(num,den);                    % Transfer function for stepper motor response dynamics (rad/s)
% %     ts = tsCalc(2,1);
% 
%     % Keep adjusting deltat and recomputing decel trajectory until final velocity is reached below final track distance
%     for deltat = 5:-dt:ts
%     
%         % Initialize dummy arrays
% %         Fdrag_aeroDecel = [Faerodrag(xdot(n-1),rho)] / (1 - eta);
% %         Fdrag_hoverDecel = [Fhoverdrag(xdot(n-1),z_nom*10^3)] / (1 - eta);
% %         Fdrag_brakeDecel = [Fbrakedrag(xdot(n-1),brakegap(n-1))] / (1 - eta);
%         Fdrag_aeroDecel = [Faerodrag(xdot(n-1),rho)];
%         Fdrag_hoverDecel = [Fhoverdrag(xdot(n-1),z_nom*10^3)];
%         Fdrag_brakeDecel = [Fbrakedrag(xdot(n-1),brakegap(n-1))];
%         xddotDecel = [xddot2];
%         xdotDecel = [xdot2];
%         xDecel = [x2];
%         tDecel = [t2];
%         bDecel = [brakegap2];
% 
%         % Deploy brake gap in 1mm increments
%         brakegapSet = brakegap2 - 1;      % Set initial brake gap target to 24mm
% 
%         % Pre-compute brake gap step response
%         [y,tstepper] = step(G, [0:dt:ts] );             % Step response function
%         dt = tstepper(2) - tstepper(1);     % Redefine time step
% 
%         i = 1;                  % Initialize counter
%         j = 0;                  % Initialize counter
%         b0 = brakegap2;
%         while xdotDecel(i) > xdotf
% 
%             i = i + 1;
%             j = j + 1;
% 
%             if( j > length(tstepper) )
%                 j = 1;
%                 b0 = bDecel(i-1);
%                 brakegapSet = bDecel(i-1) - 1;                % Deploy brake gap in 1mm increments
%                 if brakegapSet <= 2.5
%                     brakegapSet = 2.5;
%                 end
%             end
% 
%             bDecel(i) = b0 + (brakegapSet - b0) * y(j);     % Compute brakegap for step input signal
%             if bDecel(i) < 2.5
%                 bDecel(i) = 2.5;
%             end
% 
%             % Compute drag forces
% %             Fdrag_aeroDecel(i) = Faerodrag(xdotDecel(i-1),rho) / (1 - eta);
% %             Fdrag_hoverDecel(i) = Fhoverdrag(xdotDecel(i-1),z_nom*10^3) / (1 - eta);
% %             Fdrag_brakeDecel(i) = Fbrakedrag(xdotDecel(i-1),bDecel(i-1)) / (1 - eta);
%             Fdrag_aeroDecel(i) = Faerodrag(xdotDecel(i-1),rho);
%             Fdrag_hoverDecel(i) = Fhoverdrag(xdotDecel(i-1),z_nom*10^3);
%             Fdrag_brakeDecel(i) = Fbrakedrag(xdotDecel(i-1),bDecel(i-1));
%             
%             Fdrag = ( Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) );
%             
%             % Compute kinematics
%             xddotDecel(i) = -Fdrag/mpod;
%             xdotDecel(i) = xdotDecel(i-1) + xddotDecel(i)*dt;
%             xDecel(i) = xDecel(i-1) + xdotDecel(i)*dt;
%             tDecel(i) = tDecel(i-1) + dt;
%             
%         end
% 
%         if(xdotDecel(i) < xdotf && xDecel(i) <= xf)
%             break;
%         end    
%     
%     end
% 
% %     % If brake actuator settling time was to long
% %     if (xdotDecel(i) < xdotf && xDecel(i) > xf + deltax_dangerzone)
% %         formatSpec = 'Pod crashed at %0.2f m/s \n';
% %         str = sprintf(formatSpec, xdot(n));
% %         fprintf(str)
% %         
% %         crash_counter = crash_counter + 1;
% %     elseif (xdotDecel(i) < xdotf && xDecel(i) >= xf)
% %         formatSpec = 'Pod survived, but overshot target by %0.2f \n';
% %         str = sprintf(formatSpec, xDecel(i) - xf);
% %         fprintf(str)
% %         
% %         survival_counter = survival_counter + 1;
% %     elseif (xdotDecel(i) <= xdotf && xDecel(i) < xf)
% %         formatSpec = 'Pod survived, but undershot target by %0.2f \n';
% %         str = sprintf(formatSpec, xf - xDecel(i));
% %         fprintf(str)
% %         
% %         survival_counter = survival_counter + 1;
% %     end
%     
%     % step(G)
%     
%     % Compute drag forces
%     Fdrag_aero = [Fdrag_aero, Fdrag_aeroDecel];
%     Fdrag_hover = [Fdrag_hover, Fdrag_hoverDecel];
%     Fdrag_brake = [Fdrag_brake, Fdrag_brakeDecel];
%     
%     % Compute kinematics
%     xddot = [xddot, xddotDecel];
%     xdot = [xdot, xdotDecel];
%     x = [x, xDecel];
%     brakegap = [brakegap, bDecel];
%     t = [t, tDecel];
%     

    %% Print overshoot/undershoot results
    
    % Brake actuator settling time
    formatSpec = 'Brake settling time: %0.2f seconds \n';
    str = sprintf(formatSpec, ts);
    fprintf(str)
    formatSpec = 'Nominal brake gap: %0.2f mm \n';
    str = sprintf(formatSpec, brakegapNom);
    fprintf(str)

    % If overshot and crashed
    if (xdot(length(xdot)) < xdotf && x(length(x)) > xf + deltax_dangerzone)
        formatSpec = 'Pod crashed at %0.2f m/s. Top speed %0.2f m/s \n';
        str = sprintf(formatSpec, xdot(n), xdot1);
        fprintf(str)
        
        crash_counter = crash_counter + 1;
    % If survived, but overshot
    elseif (xdot(length(xdot)) <= xdotf && x(length(x)) > xf)
        formatSpec = 'Pod survived, but overshot target by %0.2fm. Top speed %0.2f m/s \n';
        str = sprintf(formatSpec, x(length(x)) - xf, xdot1);
        fprintf(str)
        
        survival_counter = survival_counter + 1;
    % If survived, but undershot
    elseif (xdot(length(xdot)) <= xdotf && x(length(x)) < xf)
        formatSpec = 'Pod survived, but undershot target by %0.2fm. Top speed %0.2f m/s \n';
        str = sprintf(formatSpec, xf - x(length(x)), xdot1);
        fprintf(str)
        
        survival_counter = survival_counter + 1;
    % If survived and right on target
    elseif (xdot(length(xdot)) <= xdotf && x(length(x)) >= xf)
        formatSpec = 'Pod is rignt on target! (Holy shit! Something must be wrong...). Top speed %0.2f m/s \n';
        str = sprintf(formatSpec, x(length(x)) - xf, xdot1);
        fprintf(str)
        
        survival_counter = survival_counter + 1;
    % If something else
    else
        fprintf('null\n')
    end
    
    % if xdotf reached, and target distance reached witin 10m, break loop
    if xdot(length(xdot)) <= xdotf && abs(x(length(x)) - xf) < 10
        break;
    % if xdotf reached, but undershot & pusher-pod separation point was at max pusher distance, then drag forces are too high, break loop 
    elseif xdot(length(xdot)) <= xdotf && xf - x(length(x)) > 5 && x1 >= deltax_pusher_max
        fprintf('Brake response may be too slow or drag forces may be too high.\n')
        break;
    end
    
end
    
    
    %% PID Override (a.k.a. "Delta Charley") Curve
    xOverride = [0];                    % Initialize distance array (m)
    xdotOverride = [120];                 % Initialize velocity array (m/s)
    xddotOverride = [gForce_pusher*g];  % Initialize acceleration array (m/s^2)
    brakegapOverride = [2.5];            % Initialize brake Gap (mm)

    Fdrag_aeroOverride = [Faerodrag(xdotOverride(1),rho)];              % Initialize array
    Fdrag_hoverOverride = [Fhoverdrag(xdotOverride(1),z_nom)];          % Initialize array
    Fdrag_brakeOverride = [Fbrakedrag(xdotOverride(1),brakegapOverride(1))];    % Initialize array

    n = 1;
    while xdotOverride(n) > xdotf
        n = n + 1;

        % Compute brakegap for step input signal
        brakegapOverride(n) = brakegapOverride(n-1);

        % Compute drag forces
        Fdrag_aeroOverride = Faerodrag(xdotOverride(n-1),rho);
        Fdrag_hoverOverride = Fhoverdrag(xdotOverride(n-1),z_nom*10^3);
        Fdrag_brakeOverride = Fbrakedrag(xdotOverride(n-1),brakegapOverride(n-1));
        Fdrag = Fdrag_aeroOverride + Fdrag_hoverOverride + Fdrag_brakeOverride;

        % Compute kinematics
        xddotOverride(n) = -Fdrag/mpod;
        xdotOverride(n) = xdotOverride(n-1) + xddotOverride(n)*dt;
        xOverride(n) = xOverride(n-1) + xdotOverride(n)*dt;
    end

%     v_override = [];
% %     for x_override = linspace(0,xf,length(x));
%     for i = 1:1:length(x)
%         v_override(i) = -3*10^(-7)*(x(i) - x(length(x)) - deltax_dangerzone)^3 - 0.0004*(x(i) - x(length(x)) - deltax_dangerzone)^2 - 0.3467*(x(i) - x(length(x)) - deltax_dangerzone) + 0.7423;
%     end
% %     plot(x,v_override)
    
    %% Time-dependent Trajectory Graphs
    figure
    subplot(411)
    hold on
    plot(t,x)
    plot([0 t(length(t))],[xf xf])
    axis([0 1.2*t(length(t)) 0 xf*1.1])
    grid on
    grid minor
%     title(['Trajectory case no. ' num2str(caseno) ': ' num2str(mpod) 'kg pod | ' num2str(P/1000,4) 'kPa | ' num2str(T,4) 'K | ' num2str(x1,4) 'm @' num2str(gForce_pusher,2) 'g push | ' num2str(xdot1,4) 'm/s max velocity | ' num2str(deltat_cruising,2) 's cruising | ' num2str(gForce_brakedrag,2) 'g brake drag | ' num2str(brakegapNom) 'mm nominal brakegap'])
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(mpod) 'kg pod | ' num2str(P/1000,4) 'kPa | ' num2str(T,4) 'K | ' num2str(x1,4) 'm @' num2str(gForce_pusher,2) 'g push | ' num2str(xdot1,4) 'm/s max velocity | ' num2str(deltat_cruising,2) 's cruising | ' num2str(gForce_brakedrag,2) 'g brake drag | ' num2str(brakegapNom) 'mm final nominal brakegap'])
    ylabel('Distance (m)')
    legend('Pod travel','Target distance');
    
    subplot(412)
    plot(t,xdot)
    axis([0 1.2*t(length(t)) 0 1.2*120])
    grid on
    grid minor
    ylabel('Velocity (m/s)')
    legend('Velocity profile');
    
    subplot(413)
    hold on
    plot(t,xddot/g)
    plot(t,Fthrust/(mpod*g))
    plot(t,-Fdrag_aero/(mpod*g))
    plot(t,-Fdrag_hover/(mpod*g))
    plot(t,-Fdrag_brake/(mpod*g))
    axis([0 1.2*t(length(t)) -1.5 1.5])
    grid on
    grid minor
    ylabel('Acceleration (gs)')
    legend('Total acceleration','Propulsion','Aerodynamic drag','Hover drag','Braking drag');
    
    subplot(414)
    plot(t,brakegap)
    axis([0 1.2*t(length(t)) 0 30])
    grid on
    grid minor
    ylabel('Brake gap (mm)')
    xlabel('time (s)')
    legend('Braking profile');
    
    %% Distance-dependent Trajectory Graphs
    
    figure
    subplot(211)
    hold on
    plot(x,xdot)
    plot(xf + deltax_dangerzone - xdotOverride*4.8,xdotOverride)
%     plot(x,v_override)
    plot([xf xf],[0 120])
    axis([0 1.1*xf 0 120])
    grid on
    grid minor
%     title(['LUT for distance vs target velocity ' num2str(mpod) 'kg pod ' num2str(P/1000) 'kPa ' num2str(T) 'K ' num2str(deltax_pusher) 'm @' num2str(gForce_pusher) 'g push ' num2str(xdot1) 'm/s max velocity ' num2str(deltat_cruising) 's cruising ' num2str(gForce_brakedrag) 'g max drag ' num2str(brakegapNom) 'mm nominal brakegap ' num2str(eta) ' eta'])
%     title(['LUT for distance vs target velocity ' num2str(mpod) 'kg pod ' num2str(P/1000) 'kPa ' num2str(T) 'K ' num2str(deltax_pusher) 'm @' num2str(gForce_pusher) 'g push ' num2str(xdot1) 'm/s max velocity ' num2str(deltat_cruising) 's cruising ' num2str(gForce_brakedrag) 'g max drag ' num2str(brakegapNom) 'mm nominal brakegap '])
    title(['Trajectory case no. ' num2str(caseno) ': ' num2str(mpod) 'kg pod | ' num2str(P/1000,4) 'kPa | ' num2str(T,4) 'K | ' num2str(x1,4) 'm @' num2str(gForce_pusher,2) 'g push | ' num2str(xdot1,4) 'm/s max velocity | ' num2str(deltat_cruising,2) 's cruising | ' num2str(gForce_brakedrag,2) 'g brake drag | ' num2str(brakegapNom) 'mm final nominal brakegap'])
%     formatSpec = 'PID Override Trigger for deltax_{dangerzone} = %0.fm';
    formatSpec = 'PID Override Trigger';
    str = sprintf(formatSpec,deltax_dangerzone);
    legend('PID Setpoint Curve',str,'Target Distance');
    ylabel('Velocity (m/s)')
    
    subplot(212)
    plot(x,brakegap)
    axis([0 1.1*xf 0 30])
    grid on
    grid minor
    ylabel('Brake gap (mm)')
    xlabel('Distance (m)')
    legend('Braking profile');
    
    %% Output Trajectory to csv
    %     header = {'time (s)', 'Distance (m)', 'Velocity (m/s)', 'Acceleration (m/s^2)', 'Aerodrag (gs)', 'Hoverdrag (gs)', 'Brakedrag (gs)', 'brakegap (mm)'};
%     data = [t; x; xdot; xddot; -Fdrag_aero/(mpod*g); -Fdrag_hover/(mpod*g); -Fdrag_brake/(mpod*g); brakegap; eta*ones(size(x)); caseno*ones(size(x))]';
    data = [t; x; xdot; xddot; Fthrust/(mpod*g); -Fdrag_aero/(mpod*g); -Fdrag_hover/(mpod*g); -Fdrag_brake/(mpod*g); brakegap; caseno*ones(size(x))]';

    formatSpec = 'Trajectory for case no. %0.f eta=%0.2f.csv';
%     filename = sprintf(formatSpec,caseno,eta);
    filename = sprintf(formatSpec,caseno);
    csvwrite(filename,data);
    
% end

%% Output Trajectory Header file to csv
formatSpec = 'Trajectory Header for case no. %0.f.csv';
filename = sprintf(formatSpec,caseno);
fid = fopen(filename,'w');
% fprintf(fid, 'time (s)\tDistance (m)\tVelocity (m/s)\tTotal Acceleration (m/s^2)\tPropulsion (gs)\tAerodrag (gs)\tHoverdrag (gs)\tBrakedrag (gs)\tbrakegap (mm)\tRelative Error\tCase No.');
fprintf(fid, 'time (s)\tDistance (m)\tVelocity (m/s)\tTotal Acceleration (m/s^2)\tPropulsion (gs)\tAerodrag (gs)\tHoverdrag (gs)\tBrakedrag (gs)\tbrakegap (mm)\tCase No.');
fclose(fid);

%% Print crash vs survival
% crash_counter
% survival_counter
