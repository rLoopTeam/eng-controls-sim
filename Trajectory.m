% brakingTrajectory.m generates arrays for desired target velocity,
% 'velocitySet', a distance, 'distanceSet'
% for GainScheduledPIDBrakingSystem.mdl
%
% Notes: 
% Need to acquire experimental data from stepper motors for modeling
% actual response dynamics, such as damping, phase lag, and time delay.
% Trajectory calcs do not account for aero-drag, hover engine drag.
% References Fdrag.m

%% Generate Simulation Constraints
simParameters

%% Initiaize crash vs survival counter
crash_counter = 0;
survival_counter = 0;

%% Compute multiple trajectory cases for -50% to +50% relative error, where relative = (actual value - approximate value)/(actual value)
% Fdrag_actual = Fdrag_approx/(1 - eta)
for eta = -0.50:0.05:0.50
    %% Simulation initial conditions
    t = [0];                    % Initialize time array (s)
    x = [0];                    % Initialize distance array (m)
    xdot = [0];                 % Initialize velocity array (m/s)
    xddot = [gForce_pusher*g];  % Initialize acceleration array (m/s^2)
    brakegap = [25];            % Initialize brake Gap (mm)

    Fdrag_aero = [Faerodrag(xdot(1),rho)];              % Initialize array
    Fdrag_hover = [Fhoverdrag(xdot(1),z_nom)];          % Initialize array
    Fdrag_brake = [Fbrakedrag(xdot(1),brakegap(1))];    % Initialize array

    %% Generate Trajectory profile for Pusher Phase
    n = 1;
    % while xdot < vpod_max %        % pusher phase constrained by velocity
    while x(n) < deltax_pusher     % pusher phase constrained by distance
        n = n + 1;
        % Compute drag forces
        Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
        Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom);
        Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
        Fdrag = ( Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) ) / (1 - eta);

        % Compute kinematics
        xddot(n) = gForce_pusher*g - Fdrag/mpod;
        xdot(n) = xdot(n-1) + xddot(n)*dt;
        x(n) = x(n-1) + xdot(n)*dt;
        brakegap(n) = brakegap(n-1);
        t(n) = t(n-1) + dt;
    end
    
    %% Generate Trajectory profile for Cruising Phase
    t1 = t(n);                              % Store time Stamp
    x1 = x(n);                              % pod distance at beginning of cruising phase (m/s)
    xdot1 = xdot(n);                        % pod velocity at beginning of cruising phase (m/s)
    while t(n) < (t1 + deltat_cruising)     % Cruising phase constrained by time
        n = n + 1;
        % Compute drag forces
        Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
        Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom);
        Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
        Fdrag = ( Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) ) / (1 - eta);
        % Compute kinematics
        xddot(n) = -Fdrag/mpod;
        xdot(n) = xdot(n-1) + xddot(n)*dt;
        x(n) = x(n-1) + xdot(n)*dt;
        brakegap(n) = brakegap(n-1);
        t(n) = t(n-1) + dt;
    end

    %% Generate Trajectory profile for Brake Engagement to Nominal Brake Gap Phase

    % Compute brakegap setpoint constrained within nominal brake gap and max brakedrag
    % brakegapSet = brakegapCalc(gForce_brakedrag*g*mpod - Fhoverdrag(xdot(n-1),z_nom), xdot(n-1));   % Nominal Brake Gap constrained at a max brake force minus hoverdrag force to avoid overpitching
    % if brakegapSet < brakegapNom
        brakegapSet = brakegapNom;
    % end

    % Calculate settling time to reach nominal brakegap
    % Extract from table or curve fitting equation (Data needed)

    % Transfer function for stepper motor response dynamics during 25mm to nomimal brakegap engagement phase
    zeta = 1.0;                                          % Damping coefficient for stepper motor response dynamics
    omega0 = omega0Calc(brakegap(n),brakegapSet);                 % Natural frequency, given settling time for 2nd order system
    num = [omega0^2];
    den = [1 2*zeta*omega0 omega0^2];
    G = tf(num,den);                                       % Transfer function for stepper motor response dynamics (rad/s)
    ts = tsCalc(brakegap(n),brakegapSet);

    [y,tstepper] = step(G,[0:0.01:ts]);                               % Step response function
    dt = tstepper(2) - tstepper(1);                                 % Redefine time step

    brakegap0 = brakegap(n);    % Store brakegap at beginning of deceleration period
    j = 0;                      % Initialize while loop counter
    while j < length(tstepper)      % Run until stepper motor settling time is reached
        n = n + 1;
        j = j + 1;
        % Compute brakegap for step input signal
    %     brakegap(n) = brakegap0 - abs(brakegapSet - brakegap0) * y(j);
        brakegap(n) = brakegap0 + (brakegapSet - brakegap0) * y(j);
        % Compute drag forces
        Fdrag_aero(n) = Faerodrag(xdot(n-1),rho);
        Fdrag_hover(n) = Fhoverdrag(xdot(n-1),z_nom);
        Fdrag_brake(n) = Fbrakedrag(xdot(n-1),brakegap(n-1));
        Fdrag = ( Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) ) / (1 - eta);
        % Compute kinematics
        xddot(n) = -Fdrag/mpod;
        xdot(n) = xdot(n-1) + xddot(n)*dt;
        x(n) = x(n-1) + xdot(n)*dt;
        t(n) = t(n-1) + dt;
    end

    %% Generate Deceleration Trajectory - Incremental brake deployment - constrained by total track distance
    % Notes: this model generates a brake gap table for constrained brakedrag force
    % Empirical data required for validating model for stepper motor response dynamics.

    % Record trajectory instances at beginning of deceleration period
    Fdrag_aero2 = Faerodrag(xdot(n-1),rho);
    Fdrag_hover2 = Fhoverdrag(xdot(n-1),z_nom);
    Fdrag_brake2 = Fbrakedrag(xdot(n-1),brakegap(n-1));
    Fdrag = ( Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) ) / (1 - eta);

    xddot2 = -Fdrag/mpod;
    xdot2 = xdot(n-1) + xddot(n)*dt;
    x2 = x(n-1) + xdot(n)*dt;
    brakegap2 = brakegap(n);                   % Store brakegap at beginning of deceleration period
    t2 = t(n-1) + dt;

    % Pre-compute stepper motor response dynamics (transfer function) for 1mm incremental brakegap deploy
    zeta = 1.0;                         % Damping coefficient for stepper motor response dynamics
    omega0 = omega0Calc(2,1);                 % Natural frequency, given settling time for 2nd order system
    % omega0 = -log(0.01)/(zeta*ts);      % Natural frequency approx, given settling time for 2nd order system
    num = [omega0^2];
    den = [1 2*zeta*omega0 omega0^2];
    G = tf(num,den);                    % Transfer function for stepper motor response dynamics (rad/s)
    ts = tsCalc(2,1);

    % Keep adjusting deltat and recomputing decel trajectory until final velocity is reached below final track distance
    for deltat = 5:-dt:ts
    
        % Initialize dummy arrays
        Fdrag_aeroDecel = [Faerodrag(xdot(n-1),rho)];
        Fdrag_hoverDecel = [Fhoverdrag(xdot(n-1),z_nom)];
        Fdrag_brakeDecel = [Fbrakedrag(xdot(n-1),brakegap(n-1))];
        xddotDecel = [xddot2];
        xdotDecel = [xdot2];
        xDecel = [x2];
        tDecel = [t2];
        bDecel = [brakegap2];

        % Deploy brake gap in 1mm increments
        brakegapSet = brakegap2 - 1;      % Set initial brake gap target to 24mm

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
                if brakegapSet < 2.5
                    brakegapSet = 2.5;
                end
            end

            bDecel(i) = b0 + (brakegapSet - b0) * y(j);     % Compute brakegap for step input signal

            % Compute drag forces
            Fdrag_aeroDecel(i) = Faerodrag(xdotDecel(i-1),rho);
            Fdrag_hoverDecel(i) = Fhoverdrag(xdotDecel(i-1),z_nom);
            Fdrag_brakeDecel(i) = Fbrakedrag(xdotDecel(i-1),bDecel(i-1));
            Fdrag = ( Fdrag_aero(n) + Fdrag_hover(n) + Fdrag_brake(n) ) / (1 - eta);
            % Compute kinematics
            xddotDecel(i) = -Fdrag/mpod;
            xdotDecel(i) = xdotDecel(i-1) + xddotDecel(i)*dt;
            xDecel(i) = xDecel(i-1) + xdotDecel(i)*dt;
            tDecel(i) = tDecel(i-1) + dt;

        end

        if(xdotDecel(i) < xdotf && xDecel(i) <= xf)
            break;
        end    
    
    end

    % If brake actuator settling time was to long
    if (xdotDecel(i) < xdotf && xDecel(i) > xf)
        formatSpec = 'Pod crashed for eta = %0.2f\n';
        str = sprintf(formatSpec,eta);
        fprintf(str)
        
        crash_counter = crash_counter + 1;
    elseif (xdotDecel(i) <= xdotf && xDecel(i) <= xf)
        formatSpec = 'Pod survived for eta = %0.2f\n';
        str = sprintf(formatSpec,eta);
        fprintf(str)
        
        survival_counter = survival_counter + 1;
    end    
    
    % step(G)
    
    % Compute drag forces
    Fdrag_aero = [Fdrag_aero, Fdrag_aeroDecel];
    Fdrag_hover = [Fdrag_hover, Fdrag_hoverDecel];
    Fdrag_brake = [Fdrag_brake, Fdrag_brakeDecel];
    % Compute kinematics
    xddot = [xddot, xddotDecel];
    xdot = [xdot, xdotDecel];
    x = [x, xDecel];
    brakegap = [brakegap, bDecel];
    t = [t, tDecel];
    
    %% Time-dependent Trajectory Graphs
    figure
    subplot(411)
    plot(t,x)
    grid on
    grid minor
    %title(['Trajectory ' num2str(mpod) 'kg pod ' num2str(rho) 'kg/m^3 air density ' num2str(gForce_pusher) 'g pusher ' num2str(vpod_max) 'm/s max velocity ' num2str(deltat_cruising) 's cruising ' num2str(gForce_brakedrag) 'g brake drag ' num2str(brakegapNom) 'mm nominal brakegap'])
    title(['Trajectory ' num2str(mpod) 'kg pod ' num2str(P/1000) 'kPa ' num2str(T) 'K ' num2str(deltax_pusher) 'm @' num2str(gForce_pusher) 'g push ' num2str(xdot1) 'm/s max velocity ' num2str(deltat_cruising) 's cruising ' num2str(gForce_brakedrag) 'g brake drag ' num2str(brakegapNom) 'mm nominal brakegap ' num2str(eta) ' eta'])
    ylabel('Distance (m)')
    
    subplot(412)
    plot(t,xdot)
    grid on
    grid minor
    ylabel('Velocity (m/s)')
    
    subplot(413)
    hold on
    plot(t,xddot/g)
    plot(t,-Fdrag_aero/(mpod*g))
    plot(t,-Fdrag_hover/(mpod*g))
    plot(t,-Fdrag_brake/(mpod*g))
    grid on
    grid minor
    legend('Total Acceleration','Aerodrag','Hoverdrag','Brakedrag');
    ylabel('Acceleration (gs)')
    
    subplot(414)
    plot(t,brakegap)
    grid on
    grid minor
    ylabel('brakegap (mm)')
    xlabel('time (s)')
    
    %% Distance-dependent Trajectory Graphs
    figure
    subplot(211)
    plot(x,xdot)
    grid on
    grid minor
    title(['LUT for distance vs target velocity ' num2str(mpod) 'kg pod ' num2str(P/1000) 'kPa ' num2str(T) 'K ' num2str(deltax_pusher) 'm @' num2str(gForce_pusher) 'g push ' num2str(xdot1) 'm/s max velocity ' num2str(deltat_cruising) 's cruising ' num2str(gForce_brakedrag) 'g max drag ' num2str(brakegapNom) 'mm nominal brakegap ' num2str(eta) ' eta'])
    ylabel('Velocity (m/s)')
    
    subplot(212)
    plot(x,brakegap)
    grid on
    grid minor
    ylabel('Brakegap (m)')
    xlabel('Distance (m)')
    
    %% Output Trajectory to csv
%     header = {'time (s)', 'Distance (m)', 'Velocity (m/s)', 'Acceleration (m/s^2)', 'Aerodrag (gs)', 'Hoverdrag (gs)', 'Brakedrag (gs)', 'brakegap (mm)'};
    data = [t; x; xdot; xddot; -Fdrag_aero/(mpod*g); -Fdrag_hover/(mpod*g); -Fdrag_brake/(mpod*g); brakegap; eta*ones(size(x))]';
    
    formatSpec = 'Trajectory for eta=%0.2f.csv';
    filename = sprintf(formatSpec,eta);
    csvwrite(filename,data);

end

%% Output Trajectory Header file to csv
formatSpec = 'Trajectory Header.csv';
filename = sprintf(formatSpec,eta);
fid = fopen(filename,'w');
fprintf(fid, 'time (s)\tDistance (m)\tVelocity (m/s)\tAcceleration (m/s^2)\tAerodrag (gs)\tHoverdrag (gs)\tBrakedrag (gs)\tbrakegap (mm)\tRelative Error');
fclose(fid);

%% Print crash vs survival
crash_counter
survival_counter
