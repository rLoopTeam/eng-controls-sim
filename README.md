# eng-controls-sim

# Installation

    cmd: 'git clone https://github.com/capsulecorplab/eng-controls-sim.git'

    Install Matlab and Simulink (code was developed using 2015a)

# Run Trajectory Simulation (Matlab)

    1. Configure simulation parameters in 'simParameters.m'

	create new case or change 'caseno' to desired case to use its corresponding simulation parameters

	e.g.,

    ```
    caseno = 48

    ...

        case 048    % Full test track case at 1.0g with skis 10mm hover height without instantaneous braking
            mpod = 441.;                 % Total pod mass (kg)
            dt = 0.01;                   % time step (s)
            xf = 1250;                   % Target distance (m)
            xdotf = 0.01;                % Target final velocity at xf (m/s)
            gForce_pusher = 1.0;         % Pusher acceleration (g's)
            deltax_pusher_max = 487.68;  % Max push distance (max: 487.68m or 1600ft) (m)
            deltat_pusher = 10;          % Desired max push distance (max: 487.68m or 1600ft) (m)
            deltat_cruising = 5.2;         % Cruising time between pusher and deceleration phase (minimum 2s required) (s)
            brakegapNom = 2.5;           % Nominal brake gap during controlled braking phase (mm)
            deltax_dangerzone = 50;      % Distance between final target and end of track (DANGER ZONE!!!) (m)
            z_nom = 0.010;               % Nominal hover height (m) based on pod mass and 8 hover engines
            ski_option = false;          % Enables/disables addition of skis
            instant_braking = false;      % true = brakes reach nominal brakegap instantaneously
            PIDcontroller = false;       % true = brakes actuators use PID controller to adjust trajectory

            %%%% Pressure %%%%
            Ppsi = 0.4;              % Atmospheric air pressure inside SpaceX test tube (Psi)
            
            % Using ideal gas law, P = rho*R*T, solve for rho 
            P = 6894.76*Ppsi;           % Atmospheric air pressure inside SpaceX test tube (Pa)
            R = 287.05;                 % Ideal gas constant (J/(kg*K))
            T = 293.15;                 % Atmospheric air temperature inside SpaceX test tube (K)
            rho = P/(R*T);              % Air density inside SpaceX test tube(kg/m^3)

            %%%% Relative Error (eta is positive for under-estimated case; negative for over-estimated case)%%%%
            eta_aerodrag = 0.0;        % Estimated aerodynamic drag relative error
            eta_hoverdrag = 0.0;       % Estimated hover-engine drag relative error
            eta_brakedrag = 0.0;       % Estimated brake drag relative error
            eta_skidrag = 0.0;         % Estimated ski drag relative error
    ```

    2. Run 'Trajectory.m' to generate scenario 

# Run Controlled Trajectory Simulation (Simulink)

    1. Generate Simulation Parameters and Setpoint Tables (Velocity & Position) from running 'simParameters.m' and 'Trajectory.m'
    
    2. Run 'GainScheduledPIDTuner.m' to generate Theoretical Gains linearized by breakpoint table
    
    3. Run 'GainScheduledPIDBrakingSystem.mdl' to visualize response dynamics
