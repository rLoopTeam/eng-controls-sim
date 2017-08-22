%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% filename: miscalc_cases.m 
% author: dr. briefs
% date: 2017/8/21
%
% purpose:
% sets simulation parameters and generates graphs for 81 possible 
% scenarios for all combinations of relative error per each drag force
%
% Dependencies:
% Trajectory.m
% Fdrag.m
% Fbrake_lift.m
% brakeactuator.m
%
% Note: Fload_brakes neglects magnetic load due to force of brakes acting on eachother.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear

%%%% Pod Physical Parameters %%%%
g = 9.81;                                   % Gravitational constant

% Input case no. (enter previous case no.)
caseno = 549;	% low pressure case at 1.0g with skis (Updated brake actuator dynamics)

crash_counter = 0;
sim_counter = 0;

%%%% Trajectory & Simulation Constraints %%%%
%%%% Relative Error (eta is positive for under-estimated case; negative for over-estimated case)%%%%
for eta_aerodrag = -0.15:0.15:0.15          % relative error for aerodynamic drag
    for eta_hoverdrag = -0.15:0.15:0.15       % relative error for hover-engine drag 
        for eta_brakedrag = -0.15:0.15:0.15   % relative for brake drag
            for eta_skidrag = -0.15:0.15:0.15 % Estimated ski drag relative error
                
                caseno = caseno + 1;

                mpod = 441.;                % Total pod mass (kg)
                dt = 0.01;                 % time step (s)
                xf = 1250;                  % Target distance (m)
                xdotf = 0.01;               % Target final velocity at xf (m/s)
                gForce_pusher_max = 1.0;        % Pusher acceleration (g's)
        %         deltax_pusher = 312;     % Desired max push distance (max: 487.68m or 1600ft) (m)
                deltat_jerk = 0.3;          % jerk time for pusher to ramp to full acceleration (s)
                deltax_pusher_max = 487.68; % Max push distance (max: 487.68m or 1600ft) (m)
        %         vpod_max = 90.;            % Constraint on max velocity (m/s)
                deltat_pusher = 10;     % Desired max push distance (max: 487.68m or 1600ft) (m)
                deltat_cruising = 4.5;        % Cruising time between pusher and deceleration phase (minimum 2s required) (s)
        %         gForce_brakedrag = 1.0;     % Constraint on max braking force (g's)
                brakegapNom = 5;          % Nominal brake gap during controlled braking phase (mm)
                deltax_dangerzone = 50;     % Distance between final target and end of track (DANGER ZONE!!!) (m)
                z_nom = 0.012;              % Nominal hover height (m) based on pod mass and 8 hover engines
                ski_option = true;          % Enables/disables addition of skis
                instant_braking = false;     % true = brakes reach nominal brakegap instantaneously
                PIDcontroller = false;       % true = brakes actuators use PID controller to adjust trajectory

                %%%% Pressure %%%%
                Ppsi = 0.4;              % Atmospheric air pressure inside SpaceX test tube (Psi)
        %         Ppsi = 14.7;              % Atmospheric air pressure for outdoor SpaceX test track (Psi)

                % Using ideal gas law, P = rho*R*T, solve for rho 
                P = 6894.76*Ppsi;           % Atmospheric air pressure inside SpaceX test tube (Pa)
                R = 287.05;                 % Ideal gas constant (J/(kg*K))
                T = 293.15;                 % Atmospheric air temperature inside SpaceX test tube (K)
                rho = P/(R*T);              % Air density inside SpaceX test tube(kg/m^3)
        %         rho = 0.100098;             % Air density inside SpaceX test tube(kg/m^3)
        %         rho = 1.2754;               % Standard Air density at 20 degC, sealevel(kg/m^3)

                Trajectory
                
                sim_counter = sim_counter + 1;
                if x(end) >= xf + deltax_dangerzone
                    crash_counter = crash_counter + 1;
                end
            end
        end
    end
end

formatSpec = '%0.f out of %0.f cases crashed!\n';
str = sprintf(formatSpec, crash_counter, sim_counter);
fprintf(str,'\n')

