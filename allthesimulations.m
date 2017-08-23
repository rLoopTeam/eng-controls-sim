%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% filename: allthesimulations.m 
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

%% Pod Physical Parameters %%
g = 9.81;                                   % Gravitational constant

% Input starting case no.
caseno = 550;	% low pressure case at 1.0g with skis (Updated brake actuator dynamics)

simtable = table();     % Initialize sim table
tldrsimtable = table(); % Initialize tl;dr sim table

crash_counter = 0;      % Initialize crash counter
sim_counter = 0;        % Initialize sim counter

% xf_nominal = 0;
% xf_max = 0;
% xf_min = 999999999999;
% xdotf_nominal = 0;
% xdotf_max = 0;
% xdotf_min = 999999999999;
% xddotf_nominal = 0;
% xddotf_max = 0;
% xddotf_min = 999999999999;
% 
% Fdrag_aero_nominal = 0;
% Fdrag_aero_max = 0;
% Fdrag_aero_min = 999999999999;
% Fdrag_hover_nominal = 0;
% Fdrag_hover_max = 0;
% Fdrag_hover_min = 999999999999;
% Fdrag_brake_nominal = 0;
% Fdrag_brake_max = 0;
% Fdrag_brake_min = 999999999999;
% Fdrag_ski_nominal = 0;
% Fdrag_ski_max = 0;
% Fdrag_ski_min = 999999999999;
% Fthrust_nominal = 0;
% Fthrust_max = 0;
% Fthrust_min = 999999999999;
% Fdrag_net_nominal = 0;
% Fdrag_net_max = 0;
% Fdrag_net_min = 999999999999;
% Fload_brakes_nominal = 0;
% Fload_brakes_max = 0;
% Fload_brakes_min = 999999999999;

%% Nested 'for loop' for 81 Possible Trajectory Simulations %%
%%%% Relative Error (eta is positive for under-estimated case; negative for over-estimated case)%%%%
caseno_start = caseno;  % store starting caseno
caseno = caseno - 1;    % subtract 1 to account for loop counter

eta_aerodrag_nominal = 0;
eta_aerodrag_min = -0.15;
eta_aerodrag_max = 0.15;
eta_hoverdrag_nominal = 0;
eta_hoverdrag_min = -0.15;
eta_hoverdrag_max = 0.15;
eta_brakedrag_nominal = 0;
eta_brakedrag_min = -0.15;
eta_brakedrag_max = 0.15;
eta_skidrag_nominal = 0;
eta_skidrag_min = -0.15;
eta_skidrag_max = 0.15;

for eta_aerodrag = [eta_aerodrag_min, eta_aerodrag_nominal, eta_aerodrag_max]   % relative error for aerodynamic drag
    for eta_hoverdrag = [eta_hoverdrag_min, eta_hoverdrag_nominal, eta_hoverdrag_max] 	% relative error for hover-engine drag 
        for eta_brakedrag = [eta_brakedrag_min, eta_brakedrag_nominal, eta_brakedrag_max]   % relative for brake drag
            for eta_skidrag = [eta_skidrag_min, eta_skidrag_nominal, eta_skidrag_max]	% Estimated ski drag relative error
                
                %% Set sim parameters
                caseno = caseno + 1;

                mpod = 441.;                % Total pod mass (kg)
                dt = 0.01;                 % time step (s)
                xf = 1250;                  % Target distance (m)
                xdotf = 0.01;               % Target final velocity at xf (m/s)
                gForce_pusher_max = 1.0;        % Pusher acceleration (g's)
                deltat_jerk = 0.3;          % jerk time for pusher to ramp to full acceleration (s)
                deltax_pusher_max = 487.68; % Max push distance (max: 487.68m or 1600ft) (m)
                deltat_pusher = 10;     % Desired max push distance (max: 487.68m or 1600ft) (m)
                deltat_cruising = 4.5;        % Cruising time between pusher and deceleration phase (minimum 2s required) (s)
                brakegapNom = 5;          % Nominal brake gap during controlled braking phase (mm)
                deltax_dangerzone = 50;     % Distance between final target and end of track (DANGER ZONE!!!) (m)
                z_nom = 0.012;              % Nominal hover height (m) based on pod mass and 8 hover engines
                hover_option = true;          % Enables/disables hover-engines
                ski_option = true;          % Enables/disables skis
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
                
                %% Run sim case
                Trajectory
                
                %% Tally crash counter
                sim_counter = sim_counter + 1;
                if x(end) >= xf + deltax_dangerzone
                    crash_counter = crash_counter + 1;
                end
                
                %% create data entry summary for the case we just ran
                data = table(caseno, eta_aerodrag, eta_hoverdrag, eta_brakedrag, eta_skidrag, x(end), max(xdot), max(xddot), max(-Fdrag_aero/(mpod*g)), max(-hover_option*Fdrag_hover/(mpod*g)), max(-ski_option*Fdrag_ski/(mpod*g)), max(-Fdrag_brake/(mpod*g)), max(100*Fdrag_aero./Fdrag_net), max(100*hover_option*Fdrag_hover./Fdrag_net), max(100*ski_option*Fdrag_ski./Fdrag_net), max(100*Fdrag_brake./Fdrag_net), max(Fload_brakes));
                simtable = [simtable; data];

                if eta_aerodrag == eta_aerodrag_nominal && eta_hoverdrag == eta_hoverdrag_nominal && eta_brakedrag == eta_brakedrag_nominal && eta_skidrag == eta_skidrag_nominal
                    %% save nominal case in tl;dr sim table
                    tldrsimtable = [tldrsimtable; data];
                end
                
                %% save under estimation case in tl;dr sim table
                if eta_aerodrag == eta_aerodrag_max && eta_hoverdrag == eta_hoverdrag_max && eta_brakedrag == eta_brakedrag_max && eta_skidrag == eta_skidrag_max
                    tldrsimtable = [tldrsimtable; data];
                end
                
                %% save over estimation case in tl;dr sim table
                if eta_aerodrag == eta_aerodrag_min && eta_hoverdrag == eta_hoverdrag_min && eta_brakedrag == eta_brakedrag_min && eta_skidrag == eta_skidrag_min
                    tldrsimtable = [tldrsimtable; data];
                end
            end
        end
    end
end

%% Print Summary

header = {'caseno', 'eta_aerodrag', 'eta_hoverdrag', 'eta_brakedrag', 'eta_skidrag', 'x_travel', 'xdot_max', 'xddot_max', 'aerodrag_max', 'hoverdrag_max', 'skidrag_max', 'brakedrag_max', 'aerodrag_contribution', 'hoverdrag_contribution', 'skidrag_contribution', 'brakedrag_contribution', 'Fload_brakes_max'};

% Save sim table to csv
fprintf('Saving sim table to csv... \n')
simtable.Properties.VariableNames = header;
formatSpec = 'simtable_case_no_%0.f-%0.f.csv';
filename = sprintf(formatSpec,caseno_start,caseno);
writetable(simtable,filename,'Delimiter',',')

% Save tl;dr sim table to csv
fprintf('Saving tl;dr sim table to csv... \n')
tldrsimtable.Properties.VariableNames = header;
formatSpec = 'tldr_simtable_case_no_%0.f-%0.f.csv';
filename = sprintf(formatSpec,caseno_start,caseno);
writetable(tldrsimtable,filename,'Delimiter',',')

fprintf('Printing summary... \n')
tldrsimtable

formatSpec = '%0.f out of %0.f cases crashed!\n';
str = sprintf(formatSpec, crash_counter, sim_counter);
fprintf(str,'\n')

%%
