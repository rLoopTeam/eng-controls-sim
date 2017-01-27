%%%% Pod Physical Parameters %%%%
mpod = 320.;                                % Total pod mass (kg)
g = 9.81;                                   % Gravitational constant

%Input
caseno = 012;

switch caseno
    %%%% Trajectory & Simulation Constraints for outdoor test track case %%%%
    case 012
        dt = 0.01;                  % time step (s)
        xf = 36;                    % Total track distance (m)
        xdot0 = 0;                  % Initial velocity (m)
        xdotf = 0.01;               % Final velocity at end of controlled braking period (MUST BE > 0) (m/s)
        gForce_pusher = 1.0;        % Pusher acceleration (g's)
        deltax_pusher = 8;          % Desired max push distance (max: 487.67m) (m)
        % vpod_max = 120;             % Desired max velocity (m/s)
        deltat_cruising = 2;        % Cruising time (s)
        gForce_brakedrag = 2.0;       % Max braking force (g's)
        brakegapNom = 24;            % Nominal brake gap during controlled braking phase (mm)
        
        %%%% Pressure %%%%
        % Ppsi = 0.1250;                 
        % Ppsi = 3.7188;
        % Ppsi = 7.3125;
        % Ppsi = 10.9063;
        Ppsi = 14.5;
        
    %%%% Trajectory & Simulation Constraints for full test track %%%%
    case 013
        dt = 0.05;                  % time step (s)
        xf = 1250;                  % Total track distance (m)
        xdot0 = 0;                  % Initial velocity (m)
        xdotf = 0.01;               % Final velocity at end of controlled braking period (MUST BE > 0) (m/s)
        gForce_pusher = 2.0;      % Pusher acceleration (g's)
        deltax_pusher = 350;      % Desired max push distance (max: 487.67m) (m)
        % vpod_max = 120;               % Desired max velocity (m/s)
        deltat_cruising = 4;        % Cruising time (s)
        gForce_brakedrag = 2.0;       % Max braking force (g's)
        brakegapNom = 12;            % Nominal brake gap during controlled braking phase (mm)

        %%%% Pressure %%%%
        Ppsi = 0.1250;                 
        % Ppsi = 3.7188;
        % Ppsi = 7.3125;
        % Ppsi = 10.9063;
        % Ppsi = 14.5;
        
end


%% Using ideal gas law, P = rho*RT, solve for rho 
P = 6894.76*Ppsi;           % PSI to Pa
R = 287.05;                 % Ideal gas constant (J/(kg*K))
T = 293.15;                 % Room temp (K)
rho = P/(R*T);              

%rho = 0.100098;             % Air density inside SpaceX test tube(kg/m^3)
%rho = 1.2754;               % Standard Air density at 20 degC, sealevel(kg/m^3)

%% Simulink Constraints %%
tsamplingperiod = 0.1;            % 10Hz sampling time
% Calculate settling time to reach nominal brakegap
% Extract from table or curve fitting equation (Data needed)
%ts_SM = 0.1;                                    % Settling time for brake actuators to reach within 1% target for 2mm operating range (s)
e_brakegap = 2;                                 % brakegap error input (mm)
ts_SM = e_brakegap*1.2/22.5;

% Transfer function for stepper motor response dynamics linearized at nomimal brakegap operating range
% Notes: transfer function to be used for simulink model
zeta_SM = 1.0;                                  % Damping coefficient for stepper motor response dynamics
omega0_SM = -log(0.01)/(zeta_SM*ts_SM);         % Natural frequency, given settling time for 2nd order system
num = [omega0_SM^2];
den = [1 2*zeta_SM*omega0_SM omega0_SM^2];
G_SM = tf(num,den);                             % Transfer function for stepper motor response dynamics (rad/s)
[y_SM,t_SM] = step(G_SM);                       % Step response function


%% Parameters for hover-engine pitch dynamics w.r.t. CG %%
% r_cx_hf = 0.86;               % Distance between the front hover-engines and CG along x-axis (m)
% r_cx_hfg = 0.53;              % Distance between the front gimballed hover-engines and CG along x-axis (m)
% r_cx_hr = 0.49;               % Distance between the rear hover-engines and CG along x-axis (m)
% r_cx_hrg = 0.82;              % Distance between the rear gimballed hover-engines and CG along x-axis (m)
% 
% % cx_offset = 0.49;
% % r_cx_hf = r_cx_hf + cx_offset;                 % Distance between the front hover-engines and CG along x-axis (m)
% % r_cx_hfg = r_cx_hfg + cx_offset;                % Distance between the front gimballed hover-engines and CG along x-axis (m)
% % r_cx_hr = r_cx_hr - cx_offset;                 % Distance between the rear hover-engines and CG along x-axis (m)
% % r_cx_hrg = r_cx_hrg - cx_offset;               	% Distance between the rear gimballed hover-engines and CG along x-axis (m)
% 
% r_cz_b = 0.59;                              % Distance between the magnetic brake pads and CG along z-axis (m)
% %r_cz_b = 0.21;                              % Distance between the magnetic brake pads and CG along z-axis (m)
% r_cz_t = 0.92;                              % Distance between the gimballed engines and CG along z-axis (m)
% l_h_h = 0.33;                               % Distance between front to front-gimballed hover engine, or rear to rear-gimballed hover engine, along x-axis (m)
% l_hf_hr = r_cx_hf + r_cx_hr;                % Distance between front and rear most hover engine along x-axis (m)
% I_c = 264.98;                               % Moment of inertia about y-axis at CG (kg*m^2), acquired from https://docs.google.com/spreadsheets/d/1uq3ggIZIbpyu-hNNmua2wM4QM-iMK1fV9sYTyJLEBbo/edit?usp=sharing
% 
% %%%% Hover-engine parameters %%%%
% RPM_he = 2000;                                  % Hover engine rpm
% k_rel1 = 0.089501;                              % Constant relating rotational and translational velocity with hover force
% k_rpm = 0.0093200503;                           % Constant relating rpm to velocity
% k_rel = atan(k_rel1*(vpod_max+k_rpm*RPM_he));   % Proportionality constant scaling hover force based on RPM & pod velocity
% %k_h = 91615.3;                                 % Approximate "hovering stiffness" for 2x hover-engine in parallel @10mm hover height (N/m) (see: https://docs.google.com/spreadsheets/d/1-igstFK75UQpAWvdLb4wZ0dIGP5MEFUwVvNvnC70Wyg/edit)
HE_load = mpod*9.81/8;                          % Load per hover engine (N)
z_nom = 0.031*exp(-0.0024*HE_load);                     % Nominal hover height (m) based on pod mass and 8 hover engines
% %z_nom = 0.01207;                               % Nominal hover height (m) based on pod mass (320kg) and 8 hover engines
% %z_nom = 0.008;                                 % Nominal hover height (m) based on pod mass (320kg) and 8 hover engines
% k_h1 = 1237.395;                                % Constant used in nonlinear hover force equation (see: https://docs.google.com/spreadsheets/d/1-igstFK75UQpAWvdLb4wZ0dIGP5MEFUwVvNvnC70Wyg/edit)
% k_h2 = -100.324;                                % Constant used in nonlinear hover force equation (see: https://docs.google.com/spreadsheets/d/1-igstFK75UQpAWvdLb4wZ0dIGP5MEFUwVvNvnC70Wyg/edit)
% k_h = -2*k_h1*k_h2*exp(k_h2*z_nom)*k_rel;       % k_h = -dF/dz = k_1*k_2*e^(k_2*z_nom), Linearized "hovering stiffness", for 2x hover-engine in parallel, normalized about nominal hover height(N/m) (see: https://docs.google.com/spreadsheets/d/1-igstFK75UQpAWvdLb4wZ0dIGP5MEFUwVvNvnC70Wyg/edit)
% %c = 0.;                                        % Oscillatory damping coefficient of ArxPax hover-engine for 10mm hover height (Ns/m)
% 
% %%%% Estimations %%%
% zetaHE = 0.7;                               % Hover engine vertical damping ratio (assumed underdamped)
% c = zetaHE*2*(mpod/4*k_h/4)^0.5;            % Estimated Hover engine vertical damping coeff

