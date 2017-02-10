%% Plots response dynamics for hover-engine due to track oscillation
% Dependencies:
% Fhoverlift.m

dt = 0.05;
w_r = .4;
A = 20;             % Track Oscillation in z-dir (mm)
v = 100;
RPM = 2000;
mpod = 350;
N_HE = 8;
g = 9.81;

% Estimations
m = mpod/N_HE;      % load per hover-engine
zeta = 8;
K_HE = (Fhoverlift(v, 12, RPM) - Fhoverlift(v, 13, RPM))/1;	% Hover stiffnesses at nominal hover height
w_HE = sqrt(K_HE/m);
c = 2*zeta*w_HE;	% Hover damping

t = 0:dt:25;

figure
j = 0;
for w_r = logspace(-1, log10(5), 5)
    j = j + 1;
    z = [12];           % Hover-engine height initial conditions (mm)
    zdot = [0];         % Hover-engine vertical velocity initial conditions (mm/s)
    r = [];             % Track initial condition
    for i = 2:1:length(t)
        r(i) = A*sin(w_r*t(i-1));
        F = Fhoverlift(v, z(i-1) - r(i-1), RPM) - c*zdot(i-1) - m*g;
        zddot = F/m;
        zdot(i) = zdot(i-1) + zddot*dt;
        z(i) = z(i-1) + zdot(i)*dt;
    end
    plotno = 510+j;
    subplot(plotno)
    hold on
    plot(t,r)
    plot(t,z)
    formatSpec = 'Input (track oscillation) @%0.2f rad/s';
    str = sprintf(formatSpec,w_r);
    legend(str,'Output (hover-engine oscillation)');
    if j == 1
        title(['Hover-engine vertical response dynamics due to track oscillation - ' num2str(mpod) 'kg pod ' num2str(v) 'm/s cruising velocity ' num2str(RPM) 'RPM ']);
    end
    if j == 3
        ylabel('Vertical Oscillation (mm)')
    end
end
xlabel('Time (s)')
