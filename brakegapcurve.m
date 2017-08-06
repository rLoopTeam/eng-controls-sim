%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author: dr. briefs
% date: 2017/8/5
%
% purpose: generates brakegap curve based on brake actuator response dynamics
% 
% input: initial brakegap (mm), target brakegap (mm)
% output: brakegap curve t vs mm (array)
%
% notes: 
% -response dynamics assumes...
% -load conditions
% -constant max velocity of 6mm/s
% -constant acceleration of 4mm/s^2
% -code output undershoots target within 0.0533%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     ts = 4.8;     % Estimated settling time for full brake deployment
% b0 = 10;
% bset = 5;
% dt = 0.001;

function [t,b] = brakegapcurve(b0,bset,dt)
    deltab = bset - b0;
    bddotmax = 4.0;
    bdotmax = 6.0;

    n = 1;
    b = [b0];
    bdot = [0];
    t = [0];
    
    % If required brakegap travel is greater than time needed to accelerate and decelerate to/from max velocity:
    if abs(deltab) > bdotmax^2/bddotmax
        %% Generate acceleration phase
%         while bdotmax^2/(2*bddotmax) - abs(b(n)) > 0.001
        while abs(bdotmax - bdot(n)) > 0

            n = n + 1;
            % Calculate speed
            bdot(n) = bdot(n-1) + bddotmax*dt;

            if abs(bdot(n)) > abs(bdotmax)
                bdot(n) = bdotmax;
            end

            % Calculate position
            b(n) = b(n-1) + bdot(n-1)*dt + 0.5*bddotmax*dt^2;
            if deltab < 0
                b(n) = b(n-1) - bdot(n-1)*dt - 0.5*bddotmax*dt^2;
            end

            t(n) = t(n-1) + dt;

        end
        
        % Create timestamp
        n1 = n;
        b1 = b(n);
        bdot1 = bdot(end);
        t1 = t(end);
        
        %% Calculate required coast period
        deltab_coast = abs(deltab) - 2*abs(b(end) - b(1));

        %% Generate coast phase
        while deltab_coast - abs(b(n) - b1) > 0.001*deltab_coast
            n = n + 1;

            % Calculate speed
            bdot(n) = bdot(n-1);

            % Calculate position
            b(n) = b(n-1) + bdot(n-1)*dt;
            if deltab < 0
                b(n) = b(n-1) - bdot(n-1)*dt;
            end

            t(n) = t(n-1) + dt;

        end
        
        % Create timestamp
        b2 = b(end);
        t2 = t(end);
        
        %% Generate deceleration phase
        while t(n) - t2 < t1
            n = n + 1;

            % Calculate speed
            bdot(n) = bdot(n-1) - bddotmax*dt;
            if bdot(n) < 0
                bdot(n) = 0;
            end

            % Calculate position
            b(n) = b(n-1) + bdot(n-1)*dt + 0.5*bddotmax*dt^2;
            if deltab < 0
                b(n) = b(n-1) - bdot(n-1)*dt - 0.5*bddotmax*dt^2;
            end

            t(n) = t(n-1) + dt;
        end
        
    %% Travel distance too short to accelerate to max speed    
    else
        %% Generate acceleration phase
        % Compute new max speed based on required travel distance
        bdotmax = sqrt(abs(deltab)*bddotmax);
        while bdotmax - abs(bdot(n)) > 0.001* abs(bdotmax)

            n = n + 1;
            % Calculate speed
            bdot(n) = bdot(n-1) + bddotmax*dt;

            if abs(bdot(n)) > abs(bdotmax)
                bdot(n) = bdotmax;
            end

            % Calculate position
            b(n) = b(n-1) + bdot(n-1)*dt + 0.5*bddotmax*dt^2;
            if deltab < 0
                b(n) = b(n-1) - bdot(n-1)*dt - 0.5*bddotmax*dt^2;
            end

            t(n) = t(n-1) + dt;

        end
        t1= t(end);
        %% Generate deceleration phase
        while t(n) < 2*t1
            n = n + 1;

            % Calculate speed
            bdot(n) = bdot(n-1) - bddotmax*dt;
            if bdot(n) < 0
                bdot(n) = 0;
            end

            % Calculate position
            b(n) = b(n-1) + bdot(n-1)*dt + 0.5*bddotmax*dt^2;
            if deltab < 0
                b(n) = b(n-1) - bdot(n-1)*dt - 0.5*bddotmax*dt^2;
            end

            t(n) = t(n-1) + dt;
        end
        
    end
    
end
%     figure(1)
%     plot(t,b)
%     figure(2)
%     plot(t,bdot)
