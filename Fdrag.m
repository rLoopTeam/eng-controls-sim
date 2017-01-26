% Input: brakegap (mm), velocity (m/s)
% Output: drag force (N)
function F = Fdrag(brakegap,v)
% Drag force
	if v < 8
		% A34 Eddy brake constants for <8 m/s
		a = -175.92*exp(-0.21369*brakegap);
		b = 3050.8*exp(-0.21398*brakegap);
		c = 0;
    elseif v > 30
		% A34 Eddy brake constants for >30 m/s
		a = 0.24153*exp(-0.21654*brakegap);
		b = -78.783*exp(-0.21665*brakegap);
		c = 12950*exp(-0.21708*brakegap);
    else
		% A34 Eddy brake constants for 8-30 m/s
		a = -3.1692*exp(-0.28239*brakegap);
		b = -9.2684*exp(-0.05037*brakegap);
		c = 13507*exp(-0.21091*brakegap);
    end
	F = a*v^2 + b*v + c;
    
end
