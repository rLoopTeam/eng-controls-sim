b=[];
t=[];
for tout=0:0.1:5
[bout,ts]=brakeactuator(23,5,tout);
b=[b,bout];
t=[t,tout];
end
plot(t,b)
ts