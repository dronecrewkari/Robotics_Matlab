function u = calOdom(time)
%Calc Input Parameter

T = 10; % [sec]
 
% [V yawrate]
V = 1.0; % [m/s]
omega = 5; % [deg/s]
 
u =[ V*(1-exp(-time/T)) ToRadian(omega)*(1-exp(-time/T))]';
end