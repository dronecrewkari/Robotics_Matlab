function u = calOdom(vt, omegat, time)
%Calc Input Parameter

T = 10; % [sec]
 
% [V yawrate]
V = vt; % [m/s]
omega = omegat; % [deg/s]
 
u =[ V*(1-exp(-time/T)) ToRadian(omega)*(1-exp(-time/T))]';
end