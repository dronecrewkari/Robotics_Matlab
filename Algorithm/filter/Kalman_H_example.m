clear all;
close all;
%匀加速直线运动的小车

T=100;       %总时间
%delta_t=0.1;  %采样时间
%t=0:delta_t:T; 

X.A=eye(2);
X.B=2*eye(size(X.A));
X.N=2;
X.Vx=1;

X.R=X.Vx*eye(2);

X.Xt=zeros(X.N,T);  %ground truth
X.Ut=zeros(X.N,T);  %estimate
X.U=ones(X.N,T);    %predict
X.Vt=cell(1,T);    %varience

Z.Vz=1;
Z.H=eye(2);
Z.P=Z.Vz*eye(2);
Z.Zt=zeros(X.N,T+2);

[X.Xt,X.Ut,X.U,X.Vt,Z.Zt]=Kalman_H(X,Z,T);

exp_KF =figure;
plot(X.Xt(1,1:T),X.Xt(2,1:T),'-b',X.Ut(1,1:T),X.Ut(2,1:T),'-r',Z.Zt(1,1:T),Z.Zt(2,1:T),'-g');
title('example of Kalman Filter','FontSize',20,'FontWeight','bold','FontName','Times New Roman');
exp_KF =legend('ground truth','Kalman Filter','measurament');
set(exp_KF,'FontSize',20,'FontWeight','bold','location','NorthWest','FontName','Times New Roman');
xlabel('x (unit:cm)','FontSize',20,'FontWeight','bold','FontName','Times New Roman');
ylabel('y (unit:cm)','FontSize',20,'FontWeight','bold','FontName','Times New Roman');
a=[max(X.Xt(1,1:T));max(X.Ut(1,1:T));max(Z.Zt(1,1:T))];
b=[max(X.Xt(2,1:T));max(X.Ut(2,1:T));max(Z.Zt(2,1:T))];

xlim([0 max(a)]);
ylim([0 max(b)]);
