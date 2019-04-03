%kalman filter written by herry
function [Xt,Ut,U,Vt,Zt]=Kalman_H( X,Z,T)

A=X.A; %transition matrix
B=X.B; %conrol matrix
N=X.N; %the number of statement
Vx=X.Vx; %the varience in noise process
Xt=X.Xt; %the varience in noise process
Ut=X.Ut;
U=X.U;
Vt=X.Vt;

U0=X.U;  %N*T
R=X.R;

Un=0;
Vn=0;
V0=eye(N);
V=eye(N);
Vt{1}=V0;

Vz=Z.Vz;
H=Z.H;
P=Z.P;

C=normrnd(Un,Vx,[2 T]);
Q=normrnd(Vn,Vz,[2 T]);


Zt=Z.Zt;
x=eye(N,T+2);
u=ones(N,T+2);

for i=2:T
    Xt(:,i)=A*x(:,i-1)+B*u(:,i)+C(:,i);
    x(:,i)=Xt(:,i);
    Zt(:,i)=H*x(:,i)+Q(:,i);    
end

for i=2:T
    U(:,i)=A*U0(:,i-1)+B*u(:,i);
    V=A*V0*A'+R;
    
    K=V*H'/(H*V*H'+P);
    
    Ut(:,i)=U(:,i)+K*(Zt(:,i)-H*U(:,i));
    Vt{i}=(eye(N)-K*H)*V;
    
    U0(:,i)=Ut(:,i);
    V0=Vt{i};
end

return; % bye, bye!!!!
    