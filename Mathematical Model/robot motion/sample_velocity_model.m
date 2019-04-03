function [xt]=sample_velocity_model(ut,xt_1,Delta_t)
%% 


%%
v=ut.v;
w=ut.w;

x=xt_1.x;
y=xt_1.y;
theta=xt_1.theta;

%% 
v_Lamda=v+sample_distribution(alpha_1*v^2+alpha_2*w^2);
w_Lamda=w+sample_distribution(alpha_3*v^2+alpha_4*w^2);
gamma_Lamda=sample_distribution(alpha_5*v^2+alpha_6*w^2);

xn=x-v_Lamda/w_Lamda*sin(theta)+v_Lamda/w_Lamda*sin(theta+w_Lamda*Delta_t);
yn=y+v_Lamda/w_Lamda*cos(theta)-v_Lamda/w_Lamda*cos(theta+w_Lamda*Delta_t);
thetan=theta+w_Lamda*Delta_t+gamma_Lamda*Delta_t;

xt.xn=xn;
xt.yn=yn;
xt.thetan=thetan;

%% sample_distribution
function [y]=sample_distribution(b,distribution)
     if distribution=='normal'
         y=0;
         for i=1:12
             y=y+1/2*rand(-sqrt(b),sqrt(b));
         end
     elseif distribution=='triangular'
         y=sqrt(6)/2*(rand(-sqrt(b),sqrt(b))+rand(-sqrt(b),sqrt(b)));
     else 
          disp('error');
     end
end
end