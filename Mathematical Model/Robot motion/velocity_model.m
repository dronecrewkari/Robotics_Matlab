function [p] = velocity_model(xt,ut,xt_1,Delta_t,alpha,distribution)
%% algorithm motion_model_veloctiy


%% inial parameter

%time t-1 pose
x=xt_1.x;         
y=xt_1.y;         
theta=xt_1.theta; 

%time t pose
xn=xt.x;         
yn=xt.y;         
thetan=xt.theta; 

%velocity vector
v=ut.v;          
w=ut.w;   

%error parameters
alpha_1=alpha.alpha_1;
alpha_2=alpha.alpha_2;
alpha_3=alpha.alpha_3;
alpha_4=alpha.alpha_4;
alpha_5=alpha.alpha_5;
alpha_6=alpha.alpha_6;

%% motion model velocity
u=1/2*((x-xn)*cos(theta)+(y-yn)*sin(theta))/((y-yn)*cos(theta)-(x-xn)*sin(theta));

x_star=(x+xn)/2+u*(y-yn);           %circle pose x*
y_star=(y+yn)/2+u*(x-xn);           %circle pose y*
r_star=sqrt((x-x_star)^2+(y-y_star)^2); %circle radius r*

Delta_theta=atan2(yn-y_star,xn-x_star)-atan2(y-y_star,x-x_star);   %change of heading direction

v_Lamda=Delta_theta/Delta_t*r_star;   
w_Lamda=Delta_theta/Delta_t;
gamma_Lamda=(thetan-theta)/Delta_t-w_Lamda;

p=prob(v-v_Lamda,alpha_1*v^2+alpha_2*w^2,distribution)*prob(w-w_Lamda,alpha_3*v^2+alpha_4*w^2,distribution)*prob(gamma_Lamda,alpha_5*v^2+alpha_6*w^2,distribution);

%% prob_distribution(a b_2)
function y=prob(a,b,distribution)
     if distribution=='normal'
         y=(1/sqrt(2*pi*b))*exp(-1/2*a.^2/b);
         %c=normpdf(a,0,b);
         %pd=makedist('normal',0,b)
         %y=pdf(pd,a)
     elseif distribution=='triangular'
         y=max(0,1/(sqrt(6*b))-abs(a)/(6*b));
     end
end
end
