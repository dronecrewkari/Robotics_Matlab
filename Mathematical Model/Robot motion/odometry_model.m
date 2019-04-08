function [p]=odometry_model(xt,ut,xt_1,alpha,distribution)
%% description
% intput:
%        xt                 pose at time t
%        ut                 odometry [xt-1;xt]
%        xt_1               pose at time t-1
%        alpha              four error parameters 
%        distribution       normal or triangular
%        p                  combine error probability              
%% initial
% time t-1 pose
x=xt_1.x;
y=xt_1.y;
theta=xt_1.theta;

% time t pose
xn=xt.x;
yn=xt.y;
thetan=xt.theta;

% odometry(xt-1,xt)
u=[ut(1);ut(2)];

x_=u(1).x;
y_=u(1).y;
theta_=u(1).theta;

x_n=u(2).x;
y_n=u(2).y;
theta_n=u(2).theta;

%error parameters
alpha_1=alpha.alpha_1;
alpha_2=alpha.alpha_2;
alpha_3=alpha.alpha_3;
alpha_4=alpha.alpha_4;
%% odometry model

delta_rot1=atan2(y_n-y_,x_n-x_)-theta_;
delta_trans=sqrt((x_-x_n)^2+(y_-y_n)^2);
delta_rot2=theta_n-theta_-delta_rot1;

delta_rot1_Lambda=atan2(yn-y,xn-x)-theta;
delta_trans_Lambda=sqrt((x-xn)^2+(y-yn)^2);
delta_rot2_Lambda=thetan-theta-delta_rot1_Lambda;

p1=prob(delta_rot1-delta_rot1_Lambda,alpha_1*(delta_rot1_Lambda^2)+alpha_2*(delta_trans_Lambda^2),distribution);
p2=prob(delta_trans-delta_trans_Lambda, alpha_3*(delta_trans_Lambda^2)+alpha_4*(delta_rot1_Lambda^2)+alpha_4*(delta_rot2_Lambda^2),distribution);
p3=prob(delta_rot2-delta_rot2_Lambda,alpha_1*(delta_rot2_Lambda^2)+alpha_2*(delta_trans_Lambda^2),distribution);

p=p1*p2*p3;

%% distribution
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







