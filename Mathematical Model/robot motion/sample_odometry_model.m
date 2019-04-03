function [xt]=sample_odometry_model(ut,xt_1)
%%
% time t-1 pose
x=xt_1.x;
y=xt_1.y;
theta=xt_1.theta;

% odometry(xt-1,xt)
u=[ut(1);ut(2)];

x_=u(1).x;
y_=u(1).y;
theta_=u(1).theta;

x_n=u(2).x;
y_n=u(2).y;
theta_n=u(2).theta;

%%
delta_rot1=atan2(y_n-y_,x_n-x_)-theta_;
delta_trans=sqrt((x_-x_n)^2+(y_-y_n)^2);
delta_rot2=theta_n-theta_-delta_rot1;

delta_rot1_Lambda = delta_rot1-sample_distribution(alpha_1*(delta_rot1^2)+alpha_2*(delta_trans^2),distribution);
delta_trans_Lambda = delta_trans-sample_distribution(alpha_3*(delta_trans^2)+alpha_4*(delta_rot1^2)+alpha_4*(delta_rot2^2),distribution);
delta_rot2_Lambda=delta_rot2-sample_distribution(alpha_1*(delta_rot2^2)+alpha_2*(delta_trans^2),distribution);

xn=x+delta_trans_Lambda*cos(theta+delta_rot1_Lambda);
yn=y+delta_trans_Lambda*sin(theta+delta_rot1_Lambda);
thetan=theta+delta_rot1_Lambda+delta_rot2_Lambda;

xt.x=xn;
xt.y=yn;
xt.theta=thetan;

%%
%% sample_distribution
function [y] = sample_distribution(b,distribution)
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



