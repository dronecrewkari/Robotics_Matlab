function [q]=likelihood_field_model(zt,xt,m)
%%
q=1;       
N=zt.N;     
z_k=zt.z_k;     % 
z_max=zt.z_max;
z_hit=zt.z_hit;
z_rand=zt.z_rand;

x=xt.x;
y=xt.y;
theta=xt.theta;
x_sens=xt.x_sens;
y_sens=xt.y_sens;
theta_sens=xt.theta_sens;

x_dot=m.x_dot;
y_dot=m.y_dot;

x_zt=zeros(1,N);
y_zt=zeros(1,N);

for k=1:N
    if z_k(k)~=z_max
        x_zt(k)=x+x_sens(k)*cos(theta)-y_sens(k)*sin(theta)+z_k(k)*cos(theta+theta_sens(k));
        y_zt(k)=y+y_sens(k)*cos(theta)+x_sens(k)*sin(theta)+z_k(k)*sin(theta+theta_sens(k));
        dist=min(sqrt((x_zt(k)-x_dot)^2+(y_zt(k)-y_dot)^2)|dot(x_dot,y_dot));
        q=q*(z_hit*prob(dist,sigma_hit,'normal')+z_rand/z_max);
    end
end

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

