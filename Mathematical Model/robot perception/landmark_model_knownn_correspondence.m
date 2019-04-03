function [q]=landmark_model_knownn_correspondence(fi,ci,xt,m,parameters)
%%


ri=fi.ri;
Phi=fi.Phi;
si=fi.si;

x=xt.x;
y=xt.y;
%theta=xt.theta;

m_x=m.m_x;
m_y=m.m_y;
s=m.s;

sigma_r=parameters.sigma_r;
sigma_Phi=parameters.sigma_Phi;
sigma_s=parameters.sigma_s;
%%
j=ci;
r_Lambda=sqrt((m_x(j)-x)^2+(m_y(j)-y)^2);
Phi_Lambda=atan2(m_y(j)-y,m_x(j)-x);
q=prob(ri-r_Lambda,sigma_r)*prob(Phi-Phi_Lambda,sigma_Phi)*prob(si-s(j),sigma_s);


%% prob_distribution(a b_2)
function y=prob(a,b)   
         y=(1/sqrt(2*pi*b))*exp(-1/2*a.^2/b);
         %c=normpdf(a,0,b);
         %pd=makedist('normal',0,b)
         %y=pdf(pd,a)
     end
end

