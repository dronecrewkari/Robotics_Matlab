function [q]=beam_range_finder_model(zt,xt,K,parameters)
%%
z_tk=zt.z_tk;
z_tk_star=zt.z_tk_star;     % caculate the z_tk_star by ray casting, which operation should be pre-computed

sigma_hit=parameters.sigma_hit;
lambda_short=parameters.lambda_short;
z_hit=parameters.z_hit;
z_short=parameters.z_short;
z_max=parameters.z_max;
z_rand=parameters.z_rand;

q=ones(1,K);
q(1)=1;
%%
for k = 1:K
    p=z_hit*p_hit(z_tk(k),z_tk_star(k),sigma_hit,z_max)+z_short*p_short(z_tk,z_tk_star(k),lambda_short)+z_max*p_max(z_tk(k),z_max)+z_rand*p_rand(z_tk(k),z_max);
    q(k)=q(k)*p;
end

%% local measurement noise ！！normal distribution p_hit
function [p_hit]=p_hit(z_tk,z_tk_star,sigma_hit,z_max)
if z_tk>=0 && z_tk<=z_max
    b = @(z_tk)normpdf(z_tk,z_tk_star,sigma_hit^2);
    a=1/integral(b,0,z_max);
    p_hit=a*normpdf(z_tk,z_tk_star,sigma_hit^2);
else
    p_hit=0;
end
end

%% unexpected objects ！！ exponential distribution
function [p_short]=p_short(z_tk,z_tk_star,lambda_short)
if z_tk>=0 && z_tk<=z_tk_star
    eta=1/(1-exp(-lambda_short*z_tk_star));
    p_short=eta*lambda_short*exp(-lambda_short*z_tk);
else
    p_short=0;
end
end

%% Failures
function [p_max]=p_max(z_tk,z_max)
if z_tk==z_max
    p_max=1;
else
    p_max=0;
end
end

%% Random measurements
function [p_rand] = p_rand(z_tk,z_max)
if z_tk>=0 && z_tk<=z_max
    p_rand=1/z_max;
else 
    p_rand=0;
end
end

end






