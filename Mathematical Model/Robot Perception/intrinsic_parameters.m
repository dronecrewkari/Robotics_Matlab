function [parameters]=intrinsic_parameters(Z,N)
%% learn intrinsic parameters


%% initial 
z_tk=Z.z_tk;
z_max=Z.z_max;
z_tk_star=Z.z_tk_star;   % ca
culate the z_tk_star by ray casting, which operation should be pre-computed
lambda_short=1;
sigma_hit=1;
norm_Z=norm(z_tk);

e_hit=zeros(1,N);
e_short=zeros(1,N);
e_max=zeros(1,N);
e_rand=zeros(1,N);

e_hit_sum=0;
e_short_sum=0;
e_max_sum=0;
e_rand_sum=0;

%% repeat
for i=1:N
    eta=1/(p_hit(z_tk(i),z_tk_star(i),sigma_hit,z_max)+p_short(z_tk(i),z_tk_star(i),lambda_short)+p_max(z_tk(i),z_max)+p_rand(z_tk(i),z_max));
    
    e_hit(i)=eta*p_hit(z_tk(i),z_tk_star(i),sigma_hit,z_max);
    e_short(i)=eta*p_short(z_tk(i),z_tk_star(i),lambda_short);
    e_max(i)=eta*p_max(z_tk(i),z_max)+p_rand(z_tk(i),z_max);
    e_rand(i)=eta*p_rand(z_tk(i),z_max);
    
    e_hit_sum=e_hit(i)+e_hit_sum;
    e_short_sum=e_short(i)+e_short_sum;
    e_max_sum=e_max(i)+e_max_sum;
    e_rand_sum=e_rand(i)+e_rand_sum;
    
    e_hit_zsum=e_hit_(i)*(z_tk(i)-z_tk_star(i))^2+e_hit_zsum;
    e_short_zsum=e_short_(i)*z_tk(i)+e_short_zsum;
    
    
    z_hit=e_hit_sum/norm_Z;
    z_short=e_short_sum/norm_Z;
    z_max=e_max_sum/norm_Z;
    z_rand=e_rand_sum/norm_Z;
    
    sigma_hit=sqrt(1/e_hit_sum*e_hit_zsum);
    lambda_short=e_short_sum/e_short_zsum;
    
    parameters.z_hit=z_hit;
    parameters.z_short=z_short;
    parameters.z_max=z_max;
    parameters.z_rand=z_rand;
    parameters.sigma_hit=sigma_hit;
    parameters.lambda_short=lambda_short;    
end
end


%% local measurement noise ！！normal distribution p_hit
function [p_hit]=p_hit(z_tk,z_tk_star,sigma_hit,z_max)
if z_tk>=0 && z_tk<=z_max
    b=@(z_tk)normpdf(z_tk,z_tk_star,sigma_hit^2);
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
function [p_rand]=p_rand(z_tk,z_max)
if z_tk>=0 && z_tk<=z_max
    p_rand=1/z_max;
else
    p_rand=0;
end
end
