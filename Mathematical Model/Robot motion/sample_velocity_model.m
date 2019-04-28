function [xt] = sample_velocity_model(ut, xt_1, Delta_t, parameter)
%%initialise
v = ut(1);
w = ut(2);

x = xt_1(1);
y = xt_1(2);
theta = xt_1(3);

alpha_1 = parameter(1);
alpha_2 = parameter(2);
alpha_3 = parameter(3);
alpha_4 = parameter(4);
alpha_5 = parameter(5);
alpha_6 = parameter(6);

xt = zeros(3, 1);

%% 
v_Lamda = v + prob(alpha_1 * v^2 + alpha_2*w^2);
w_Lamda = w + prob(alpha_3 * v^2 + alpha_4 * w^2);
gamma_Lamda = prob(alpha_5 * v^2 + alpha_6 * w^2);

xn = x - v_Lamda/w_Lamda * sin(theta) + v_Lamda/w_Lamda * sin(theta+w_Lamda*Delta_t);
yn = y + v_Lamda/w_Lamda * cos(theta) - v_Lamda/w_Lamda * cos(theta+w_Lamda*Delta_t);
thetan = theta + w_Lamda * Delta_t + gamma_Lamda * Delta_t;

xt(1) = xn;
xt(2) = yn;
xt(3) = thetan;

%% sample_distribution
function [y] = prob(b, varargin) % default: normal
     if nargin == 1
         y = 0;
         for i=1:12
             y= y + rand(-sqrt(b), sqrt(b));
         end
         y = y/2;
     elseif nargin == 2 && strcmp('triangular', varargin{1})
         y = (sqrt(6)/2) * (rand(-sqrt(b), sqrt(b)) + rand(-sqrt(b),sqrt(b)));
     else 
          disp('error distribution');
     end
end
end