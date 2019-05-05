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
v_Lamda = v + sample_distribution(alpha_1 * v^2 + alpha_2 * w^2);
w_Lamda = w + sample_distribution(alpha_3 * v^2 + alpha_4 * w^2);
gamma_Lamda = sample_distribution(alpha_5 * v^2 + alpha_6 * w^2);

xn = x - v_Lamda/w_Lamda * sin(theta) + v_Lamda/w_Lamda * sin(theta + w_Lamda*Delta_t);
yn = y + v_Lamda/w_Lamda * cos(theta) - v_Lamda/w_Lamda * cos(theta + w_Lamda*Delta_t);
thetan = theta + w_Lamda * Delta_t + gamma_Lamda * Delta_t;

xt(1) = xn;
xt(2) = yn;
xt(3) = thetan;

end