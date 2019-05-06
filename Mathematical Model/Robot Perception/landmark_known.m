function [q] = landmark_known(fi, xt, m, parameters)
%% initialise
numObser = size(fi, 1);
ri = fi(:, 1);
Phi = fi(:, 2);
si = fi(:, 3);

j = si;

x = xt(1);
y = xt(2);
theta = xt(3);
%theta=xt.theta;

m_x = m(j, 1);
m_y = m(j, 2);
s = m(j, 3);

sigma_r = parameters(1);
sigma_Phi = parameters(2);
sigma_s = parameters(3);
%%
q = 1;
for i = 1:numObser
    r_Lambda = sqrt( (m_x(i) - x)^2 + (m_y(i) - y)^2 );
    Phi_Lambda = atan2(m_y(i) - y, m_x(i) - x) - theta;
    Phi_Lambda = wrapToPi(Phi_Lambda);
    pp = Phi(i) - Phi_Lambda;
    if pp > pi
        pp = pp - pi;
    end
    q = q * prob_distribution(ri(i) - r_Lambda, sigma_r) * prob_distribution(pp, sigma_Phi) * prob_distribution(si(i) - s(i), sigma_s);
end

