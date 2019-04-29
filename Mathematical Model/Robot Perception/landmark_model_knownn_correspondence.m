function [q] = landmark_model_knownn_correspondence(fi, xt, m, parameters)
%% initialise
ri = fi(1);
Phi = fi(2);
si = fi(3);

j = si;

x = xt(1);
y = xt(2);
%theta=xt.theta;

m_x = m(1, j);
m_y = m(2, j);
s = m(3, j);

sigma_r = parameters(1);
sigma_Phi = parameters(2);
sigma_s = parameters(3);
%%

r_Lambda = sqrt( (m_x - x)^2 + (m_y - y)^2 );
Phi_Lambda = atan2(m_y - y, m_x - x);
q = prob_distribution(ri - r_Lambda, sigma_r) * prob_distribution(Phi - Phi_Lambda, sigma_Phi) * prob_distribution(si - s, sigma_s);

end

