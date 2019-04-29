% prob_distribution(a b^2)
%default: normal
function y = prob_distribution(a, b, varargin)    
        if nargin == 2
             y = (1/sqrt(2*pi*b)) * exp(-0.5 * a^2 / b);
        elseif nargin == 3 && strcmp('triangular', varargin{1})
             y = max(0, 1/(sqrt(6 * b)) - abs(a)/(6 * b));
        end
end