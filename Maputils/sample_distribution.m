%% sample_distribution
function [y] = sample_distribution(b, varargin) % default: normal
     if nargin == 1
         y = 0;
         for i=1:12
             y = y + 2 * sqrt(b) * rand - sqrt(b);
         end
         y = y/2;
     elseif nargin == 2 && strcmp('triangular', varargin{1})  
         
         y = (sqrt(6)/2) * (2 * sqrt(b) * rand - sqrt(b) + 2 * sqrt(b) * rand - sqrt(b));
     else 
          disp('error distribution');
     end
end