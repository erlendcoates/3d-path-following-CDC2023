function x_par = proj_par(x, v)
% proj_par - Projects x parallel to unit vector v
%
% Inputs:
%   x - vector to project
%   v - unit direction vector
%
% Output:
%   x_par - component of x parallel to v

x_par = (v' * x) * v;
end
