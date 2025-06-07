function x_perp = proj_perp(x, v)
% proj_perp - Projects x orthogonally to unit vector v
%
% Inputs:
%   x - vector to project
%   v - unit direction vector
%
% Output:
%   x_perp - component of x orthogonal to v

x_perp = x - proj_par(x, v);
end
