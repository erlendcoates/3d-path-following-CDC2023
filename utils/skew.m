function S = skew(a)
% skew - Returns the skew-symmetric matrix of a 3x1 vector a
% 
% Input:
%   a - 3x1 vector
%
% Output:
%   S - 3x3 skew-symmetric matrix such that S*b = cross(a,b)

S = [   0    -a(3)   a(2);
      a(3)     0   -a(1);
     -a(2)   a(1)    0 ];
end