function [xi_r, eta_r, lambda_r] = helix(s, R, h, p_0)
% helix - Generates position and derivatives along a 3D helix path
%
% Inputs:
%   s    - arc length along the helix
%   R    - radius of the helix
%   h    - vertical height per revolution
%   p_0  - center offset [3x1]
%
% Outputs:
%   xi_r     - reference position on path [3x1]
%   eta_r    - tangent vector (unit) [3x1]
%   lambda_r - curvature derivative of eta_r [3x1]

c = h / (2*pi);
L = sqrt(R^2 + c^2);           % Length scaling for parameterization

phi = s / L;                   % Angular position

xi_r = p_0 + [R*cos(phi);
              R*sin(phi);
             -c*phi];

eta_r = [ -R*sin(phi)/L;
           R*cos(phi)/L;
          -c/L ];

lambda_r = [ -R*cos(phi)/(R^2 + c^2);
             -R*sin(phi)/(R^2 + c^2);
              0 ];
end
