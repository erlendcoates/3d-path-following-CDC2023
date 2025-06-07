function dxdt = secondOrderKinematicsWind(x, u, V_a, v_w, K_norm)
% secondOrderKinematicsWind - Dynamics of second-order kinematic model with wind
%
% Inputs:
%   x      - state vector [xi; eta_a] (6x1)
%   u      - lateral acceleration input (3x1)
%   V_a    - airspeed (scalar)
%   v_w    - wind vector (3x1)
%   K_norm - gain to enforce unit-norm of eta_a
%
% Output:
%   dxdt   - time derivative of state [dot(xi); dot(eta_a)]

xi_dot     = V_a * x(4:6) + v_w;
eta_a_dot  = u / V_a + K_norm * (1 - norm(x(4:6))^2) * x(4:6);

dxdt = [xi_dot; eta_a_dot];
end