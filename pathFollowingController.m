function [V_r, a_a, eta_d, eta_a_d] = pathFollowingController(xi, eta_a, V_a, v_w, xi_r, eta_r, lambda_r)
% pathFollowingController
% Computes path-following control inputs and guidance vectors for 3D path
%
% Inputs:
%   xi       - vehicle position [3x1]
%   eta_a    - air-relative heading (unit vector) [3x1]
%   V_a      - airspeed (scalar)
%   v_w      - wind vector [3x1]
%   xi_r     - reference position on the path [3x1]
%   eta_r    - reference tangent (unit vector) [3x1]
%   lambda_r - path curvature scalar
%
% Outputs:
%   V_r      - reference progression speed along path
%   a_a      - commanded lateral acceleration (normal to eta_a)
%   eta_d    - desired inertial heading (unit vector)
%   eta_a_d  - desired air-relative heading (unit vector)

% --- Tuning parameters ---
k_1     = 20;        % Gain for parallel correction
k_2     = 0.01;      % Gain for perpendicular guidance
k_eta   = 0.025;     % Heading control gain
Delta_1 = 50;        % Saturation limit

% --- Velocity and position error ---
v         = V_a * eta_a + v_w;             % Ground-relative velocity
xi_tilde  = xi - xi_r;                     % Position error

% --- Reference speed computation (with tanh saturation) ---
xi_tilde_par = eta_r' * xi_tilde;
V_r = eta_r' * v + Delta_1 * tanh(k_1 * xi_tilde_par / Delta_1);

% --- Position error derivative ---
xi_tilde_dot = v - V_r * eta_r;

% --- Desired inertial heading (eta_d) ---
kappa = eta_r - k_2 * proj_perp(xi_tilde, eta_r);
eta_d = kappa / norm(kappa);

% --- Desired air-relative heading (eta_a_d) ---
V_w     = norm(v_w);
vw_proj = v_w' * eta_d;
V_d     = vw_proj + sqrt(vw_proj^2 + V_a^2 - V_w^2);
eta_a_d = (V_d * eta_d - v_w) / V_a;

% --- Derivative of eta_d (needed for a_a) ---
eta_r_dot   = lambda_r * V_r;
Omega_r     = cross(eta_r, eta_r_dot);              % Angular rate of eta_r
Omega_r_x   = skew(Omega_r);                        % Skew matrix

% Compute derivative of kappa (chain rule)
kappa_dot = eta_r_dot ...
          - k_2 * proj_perp(xi_tilde_dot, eta_r) ...
          - k_2 * (eta_r * eta_r' * Omega_r_x - Omega_r_x * eta_r * eta_r') * xi_tilde;

% Project and normalize to get eta_d_dot
eta_d_dot = proj_perp(kappa_dot, eta_d) / norm(kappa);

% --- Derivative of eta_a_d (for dynamic heading tracking) ---
V_d_dot     = V_d * (v_w' * eta_d_dot) / sqrt(vw_proj^2 + V_a^2 - V_w^2);
eta_a_d_dot = (V_d * eta_d_dot + V_d_dot * eta_d) / V_a;

% --- Lateral acceleration control input ---
a_a = V_a^2 * k_eta * proj_perp(eta_a_d, eta_a) ...
    - V_a * cross(eta_a, cross(eta_a_d, eta_a_d_dot));

end
