close all; clear; clc;

%% Reference Path Definition
R = 200; 
h = 100;
p_0 = [0; 0; 0];
ref = @(s) helix(s, R, h, p_0);

%% Simulation Parameters
V_a = 18;            % Airspeed [m/s]
v_w = [10; 0; 0];    % Wind [m/s]
K_norm = 10;         % Normalization gain

simTime = 100;       % Total time [s]
dt = 1/20;           % Time step [s]
N = simTime / dt;    % Number of steps

%% Initial Conditions
s_r_0 = 0;             
xi_0 = p_0;
eta_a_0 = [-1; 0; 0];
x_0 = [s_r_0; xi_0; eta_a_0];

%% Allocate Memory
x = zeros(7, N+1);     % State: [s_r; xi; eta_a]
u = zeros(4, N);       % Input: [V_r; a_a]
eta_d = zeros(3, N);
eta_a_d = zeros(3, N);

xi_tilde_perp_norm = zeros(1, N);
xi_tilde_par       = zeros(1, N);
eta_tilde_norm     = zeros(1, N);

x(:,1) = x_0;
t = 0:dt:simTime;

%% ODE Definition
odefun = @(x,u) [u(1); secondOrderKinematicsWind(x(2:7), u(2:4), V_a, v_w, K_norm)];

%% Main Simulation Loop
for i = 1:N
    s_r_i = x(1,i);
    xi_i = x(2:4,i);
    eta_a_i = x(5:7,i) / norm(x(5:7,i)); % Normalize

    [xi_r_i, eta_r_i, lambda_r_i] = ref(s_r_i);
    [V_r, a_a, eta_d_i, eta_a_d_i] = pathFollowingController(xi_i, eta_a_i, V_a, v_w, xi_r_i, eta_r_i, lambda_r_i);

    u(:,i) = [V_r; a_a];
    eta_d(:,i) = eta_d_i;
    eta_a_d(:,i) = eta_a_d_i;

    xi_tilde = xi_i - xi_r_i;
    xi_tilde_perp_norm(i) = norm((eye(3) - eta_r_i * eta_r_i') * xi_tilde);
    xi_tilde_par(i) = eta_r_i' * xi_tilde;
    eta_tilde_norm(i) = norm(eta_a_i - eta_a_d_i);

    [~, x_] = ode45(@(t, y) odefun(x(:,i), u(:,i)), [t(i) t(i+1)], x(:,i));
    x(:, i+1) = x_(end,:)';
end

%% Post-processing
s_r   = x(1,:);
xi    = x(2:4,:);
eta_a = x(5:7,:);
v     = V_a * eta_a + v_w;
V     = vecnorm(v, 2, 1);
eta   = v ./ V;
a_lat = u(2:4,:);
a_norm = vecnorm(a_lat, 2, 1);

% Initialize
xi_r = zeros(3, N);
eta_r_vec = zeros(3, N);
theta = zeros(1, N);
gamma = zeros(1, N);
chi = zeros(1, N);
gamma_a = zeros(1, N);
chi_a = zeros(1, N);
gamma_d = zeros(1, N);
chi_d = zeros(1, N);
gamma_a_d = zeros(1, N);
chi_a_d = zeros(1, N);
gamma_r = zeros(1, N);
chi_r = zeros(1, N);

for i = 1:N
    [xi_r_i, eta_r_i, ~] = ref(s_r(i));
    xi_r(:,i) = xi_r_i;
    eta_r_vec(:,i) = eta_r_i;

    theta(i) = acos(dot(eta_a(:,i), eta_a_d(:,i)) / (norm(eta_a(:,i)) * norm(eta_a_d(:,i))));
    
    gamma(i)     = asin(-eta(3,i));
    chi(i)       = atan2(eta(2,i), eta(1,i));
    gamma_a(i)   = asin(-eta_a(3,i));
    chi_a(i)     = atan2(eta_a(2,i), eta_a(1,i));
    gamma_d(i)   = asin(-eta_d(3,i));
    chi_d(i)     = atan2(eta_d(2,i), eta_d(1,i));
    gamma_a_d(i) = asin(-eta_a_d(3,i));
    chi_a_d(i)   = atan2(eta_a_d(2,i), eta_a_d(1,i));
    gamma_r(i)   = asin(-eta_r_vec(3,i));
    chi_r(i)     = atan2(eta_r_vec(2,i), eta_r_vec(1,i));
end

%% Plots

% 3D Path
f = figure;
plot3(xi_r(1,:), xi_r(2,:), xi_r(3,:), 'b'); hold on;
plot3(xi(1,:), xi(2,:), xi(3,:), 'r', 'LineWidth', 2); hold off;
f.CurrentAxes.ZDir = 'reverse';
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid on;
view(-240.6, 18.6); % same as original
print('figures/helix', '-depsc');

% Acceleration, Position Error, Heading Error
figure;
subplot(3,1,1);
plot(t(1:end-1), a_norm);
ylabel('$\|a_a^\perp\|$ [m/s$^2$]', 'Interpreter', 'latex');
grid on;

subplot(3,1,2);
plot(t(1:end-1), xi_tilde_perp_norm);
ylabel('$\|\Pi_{\eta_r}\tilde{\xi}\|$ [m]', 'Interpreter', 'latex');
grid on;

subplot(3,1,3);
plot(t(1:end-1), theta * 180/pi);
ylabel('$\theta = \arccos(\eta_a^\top \eta_a^d)$ [deg]', 'Interpreter', 'latex');
xlabel('Time [s]');
grid on;
print('figures/plots', '-depsc');

% Speed and Longitudinal Error
figure;
subplot(2,1,1);
plot(t(1:end-1), u(1,:), 'DisplayName', 'V_r'); hold on;
plot(t, V, 'DisplayName', 'V');
plot(t, V_a * ones(size(t)), 'DisplayName', 'V_a'); hold off;
legend;
ylabel('Speed [m/s]');
grid on;

subplot(2,1,2);
plot(t(1:end-1), xi_tilde_par);
xlabel('Time [s]');
ylabel('$\eta_r^\top \tilde{\xi}$ [m]', 'Interpreter', 'latex');
grid on;
print('figures/V_r_xi_tilde_par', '-depsc');

% Azimuth and Elevation Angles
figure;
subplot(2,1,1);
plot(t(1:end-1), chi * 180/pi); hold on;
plot(t(1:end-1), chi_a * 180/pi);
plot(t(1:end-1), chi_d * 180/pi);
plot(t(1:end-1), chi_a_d * 180/pi);
plot(t(1:end-1), chi_r * 180/pi); hold off;
legend('\chi', '\chi_a', '\chi_d', '\chi_a^d', '\chi_r');
ylabel('Azimuth angle [deg]');
grid on;

subplot(2,1,2);
plot(t(1:end-1), gamma * 180/pi); hold on;
plot(t(1:end-1), gamma_a * 180/pi);
plot(t(1:end-1), gamma_d * 180/pi);
plot(t(1:end-1), gamma_a_d * 180/pi);
plot(t(1:end-1), gamma_r * 180/pi); hold off;
legend('\gamma', '\gamma_a', '\gamma^d', '\gamma_a^d', '\gamma_r');
ylabel('Elevation angle [deg]');
xlabel('Time [s]');
grid on;
print('figures/AzimuthElevation', '-depsc');


