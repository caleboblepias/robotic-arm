function plot_quintic_traj(traj, waypoints)
% PLOT_QUINTIC_TRAJ_1KHZ
%   Runs a 1 kHz simulation of the trajectory and visualizes it.
%
%   Inputs:
%       traj: trajectory struct
%       waypoints: Nx3 points (optional)

%% ========= Simulation Parameters ==========
dt = 0.001;     % 1 kHz control loop
T_total = traj.t_breaks(end);
N = floor(T_total / dt);

%% ========= Pre-allocate ===================
P = zeros(N,3);
V = zeros(N,3);
A = zeros(N,3);

%% ========= Run 1 kHz Simulation ===========
t = 0;

for k = 1:N
    [P(k,:), V(k,:), A(k,:)] = evaluate_quintic_traj(traj, t);
    t = t + dt;
end

%% ========= Create 3D Figure ================
figure; hold on; grid on; axis equal;

% Draw entire smooth trajectory
plot3(P(:,1), P(:,2), P(:,3), 'b-', 'LineWidth', 2);

% Plot waypoints as reference
if nargin > 1
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
end

% Animated marker
h = plot3(P(1,1), P(1,2), P(1,3), 'bo', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'b');

xlabel('X'); ylabel('Y'); zlabel('Z');
title('1 kHz Trajectory Execution');

view(3);

%% ========= Animate (not at 1 kHz!) ==========
visualize_rate = 20; % visualize at 50 Hz (every 20 ms)
step = round(visualize_rate / (1000 * dt));

for k = 1:step:N
    set(h, 'XData', P(k,1), 'YData', P(k,2), 'ZData', P(k,3));
    drawnow;
end

hold off;

end
