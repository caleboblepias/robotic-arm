%% ============================================================
%   test_full_trajectory.m
%   VALIDATION FOR NEW GEOMETRIC + TIME-SCALED TRAJECTORY SYSTEM
% =============================================================

clc; clear; close all;

fprintf("\n=============================================\n");
fprintf("    TESTING GEOMETRIC + TIME-SCALED TRAJECTORY\n");
fprintf("=============================================\n\n");

%% ------------------------------------------------------------
% 1. Define raw waypoints (coarse)
% ------------------------------------------------------------

% Parameters
R = 10;
theta = linspace(0, 2*pi, 9);
theta(end) = [];     % remove duplicate endpoint

% Circle in XY plane before tilt
X = R * cos(theta);
Y = R * sin(theta);
Z = zeros(size(theta));

% Rotation matrix for 45° tilt about X-axis
alpha = pi/4; % 45 degrees
Rx = [1      0           0;
      0 cos(alpha) -sin(alpha);
      0 sin(alpha)  cos(alpha)];

% Apply rotation to every point
pts = Rx * [X; Y; Z];

% Extract final waypoints
raw_waypoints = pts';    % Nx3

disp(raw_waypoints)




N = size(raw_waypoints,1);
fprintf("Using %d raw waypoints:\n", N);
disp(raw_waypoints);


%% ------------------------------------------------------------
% 2. Parameters for geometric sampling
% ------------------------------------------------------------
n_samples = 200;     % number of geometric samples
dt = 0.01;           % 100 Hz sampling (for v/a estimation)

fprintf("Sampling geometric curve with %d samples...\n", n_samples);


%% ------------------------------------------------------------
% 3. Generate full trajectory using NEW pipeline
% ------------------------------------------------------------
traj = generate_full_trajectory(raw_waypoints, n_samples, dt);

fprintf("Trajectory struct fields:\n");
disp(fieldnames(traj));


%% ------------------------------------------------------------
% 4. Evaluate and log samples
% ------------------------------------------------------------
fprintf("\nSampling final time trajectory:\n");

T_total = traj.t_breaks(end);
ts = linspace(0, T_total, 6);

for t = ts
    [p, v, a] = evaluate_quintic_traj(traj, t);
    fprintf("t = %.2f | p = [%6.2f %6.2f %6.2f]\n", t, p);
end


%% ------------------------------------------------------------
% 5. Visualize final trajectory
% ------------------------------------------------------------
fprintf("\nLaunching 3D trajectory visualization...\n");

plot_quintic_traj(traj);   % you can overlay raw waypoints if desired


fprintf("\n=============================================\n");
fprintf("     NEW TRAJECTORY SYSTEM TEST COMPLETE\n");
fprintf("=============================================\n\n");

spline = bspline_fit(raw_waypoints);
pts = bspline_sample(spline, 30);
disp(pts(1:5,:))

pts(end-10:end, :)
