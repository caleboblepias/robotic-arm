%% ============================================================
%   test_trajectory_generator.m
%   VALIDATION SUITE FOR: generate_quintic_trajectory,
%                         evaluate_quintic_traj,
%                         compute_c2_velocities,
%                         compute_c2_accels,
%                         quintic_segment
%
%   Author: Caleb
% =============================================================

clc; clear; close all;

fprintf("\n=============================================\n");
fprintf("      TESTING TRAJECTORY GENERATOR MODULE\n");
fprintf("=============================================\n\n");

%% ------------------------------------------------------------
% 1. Define SMOOTH 3D S-CURVE WAYPOINTS (NO STOPPING)
% ------------------------------------------------------------

waypoints = [
    0     0     0;
   10     5     2;
   25    15     4;
   40    10     6;
   55    -5     9;
   70   -15    10;
   85   -10     9;
  100     0     7;
];

N = size(waypoints,1);
fprintf("Using %d smooth 3D waypoints:\n", N);
disp(waypoints);

%% ------------------------------------------------------------
% 2. Define time per segment
% ------------------------------------------------------------
% Use durations proportional to distance between waypoints

times = zeros(N-1,1);
for i = 1:N-1
    dist = norm( waypoints(i+1,:) - waypoints(i,:) );
    times(i) = dist / 20;  % assume desired ~20 units/sec
end

fprintf("Segment times (based on distance):\n");
disp(times');


%% ------------------------------------------------------------
% 3. Generate full C2 trajectory
% ------------------------------------------------------------
fprintf("\nGenerating complete C2 trajectory...\n");

traj = generate_quintic_trajectory(waypoints, times);

fprintf("Trajectory structure fields:\n");
disp(fieldnames(traj));


%% ------------------------------------------------------------
% 4. Evaluate trajectory at sample times
% ------------------------------------------------------------

fprintf("\nSampling trajectory at several points:\n");

T_total = traj.t_breaks(end);
ts = linspace(0, T_total, 6);

for t = ts
    [p, v, a] = evaluate_quintic_traj(traj, t);
    fprintf("t = %.2f  |  p = [%6.2f %6.2f %6.2f]\n", t, p);
end


%% ------------------------------------------------------------
% 5. Visualize (1 kHz simulated control loop)
% ------------------------------------------------------------
fprintf("\nLaunching 3D trajectory visualization...\n");

plot_quintic_traj(traj, waypoints);


fprintf("\n=============================================\n");
fprintf("     TRAJECTORY GENERATOR TEST COMPLETE\n");
fprintf("=============================================\n\n");

