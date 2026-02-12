function traj = generate_full_trajectory(raw_waypoints, n_samples, dt)
% GENERATE_FULL_TRAJECTORY
%   Full pipeline for generating a smooth, time-scaled trajectory.
%
%   Uses Catmull–Rom spline for geometric curve interpolation.
%   (B-spline version retained but commented out.)
%
%   Inputs:
%       raw_waypoints : Nx3 waypoints
%       n_samples     : number of geometric samples
%       dt            : sampling timestep
%
%   Output:
%       traj : complete trajectory struct


%% ------------------------------------------------------------
% 1. Create geometric curve
% ------------------------------------------------------------

% === Option A: B-spline (NOT recommended — does NOT interpolate endpoints) ===
% spline = bspline_fit(raw_waypoints);
% pts    = bspline_sample(spline, n_samples);

% === Option B: Catmull–Rom (RECOMMENDED — interpolates all points) ===
spline = catmull_rom_fit(raw_waypoints);
pts    = catmull_rom_sample(spline, n_samples);

disp("FIRST 10 SAMPLED POINTS:");
disp(pts(1:10,:));

disp("LAST 10 SAMPLED POINTS:");
disp(pts(end-9:end,:));

%% ------------------------------------------------------------
% 2. Compute velocities & accelerations from geometric curve
% ------------------------------------------------------------
v = calc_velocities(pts, dt);
a = calc_accelerations(pts, dt);


%% ------------------------------------------------------------
% 3. Build time vector for trajectory segments
% ------------------------------------------------------------
times = ones(n_samples - 1, 1) * dt;


%% ------------------------------------------------------------
% 4. Generate full piecewise quintic trajectory
% ------------------------------------------------------------
traj = generate_quintic_trajectory(pts, v, a, times);


end
