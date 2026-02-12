%% ChatGPT 5.1 Generated Test Case

%% ============================================================
%   TRAJECTORY TEST MENU (Interactive Path Selection)
% =============================================================

clear; close all; clc;

fprintf("\n=============================================\n");
fprintf("         TRAJECTORY TESTING SYSTEM\n");
fprintf("=============================================\n\n");

fprintf("Select a path type:\n");
fprintf("  1) Tilted Circle\n");
fprintf("  2) Helix\n");
fprintf("  3) Figure-8 Curve\n");
fprintf("  4) 3D S-Curve\n");
fprintf("  5) Trefoil Knot (complex)\n\n");

choice = input("Enter your choice (1–5): ");

%% -------------------------------
% 1. Generate geometric path
% -------------------------------
N = 400;   % number of points along geometric curve

switch choice
    case 1
        fprintf("Generating Tilted Circle...\n");
        params.R = 10;
        params.tilt = pi/4;
        path = geometric_curve('tilted_circle', params, N);

    case 2
        fprintf("Generating Helix...\n");
        params.R = 5;
        params.turns = 3;
        params.height = 12;
        path = geometric_curve('helix', params, N);

    case 3
        fprintf("Generating Figure-8 Curve...\n");
        params.a = 8;
        path = geometric_curve('figure8', params, N);

    case 4
        fprintf("Generating 3D S-Curve...\n");
        path = geometric_curve('s_curve3d', struct(), N);

    case 5
        fprintf("Generating Trefoil Knot...\n");
        params.scale = 8;
        path = geometric_curve('trefoil', params, N);

    otherwise
        error("Invalid selection.");
end

%% -------------------------------
% 2. Time parameters
% -------------------------------
T = 6.0;            % total duration
dt = 0.01;
ts = 0:dt:T;

%% -------------------------------
% 3. Set up animation figure
% -------------------------------
figure; hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

title('3D Trajectory Animation');
plot3(path(:,1), path(:,2), path(:,3), 'k--', 'LineWidth', 1);

% Animated ball
h_ball = plot3(path(1,1), path(1,2), path(1,3), 'ro', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Force isometric view
axis equal;                % equal scale on all axes
view(45, 30);              % classic iso angle (az=45°, el=30°)
axis vis3d;                % lock aspect ratio during animation

%% -------------------------------
% 4. Run animation
% -------------------------------
for t = ts
    [p, ~, ~] = evaluate_trajectory(path, T, t);
    set(h_ball, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
    drawnow;
end

fprintf("\nAnimation complete.\n");
