%% ChatGPT 5.1 Generated Test Case

%% ============================================================
%   test_robot_model.m
%   VALIDATION SUITE FOR: fk.m, jacobian.m, position, rotation
%   Author: Caleb
% =============================================================

clc; clear; close all;

fprintf("\n=============================================\n");
fprintf("      TESTING ROBOT MODEL MODULE\n");
fprintf("=============================================\n\n");

%% ------------------------------------------------------------
% 1. Define DH Table (EDIT THIS FOR YOUR ROBOT)
% ------------------------------------------------------------
% DH = [a, alpha, d, theta_offset]
DH = [
    0     pi/2   0.4    0;
    0.3   0      0      0;
    0.2   0      0      0;
    0     0      0.1    0
];

n = size(DH,1);

% Random test joint angles
q = rand(n,1)*2*pi - pi;

fprintf("Using random q:\n");
disp(q');


%% ------------------------------------------------------------
% 2. Test FK
% ------------------------------------------------------------

fprintf("Testing FK...\n");

[T, p, R, T_all] = fk(DH, q);

disp("End-effector transform T:");
disp(T);

disp("Position p:");
disp(p');

disp("Rotation R:");
disp(R);


%% ------------------------------------------------------------
% 3. Test Jacobian
% ------------------------------------------------------------

fprintf("\nTesting analytic Jacobian...\n");

J = jacobian(DH, q);

disp("Jacobian J:");
disp(J);


%% ------------------------------------------------------------
% 4. Numerical Jacobian Check (VERY IMPORTANT)
% ------------------------------------------------------------

fprintf("\nPerforming NUMERICAL JACOBIAN CHECK...\n");

eps_val = 1e-6;
J_numeric = zeros(6,n);

% baseline FK
[~, p0, R0] = fk(DH, q);

for i = 1:n
    dq = zeros(n,1);
    dq(i) = eps_val;

    [~, p1, R1] = fk(DH, q + dq);

    % Linear velocity approx
    dp = (p1 - p0) / eps_val;

    % Angular velocity approx (using small-angle approximation)
    dR = R1 * R0';
    w = [ dR(3,2); dR(1,3); dR(2,1) ] / eps_val;

    J_numeric(:,i) = [dp; w];
end

disp("Numeric Jacobian J_numeric:");
disp(J_numeric);

disp("Difference (J - J_numeric):");
disp(J - J_numeric);

max_err = max(abs(J - J_numeric), [], 'all');
fprintf("Max error between analytic and numeric: %e\n", max_err);

if max_err < 1e-3
    fprintf("Jacobian test: PASSED ✔✔✔\n");
else
    fprintf("Jacobian test: FAILED ❌\n");
end


%% ------------------------------------------------------------
% 5. Visual FK check (optional)
% ------------------------------------------------------------

fprintf("\nDrawing FK visualization...\n");
visualize_fk(DH, q);

fprintf("\n=============================================\n");
fprintf("       ROBOT MODEL TEST COMPLETE\n");
fprintf("=============================================\n\n");
