function [T, p, R, T_all] = fk(DH, q)
%   forward kinematics using standard DH parameters.
%
%   inputs:
%       DH : n×4 Denavit-Hartenberg table [a, alpha, d, theta0]
%       q  : joint angles added to theta0 for rows corresponding to joints
%
%   outputs:
%       T     : 4×4 transform of the end effector
%       p     : 3×1 end-effector position
%       R     : 3×3 end-effector rotation matrix
%       T_all : 4×4×n transforms of all intermediate frames
%
%   multiplies successive DH transforms to compute the full chain.


n = size(DH,1);
T = eye(4);
T_all = zeros(4,4,n);

for i = 1:n
    a     = DH(i,1);
    alpha = DH(i,2);
    d     = DH(i,3);
    theta0 = DH(i,4);

    % extra check to account for end effector offset
    if i <= length(q)
        theta = theta0 + q(i);
    else
        theta = theta0;
    end

    Ti = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
           0,            sin(alpha),             cos(alpha),           d;
           0 0 0 1 ];

    T = T * Ti;
    T_all(:,:,i) = T;
end

p = T(1:3,4);
R = T(1:3,1:3);
end
