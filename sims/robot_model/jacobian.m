function J = jacobian(DH, q)
%   compute the 6×n geometric Jacobian for a serial manipulator.
%
%   inputs:
%       DH : n×4 Denavit–Hartenberg table
%       q  : n×1 joint angles (revolute joints)
%
%   output:
%       J  : 6×n Jacobian matrix [linear; angular]
%
%   method:
%       - use forward kinematics to obtain all frame origins and z-axes
%       - for each joint i:
%           Jv_i = z_i × (p_e - p_i)
%           Jw_i = z_i

n = length(q);

% get FK and intermediate frames
[~, p_e, ~, T_all] = fk(DH, q);

J = zeros(6, n);

for i = 1:n
    if i == 1
        % base frame: standard z-axis
        z_i = [0; 0; 1];
        p_i = [0; 0; 0];
    else
        % extract z-axis of frame i-1
        z_i = T_all(1:3, 3, i-1);
        % extract position of frame i-1
        p_i = T_all(1:3, 4, i-1);
    end

    % revolute joint Jacobian entries
    J(1:3, i) = cross(z_i, p_e - p_i);  % linear velocity part
    J(4:6, i) = z_i;                    % angular velocity part
end
