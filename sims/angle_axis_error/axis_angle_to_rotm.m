function R = axis_angle_to_rotm(axis, theta)
%   convert axis-angle representation into a 3×3 rotation matrix.
%
%   inputs:
%       axis  : 3×1 rotation axis (does not need to be unit length)
%       theta : rotation angle (radians)
%
%   output:
%       R     : 3×3 rotation matrix
%
%   uses Rodrigues' rotation formula.

    % ensure axis is column vector
    axis = axis(:);

    % normalize axis
    if norm(axis) < 1e-12
        R = eye(3);
        return;
    end
    u = axis / norm(axis);

    ux = u(1);
    uy = u(2);
    uz = u(3);

    % skew-symmetric cross-product matrix of u
    K = [   0   -uz   uy;
           uz     0  -ux;
          -uy    ux    0 ];

    % rodrigues' rotation formula
    R = eye(3) * cos(theta) + ...
        (1 - cos(theta)) * (u * u') + ...
        sin(theta) * K;

end
