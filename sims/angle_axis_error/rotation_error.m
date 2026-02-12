function [w_err, theta_err, u_err] = rotation_error(Rd, R)
% compute orientation error between desired rotation Rd and actual rotation R.
% inputs:
%       Rd  : desired rotation
%       R   : actual rotation
% 
% outputs:
%       w_err     : 3x1 rotation error vector (theta * axis)
%       theta_err : scalar rotation error magnitude (rad)
%       u_err     : 3x1 unit axis of rotation

% rotation error matrix
R_err = Rd' * R;

% clamp numerical trace
tr = max(min((trace(R_err) - 1) / 2, 1), -1);

% error angle
theta_err = acos(tr);

% handle small angle
if abs(theta_err) < 1e-9
    w_err = [0;0;0];
    u_err = [1;0;0]; % arbitrary axis
    return;
end

% error axis
u_err = 1/(2*sin(theta_err)) * [ ...
    R_err(3,2) - R_err(2,3);
    R_err(1,3) - R_err(3,1);
    R_err(2,1) - R_err(1,2) ];

u_err = u_err / norm(u_err);

% angular-error vector (used in control laws)
w_err = theta_err * u_err;

end
