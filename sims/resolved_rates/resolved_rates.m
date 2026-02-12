function q_dot = resolved_rates(x_dot, J_pinv)
%   compute joint velocities using the resolved-rates control law.
%
%   inputs:
%       x_dot  : 6x1 desired end-effector twist [v; w]
%       J_pinv : damped or regular pseudoinverse of the Jacobian
%
%   output:
%       q_dot  : joint velocity command
%
%   basic method:
%       q_dot = J_pinv * x_dot

q_dot = J_pinv * x_dot;


end

