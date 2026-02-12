function J_damped = damped_pinv(J, lambda)
% computes damped pseudoinverse of J.
%
%   J_damped = (J'J + lambda^2 I)^(-1) J'
%
% inputs:
%   J      : m x n Jacobian
%   lambda : scalar damping coefficient (e.g. 0.01 or 0.1)
%
% output:
%   J_damped : n x m damped pseudoinverse

[m,n] = size(J);
J_damped = (J.' * J + (lambda^2) * eye(n)) \ J.';
end
