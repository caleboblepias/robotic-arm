function v = calc_velocities(pts, dt)
% CALC_VELOCITIES
%   Estimates velocity vectors from sampled geometric points.
%
%   Inputs:
%       pts : N x 3 sampled positions (from geometric B-spline)
%       dt  : time step between samples (desired temporal resolution)
%
%   Output:
%       v   : N x 3 velocity vectors

N = size(pts, 1);
v = zeros(N,3);

% ---------------------
% Interior points (central differences)
% ---------------------
for i = 2:N-1
    v(i,:) = (pts(i+1,:) - pts(i-1,:)) / (2*dt);
end

% ---------------------
% Endpoints (one-sided differences)
% ---------------------
v(1,:)   = (pts(2,:)   - pts(1,:))   / dt;
v(N,:)   = (pts(N,:)   - pts(N-1,:)) / dt;

end
