function a = calc_accelerations(pts, dt)
% CALC_ACCELERATIONS
%   Estimates accelerations from sampled geometric points.
%
%   Inputs:
%       pts : N x 3 sampled positions
%       dt  : sampling timestep
%
%   Output:
%       a   : N x 3 acceleration vectors

N = size(pts, 1);
a = zeros(N,3);

% ---------------------
% Interior points: central second difference
% ---------------------
for i = 2:N-1
    a(i,:) = (pts(i+1,:) - 2*pts(i,:) + pts(i-1,:)) / (dt^2);
end

% ---------------------
% Endpoints: forward/backward diffs
% ---------------------
a(1,:) = (pts(3,:) - 2*pts(2,:) + pts(1,:)) / (dt^2);
a(N,:) = (pts(N,:) - 2*pts(N-1,:) + pts(N-2,:)) / (dt^2);

end
