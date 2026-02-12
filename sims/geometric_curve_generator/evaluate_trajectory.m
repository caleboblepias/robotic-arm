function [p, v, a] = evaluate_trajectory(path, T, t)
%   evaluate position, velocity, and acceleration along a geometric path
%   using a quintic time-scaling s(t).
%
%   inputs:
%       path : N×3 geometric waypoints defining the trajectory
%       T    : total trajectory duration (seconds)
%       t    : query time (0 ≤ t ≤ T)
%
%   outputs:
%       p    : 3×1 position at time t
%       v    : 3×1 velocity at time t
%       a    : 3×1 acceleration at time t
%
%   method:
%       - compute s(t), sdot(t), sddot(t) from normalized quintic
%       - interpolate path at s using pchip
%       - compute dp/ds and d^2p/ds^2 numerically
%       - apply chain rule to obtain v and a

N = size(path,1);

% compute normalized position along curve
s = time_law(t, T);

% numerical derivatives of s(t)
ds  = (30*(t/T)^2 - 60*(t/T)^3 + 30*(t/T)^4) / T;
dds = (60*(t/T)   - 180*(t/T)^2 + 120*(t/T)^3) / T^2;

% interpolate geometric curve at s
s_grid = linspace(0,1,N);
p = interp1(s_grid, path, s, 'pchip'); % smooth interpolation

% derivatives of p(s)
dpds  = gradient(path) ./ gradient(s_grid'); % Nx3 approximate dp/ds
dpds_interp = interp1(s_grid, dpds, s, 'pchip');

d2pds2 = gradient(dpds) ./ gradient(s_grid'); % Nx3 d^2p/ds^2
d2pds2_interp = interp1(s_grid, d2pds2, s, 'pchip');

% chain rule
v = dpds_interp  * ds;
a = dpds_interp  * dds + d2pds2_interp * (ds^2);

p = p(:);
v = v(:);
a = a(:);

end
