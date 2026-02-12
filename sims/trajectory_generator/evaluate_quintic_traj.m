function [p, v, a] = evaluate_quintic_traj(traj, t)
% EVALUATE_QUINTIC_TRAJ
%   Evaluates a piecewise-quintic trajectory at global time t.
%
%   This version correctly:
%       - selects the right segment
%       - clamps time only at boundaries
%       - avoids jumping into segment N+1
%       - prevents evaluation artifacts at the end of the path
%
%   Inputs:
%       traj : trajectory struct (from generate_quintic_trajectory)
%       t    : time to evaluate
%
%   Outputs:
%       p, v, a : 1x3 position, velocity, acceleration


%% ---------------------------------------------------------------
% Extract info
%% ---------------------------------------------------------------
t_breaks = traj.t_breaks;     % column vector, length = N_segments + 1
segments = traj.segments;
N_segments = length(segments);

T_start = t_breaks(1);
T_end   = t_breaks(end);


%% ---------------------------------------------------------------
% Clamp time to valid range
%% ---------------------------------------------------------------
if t <= T_start
    seg_idx = 1;
    tau = 0;
elseif t >= T_end
    seg_idx = N_segments;
    tau = segments(seg_idx).T;   % evaluate exactly at end of last segment
else
    % Find segment such that t ∈ [t_k, t_{k+1})
    % (strict upper-bound search)
    seg_idx = find(t_breaks > t, 1) - 1;

    % Safety fallback (should never trigger)
    if seg_idx < 1
        seg_idx = 1;
    elseif seg_idx > N_segments
        seg_idx = N_segments;
    end

    tau = t - t_breaks(seg_idx);
end


%% ---------------------------------------------------------------
% Retrieve segment polynomial coefficients
%% ---------------------------------------------------------------
seg = segments(seg_idx);

cx = seg.coeffs_x;    % [a0 a1 a2 a3 a4 a5]
cy = seg.coeffs_y;
cz = seg.coeffs_z;

% Convert coefficients for polyval:
% polyval expects [a5 a4 a3 a2 a1 a0]
px_poly = cx(end:-1:1);
py_poly = cy(end:-1:1);
pz_poly = cz(end:-1:1);


%% ---------------------------------------------------------------
% Compute position
%% ---------------------------------------------------------------
px = polyval(px_poly, tau);
py = polyval(py_poly, tau);
pz = polyval(pz_poly, tau);


%% ---------------------------------------------------------------
% Compute velocity
%% ---------------------------------------------------------------
dpx_poly = polyder(px_poly);
dpy_poly = polyder(py_poly);
dpz_poly = polyder(pz_poly);

vx = polyval(dpx_poly, tau);
vy = polyval(dpy_poly, tau);
vz = polyval(dpz_poly, tau);


%% ---------------------------------------------------------------
% Compute acceleration
%% ---------------------------------------------------------------
ddpx_poly = polyder(dpx_poly);
ddpy_poly = polyder(dpy_poly);
ddpz_poly = polyder(dpz_poly);

ax = polyval(ddpx_poly, tau);
ay = polyval(ddpy_poly, tau);
az = polyval(ddpz_poly, tau);


%% ---------------------------------------------------------------
% Output
%% ---------------------------------------------------------------
p = [px, py, pz];
v = [vx, vy, vz];
a = [ax, ay, az];

end
