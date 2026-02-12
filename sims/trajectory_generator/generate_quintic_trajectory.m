function traj = generate_quintic_trajectory(pts, v, a, times)
% GENERATE_QUINTIC_TRAJECTORY (updated signature)
%
%   Inputs:
%       pts   : Nx3 sampled positions (geometric curve)
%       v     : Nx3 velocities
%       a     : Nx3 accelerations
%       times : (N-1)x1 segment durations
%
%   Output:
%       traj  : trajectory struct with quintic segments

N = size(pts,1);
segments = struct;

for i = 1:N-1
    T = times(i);

    p0 = pts(i,:);
    p1 = pts(i+1,:);

    v0 = v(i,:);
    v1 = v(i+1,:);

    a0 = a(i,:);
    a1 = a(i+1,:);

    segments(i).T = T;
    segments(i).coeffs_x = quintic_segment(p0(1), v0(1), a0(1), p1(1), v1(1), a1(1), T);
    segments(i).coeffs_y = quintic_segment(p0(2), v0(2), a0(2), p1(2), v1(2), a1(2), T);
    segments(i).coeffs_z = quintic_segment(p0(3), v0(3), a0(3), p1(3), v1(3), a1(3), T);
end

traj.t_breaks = [0; cumsum(times)];
traj.segments = segments;

end
