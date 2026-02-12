function a = compute_c2_accels(waypoints, times, v)
% COMPUTE_C2_ACCELS
%   Computes globally C2-continuous accelerations at waypoints
%   to be used with quintic spline segments.
%
%   Inputs:
%       waypoints : Nx3 matrix of Cartesian points (unused here)
%       times     : (N-1)x1 vector of durations
%       v         : Nx3 matrix of C1-consistent velocities
%
%   Outputs:
%       a         : Nx3 matrix of C2-consistent accelerations
%
%   Boundary conditions:
%       a(1,:) = 0
%       a(N,:) = 0


N = size(waypoints,1);
a = zeros(N,3);  % Initialize with boundary a(1)=0, a(N)=0


% Solve for x, y, z independently
for dim = 1:3
    vi = v(:,dim);

    % Build tridiagonal system
    A = zeros(N,N);
    b = zeros(N,1);

    % Boundary rows
    A(1,1) = 1;   b(1)   = 0;
    A(N,N) = 1;   b(N)   = 0;

    % Interior rows
    for i = 2:N-1
        h_im1 = times(i-1);
        h_i   = times(i);

        A(i,i-1) = h_im1;
        A(i,i)   = 2*(h_im1 + h_i);
        A(i,i+1) = h_i;

        % RHS:
        % 6 * ( (v_{i+1}-v_i)/h_i - (v_i - v_{i-1})/h_{i-1} )
        dv_i   = (vi(i+1) - vi(i)) / h_i;
        dv_im1 = (vi(i)   - vi(i-1)) / h_im1;

        b(i) = 6*(dv_i - dv_im1);
    end

    % Solve system
    ai = A \ b;
    a(:,dim) = ai;
end

end
