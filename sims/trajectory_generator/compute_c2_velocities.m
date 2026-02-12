function v = compute_c2_velocities(waypoints, times)
% COMPUTE_C2_VELOCITIES
%   Computes velocities at each waypoint to ensure global C2 continuity.
%
%   Inputs:
%       waypoints: Nx3 matrix of Cartesian points
%       times:     (N-1)x1 vector of segment durations
%
%   Output:
%       v: Nx3 matrix of velocities at each waypoint
%
%   The system solved per dimension is:
%
%   h_{i-1} v_{i-1}
%   + 2(h_{i-1}+h_i) v_i
%   + h_i v_{i+1}
%   = 3( (p_{i+1}-p_i)/h_i - (p_i-p_{i-1})/h_{i-1} )
%
%   with boundary conditions v(1)=0, v(N)=0.
%
%   This is the classical natural cubic-spline slope system and produces
%   minimum-jerk, C2-continuous velocities for the final quintic interpolation.

N = size(waypoints,1);
v = zeros(N,3);   % v(1,:) and v(N,:) are fixed to zero

% Solve for x, y, z independently
for dim = 1:3
    p = waypoints(:,dim);

    % Build tridiagonal matrix A and RHS b
    A = zeros(N,N);
    b = zeros(N,1);

    % Boundary conditions
    A(1,1) = 1; 
    A(N,N) = 1;
    b(1) = 0;
    b(N) = 0;

    % Interior continuity equations
    for i = 2:N-1
        h_im1 = times(i-1);   % duration of segment (i-1)
        h_i   = times(i);     % duration of segment i

        % Fill A matrix (tridiagonal)
        A(i, i-1) = h_im1;
        A(i, i)   = 2*(h_im1 + h_i);
        A(i, i+1) = h_i;

        % RHS term = 3 * (s_i - s_{i-1})
        s_i   = (p(i+1) - p(i)) / h_i;
        s_im1 = (p(i)   - p(i-1)) / h_im1;
        b(i) = 3 * (s_i - s_im1);
    end

    % Solve the tridiagonal linear system
    v(:,dim) = A \ b;
end

end
