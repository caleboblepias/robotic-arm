function coeffs = quintic_segment(p0, v0, a0, p1, v1, a1, T)
% Returns 6×1 coefficient vector for quintic polynomial

% Solve system:
% p(0)=p0, p(T)=p1,
% p'(0)=v0, p'(T)=v1,
% p''(0)=a0, p''(T)=a1

A = [ ...
    1      0       0        0         0        0;
    0      1       0        0         0        0;
    0      0       2        0         0        0;
    1      T      T^2      T^3       T^4      T^5;
    0      1      2*T      3*T^2     4*T^3    5*T^4;
    0      0       2       6*T       12*T^2   20*T^3 ];

b = [p0; v0; a0; p1; v1; a1];

coeffs = A \ b;
end
