function s = time_law(t, T)
%   quintic minimum-jerk timing law.
%
%   inputs:
%       t : current time (0 to T)
%       T : total duration
%
%   output:
%       s : normalized path parameter in [0, 1]
%
%   uses the standard 10-15-6 quintic polynomial for smooth start/stop.

tau = t / T;

s = 10*tau^3 - 15*tau^4 + 6*tau^5;
end
