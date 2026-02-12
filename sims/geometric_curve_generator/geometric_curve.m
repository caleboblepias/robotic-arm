function path = geometric_curve(type, params, N)
%   generate an N×3 geometric path for trajectory planning.
%
%   inputs:
%       type   : string specifying path shape
%       params : struct containing curve parameters
%       N      : number of samples along the path
%
%   output:
%       path   : N×3 array of positions [X Y Z]
%
%   supported types:
%       'tilted_circle'          – radius + tilt
%       'waypoints_linear_interp'– linear interp of given waypoints
%       'helix'                  – radius, turns, height
%       'figure8'                – planar figure-eight
%       's_curve3d'              – smooth 3-D S-shaped curve
%       'trefoil'                – 3-D trefoil knot
%       'offset_circle'          – circle with tilt and translation

switch type
    case 'tilted_circle'
        R = params.R;
        tilt = params.tilt; % radians
        
        theta = linspace(0, 2*pi, N)';
        X = R*cos(theta);
        Y = R*sin(theta)*cos(tilt);
        Z = R*sin(theta)*sin(tilt);

        path = [X Y Z];

    case 'waypoints_linear_interp'
        W = params.waypoints;
        t = linspace(0,1,N)';
        path = interp1(linspace(0,1,size(W,1)), W, t, 'linear');

    case 'helix'
        R = params.R;
        turns = params.turns;
        height = params.height;
    
        theta = linspace(0, 2*pi*turns, N)';
        X = R * cos(theta);
        Y = R * sin(theta);
        Z = linspace(0, height, N)';
    
        path = [X Y Z];

    case 'figure8'
        a = params.a;
    
        t = linspace(0, 2*pi, N)';
        X = a * sin(t);
        Y = a * sin(t).*cos(t);
        Z = zeros(N,1);
    
        path = [X Y Z];

    case 's_curve3d'
        t = linspace(-2,2,N)';
        X = t;
        Y = tanh(t);
        Z = 0.5 * sin(2*t);
        path = [X Y Z];

    case 'trefoil'
        s = linspace(0, 2*pi, N)';
        scale = params.scale;
        X = scale * (sin(s) + 2*sin(2*s));
        Y = scale * (cos(s) - 2*cos(2*s));
        Z = scale * (-sin(3*s));
        path = [X Y Z];


    case 'offset_circle'
        R = params.R;
        C = params.center;    
        tilt = params.tilt;
    
        theta = linspace(0, 2*pi, N)';
    
        % base circle
        base = [R*cos(theta), R*sin(theta), zeros(N,1)];
    
        % tilt rotation about Y axis
        Ry = [cos(tilt) 0 sin(tilt);
              0         1     0;
             -sin(tilt) 0 cos(tilt)];
    
        rotated = (Ry * base')';
    
        % translate
        path = rotated + C;



    otherwise
        error('Unknown curve type');
end
end
