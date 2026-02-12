function [p_log, p_ref_log, perr_log, werr_log, q_log,t_log, p_act_log] = control_loop(choice, Kp_t, Kd_t, Kp_r, Kd_r)
%{

RETURN VALUES

p_log       : [steps x 3]  
              Actual end-effector Cartesian position at each timestep.
              Columns correspond to X, Y, Z (cm).

p_ref_log   : [steps x 3]  
              Desired reference Cartesian position sampled from the path.

perr_log    : [steps x 1]  
              Scalar position-error norm  || p_d - p ||  (cm).

werr_log    : [steps x 1]  
              Scalar orientation-error norm  || e_R ||  (rad).

q_log       : [steps x nJoints]  
              Joint angles over time (rad).  
              For this model, nJoints = length(q) = 4.

t_log       : [steps x 1]  
              Simulation time vector (s).

p_act_log   : [steps x 3]  
              Actual 3-D end-effector trajectory for visualization and
              system-response comparison.

%}




clc; close all;

%% DH
d1=5; d2=5; d3=25; d4=5; d5=25;
% a alpha d theta
DH = [  0     0        0      0 ;
       -d2   pi/2     d1     0 ;
        d3   0     0      0 ;  
        0    pi/2    -d4     0 ;
        0     0      -d5     0 ];

%% TUNING
%Kp_t = 10;     Kd_t = 0.05; % translation (cm/s  per cm,  cm/s  per cm/s)
%Kp_r = 0.1;     Kd_r = 0.02; % rotation (rad/s per rad, rad/s per rad/s)

lambda    = 0.2; % damped-pinv factor
vmax_cm   = 50; % Cartesian speed cap (cm/s)
qd_max    = 5; % joint-speed cap (rad/s)

%% SETTINGS
dt      = 0.05; % 1 kHz
T_final = 8; % simulate 8 s
steps   = round(T_final/dt);

%q = [0.3 -1.0  1.2  0.2]; % non-singular start pose (rad)
%q = [0 -30 -65 0];
q = [0 0 0 0];
t = 0;

%% GEOMETRY PROFILE

N = 1200; % number of points along geometric curve

switch choice
    case 1
        % circle
        u  = linspace(0,1,N).';
        
        % circle parameters
        R  = 20;          % radius in cm
        xc = 10;          % circle center (x)
        yc = -5;          % circle center (y)
        zc = 30;          % constant height (z)
        
        % circular geometry
        path = [ xc + R*cos(2*pi*u) , ...
                 yc + R*sin(2*pi*u) , ...
                 zc*ones(N,1)       ];

    case 2
        
        % ride
        u = linspace(0,1,N)';

        R = 25;     % spiral radius
        D = 20;     % vertical drop
        H = 4;      % track vibration amplitude
        z0 = 40;    % starting height
        
        x = R*cos(4*pi*u);
        y = R*sin(4*pi*u);
        z = z0 - D*u + H*sin(6*pi*u);
        
        path = [x y z];
        
    case 3
        % trefoil
        u = linspace(0,2*pi,N).';
        
        x = (2 + cos(3*u)) .* cos(2*u);
        y = (2 + cos(3*u)) .* sin(2*u);
        z = sin(3*u);
        
        scale = 10;   % scale up for workspace
        path = scale * [x, y, z] + [10, -5, 30];  % shift into workspace

    otherwise
        error("Invalid selection.");
end

% GEOMETRY TESTS
%{

% S curve
N  = 1200;
u  = linspace(0,1,N).';
S  = 3*u.*(1-u);                 % smooth scalar for S-shape
path = [ 15*u           , ...    % X: 0 → 15 cm
         10*sin(2*pi*u) , ...    % Y: ±10 cm sinusoid
         40 - 25*S      ];       % Z: 40→15 cm w/ S curve
%}

%% ORIENTATION PROFILE
% spin the EE tool visually, not through IK
theta_spin = linspace(0, 8*pi, N).';   % 4 full spins
thetad_spin = gradient(theta_spin)/dt;
thetadd_spin = gradient(thetad_spin)/dt;

angle_path = [theta_spin, thetad_spin, thetadd_spin];

%% FIGURE
figure; hold on; grid on; axis equal;
xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
title('6-DoF PD Control demo (cm)');
view(135,25);
plot3(path(:,1),path(:,2),path(:,3),'b--','LineWidth',1.0);
hLinks = plot3(0,0,0,'ro-','LineWidth',3,'MarkerFaceColor','r');
hTrace = plot3(0,0,0,'g','LineWidth',2);

%% MEMORY FOR Kd
e_p_prev   = zeros(3,1);
w_err_prev = zeros(3,1);


%% INITIAL CONFIG
[T0,~,R0,~] = fk(DH, q); 


%% LOGGING
p_log      = zeros(steps,3); % actual EE position
p_ref_log  = zeros(steps,3); % desired EE position
perr_log   = zeros(steps,1); % ||p_d - p||
werr_log   = zeros(steps,1); % orientation error norm
q_log      = zeros(steps,length(q)); % joint history
t_log      = zeros(steps,1); % time history
p_act_log = zeros(steps, 3);
manip_log = zeros(steps,1);




%% MAIN LOOP

for k = 1:steps
    % current using FK
    [~,p,R,TF] = fk(DH,q);
    
    % desired using quintic
    idx = min(N, round(1 + (N-1)*(k-1)/(steps-1)));
    p_d = path(idx,:).';
    v_d = zeros(3,1);
    
    theta_d = angle_path(idx,1);
    thetad_d  = angle_path(idx,2);
    
    axis_body = [0;0;1];
    R_d = R0 * axis_angle_to_rotm(axis_body, theta_d);   % <-- R0 defined!
    w_d = (R_d*axis_body) * thetad_d;
    
    % errors/PD
    e_p = p_d - p;
    e_p_dot = (e_p - e_p_prev)/dt;  e_p_prev = e_p;
    
    [w_err,~,~] = rotation_error(R_d,R);
    w_err_dot = (w_err - w_err_prev)/dt;  w_err_prev = w_err;
    
    v_cmd = v_d + Kp_t*e_p   + Kd_t*e_p_dot;
    w_cmd = w_d + Kp_r*w_err + Kd_r*w_err_dot;
    
    if norm(v_cmd)>vmax_cm, v_cmd = vmax_cm*v_cmd/norm(v_cmd); end % clamp Cartesian v
    x_dot = [v_cmd ; w_cmd];
    
    J = jacobian(DH,q);
    q_dot = damped_pinv(J,lambda) * x_dot;
    
    %{
    if any(~isfinite(q_dot))
        warning('q̇ became NaN/Inf – skipping this step'); % guard
        t = t + dt;  continue
    end
    %}

    if norm(q_dot)>qd_max, q_dot = qd_max*q_dot/norm(q_dot); end % clamp joint v
    
    % integration
    q = q + q_dot'*dt;  t = t + dt;

    % logging
    p_log(k,:)     = p.';
    p_ref_log(k,:) = p_d.';
    perr_log(k)    = norm(e_p);
    werr_log(k)    = norm(w_err);
    q_log(k,:)     = q;
    t_log(k)       = t;
    p_act_log(k,:) = p.';
    manip_log(k) = sqrt(det(J*J.'));



    
    % animate
    if mod(k,20)==0
        pts=[0 0 0]; for i=1:size(TF,3), pts=[pts;TF(1:3,4,i)']; end        
        set(hLinks,'XData',pts(:,1),'YData',pts(:,2),'ZData',pts(:,3));
        kk = min(k,N); % safe index
        set(hTrace,'XData',path(1:kk,1), ...
                   'YData',path(1:kk,2), ...
                   'ZData',path(1:kk,3));
        drawnow;

    end

    

    
end


%disp('Demo finished.');

%% SYSTEM RESPONSE


figure; 
subplot(3,1,1); hold on; grid on;
plot(t_log, p_ref_log(:,1),'b--','LineWidth',1.2);
plot(t_log, p_log(:,1),'r','LineWidth',1.3);
title('X Position Tracking'); ylabel('X (cm)'); legend('Ref','Actual');

subplot(3,1,2); hold on; grid on;
plot(t_log, p_ref_log(:,2),'b--','LineWidth',1.2);
plot(t_log, p_log(:,2),'r','LineWidth',1.3);
title('Y Position Tracking'); ylabel('Y (cm)'); legend('Ref','Actual');

subplot(3,1,3); hold on; grid on;
plot(t_log, p_ref_log(:,3),'b--','LineWidth',1.2);
plot(t_log, p_log(:,3),'r','LineWidth',1.3);
title('Z Position Tracking'); ylabel('Z (cm)'); xlabel('Time (s)');
legend('Ref','Actual');

figure; 
subplot(2,1,1); hold on; grid on;
plot(t_log, perr_log,'LineWidth',1.5);
title('Position Error Norm'); ylabel('||p_d - p|| (cm)');

subplot(2,1,2); hold on; grid on;
plot(t_log, werr_log,'LineWidth',1.5);
title('Orientation Error Norm'); ylabel('||e_R|| (rad)'); xlabel('Time (s)');

figure; hold on; grid on;
plot(t_log, q_log,'LineWidth',1.3);
title('Joint Angles Over Time'); xlabel('Time (s)'); ylabel('q_i (rad)');
legend('q_1','q_2','q_3','q_4','Location','best');

figure; hold on; axis equal; grid on;
plot3(path(:,1),path(:,2),path(:,3),'b--','LineWidth',1.2);   % reference
plot3(p_act_log(:,1),p_act_log(:,2),p_act_log(:,3),'r','LineWidth',1.5);  % actual
legend('Reference Path','Actual EE Path');
title('Actual vs. Reference End-Effector Path');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135,25);

figure; hold on; grid on;
plot(t_log, manip_log, 'LineWidth', 1.5);
title('Manipulability Index Over Time');
xlabel('Time (s)');
ylabel('Manipulability');



end



