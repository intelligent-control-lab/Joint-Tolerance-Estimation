%% adversarial optimization for the joint tolerance bound 
% robot configurations
clc
clear
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
% theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20];
% nlink = 6; % only consider the front 3 joint
% epos = get_endpose(theta_ini',robot.DH,robot.base,robot.cap,nlink);
% xdist = 0.04; 

%% robot set up parameters 
% real theta 
theta_ini = [-0.0452   -1.5247    0.2224   -0.0072    0.1575   -1.9004];
nlink = 6;

%% before the solution
seed = 1;
max_iter = 100;
xs = [];
seeds = [];
cs = [];
%% automatic decomposition tool 
% set random seed
for i = 1:max_iter
    rng(seed);
    
    options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',5e-13);
    obj = @(x)cost_obj(x, theta_ini, nlink);
    
    xref = theta_ini(1:nlink) + pi*rand(1,nlink); % set reference to be perturbation around initial theta
    LB = -pi*ones(1,nlink); 
    UB = pi*ones(1,nlink);
    
    % start optimization 
    [x, fval, exitflag,output] = fmincon(obj,xref,[],[],[],[],LB,UB,@(x)nonlinear_fk_cons(x,robot,nlink,theta_ini),options);
    
    % log the result if feasible solution found 
    if exitflag > 0
        c = nonlinear_fk_cons(x,robot,nlink,theta_ini);
        if c < 0 % wall is not moved 
            % good solution 
            xs = [xs obj(x)];
            seeds = [seeds seed];
            cs = [cs c];
        end
    end
    seed = seed + 1;
end

% get the computed bound 
computed_bound = min(xs);
fprintf('the computed bound is: %d', computed_bound);


function [c, ceq] = nonlinear_fk_cons(theta,robot,nlink,theta_ini)
    if nlink < 6
        theta_full = [theta theta_ini(nlink+1:end)];
    else
        theta_full = theta;
    end

    % anchor point 
    anchor_point = [1.6343;0.0060;1.1336];

    %epos = FK
    epos = ForKine(theta_full', robot.DH, robot.base, robot.cap);

    % plane and normal vector
    plane = [-0.4758   -0.0135   -1.0000    1.7601];
    n = normal2hole(plane,anchor_point);
    % epos = ForKine_jointbound_approx(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    proj = proj_plane(epos, plane); % get projection point on the plane
    vec = epos - proj;
    dist = dot(n,vec); 

    % if dist < 0, then it is a violation 
    c = dist; % minimize lmd, such that there exists unsafe point

    ceq = [];
end

function lmd = cost_obj(theta, theta_ini, nlink)
    deviation = theta - theta_ini(1:nlink);
    lmd = max(abs(deviation));
end