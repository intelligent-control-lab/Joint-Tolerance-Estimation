%% adversarial optimization for the joint tolerance bound 
% robot configurations
clc
clear
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20];
nlink = 6; % only consider the front 3 joint
epos = get_endpose(theta_ini',robot.DH,robot.base,robot.cap,nlink);
xdist = 0.04; 

% forward kinematics to compute the boundary wall 
xwall = epos(1) + xdist;

% %% automatic decomposition tool 
% % set random seed
% rng(1);
% 
% options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',5e-14);
% obj = @(x)cost_obj(x, theta_ini, nlink);
% 
% xref = theta_ini(1:nlink) + 1*pi*rand(1,nlink); % set reference to be perturbation around initial theta
% LB = -pi*ones(1,nlink); 
% UB = pi*ones(1,nlink);
% 
% % start optimization 
% [x, fval, exitflag,output] = fmincon(obj,xref,[],[],[],[],LB,UB,@(x)nonlinear_fk_cons(x,robot.DH,robot.base,robot.cap,nlink,xwall,theta_ini),options);
% 
% % get the computed bound 
% computed_bound = obj(x);
% fprintf('the computed bound is: %d', computed_bound);

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
    [x, fval, exitflag,output] = fmincon(obj,xref,[],[],[],[],LB,UB,@(x)nonlinear_fk_cons(x,robot.DH,robot.base,robot.cap,nlink,xwall,theta_ini),options);
    
    % log the result if feasible solution found 
    if exitflag > 0
        c = nonlinear_fk_cons(x,robot.DH,robot.base,robot.cap,nlink,xwall,theta_ini);
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


function [c, ceq] = nonlinear_fk_cons(theta,DH,base,RoCap,nlink,xwall,theta_ini)
    if nlink < 6
        theta_full = [theta theta_ini(nlink+1:end)];
    else
        theta_full = theta;
    end
    epos = get_endpose(theta_full',DH,base,RoCap,nlink);
    xpos = epos(1);
    c = -(xpos - xwall); % minimize lmd, such that there exists unsafe point

    ceq = [];
end

function lmd = cost_obj(theta, theta_ini, nlink)
    deviation = theta - theta_ini(1:nlink);
    lmd = max(abs(deviation));
end