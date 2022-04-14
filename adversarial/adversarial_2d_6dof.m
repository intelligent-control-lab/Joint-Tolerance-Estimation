%% adversarial optimization for the joint tolerance bound 
% robot configurations
theta1 = pi/4;
theta2 = pi/4;
theta3 = pi/4;
theta4 = pi/4;
theta5 = pi/4;
theta6 = pi/4;
xdist = 0.09;
ydist = 0.05;
nlink = 6;

% forward kinematics to compute the boundary wall 
xpos = cos(theta1) + cos(theta2) + cos(theta3) + cos(theta4) + cos(theta5) + cos(theta6);
xwall = xpos + xdist;

% theta_ini
theta_ini = [];
for i = 1:nlink
    eval(['theta_ini = [theta_ini theta' num2str(i) ']']);
end

%% automatic decomposition tool 
% set random seed
% rng(6);
% 
% options = optimoptions('fmincon','Display','iter','Algorithm','active-set','ConstraintTolerance',0);
% obj = @(x)cost_obj(x, theta_ini);
% 
% xref = theta_ini + pi*rand(1,nlink); % set reference to be perturbation around initial theta within 0.1 tolerance bound
% LB = -pi*ones(1,nlink); 
% UB = pi*ones(1,nlink);
% 
% % start optimization 
% [x, fval, exitflag,output] = fmincon(obj,xref,[],[],[],[],LB,UB,@(x)nonlinear_fk_cons(x,xwall),options);
% 
% % get the computed bound 
% computed_bound = obj(x);
% fprintf('the computed bound is: %d', computed_bound);
% 
% [c, ceq] = nonlinear_fk_cons(x, xwall);
% fprintf('the constraint violation is: %d', c);


%% before the solution
seed = 1;
max_iter = 100;
xs = [];
seeds = [];
cs = [];

%% repeat to solve
for i = 1:max_iter
    rng(seed);
    
    options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',0);
    obj = @(x)cost_obj(x, theta_ini);
    
    xref = theta_ini + pi*rand(1,nlink); % set reference to be perturbation around initial theta within 0.1 tolerance bound
    LB = -pi*ones(1,nlink); 
    UB = pi*ones(1,nlink);
    
    % start optimization 
    [x, fval, exitflag,output] = fmincon(obj,xref,[],[],[],[],LB,UB,@(x)nonlinear_fk_cons(x,xwall),options);
    
    % log the result if feasible solution found 
    if exitflag > 0
        c = nonlinear_fk_cons(x,xwall);
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


function [c, ceq] = nonlinear_fk_cons(theta, xwall)
    xpos = 0;
    for i = 1:size(theta,2)
        xpos = xpos + cos(theta(i));
    end
    c = -(xpos - xwall); % minimize lmd, such that there exists unsafe point
    ceq = [];
end

function lmd = cost_obj(theta, theta_ini)
    deviation = theta - theta_ini;
    lmd = max(abs(deviation));
end