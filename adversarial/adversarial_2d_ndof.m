%% adversarial optimization for the joint tolerance bound 
% robot configurations
%% experimental parameters
slack = 1e-2;
max_iter = 100;
tolerance = 0;
nlink = 20;

%% start experiments
xdist = 0.09 - slack;
ydist = 0.05;

% forward kinematics to compute the boundary wall 
xpos = 0;
for i = 1:nlink
    xpos = xpos + cos(pi/4);
end
xwall = xpos + xdist;

% theta_ini
theta_ini = [];
for i = 1:nlink
%     eval(['theta_ini = [theta_ini theta' num2str(i) ']']);
    theta_ini = [theta_ini pi/4];
end

% %% automatic decomposition tool 
% % set random seed
% rng(1);
% 
% options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',5e-13);
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
xs = [];
seeds = [];
cs = [];
%% automatic decomposition tool 
% set random seed
tic;
for i = 1:max_iter
    rng(seed);
    
    options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',tolerance);
    obj = @(x)cost_obj(x, theta_ini);
    
    xref = theta_ini + pi*rand(1,nlink); % set reference to be perturbation around initial theta
    LB = -pi*ones(1,nlink); 
    UB = pi*ones(1,nlink);
    
    % start optimization 
    [x, fval, exitflag,output] = fmincon(obj,xref,[],[],[],[],LB,UB,@(x)nonlinear_fk_cons(x,xwall),options);
    
    % log the result if feasible solution found 
    if exitflag > 0
        c = nonlinear_fk_cons(x,xwall);
        if c < tolerance % wall is not moved 
%         if 0. < c && c < 1e-3 % wall is not moved 
            % good solution 
            xs = [xs obj(x)];
            seeds = [seeds seed];
            cs = [cs c];
        end
    end
    seed = seed + 1;
end

% tEnd = toc;
% total_time = tEnd - tStart;
total_time = toc;


% get the computed bound 
% computed_bound = min(xs);
% fprintf('the computed bound is: %d', computed_bound);

% get performance status 
[time2valid, optimality, percent, variance] = eval_performance(xs, total_time, nlink);
fprintf("average time for a valid solution is %d \n", time2valid);
fprintf("estimation optimality is %d \n", optimality);
fprintf("valid solution percentage is %d \n", percent);
fprintf("solution variance is %d \n", variance);

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

function [time2valid, optimality, percent, variance] = eval_performance(xs, total_time, nlink)
    valid_count = 0;
    for i = 1:length(xs)
        lmd = xs(i);
        distance = nlink * cos(pi/4) + 0.09 - nlink * cos(pi/4 - lmd);
        if distance > 0
            valid_count = valid_count + 1;
        end
    end
    percent = valid_count / 100;
    variance = var(xs);
    time2valid = [];
    if valid_count > 0
        time2valid = total_time / valid_count;
    end
    
    % optimal joint 
    lmd_opt = min(xs);
    optimality = nlink * cos(pi/4) + 0.09 - nlink * cos(pi/4 - lmd_opt);
    if optimality < 0
        optimality = [];
    end
end