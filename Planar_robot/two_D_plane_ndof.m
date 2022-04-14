%% the 2D plane with 2D link experiments
clc
clear

%% experimental parameters
maximum_iter = 100;
tolerance = 0;
nlink = 20;

%% start experiments
% define the auxiliary variables 
syms lmd;
for i = 1:nlink
    eval(['syms y' num2str(i) ' real']);
end

% robot configurations
xdist = 0.09;
ydist = 0.05;

% forward kinematics 
xpos = 0;
for i = 1:nlink
    xpos = xpos + cos(pi/4);
end

% compute lower bound 
theta_ini = [];
for i = 1:nlink
    theta_ini = [theta_ini; pi/4];
end
lmd_maximum = 0.1;
maximum_delta = compute_lower_bound_shift_planar_robot(theta_ini, lmd_maximum, nlink);
% maximum_delta = 0.000179; % precomputed, don't need to redo the computation every time

xwall = xpos + xdist - maximum_delta;

%% automatic decomposition tool 
% forward kinematics within joint toleranc, and without lmd 
xfk = 0;
for i = 1:nlink
    eval(['xfk = xfk + cossym(theta_ini(i), y' num2str(i) ');']);
end
% xfk = cossym(theta1, y1) + cossym(theta2, y2) + cossym(theta3, y3) + cossym(theta4, y4) + cossym(theta5, y5) + cossym(theta6, y6);
% yfk = sinsym(theta1, y1) + sinsym(theta2, y2) + sinsym(theta3, y3) + sinsym(theta4, y4) + sinsym(theta5, y5) + sinsym(theta6, y6);

% refute set polynomials
% construct the coefficient and monomoials
f1 = xfk - xwall;
% f1 = yfk - ywall;
[c,t] = coeffs(f1);
deci_coe = vpa(c,3);
% decompose to Y^T Q Y = f1
% decide which term
% differentiation to determine the order 
tic
for i = 1:nlink
    eval(['gys' num2str(i) '= diff(t,y' num2str(i) ');'])
end

% convert to real number
ys = [];
numones = [];
for i = 1:nlink
    eval(['ys = [ys y' num2str(i) '];']);
    numones = [numones 1];
end
numones = num2cell(numones);
ys = num2cell(ys);
for i = 1:nlink
    eval(['gys' num2str(i) '= double(subs(gys' num2str(i) ', ys, numones));']);
end

% assign the Q matrix
for num = 1:size(t,2) 
    % determine the order sequence 
    weight = eval(deci_coe(num));
    order_seq = [];
    for i =  1:nlink
        eval(['order_seq = [order_seq gys' num2str(i) '(num)];'])
    end
    
    % get the row and column and weight, directly using the switch case
    % since it is too simple, Y = [1,y1,y2,y3,y4,y5,y6]
    disp(order_seq);
    % the second order term
    for j = 1:nlink
        if order_seq(j) == 2
            Q(j+1,j+1) = weight;
            continue;
        end
    end
    % the first order term
    for j = 1:nlink
        if order_seq(j) == 1
            Q(1,j+1) = weight/2;
            continue;
        end
    end
    % the constant term 
    if max(order_seq) == 0 
        % constant term
        Q(1,1) = weight;
        continue;
    end
end

% make the Q matrix as symmetric 
dim = nlink + 1; % due to Y = [1 y1 y2 y3 y4 y5 y6];
for i = 1:dim
    for j = 1:i-1
        assert(j < i);
        Q(i,j) = Q(j,i);
    end
end

% auxiliary decomposition of the constraints for y1, y2
Qs = {};
for i = 1:nlink
    Q_tmp = zeros(nlink+1,nlink+1);
    Q_tmp(1,1) = 1;
    Q_tmp(i+1,i+1) = -1;
    Qs{i} = Q_tmp;
end
Q_tmp = zeros(nlink+1,nlink+1);
Q_tmp(1,1) = 1;
Q_cons = Q_tmp;


%% before the solution 
seed = 1;
xs = [];
seeds = [];
numericals = [];
exitflags = [];

%% SOS formulation to formulate the nonlinear constraints
options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',5e-13);
% options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

tic
for iter = 1:maximum_iter
    A = [];
    b = [];
    normalize = 0; % whether to use the normalization trick
    
    rng(seed);
    xref = [0 1*rand(1,nlink+1)]; % lmd b c d e f g h
    
    % non-normalized formulation 
    if normalize == 0
        LB = [0, 0.5*ones(1,nlink+1)];
        UB = [0.1, inf*ones(1,nlink+1)];
        [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D_ndof(x,Q,Qs,Q_cons),options);
    end
    
    % normalized trick formulation 
    if normalize == 1
        LB = [0, 0, 0, 0, 0, 0];
        UB = [];
        [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D_ndof(x,Q,Qs,Q_cons),options);
    end

    if exitflag >= 0
        % log the solution 
        % valid solution found 
        c = nonlcon_hierarchy_2D_ndof(x,Q,Qs,Q_cons);
        numerical = c - 1e-12; % minus the 1e-12 offset from the constraints 
        if numerical < 5e-14 % sufficiently small numerical error
%         if 1
            % good solution 
            xs = [xs x(1)];
            seeds = [seeds seed];
            numericals = [numericals numerical];
            exitflags = [exitflags exitflag];
        end
    end
    seed = seed + 1;
end

total_time = toc;
% computed_bound = max(xs);
% fprintf('the computed bound is: %d', computed_bound);

% get performance status 
[time2valid, optimality, percent, variance] = eval_performance(xs, total_time, nlink);
fprintf("average time for a valid solution is %d \n", time2valid);
fprintf("estimation optimality is %d \n", optimality);
fprintf("valid solution percentage is %d \n", percent);
fprintf("solution variance is %d \n", variance);

function c = cossym(t, y)
    c = cos(t)*(1 - y^2/2) - sin(t)*y;
end

function s = sinsym(t, y)
    s = sin(t)*(1 - y^2/2) + cos(t)*y;
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