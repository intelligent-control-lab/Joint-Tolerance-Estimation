%% the 2D plane with 2D link experiments
clc
clear
% define the auxiliary variables 
syms lmd y1 y2 y3 y4 y5 y6 real;
nlink = 6;

% robot configurations
theta1 = pi/4;
theta2 = pi/4;
theta3 = pi/4;
theta4 = pi/4;
theta5 = pi/4;
theta6 = pi/4;
xdist = 0.09;
ydist = 0.05;

% forward kinematics 
xpos = cos(theta1) + cos(theta2) + cos(theta3) + cos(theta4) + cos(theta5) + cos(theta6);
ypos = sin(theta1) + sin(theta2) + sin(theta3) + sin(theta4) + sin(theta5) + sin(theta6);

% compute lower bound 
theta_ini = [theta1;theta2;theta3;theta4;theta5;theta6];
lmd_maximum = 0.1;
maximum_delta = compute_lower_bound_shift_planar_robot(theta_ini, lmd_maximum, nlink);

xwall = xpos + xdist - maximum_delta;
ywall = ypos + ydist;
% xwall = 1.4;

%% automatic decomposition tool 
% Y vector = [1 y1 y2];
% forward kinematics within joint toleranc, and without lmd 
xfk = cossym(theta1, y1) + cossym(theta2, y2) + cossym(theta3, y3) + cossym(theta4, y4) + cossym(theta5, y5) + cossym(theta6, y6);
yfk = sinsym(theta1, y1) + sinsym(theta2, y2) + sinsym(theta3, y3) + sinsym(theta4, y4) + sinsym(theta5, y5) + sinsym(theta6, y6);

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
for i = 1:nlink
    eval(['gys' num2str(i) '= double(subs(gys' num2str(i) ', {y1,y2,y3,y4,y5,y6}, {1,1,1,1,1,1}));']);
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
    if order_seq(6) == 2
        % y5^2
        Q(7,7) = weight;
        continue;
    end
    if order_seq(5) == 2
        % y5^2
        Q(6,6) = weight;
        continue;
    end
    if order_seq(4) == 2
        % y4^2
        Q(5,5) = weight;
        continue;
    end
    if order_seq(3) == 2
        % y3^2
        Q(4,4) = weight;
        continue;
    end
    if order_seq(2) == 2
        % y2^2
        Q(3,3) = weight;
        continue;
    end
    if order_seq(1) == 2
        % y1^2
        Q(2,2) = weight;
        continue;
    end

    % the first order term
    if order_seq(6) == 1
        % y4
        Q(1,7) = weight/2;
        continue;
    end
    if order_seq(5) == 1
        % y4
        Q(1,6) = weight/2;
        continue;
    end
    if order_seq(4) == 1
        % y4
        Q(1,5) = weight/2;
        continue;
    end
    if order_seq(3) == 1
        % y3
        Q(1,4) = weight/2;
        continue;
    end
    if order_seq(2) == 1
        % y2
        Q(1,3) = weight/2;
        continue;
    end
    if order_seq(1) == 1
        % y1
        Q(1,2) = weight/2;
        continue;
    end

    % the constant term 
    if order_seq(1) == 0 && order_seq(2) == 0 && order_seq(3) == 0 && order_seq(4) == 0 && order_seq(5) == 0 && order_seq(6) == 0 
        % constant term
        Q(1,1) = weight;
        continue;
    end
end

% make the Q matrix as symmetric 
dim = 7; % due to Y = [1 y1 y2 y3 y4 y5 y6];
for i = 1:dim
    for j = 1:i-1
        assert(j < i);
        Q(i,j) = Q(j,i);
    end
end

% auxiliary decomposition of the constraints for y1, y2
Q1 = [1 0 0 0 0 0 0; 0 -1 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0];
Q2 = [1 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 -1 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0];
Q3 = [1 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 -1 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0];
Q4 = [1 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 -1 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0];
Q5 = [1 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 -1 0; 0 0 0 0 0 0 0];
Q6 = [1 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 -1];
Q_cons = [1 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0; 0 0 0 0 0 0 0];

%% before the solution 
seed = 1;
xs = [];
seeds = [];
numericals = [];
maximum_iter = 30;
exitflags = [];

%% SOS formulation to formulate the nonlinear constraints
options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',5e-13);
obj = @(x)-x(1);

for iter = 1:maximum_iter
    A = [];
    b = [];
    normalize = 0; % whether to use the normalization trick
    
    rng(seed);
    xref = [0 100*rand(1,nlink+1)]; % lmd b c d e f g h
    
    % non-normalized formulation 
    if normalize == 0
        LB = [0, 1*ones(1,nlink+1)];
        UB = [0.1, inf*ones(1,nlink+1)];
        [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D_6dof(x,Q,Q1,Q2,Q3,Q4,Q5,Q6,Q_cons),options);
    end
    
    % normalized trick formulation 
    if normalize == 1
        LB = [0, 0, 0, 0, 0, 0];
        UB = [];
        [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D_normalize(x,Q,Q1,Q2,Q3,Q_cons),options);
    end

    if exitflag >= 0
        % log the solution 
        % valid solution found 
        c = nonlcon_hierarchy_2D_6dof(x,Q,Q1,Q2,Q3,Q4,Q5,Q6,Q_cons);
        numerical = c - 1e-12; % minus the 1e-12 offset from the constraints 
        if numerical < 5e-14 % sufficiently small numerical error 
            % good solution 
            xs = [xs x(1)];
            seeds = [seeds seed];
            numericals = [numericals numerical];
            exitflags = [exitflags exitflag];
        end
    end
    seed = seed + 1;
end

toc
computed_bound = max(xs);
fprintf('the computed bound is: %d', computed_bound);

function c = cossym(t, y)
    c = cos(t)*(1 - y^2/2) - sin(t)*y;
end

function s = sinsym(t, y)
    s = sin(t)*(1 - y^2/2) + cos(t)*y;
end