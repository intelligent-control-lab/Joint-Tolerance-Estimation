%% compute the symbolic forward kinematics 
clc
clear
syms lmd y1 y2 y3 y4 y5 y6 real;
ys = [y1 y2 y3 y4 y5 y6];
% syms b c d e g h k real;
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20]';
seed = 0;
% no lmd forward kinematics
c_pre = ForKine_sym_nolmd(theta_ini, robot.DH, robot.base, robot.cap, ys);

% hyper-params
seed = 1;
lb = 0.0127;

%% testing for the composition 
% now only focus on the decompose of the original f1 function: forward
% kinematics
f1 = c_pre(3)-1.35; % for y axis wall 
[c,t] = coeffs(f1);
deci_coe = vpa(c,3);


%% the real decomposition of the polynomial 
% initialize the empty matrix
% make it as automatic process
% monitor the time 
tic
nlink = 6;
dim = 2^nlink;
Q = zeros(dim, dim);
ys = [];
for i = 1:nlink
    eval(['ys = [ys y' num2str(i) '];'])
end

for i = 1:nlink
    eval(['gys' num2str(i) '= diff(t,y' num2str(i) ');'])
end

% acceleration the computation 
for i = 1:nlink
    eval(['gys' num2str(i) '= double(subs(gys' num2str(i) ', {y1,y2,y3,y4,y5,y6}, {1,1,1,1,1,1}));']);
end

% do the batch gradient first 
tic
for num = 1:size(t,2) 
    % determine the order sequence 
    weight = eval(deci_coe(num));
    order_seq = [];
    for i =  1:nlink
        eval(['order_seq = [order_seq gys' num2str(i) '(num)];'])
    end
    
    % get the row and column and weight 
    [q,row,col] = coe_assign(weight, order_seq, nlink);
    Q(row,col) = q;
end
toc

% get the symmetric matrix Q 
for i = 1:dim
    for j = 1:i-1
        assert(j < i);
        Q(i,j) = Q(j,i);
    end
end

%% the auxiliary decomposed matrix 
% the each joint constriant is 1 - y_i^2
% j1 
for i = 1:6
    eval(['Q' num2str(i) '= zeros(dim,dim);'])
    eval(['Q' num2str(i) '(1,1) = 1;'])
    % diagnoal terms 
    eval(['order = 2^(' num2str(i) '-1)+1;']) % compensate 1 to match MATLAB order rule 
    eval(['Q' num2str(i) '(order,order) = -1;'])
end

% the constant term
Q_cons = zeros(dim,dim);
Q_cons(1,1) = 1;


%% SOS formulation to formulate the nonlinear constraints

clc
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

A = [];
b = [];
rng(seed);
xref = [0 100000*rand(1,7)]; % lmdb b c d e g h k
LB = [0, lb*ones(1,7)];
UB = [0.1, inf, inf, inf, inf, inf, inf, inf];

[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_fk(x,Q,Q1,Q2,Q3,Q4,Q5,Q6,Q_cons),options);
toc

fprintf('the computed bound is: %d', x(1));
