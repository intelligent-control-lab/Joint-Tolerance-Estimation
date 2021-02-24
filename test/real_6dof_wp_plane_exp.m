%% test for the distance to plane 
clc
clear
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
%% robot set up parameters 
% real theta 
theta_ini = [-0.0452   -1.5247    0.2224   -0.0072    0.1575   -1.9004]';
% anchor poitn 
anchor_point = [1.6343;0.0060;1.1336];
%epos = FK
epos = ForKine(theta_ini, robot.DH, robot.base, robot.cap);
plane = [-0.4758   -0.0135   -1.0000    1.7601];

%% examination to verify distance currently is positive
n = normal2hole(plane,anchor_point);
% get projection point on the plane
proj = proj_plane(epos, plane);
vec = epos - proj;
dist = dot(n,vec);
disp(['plane ' num2str(5) ' with reference configured robot, distance is ' num2str(dist)]);

%% SOS testing to get the provably safe joint tolerance bound 
syms lmd y1 y2 y3 y4 y5 y6 real;
ys = [y1 y2 y3 y4 y5 y6];
c_pre = ForKine_sym_nolmd(theta_ini, robot.DH, robot.base, robot.cap, ys);
proj = proj_plane(c_pre, plane);
vec = c_pre - proj;
dist = dot(n,vec);
% original requirement is dist > 0 + thres
% construct the refute set
thres = 0.02;
f1 = -dist; % the dist < 0 is empty, which is -dist >= 0 is empty
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
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
obj = @(x)-x(1);

A = [];
b = [];
seed = seed + 1;
rng(seed);
xref = [0 100000*rand(1,7)]; % lmdb b c d e g h k

% LB = 1*[0, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4];

LB = 0*[0, 1, 1, 1, 1, 1, 1, 1];
% UB = [];
% UB = [0.1, inf, inf, inf, inf, inf, inf, inf];

% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_fk(x,Q,Q1,Q2,Q3,Q4,Q5,Q6,Q_cons),options);
toc


