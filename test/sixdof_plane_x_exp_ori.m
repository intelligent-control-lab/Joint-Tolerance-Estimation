function [lmd, min_dist, violate_rate, xwall, min_delta]=sixdof_plane_x_exp_ori(theta_ini, wall_dist)
global x_fail;
%% compute the symbolic forward kinematics 
syms lmd_sym y1 y2 y3 y4 y5 y6 real ;
ys = [y1 y2 y3 y4 y5 y6];
% syms b c d e g h k real;
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
% theta_ini = [3*pi/20   -9*pi/20    3*pi/20   pi/20    3*pi/20   pi/20]';
% no lmd forward kinematics
c_pre = ForKine_sym_nolmd(theta_ini, robot.DH, robot.base, robot.cap, ys);

% hyper-params
seed = 7;
lb = 0;

%% numerically compute max delta
max_lmd=0.1;
delta=@(x)(ForKine_sym_nolmd(theta_ini, robot.DH, robot.base, robot.cap, x)...
          -ForKine_sym_real( theta_ini, robot.DH, robot.base, robot.cap, x));
      
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',2e3);
obj = @(x)[1,0,0]*delta(x);

A = [];
b = [];
LB =-max_lmd*ones(6,1);
UB = max_lmd*ones(6,1);

mins=zeros(20,1);
for i=1:20
    xref = max_lmd*(-1+2*rand(6,1));
    [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,[],options);
    if exitflag>0
        mins(i)=obj(x);
    end
%     maxs(i)=-obj(xref);
end
min_delta=min(mins);

disp([min_delta]);

%% testing for the composition 
% now only focus on the decompose of the original f1 function: forward
% kinematics
% xwall = 1.9;

epos_ini = ForKine_sym_real( theta_ini, robot.DH, robot.base, robot.cap, zeros(6,1));
xwall = epos_ini(1)+wall_dist;

% min_delta=0;

f1 = c_pre(1)-xwall-min_delta; % for x axis wall 
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

options = optimoptions('fmincon','Display','iter','Algorithm','SQP','ConstraintTolerance',1e-12);
obj = @(x)-x(1);
judge = @(x)nonlcon_hierarchy_fk_64(x,Q,Q_cons);
% mask = [1, zeros(1,63)];

A = [];
b = [];
%rng(seed);
lmd=0;

N=1;
while N<=30
    xref = [0 1000*rand(1,65)]; % lmdb b c d e g h k
    LB = [0, lb*ones(1,65)];
    UB = [max_lmd, +inf*ones(1,65)];
    
    [x, fval, exitflag,~] = fmincon(obj,xref,[],[],A,b,LB,UB,judge,options);
    if exitflag>0
        if max(judge(x)) <= 1e-10
            if x(1)>lmd
                lmd=x(1);
                x_sos=x;
            end
            N=N+1;
        else
            disp('strange');
            disp(max(judge(x)));
        end
    end
end

disp(['the computed bound is: ', num2str(lmd)]);

%% sampling to verify the joint bound exact forward kinematics
% made sure cpre.x = 1.7421 

% precomputed lmd as the joint bound 
% for x wall 
% lmd = 5.485567e-02; % x wall = 1.8


sample_num = 200000;
xpos_samples = zeros(sample_num,1);
% sampling a y vector within [-1,1]
violate = 0;
min_dist = 999;

for i = 1:sample_num
    xs = lmd*(-1 + 2*rand(6,1));
    epos = ForKine_sym_real( theta_ini, robot.DH, robot.base, robot.cap, xs);
    xpos_samples(i) = epos(1); % x axis wall
    
    % update optimality 
    dist = xwall-epos(1);
    if dist < min_dist
        min_dist = dist;
    end
    
    % violation check
    if dist<0
        violate = violate + 1;
    end
end
if violate>0
    x_fail=[x_fail; x_sos];
end

% figure
% sorted_x=sort(xpos_samples);
% plot(sorted_x,'.');
% hold on 
% % plot the solidline to demonstrate 1.8
% 
% yline = xwall * ones(sample_num,1); % x axis wall
% 
% plot(yline,'-','lineWidth',2);
% xlabel('sample number');
% ylabel('x coordinate / m'); 
% % limitation 
% %ylim([xwall-0.3, xwall+0.1]);  % x axis wall
% hold off

violate_rate=violate/sample_num;
disp( ['violate rate: ', num2str(violate_rate)]);
disp( ['min_dist: ', num2str(min_dist)]);
disp( 'theta_ini: ');
disp( theta_ini );
disp( ['x_ini: ', num2str(epos_ini(1))]);
disp( ['x_wall: ', num2str(xwall)]);

assert(violate==0);