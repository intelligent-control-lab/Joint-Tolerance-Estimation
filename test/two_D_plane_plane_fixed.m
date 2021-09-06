function [lmd, min_dist, violate_rate, xwall]=two_D_plane_plane_fixed(wall_dist, ys_ori)
%% the 2D plane with 2D link experiments

% define the auxiliary variables 
syms lmd_sym y1 y2 real;
nlink = 2;

% robot configurations
% ys_ori= [4*pi/6; 1*pi/6];
theta1 = ys_ori(1);
theta2 = ys_ori(2);

% forward kinematics 
xpos = cos(theta1) + cos(theta2);
ypos = sin(theta1) + sin(theta2);

% arbitrary wall: y <= x + bias
% max_b = 2*(sqrt(2)/2  + sqrt(2)/2); % 2.8284
% min_b = xpos + ypos; % 2.7321
bias = 2.8; % 2.7321 < 2.8 < 2.8284
% wall_dist=0.2;
xwall = xpos + wall_dist;

%% automatic decomposition tool 
% Y vector = [1 y1 y2];
% forward kinematics within joint toleranc, and without lmd 
xfk = cossym(theta1, y1) + cossym(theta2, y2);
yfk = sinsym(theta1, y1) + sinsym(theta2, y2);

b1=cos(theta1);
b2=cos(theta2);

% refute set polynomials
% construct the coefficient and monomoials
f1 = xfk - xwall;
% f1 = xfk + yfk - bias;
[c,t] = coeffs(f1);
deci_coe = vpa(c,3);
% decompose to Y^T Q Y = f1
% decide which term
% differentiation to determine the order 
tic
for i = 1:2
    eval(['gys' num2str(i) '= diff(t,y' num2str(i) ');'])
end
% convert to real number
for i = 1:2
    eval(['gys' num2str(i) '= double(subs(gys' num2str(i) ', {y1,y2}, {1,1}));']);
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
    % since it is too simple, Y = [1,y1,y2]
    % disp(order_seq);
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
    if order_seq(1) == 0 && order_seq(2) == 0
        % y1
        Q(1,1) = weight;
        continue;
    end
end

% make the Q matrix as symmetric 
dim = 3; % due to Y = [1 y1 y2];
for i = 1:dim
    for j = 1:i-1
        assert(j < i);
        Q(i,j) = Q(j,i);
    end
end

% auxiliary decomposition of the constraints for y1, y2
Q1 = [1 0 0; 0 -1 0; 0 0 0];
Q2 = [1 0 0; 0 0 0; 0 0 -1];
Q_cons = [1 0 0; 0 0 0; 0 0 0];


%% SOS formulation to formulate the nonlinear constraints
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',4e3);
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
obj = @(x)-x(1);

% A = [0 b1 -1 0; 0 b2 0 -1];
% b = [0; 0];
A=[];
b=[];
normalize = 0; % whether to use the normalization trick
lmda=zeros(30,1);

for i=1:30
    %rng(1);
    xref = [0 100*rand(1,3)]'; % lmdb b c d
    
    % non-normalized formulation
    if normalize == 0
        LB = [0, 1, 1, 1]';
        UB = [0.5, inf, inf, inf];
        [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D(x,Q,Q1,Q2,Q_cons),options);
    end
    
    % normalized trick formulation
    if normalize == 1
        LB = [0, 0, 0, 0]';
        UB = [];
        [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D_normalize(x,Q,Q1,Q2,Q_cons),options);
    end
    
    if exitflag>0
        lmda(i)=x(1);
    end
end

lmd=max(lmda);
disp(['the computed bound is: ', num2str(lmd)]);

toc

%% eval
sample_num = 200000;
xpos_approx_samples = zeros(sample_num,1);
violate = 0;
min_dist = 999;

for i = 1:sample_num
    ys = -1 + 2*rand(2,1); % sampling a y vector within [-1,1]
    ys_pert = ys*lmd + ys_ori; % perturbed y vector
    xpos_per = cos(ys_pert(1)) + cos(ys_pert(2));
    ypos_per = sin(ys_pert(1)) + sin(ys_pert(2));
    xpos_approx_samples(i) = xpos_per; % x wall 
%     xpos_approx_samples(i) = ypos_per; % y wall 
    % violation check
    if xpos_per > xwall
        violate = violate + 1;
    end
    % update optimality 
    dist = xwall - xpos_per;
    if dist < min_dist
        min_dist = dist;
    end
end

% figure
% plot(sort(xpos_approx_samples),'.');
% hold on 
% % plot the solidline to demonstrate 1.8
% yline = xwall * ones(sample_num,1); % x wall
% % yline = ywall * ones(sample_num,1); % x wall
% plot(yline,'-','lineWidth',2);
% hold off
% % limitation 
% xlabel('sample number');
% ylabel('x coordinate / m');
% ylabel('y coordinate / m');
% ylim([xwall-0.2 xwall + 0.1]); % x wall 
% ylim([ywall-0.15, ywall + 0.1]); % y wall 

% disp(max(xpos_approx_samples));
disp( violate);
disp( min_dist);
violate_rate=violate/sample_num;
end
%% functions
function c = cossym(t, y)
    c = cos(t)*(1 - y^2/2) - sin(t)*y;
end

function s = sinsym(t, y)
    s = sin(t)*(1 - y^2/2) + cos(t)*y;
end
