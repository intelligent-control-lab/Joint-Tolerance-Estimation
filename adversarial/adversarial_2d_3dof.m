%% adversarial optimization for the joint tolerance bound 
% robot configurations
theta1 = pi/4;
theta2 = pi/4;
theta3 = pi/4;
xdist = 0.09;
ydist = 0.05;
nlink = 3;

% forward kinematics to compute the boundary wall 
xpos = cos(theta1) + cos(theta2) + cos(theta3);
xwall = xpos + xdist;

% theta_ini
theta_ini = [];
for i = 1:nlink
    eval(['theta_ini = [theta_ini theta' num2str(i) ']']);
end

%% automatic decomposition tool 
% set random seed
rng(1);

options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)cost_obj(x, theta_ini);

xref = theta_ini + pi*rand(1,nlink); % set reference to be perturbation around initial theta within 0.1 tolerance bound
LB = -pi*ones(1,nlink); 
UB = pi*ones(1,nlink);

% start optimization 
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],[],[],LB,UB,@(x)nonlinear_fk_cons(x,xwall),options);

% get the computed bound 
computed_bound = obj(x);
fprintf('the computed bound is: %d', computed_bound);


function [c, ceq] = nonlinear_fk_cons(theta, xwall)
    xpos = cos(theta(1)) + cos(theta(2)) + cos(theta(3));
    c = -(xpos - xwall); % minimize lmd, such that there exists unsafe point
    ceq = [];
end

function lmd = cost_obj(theta, theta_ini)
    deviation = theta - theta_ini;
    lmd = max(abs(deviation));
end