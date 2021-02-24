%% comparison of SOS formulation and not using SOS formulation 
clear
clc
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
% options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

%% quadratic testing 
% load the matrix 
% load('data/A.mat');
% load('data/b.mat');
A = [0     0     1     0     0     0;
     -sqrt(3)/2     0     0     1     1     0;
     0     -0.25     0     0     0     1];
 
% A = [-0.8660   -0.2500    1.0000         0         0         0         0;
%          0    0.5000         0    1.0000    1.0000         0    1.0000;
%     0.8660   -0.2500         0         0         0    1.0000         0];
b = [0.25, 0, 0]';
% b = [0.25, 0.5, 0.25]';

% A = At';

% boundary 
lmd = pi/4;
%% initialize the upper and lower bound 
new_vec_dim = size(A,2);
% LB = [];
% UB = [];
% for i = 1:new_vec_dim
%     LB = [LB;-inf];
%     UB = [UB;inf];
% end
% update the lmd entrance limit 
% for i = 1:4
%     LB(i) = 0.3;
%     UB(i) = lmd^i;
% end
lmdbound = 0.8;
LB = [0,0,0.25,-inf,-inf,0]';
UB = [lmdbound,lmdbound^2,0.25,inf,inf,lmdbound^2/4]';

%% initialize the xref warm start 
optlmd = 0.3245;
xref = [];
for i = 1:new_vec_dim
    xref = [xref;0];
end
% update the lmd entrance limit 
for i = 1:2
    xref(i) = optlmd^i;
end
xref(6) = optlmd^2/4;

disp(xref);
% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_onelink(x),options);
