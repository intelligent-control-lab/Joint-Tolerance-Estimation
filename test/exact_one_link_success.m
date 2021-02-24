



%% comparison of SOS formulation and not using SOS formulation 
clear
clc
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);


% A =  [1, 1, 0,     0,     0;
%      0,     0,     1,     1,     0;
%      0,     0,     0,     0,     1];
%  
% b = [-4,2,1]';
% 
% xref = [-3,1,1,1,1]';
% [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x),options);

%% quadratic testing 
% y limit original 
% A = [0     1     0     0     0;
%     -1     0     1     1     0;
%      0     0     0     0     1];
% b = [1,0,1]';
 
% one link problem
A = [   -0.8660   -0.2500    1.0000         0         0         0         0;
         0    0.5000         0    1.0000    1.0000         0    1.0000;
    0.8660   -0.2500         0         0         0    1.0000         0];

b = [0.25,0.5,0.25]';
 
 
% A = [-0.5000    1.0000         0         0         0         0;
%          0         0    1.0000    1.0000         0    1.0000;
%     0.5000         0         0         0    1.0000         0];
% 
% b = [1.25,1.5,1.25]';
 
% opt = 5 * (sqrt(5/4) - 2.5);
% xref = [opt, opt^2/25, -4-opt^2/25-opt, 1, 1, 1]';

opt = 2.5;
y = -1;
% xref = [opt, 2.5, -y, y, 0, 1.5]';
% xref = [0,1.25,0,0,1.25,1.5]';
% xref = [0,0,0,0,0,0]';
xref = [0,0,0,0,0,0,0]';
% xref = [opt, 1, -y, opt+y, 1, 1, 1, -1, -1]';

% LB = [-10, -10, -10, -10, -10];
% UB = [10, 10, 10, 10, 10];
LB = [];
UB = [];

% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon2(x),options);

