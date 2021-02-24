%% comparison of SOS formulation and not using SOS formulation 
clear
clc
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

%% quadratic testing 
% load the matrix 
A = [0, -0.25, 0, 1, 0;
     -sqrt(3)/2, 0, 0, 0, 1];
b = [0, 0];


% boundary 
lmd = pi/2;
%% initialize the upper and lower bound 
new_vec_dim = size(A,2);
LB = [];
UB = [];

max = 2/180*pi; % 2 degree
LB = [0,0,-1,0,0]';
UB = [max, max^2, 1, max^2/4, sqrt(3)/2*max]';

%% initialize the xref warm start 
optlmd = 0.017; % one degree 
% update the lmd entrance limit 
xref = [optlmd, optlmd^2, 1, optlmd^2/4, sqrt(3)/2*optlmd]';

disp(xref);
% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_onelink(x),options);
