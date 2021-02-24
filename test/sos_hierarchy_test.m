



%% comparison of SOS formulation and not using SOS formulation 
clear
clc
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

 
% test case 1
% A = [0 -1 1 1 0 -1;
%      0 0 0 0 1 -1];
%  
% b = [1,0]';

% test case 2
% A = [0 0 0 0 1 -1;
%      0 0 1 1 0 -2];
% b = [0;0];

% test case 3
A = [0 -1 1 1 0 -1 0;
     0 0 0 0 1 -1 0];
 
b = [1,0]';


 
xref = [0,0,0,0,0,0,0]';


LB = [];
UB = [];

% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy(x),options);

