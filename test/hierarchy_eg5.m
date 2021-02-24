



%% comparison of SOS formulation and not using SOS formulation 
clear
clc
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

 
A = [];
b = [];

 
xref = [0 0 0]; % lmdb b c 


LB = [0,0,0];
UB = [];

% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy(x),options);

