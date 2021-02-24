

%% exactly solution for one link problem 

%% comparison of SOS formulation and not using SOS formulation 
clear
clc
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);


%% working solution: concise the decision variable 
A = [];
b = [];

 
xref = [0 0 0]; % lmdb x y 
% xref = [0,1,1];


% LB = [0,1,1];
LB = [0,0,0];
UB = [];

% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
tic
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy(x),options);
time = toc


%% general solution 
% A = [];
% b = [];
% 
%  
% xref = [0 0 0 0 0 0 0]; % lmdb b c 4varaible  
% 
% 
% LB = [0,0,0,0,-inf,-inf,-inf];
% UB = [];
% 
% % [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
% [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy(x),options);
