



%% comparison of SOS formulation and not using SOS formulation 
clear
clc
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

% syms  q11 q12 q13 q22 q23 q33 x1 x2 real;
% Q = [q11 q12 q13;
%      q12 q22 q23;
%      q13 q23 q33];
%  
% func = simplify([1 x1 x2]*Q*[1 x1 x2]');


%% general solution 

% xref = [0 0 0 0 0 0 0 0 0 0];
% % xref = [0 0 0 0 0 0 0 0 0 0];
% 
% A = [];
% b = [];
% 
%  
% % xref = [0 0.001 0.001]; % lmdb b c 
% 
% LB = [0 0 0 0 0 -inf -inf -inf -inf -inf];
% % LB = [];
% UB = [];
% 
% [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy(x),options);

%% simplified solution 

A = [];
b = [];

% xref = [0 1 1 1]; % lmd a b c 
xref = [0 100*rand(1,3)] % lmd a b c 

LB = [0 1 1 1];
UB = [];

[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy(x),options);
