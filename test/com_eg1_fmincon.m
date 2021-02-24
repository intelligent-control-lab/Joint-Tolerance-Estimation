%% comparison of SOS formulation and not using SOS formulation 
clear
clc
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
obj = @(x)-x(1);

% xref = [0,0]';
% 
% A = [];
% b = [];
% A =  [1, 1, 0,     0,     0;
%      0,     0,     1,     1,     0;
%      0,     0,     0,     0,     1];
%  
% b = [-4,2,1]';
% 
% xref = [-3,1,1,1,1]';
% [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x),options);

%% quadratic testing
A = [1     1     1     0     0     0;
     0     0     0     1     1     0;
     0     0     0     0     0     1];
 
b = [-4,2,1]';
% opt = 5 * (sqrt(5/4) - 2.5);
% xref = [opt, opt^2/25, -4-opt^2/25-opt, 1, 1, 1]';

opt = 5 * (sqrt(5/4) - 2.5) - 4;
xref = [opt, opt^2/25+2, -4-opt^2/25-opt, 1, 1, 1]';
% xref = [0, 0, 0, 0, 0, 0]';
% [x, fval] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x));
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,[],[],@(x)nonlcon2(x),options);
