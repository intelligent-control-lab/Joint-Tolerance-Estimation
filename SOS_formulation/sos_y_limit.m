% SOSDEMO3 --- Bound on Global Extremum
% Section 3.3 of SOSTOOLS User's Manual
% 

clear; echo on;
syms x1 gam;
% vartable = [x1, x2];
vartable = [x1];
% =============================================
% First, initialize the sum of squares program
prog = sosprogram(vartable);

% =============================================
% Declare decision variable gam too
prog = sosdecvar(prog,[gam]);

% =============================================
% Next, define SOSP constraints

% Constraint : r(x)*(f(x) - gam) >= 0
% f(x) is the Goldstein-Price function

f = x1^2 + x1*gam + 1;
g = 1 - x1^2;

prog = sosineq(prog,(f));
prog = sosineq(prog,(g));
% prog = sosineq(prog,(gam-gam2^2));

% =============================================
% Set objective : maximize gam
prog = sossetobj(prog,-gam);

% =============================================
% And call solver
solver_opt.solver = 'sedumi';
prog = sossolve(prog,solver_opt);
% =============================================
% Finally, get solution
SOLgamma = sosgetsol(prog,gam+3)
echo off