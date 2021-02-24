% SOSDEMO3 --- Bound on Global Extremum
% Section 3.3 of SOSTOOLS User's Manual
% 

clear; echo on;
syms x1 gam gam1;
% vartable = [x1, x2];
vartable = [x1];
% =============================================
% First, initialize the sum of squares program
prog = sosprogram(vartable);

% =============================================
% Declare decision variable gam too
prog = sosdecvar(prog,[gam gam1]);

% =============================================
% Next, define SOSP constraints

% Constraint : r(x)*(f(x) - gam) >= 0
f = -gam1 + 2*gam + 3;

prog = sosineq(prog,(f));

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