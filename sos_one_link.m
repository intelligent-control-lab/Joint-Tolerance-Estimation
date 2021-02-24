% SOSDEMO3 --- Bound on Global Extremum
% Section 3.3 of SOSTOOLS User's Manual
% 

clear; echo on;
syms u1 u2 y;
vartable = [y];
% =============================================
% First, initialize the sum of squares program
prog = sosprogram(vartable);

% =============================================
% Declare decision variable gam too
prog = sosdecvar(prog,[u1,u2]);

% =============================================
% Next, define SOSP constraints

% Constraint : f(y) >= 0

f = 0.25*y^2*u2 + sqrt(3)/2*y*u1 + 0.25;
% f = 0.25*y^2*u2 + sqrt(3)/2*y*u1 + 0.015;
% f = 0.25*y^2 + sqrt(3)/2*y + 0.25;
% g = 1 - y^2;

prog = sosineq(prog,(f),[-1,1]);
% prog = sosineq(prog,(g));
% prog = sosineq(prog,(gam-gam2^2));
% =============================================
% Set objective : maximize gam
prog = sossetobj(prog,-u1);

% =============================================
% And call solver
solver_opt.solver = 'sedumi';
prog = sossolve(prog,solver_opt);
% =============================================
% Finally, get solution
SOLgamma = sosgetsol(prog,u1)
echo off