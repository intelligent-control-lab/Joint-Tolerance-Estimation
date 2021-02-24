%% fmincon solution for find joint limit 

%% fmincon solution 
clear; echo on;
syms u1 u2 u3 u4 y;
vartable = [y];
% =============================================
% First, initialize the sum of squares program
prog = sosprogram(vartable);

% =============================================
% Declare decision variable gam too
prog = sosdecvar(prog,[u1 u2 u3 u4]);

% =============================================
% Next, define SOSP constraints

% Constraint : f(y) >= 0

% f = 0.25*y^2*u2 + sqrt(3)/2*y*u1 + 0.25;
f = 3/4 - 0.5*(1 - 0.5*y^2*u2 + 1/24*y^4*u4) + sqrt(3)/2*(y*u1 - 1/6*y^3*u3 - 0.08);
g = y^2 - 1;


prog = sosineq(prog,(f));
prog = sosineq(prog,(g));
% prog = sosineq(prog,(gam-gam2^2));

% =============================================
% Set objective : maximize gam
prog = sossetobj(prog,-u1-u2-u3-u4);

% =============================================
% And call solver
solver_opt.solver = 'sedumi';
prog = sossolve(prog,solver_opt);
% =============================================
% Finally, get solution
SOLgamma = sosgetsol(prog,u1)
echo off