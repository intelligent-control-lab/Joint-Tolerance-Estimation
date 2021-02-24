% SOSDEMO3 --- Bound on Global Extremum
% Section 3.3 of SOSTOOLS User's Manual
% 

clear; echo on;
syms x gam gam1;
% vartable = [x1, x2];
vartable = [x];
% =============================================
% First, initialize the sum of squares program
prog = sosprogram(vartable);

% =============================================
% Declare decision variable gam too
prog = sosdecvar(prog,[gam]);
% prog = sosdecvar(prog,[gam, gam1]);

% =============================================
% Next, define SOSP constraints

% Constraint : r(x)*(f(x) - gam) >= 0
% f(x) is the Goldstein-Price function

% y limit testing

% one link problem 
% f = 0.25*x1^2*gam1 + sqrt(3)/2*x1*gam + 0.25;
% f = x^2 + gam*x + 1;
% prog = sosineq(prog,(f),[-1,1]);


% simplest problem to test SOS hierarchy 
f = x - gam;
% prog = sosineq(prog,(f),[-1,1]);
prog = sosineq(prog,(f));


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
SOLgamma = sosgetsol(prog,gam)
echo off