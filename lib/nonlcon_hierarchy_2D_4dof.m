function [c,ceq] = nonlcon_hierarchy_2D_4dof(x,Q,Q1,Q2,Q3,Q4,Q_cons)
%nonlinear condition
%   ceq = 0
%   c <= 0
% the nonlinear function for 

%% 1st hierarchy problem 
%% ineq
% test case for forward kinematics
% lmdb b c d e g h k 
lmd = x(1);
b = x(2);
c = x(3);
d = x(4);
e = x(5);
f = x(6);

% incorporate the lmd into Q matrix
% list the index
% y number for the vector
Q_lmd = [1 lmd lmd lmd lmd;
         lmd lmd^2 lmd^2 lmd^2 lmd^2;
         lmd lmd^2 lmd^2 lmd^2 lmd^2;
         lmd lmd^2 lmd^2 lmd^2 lmd^2;
         lmd lmd^2 lmd^2 lmd^2 lmd^2];

Q_lmd_final = Q .* Q_lmd;

% sum of Q matrix
Q_sum = -1*(b*Q_lmd_final + c*Q1 + d*Q2 + e*Q3 + f*Q4 + Q_cons);

% get the nonlinear constraints for SDP
c = [];
assert(size(Q_sum,1) == 5 && size(Q_sum,2) == 5)
for i = 1:5 % 2d plan 2d link case: Q is 5*5 dimension
    c = [c; -det(Q_sum(1:i,1:i))];
end
c = c + 1e-12;
c = max(c);

% c = -eig(Q_sum);

%% eq
ceq = [];

end

