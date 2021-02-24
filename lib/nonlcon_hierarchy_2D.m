function [c,ceq] = nonlcon_hierarchy_2D(x,Q,Q1,Q2,Q_cons)
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

% incorporate the lmd into Q matrix
% list the index
% y number for the vector
Q_lmd = [1 lmd lmd;
         lmd lmd^2 lmd^2;
         lmd lmd^2 lmd^2];

Q_lmd_final = Q .* Q_lmd;

% sum of Q matrix
Q_sum = -1*(b*Q_lmd_final + c*Q1 + d*Q2 + Q_cons);

% get the nonlinear constraints for SDP
c = [];
assert(size(Q_sum,1) == 3 && size(Q_sum,2) == 3)
for i = 1:3 % 2d plan 2d link case: Q is 3*3 dimension
    c = [c; -det(Q_sum(1:i,1:i))];
end

%% eq
ceq = [];

end

