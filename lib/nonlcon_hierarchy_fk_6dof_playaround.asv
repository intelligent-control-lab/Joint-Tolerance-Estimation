function [c,ceq] = nonlcon_hierarchy_fk_6dof_playaround(x,Q,Q1,Q2,Q3,Q4,Q5,Q6,Q_cons)
%nonlinear condition
%   ceq = 0
%   c <= 0

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
g = x(7);
h = x(8);

% incorporate the lmd into Q matrix
% list the index
% y number for the vector
dim = 2^6; % this is the 6dof robot
vec_ynum = zeros(dim,1);
for num = 0:dim-1
    dec_num = dec2binary_6dof(num);
    sum_y = sum(dec_num);
    vec_ynum(num+1) = sum_y;
end

Q_lmd = zeros(dim,dim);
for i = 1:dim
    for j = 1:dim
        Q_lmd(i,j) = lmd^(vec_ynum(i) + vec_ynum(j));
    end
end

Q_lmd_final = Q .* Q_lmd;

% sum of Q matrix
Q_sum = -1*(Q_lmd_final + c*Q1 + d*Q2 + e*Q3 + f*Q4 + g*Q5 + h*Q6 + b*Q_cons);

% get the nonlinear constraints for PD
c = [];
for i = 1:dim
    c = [c; -det(Q_sum(1:i,1:i))];
end
% c = c + 1e-12;
% c = max(c);

% c=-eig(Q_sum);

%% eq
ceq = [];

end

