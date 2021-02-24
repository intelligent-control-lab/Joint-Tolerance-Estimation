function [c,ceq] = nonlcon_hierarchy_fk(x,Q,Q1,Q2,Q3,Q4,Q5,Q6,Q_cons)
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
g = x(6);
h = x(7);
k = x(8);

% incorporate the lmd into Q matrix
% list the index
% y number for the vector
vec_ynum = zeros(64,1);
for num = 0:2^6-1
    dec_num = dec2binary_6dof(num);
    sum_y = sum(dec_num);
    vec_ynum(num+1) = sum_y;
end

Q_lmd = zeros(64,64);
for i = 1:2^6
    for j = 1:2^6
        Q_lmd(i,j) = lmd^(vec_ynum(i) + vec_ynum(j));
    end
end

Q_lmd_final = Q .* Q_lmd;

% sum of Q matrix
Q_sum = -1*(Q_lmd_final + c*Q1 + d*Q2 + e*Q3 + g*Q4 + h*Q5 + k*Q6 + b*Q_cons);

% get the nonlinear constraints for SDP
c = [];
for i = 1:64
    c = [c; -det(Q_sum(1:i,1:i))];
end

%% eq
ceq = [];

end

