function [c,ceq] = nonlcon_hierarchy_2D_ndof(x,Q,Qs,Q_cons)
%nonlinear condition
%   ceq = 0
%   c <= 0
% the nonlinear function for 

%% 1st hierarchy problem 
%% ineq
lmd = x(1);

% incorporate the lmd into Q matrix
Q_lmd = ones(size(Q));
for i = 2:size(Q,2)
    Q_lmd(1,i) = lmd;
    Q_lmd(i,1) = lmd;
end
for i = 2:size(Q,2)
    for j = 2:size(Q,1)
        Q_lmd(i,j) = lmd^2;
    end
end

Q_lmd_final = Q .* Q_lmd;

% sum of Q matrix
Q_sum = x(2)*Q_lmd_final;
assert(size(Qs,2) + 2 == size(x,2));
for i = 3:size(x,2)
    Q_sum = Q_sum + x(i)*cell2mat(Qs(i-2));
end
Q_sum = Q_sum + Q_cons;
Q_sum = -1*Q_sum;

% get the nonlinear constraints for SDP
c = [];
for i = 1:size(Q_sum,2)
    c = [c; -det(Q_sum(1:i,1:i))];
end
c = c + 1e-12;
c = max(c);

% c = -eig(Q_sum);

%% eq
ceq = [];

end

