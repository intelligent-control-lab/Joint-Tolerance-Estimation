function [c,ceq] = nonlcon_hierarchy_fk_64(x,Q,Q_cons)
%nonlinear condition
%   ceq = 0
%   c <= 0

%% 1st hierarchy problem 
%% ineq
% test case for forward kinematics
% lmdb b c d e g h k 
lmd = x(1);
b = x(2);
coeff=x(3:66);

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

%% generating Qf
Qf=zeros(64,64);
for num = 1:2^6-1
    qi=zeros(64,1);
    dec_num = dec2binary_6dof(num);
    for j=0:2^6-1
        dec_j = dec2binary_6dof(j);
        if min(dec_num - dec_j)>=0
            qi(j+1)=(-1)^sum(dec_j);
        end
    end
    
    Qi=diag(qi);
    Qf=Qf+Qi*coeff(num+1);
end

%% finishing

% sum of Q matrix
Q_sum = -1*(Q_lmd_final + Qf + b*Q_cons);

% get the nonlinear constraints for SDP
c = [];
% for i = 1:64
%     c = [c; -det(Q_sum(1:i,1:i))];
% end
c=-eig(Q_sum);
%% eq
ceq = [];

end

