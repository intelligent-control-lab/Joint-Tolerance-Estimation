%% unit test
clc
clear
vec_ynum = zeros(64,1);
for num = 0:2^6-1
    dec_num = dec2binary_6dof(num);
    sum_y = sum(dec_num);
    vec_ynum(num+1) = sum_y;
end

Q_lmd = zeros(64,64);
for i = 1:2^6
    for j = 1:2^6
        Q_lmd(i,j) = vec_ynum(i) + vec_ynum(j);
    end
end

Q = 2*ones(64,64);
Q1 = Q_lmd .*Q; 