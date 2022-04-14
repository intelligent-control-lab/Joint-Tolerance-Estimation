%% compute the maximum s1s2... - x1x2.. 
function d_sin_max = delta_sin_max(lmd, sin_num)
    d_sin_max = lmd^sin_num - sin(lmd)^sin_num;
end