function d_cos_m = delta_cos_max(lmd, sin_num, cos_num)
    d_cos_m = sin(lmd)^sin_num * (1 - (1 - lmd^2/2)^cos_num);
end