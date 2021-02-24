function [c,ceq] = nonlcon1(x)
%nonlinear condition for one link example
%   ceq = 0
%   c <= 0
c = x^2 - 2*x - 3;
ceq = [];
end

