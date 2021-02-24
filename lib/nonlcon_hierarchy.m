function [c,ceq] = nonlcon_hierarchy(x)
%nonlinear condition
%   ceq = 0
%   c <= 0

%% 1st hierarchy problem 
%% ineq
% test case 1
% A = [x(2),x(3);x(4),x(5)];
% B = (A+A')/2;
% c1 = -B(1,1);
% c2 = -(B(1,1)*B(2,2) - B(2,1)*B(1,2));
% c3 = -x(6);
% c = [c1;c2;c3];

% test case 2
% A = [x(2),x(3);x(4),x(5)];
% B = (A+A')/2;
% c1 = -B(1,1);
% c2 = -(B(1,1)*B(2,2) - B(2,1)*B(1,2));
% c3 = -x(6);
% c = [c1;c2;c3];

% test case 3
% A = [x(2),x(3);x(4),x(5)];
% B = (A+A')/2;
% c1 = -B(1,1);
% c2 = -(B(1,1)*B(2,2) - B(2,1)*B(1,2));
% c3 = -x(6);
% c4 = -x(7);
% c = [c1;c2;c3;c4];

% test case 6 (eg6)
% c = 3/4*x(1)^2 - (x(1)^2 + 4*x(2))*(1/4 - x(2) - x(3));
% lmd = x(1);
% b = x(2);
% c = x(3);
% Q = [0.25*b-c-1, sqrt(3)/4*lmd*b;
%      sqrt(3)/4*lmd*b, 0.25*lmd^2*b+c];
% c = -det(Q);

% test case 6.1 (eg6.1)
% Q = [x(5) x(6);
%      x(6) x(7)];
% c1 = -Q(1,1);
% c2 = -(Q(1,1)*Q(2,2) - Q(2,1)*Q(1,2));
% c = [c1;c2];

% test case 7 (two link)
% Q = [x(6) x(7) x(8);
%      x(7) x(9) 0;
%      x(8) 0 x(10)];
% Q2 = [x(6) x(7);
%       x(7) x(9)];
% c1 = -Q(1,1);
% % c2 = -(Q(1,1)*Q(2,2) - Q(2,1)*Q(1,2));
% c2 = -det(Q2);
% c3 = -det(Q);
% % c4 = -[x(1);x(2);x(3);x(4);x(5)];
% c = [c1;c2;c3];

% test case 7.1 (two link) simplified 
lmd = x(1);
a = x(2);
b = x(3);
c = x(4);
Q = [(0.034-0.01)*a-b-c-1, sqrt(3)/4*lmd*a, lmd/4*a;
    sqrt(3)/4*lmd*a, lmd^2/4*a+b, 0;
    lmd/4*a, 0, sqrt(3)/4*lmd^2*a+c];
c1 = -Q(1,1);
c2 = -det(Q(1:2,1:2));
c3 = -det(Q);
c = [c1;c2;c3];


%% eq
% test case 1
% ceq1 = x(4)+x(3) + x(1)*x(6) - x(6);
% ceq2 = x(2) + x(1)*x(6) + 1;
% ceq = [ceq1;ceq2]; 

% test case 2 
% ceq1 = x(2) + x(6) + x(1)*x(6) + 1;
% ceq = [ceq1];

% test case 4
% ceq1 = x(4)+x(3) + x(1)*x(6) - x(6) + x(7);
% ceq2 = x(2) + x(1)*x(6) + 1 + x(7);
% ceq = [ceq1;ceq2]; 

% test case 5
% ceq = [];

% test case 6
% ceq = [];

% test case 6.1
% ceq1 = x(5) - x(3)/4 + x(4) + 1;
% ceq2 = x(6) - sqrt(3)/4*x(1)*x(3);
% ceq3 = x(7) - 1/4*x(2)*x(3) - x(4);
% ceq4 = x(2) - x(1)^2;
% ceq = [ceq1;ceq2;ceq3;ceq4];

% test case 7 (two link)
% ceq1 = x(9) - x(2)/4*x(3) - x(4);
% ceq2 = x(7) - sqrt(3)/4*x(1)*x(3);
% ceq3 = x(10) - sqrt(3)/4*x(2)*x(3) - x(5);
% ceq4 = x(8) - x(1)/4*x(3);
% ceq5 = x(6) - 0.034*x(3) + x(4) + x(5) + 1;
% ceq6 = x(2) - x(1)^2;
% ceq = [ceq1;ceq2;ceq3;ceq4;ceq5;ceq6];

% test case 7.1 (two link) simplified
ceq = [];

end

