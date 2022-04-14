function [c,ceq] = nonlcon_onelink(x)
% nonlinear condition
%   ceq = 0
%   c <= 0

%% with sos quadratic relationship: non-sound approximation 
% ineq
% c1 = x(4)*x(5) - x(3)*x(6); % original ineq
A = [x(3),x(4);x(5),x(6)]';
ev = eig(A);
c1 = -ev;
c2 = 1/4 - x(4)^2; % y range ineq
c3 = -x(3); % additional constraints for sdp 
c = [c1;c2;c3]; % joint ineq 

% eq 
ceq = x(2) - x(1)^2; % lmd1 = lmd^2


%% with sos quadratic relationship: sound approximation 
% c1 = -det([x(5),x(6),x(7);x(8),x(9),x(10);x(11),x(12),x(13)]');
% c2 = -det([x(5),x(6);x(8),x(9)]');
% c3 = -det(x(5));
% c4 = -det([x(14),x(15);x(16),x(17)]');
% c5 = -det(x(14));
% c = [c1;c2;c3;c4;c5];
% % c = [c1;c2;c4];
% 
% ceq1 = x(2) - x(1)^2;
% ceq2 = x(3) - x(1)^3;
% ceq3 = x(4) - x(1)^4;
% ceq = [ceq1;ceq2;ceq3];

%% with muanually decomposition 
% c = -(x(3)*(x(4)*x(3) + sqrt(3)/2*x(1)) + 0.015);
% ceq = x(2) - x(1)^2; % lmd1 = lmd^2
end

