function [c,ceq] = nonlcon2(x)
%nonlinear condition
%   ceq = 0
%   c <= 0

% w/o sos
% c = -x(1)^2 - 2*x(1) + 4 + x(2);
% ceq = [];

% with sos linear
% % c = x(3)*x(4) - x(2)*x(5);
% A = [x(2),x(3);x(4),x(5)]';
% ev = eig(A);
% c = -ev;
% ceq = [];

%% with sos quadratic relationship 
% c1 = x(4)*x(5) - x(3)*x(6);
% % c2 = x(2) - 1/25*x(1)^2;
% % c = [c1;c2];
% c = c1;

% A = [x(3),x(4);x(5),x(6)]';
% ev = eig(A);
% c = -ev;
% ceq = x(2) - 1/25*x(1)^2; % lmd1 = 1/25 lmd^2


%% y limite examination test

% y limit testing 
% c3 = -x(6);
% c1 = x(3)*x(4) - x(2)*x(5);
% c2 = -x(2);
% c3 = -x(6);
% c = [c1, c2, c3];


% one link case 
A = [x(3),x(4);x(5),x(6)]';
B = (A+A')/2;
c1 = -B(1,1);
c2 = -(B(1,1)*B(2,2) - B(2,1)*B(1,2));
c3 = -x(7);
c = [c1, c2, c3];
ceq = x(2) - x(1)^2; % lmd1 = lmd^2



% original definition
% c1 = x(3)*x(4) - x(2)*x(5);
% c3 = -x(2);
% c2 = x(3)^2 - 0.25; % y limit specification 
% c = [c1, c2, c3];

% symmetric PSD matrix
% A = [x(2),x(3);x(4),x(5)]';
% B = (A+A')/2;
% c1 = -B(1,1);
% c2 = -(B(1,1)*B(2,2) - B(2,1)*B(1,2));
% c3 = -B(2,2);
% c4 = x(3)^2 - 0.25; % y limit specification 
% % c = [c1;c2;c3];
% c = [c1;c2;c3;c4];


% eigen value condition 
% A = [x(2),x(3);x(4),x(5)]';
% ev = eig(A);
% c1 = -ev;
% c2 = x(3)^2 - 1; % y limit specification 
% c = [c1;c2];

end

