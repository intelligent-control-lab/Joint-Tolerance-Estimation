%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% weiye modification, new SOS formulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Atn = zeros(3+4,6+4);
Atn(1:3,1:6) = At';
bn = zeros(3+4,1);
bn(1:3,:) = b;

% set lambda and lambda1 relationship
Atn(4,1) = 1/5;
Atn(4,7) = -1;
bn(4,1) = 0;

Atn(5,8) = 1;
bn(5,1) = 1;

Atn(6,2) = 1;
Atn(6,9) = -1;
bn(6,1) = 0;

Atn(7,1) = 1/5;
Atn(7,10) = -1;
bn(7,1) = 0;


c = zeros(6+4,1);
c(1,1) = -1;
K.s = [2 2];

At = Atn';
b = bn;