%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% weiye modification, new SOS formulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Atn = zeros(1+4,3+4);
Atn(1,1:3) = At';
bn = zeros(1+4,1);
bn(1,:) = b;

% set lambda and lambda1 relationship
Atn(2,1) = 1;
Atn(2,4) = -1;
bn(2,1) = 0;

Atn(3,5) = 1;
bn(3,1) = 1;

Atn(4,2) = 1;
Atn(4,6) = -1;
bn(4,1) = 0;

Atn(5,1) = 1;
Atn(5,7) = -1;
bn(5,1) = 0;

% set inverse lambda and lambda1 relationship
Atn(6,8) = 1;
bn(6,1) = 1;

Atn(7,1) = 1;
Atn(7,9) = -1;
bn(7,1) = 0;

Atn(8,1) = 1;
Atn(8,10) = -1;
bn(8,1) = 0;

Atn(9,2) = 1;
Atn(9,11) = -1;
bn(9,1) = 0;


c = zeros(3+4+4,1);
c(1,1) = -1;
K.s = [1 2 2];

At = Atn';
b = bn;