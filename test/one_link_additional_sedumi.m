    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % weiye modification, new SOS formulation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Atn = zeros(3+4+4,6+4+4);
    Atn(1:3,1:6) = At';
    bn = zeros(3+4+4,1);
    bn(1:3,:) = b;
    
    % set lambda and lambda1 relationship
    Atn(4,7) = 1;
    bn(4,1) = 1;
    
    Atn(5,1) = 1;
    Atn(5,8) = -1;
    bn(5,1) = 0;
    
    Atn(6,1) = 1;
    Atn(6,9) = -1;
    bn(6,1) = 0;
    
    Atn(7,2) = 1;
    Atn(7,10) = -1;
    bn(7,1) = 0;
   
    % set y relationship
    Atn(8,5) = 4;
    Atn(8,11) = 1;
    bn(8,1) = 0;
    
    Atn(9,12) = 1;
    bn(9,1) = 1;
    
    Atn(10,13) = 1;
    bn(10,1) = 1;
    
    Atn(11,5) = 4;
    Atn(11,14) = 1;
    bn(11,1) = 0;

   
    c = zeros(6+4+4,1);
    c(1,1) = -1;
    K.s = [2 2 2];
    
    At = Atn';
    b = bn;