%% Robot Property
% note: Please keep the parameter constant
function robot=robotproperty(id)
robot.name = id;
switch id    
        case 'GP50'
        %the constants
        robot.nlink=6; % robot links
        robot.umax=10; % max acceleration
        robot.delta_t=0.5; % the sample time
        
        robot.lb = [-pi;-pi;-pi;-pi;-pi;-pi];
        robot.ub = [pi;pi;pi;pi;pi;pi];
%         robot.DH=[0, 0.281, 0.145, -pi/2; % the DH parameter
%                   -pi/2, 0, 0.87, 0;
%                   0, 0, 0.21, -pi/2;
%                   0, 1.025, 0, pi/2;
%                   0, 0, 0, -pi/2;
%                   0, 0.675, 0, 0];%theta,d,a,alpha
        robot.DH=[0, 0.281, 0.145, -pi/2; % the DH parameter
                  -pi/2, 0, 0.87, 0;
                  0, 0, 0.21, -pi/2;
                  0, 1.025, 0, pi/2;
                  0, 0, 0, -pi/2;
                  0, 0.175, 0, 0];%theta,d,a,alpha
              
        robot.dfDH=[0, 0.281, 0.145, -pi/2; % the DH parameter
                  -pi/2, 0, 0.87, 0;
                  0, 0, 0.21, -pi/2;
                  0, 1.025, 0, pi/2;
                  0, 0, 0, -pi/2;
                  0, 0.66139, 0, 0];%theta,d,a,alpha  
%                   0, 0.675, 0, 0];%theta,d,a,alpha    
           
        robot.base=[0;0;0.259]; % base link offset
        
        robot.cap={};
        % robot capsule for each link. 
        % Current capsule is not specified. 
        % End-effector measured offset, with respect to last coorfinate
        robot.cap{1}.p = [-0.145 -0.145;0.105 0.105;0 0];
        robot.cap{1}.r = 0.385;
        
        robot.cap{2}.p = [-0.87 0;0 0;-0.1945 -0.1945];
        robot.cap{2}.r = 0.195;
        
        robot.cap{3}.p = [-0.02 -0.09;0.073 0.073;0.115 0.115];
        robot.cap{3}.r = 0.33;
        
        robot.cap{4}.p = [0 0;-0.65 0;-0.0235 -0.0235];
        robot.cap{4}.r = 0.115;
        
        robot.cap{5}.p = [0 0;0.0145 0.0145;0.025 0.025];
        robot.cap{5}.r = 0.15;

        
        robot.cap{6}.p1 = [-0.142391115489123,-0.00817305105262633;0.0844531341176073,0.0515485707207726;-0.448552826957168,-0.216598563316495];
        robot.cap{6}.p2 = [-0.00407589262314147,-0.00777057759414465;0.0819148210462628,0.0218344612710158;-0.209517946591771,-0.215902972600098];
        robot.cap{6}.p3 = [0.00252978728478826,0.000390496336481267;6.28378116607958e-10,1.00300828261106e-10;-0.170767309373314,-0.0344384157898974];
        robot.cap{6}.r1 = 0.065;
        robot.cap{6}.r2 = 0.05;
        robot.cap{6}.r3 = 0.03;

%         ratio = 0.3;
%         pleft = [-0.142391115489123;0.0844531341176073;-0.448552826957168];
%         pright = [-0.00817305105262633;0.0515485707207726;-0.216598563316495];
%         diff = pright - pleft;
%         prnew = pleft + ratio * diff;
%         robot.cap{6}.p1 = [pleft, prnew];
% 
% 
        ratio = 0.1;
        pleft = [0.00252978728478826;6.28378116607958e-10;-0.170767309373314];
        pright = [0.000390496336481267;1.00300828261106e-10;-0.0344384157898974];
        diff = pright - pleft;
        prnew = pleft + ratio * diff;
        robot.cap{6}.p3 = [pleft, prnew];
end

% the kinematic matrices
robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);
        zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);
        robot.delta_t*eye(robot.nlink)];
robot.Ac=[eye(3) robot.delta_t*eye(3);
        zeros(3) eye(3)];
robot.Bc=[0.5*robot.delta_t^2*eye(3);
        robot.delta_t*eye(3)];


end