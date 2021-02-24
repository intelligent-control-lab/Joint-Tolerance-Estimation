%% evaluation of the 6dof forward kinematics problem solution
% forward kinematics for original configuration


%% robot set up parameters 
% real theta 
theta_ini = [-0.0452   -1.5247    0.2224   -0.0072    0.1575   -1.9004]';
% anchor poitn 
anchor_point = [1.6343;0.0060;1.1336];
%epos = FK
epos = ForKine(theta_ini, robot.DH, robot.base, robot.cap);
plane = [-0.4758   -0.0135   -1.0000    1.7601];
n = normal2hole(plane,anchor_point);
%% sampling to verify distance currently is positive
% lmd = 0.0302; % optimal joint bound 
lmd = 0.0364; % optimized joint bound 

sample_num = 10000;
xpos_samples = zeros(10000,1);
% sampling a y vector within [-1,1]

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
%     epos = ForKine_jointbound(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    epos = ForKine_jointbound_approx(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    proj = proj_plane(epos, plane); % get projection point on the plane
    vec = epos - proj;
    dist = dot(n,vec);
    xpos_samples(i) = dist; % y axis wall 
end

figure
plot(xpos_samples,'.');
hold on 

yline = 0 * ones(10000,1); % constraint distance to plane > 0

plot(yline,'-','lineWidth',2);
hold on 
ylim([-0.5 0.5]);  % distance to plane 
