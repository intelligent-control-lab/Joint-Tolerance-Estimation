%% evaluation of the 6dof forward kinematics problem solution
% forward kinematics for original configuration

% robot definition 
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20]';

cpre = ForKine(theta_ini, robot.DH, robot.base, robot.cap);

%% sampling to verify the joint bound exact forward kinematics
% made sure cpre.x = 1.7421 

% precomputed lmd as the joint bound 
% for x wall 
% lmd = 0.0346; % x wall = 1.8

% for y axis wall 
lmd = 0.0265; % y wall = 0.45

% for z axis wall 
% lmd = 0.0230; % z wall = 1.3
% lmd = 0.0403; % z wall = 1.35

sample_num = 10000;
xpos_samples = zeros(sample_num,1);
% sampling a y vector within [-1,1]

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
    epos = ForKine_jointbound(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
%     xpos_samples(i) = epos(1); % x axis wall
    xpos_samples(i) = epos(2); % y axis wall 
%     xpos_samples(i) = epos(3); % z axis wall
end

figure
plot(xpos_samples,'.');
hold on 
% plot the solidline to demonstrate 1.8

% yline = 1.8 * ones(sample_num,1); % x axis wall
yline = 0.45 * ones(sample_num,1); % y axis walll 
% yline = 1.4 * ones(sample_num,1); % z axis walll 

plot(yline,'-','lineWidth',2);
hold on 
% limitation 
% ylim([1.4 1.9]);  % x axis wall
ylim([0 0.5]);  % y axis wall
% ylim([0 1.4]);  % z axis wall

%% sampling to verify the joint bound approximated forward kinematics
% precomputed lmd as the joint bound 
% xwall
% safe distance is 1.8 m 
dist = 1.8;
% lmd = 0.3283; % feasibility = e-3
% lmd = 0.0601; % feasibility = e-12
lmd = 0.0478; % feasibility = e-14

% safe distance is 1.83 m 
% dist = 1.83;
% % lmd = 0.3595; % feasibility = e-3
% % lmd = 0.0844; % feasibility = 5.992e-12
% lmd = 0.0639; % feasibility = 3.479e-14

% ywall 
dist = 0.45;
lmd = 0.0338;

sample_num = 10000;
xpos_approx_samples = zeros(sample_num,1);
% sampling a y vector within [-1,1]

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
    epos = ForKine_jointbound_approx(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    xpos_approx_samples(i) = epos(2);
end

figure
plot(xpos_approx_samples,'.');
hold on 
% plot the solidline to demonstrate 1.8
yline = dist * ones(sample_num,1);
plot(yline,'-','lineWidth',2);
hold on 
% limitation 
% ylim([0 1.9]);
ylim([0 0.5]);
disp(max(xpos_approx_samples));