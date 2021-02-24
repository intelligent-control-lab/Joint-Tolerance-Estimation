%% evaluation of the 2d plane with 2d  forward kinematics problem solution
% forward kinematics for original configuration

% robot definition 
% robot configurations
ROBOT = '2d2linkBot';
theta1 = pi/3;
theta2 = pi/6;
ys_ori = [theta1; theta2];


% forward kinematics 
xpos = cos(theta1) + cos(theta2);
ypos = sin(theta1) + sin(theta2);

% cases 
% xdist = 0.2;
% lmd = 0.1591; % feasibility = e-14

% xdist = 0.25;
% lmd = 0.2038;

% xdist = 0.15;
% lmd = 0.1166;

% xdist = 0.07;
% lmd = 0.0526;

ydist = 0.05;
lmd = 0.0372;

%% sampling to verify the joint bound approximated forward kinematics
% precomputed lmd as the joint bound 
xwall = xpos + xdist;
ywall = xpos + ydist;

sample_num = 30000;
xpos_approx_samples = zeros(sample_num,1);


for i = 1:sample_num
    ys = -1 + 2*rand(2,1); % sampling a y vector within [-1,1]
    ys_pert = ys*lmd + ys_ori; % perturbed y vector
    xpos_per = cos(ys_pert(1)) + cos(ys_pert(2));
    ypos_per = sin(ys_pert(1)) + sin(ys_pert(2));
%     xpos_approx_samples(i) = xpos_per; % x wall 
    xpos_approx_samples(i) = ypos_per; % y wall 
end

figure
plot(xpos_approx_samples,'.');
hold on 
% plot the solidline to demonstrate 1.8
% yline = xwall * ones(sample_num,1); % x wall
yline = ywall * ones(sample_num,1); % x wall
plot(yline,'-','lineWidth',2);
hold on 
% limitation 
% ylim([0 xwall + 0.2]); % x wall 
ylim([ywall-0.2, ywall + 0.2]); % x wall 
disp(max(xpos_approx_samples));