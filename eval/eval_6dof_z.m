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
% for z axis wall 
lmd = 0.035 ; % z wall = 1.35

sample_num = 10000;
xpos_samples = zeros(sample_num,1);
% sampling a y vector within [-1,1]
violate = 0;
min_dist = 999;

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
    epos = ForKine_jointbound(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    xpos_samples(i) = epos(3); % z axis wall
    
    % violation check
    if epos(2) > 1.35
        violate = violate + 1;
    end
    % update optimality 
    dist = 1.35 - epos(3);
    if dist < min_dist
        min_dist = dist;
    end
end

figure
plot(xpos_samples,'.');
hold on 
% plot the solidline to demonstrate 1.8

% yline = 1.8 * ones(sample_num,1); % x axis wall
% yline = 0.45 * ones(sample_num,1); % y axis walll 
yline = 1.35 * ones(sample_num,1); % z axis walll 

plot(yline,'-','lineWidth',2);
xlabel('sample number');
ylabel('z coordinate / m'); 
hold on 
% limitation 
% ylim([1.65 1.85]);  % x axis wall
% ylim([0.3 0.5]);  % y axis wall
ylim([1 1.45]);  % z axis wall
disp( violate);
disp( min_dist);