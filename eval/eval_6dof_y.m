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
% for y axis wall 
lmd = 0.0265; % y wall = 0.45

sample_num = 10000;
xpos_samples = zeros(sample_num,1);
% sampling a y vector within [-1,1]
violate = 0;
min_dist = 999;

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
    epos = ForKine_jointbound(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    xpos_samples(i) = epos(2); % y axis wall 
    
    % violation check
    if epos(2) > 0.45
        violate = violate + 1;
    end
    % update optimality 
    dist = 0.45 - epos(2);
    if dist < min_dist
        min_dist = dist;
    end
end

figure
plot(xpos_samples,'.');
hold on 
% plot the solidline to demonstrate 1.8

yline = 0.45 * ones(sample_num,1); % y axis walll 

plot(yline,'-','lineWidth',2);
xlabel('sample number');
ylabel('x coordinate / m'); 
hold on 
% limitation 
ylim([0.3 0.5]);  % y axis wall
disp( violate);
disp( min_dist);
