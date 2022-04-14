%% evaluation of the 2d plane with 2d  forward kinematics problem solution
% forward kinematics for original configuration

% robot definition 
% robot configurations
ROBOT = '2d3linkBot';
theta1 = pi/4;
theta2 = pi/4;
theta3 = pi/4;
ys_ori = [theta1; theta2; theta3];


% forward kinematics 
xpos = cos(theta1) + cos(theta2) + cos(theta3);
ypos = sin(theta1) + sin(theta2) + sin(theta3);

xdist = 0.09;

% computed joint tolerance: JTE V.S. Adversarial Optimization 
% lmd = 0.0428; $ JTE
lmd = 0.0433; % Adversarial 


%% sampling to verify the joint bound approximated forward kinematics
% precomputed lmd as the joint bound 
xwall = xpos + xdist;

sample_num = 1000000;
xpos_approx_samples = zeros(sample_num,1);
violate = 0;
min_dist = 999;

for i = 1:sample_num
    ys = -1 + 2*rand(3,1); % sampling a y vector within [-1,1]
    ys_pert = ys*lmd + ys_ori; % perturbed y vector
    xpos_per = cos(ys_pert(1)) + cos(ys_pert(2)) + cos(ys_pert(3));
    ypos_per = sin(ys_pert(1)) + sin(ys_pert(2)) + sin(ys_pert(3));
    xpos_approx_samples(i) = xpos_per; % x wall 
%     xpos_approx_samples(i) = ypos_per; % y wall 
    % violation check
    if xpos_per > xwall
        violate = violate + 1;
    end
    % update optimality 
    dist = xwall - xpos_per;
    if dist < min_dist
        min_dist = dist;
    end
end

figure
plot(xpos_approx_samples,'.');
hold on 
% plot the solidline to demonstrate 1.8
yline = xwall * ones(sample_num,1); % x wall
% yline = ywall * ones(sample_num,1); % x wall
plot(yline,'-','lineWidth',2);
hold on 
% limitation 
xlabel('sample number');
ylabel('x coordinate / m');
% ylabel('y coordinate / m');
ylim([xwall-0.2 xwall + 0.1]); % x wall 
% ylim([ywall-0.15, ywall + 0.1]); % y wall 
% disp(max(xpos_approx_samples));
disp( violate);
disp( min_dist);