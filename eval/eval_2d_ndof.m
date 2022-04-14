%% evaluation of the 2d plane with 2d  forward kinematics problem solution
% forward kinematics for original configuration

% robot definition 
% robot configurations
ROBOT = '2dnlinkBot';
% ys_ori = [theta1; theta2; theta3; theta4; theta5; theta6];
nlink = 20;
ys_ori = [];
for i = 1:nlink
    ys_ori = [ys_ori;pi/4];
end

% forward kinematics 
xpos = 0;
for i = 1:nlink
    xpos = xpos + cos(pi/4);
end

xdist = 0.09;

% 100 random seed computation 
% lmd = 0.0202; % adversarial computed 50
% lmd = 0.0143; % adversarial computed 25
% lmd = 0.03196; % adversarial computed 20
% lmd = 0.02144; % adversaial computed 6
% lmd = 0.0128; % adversaial computed 10
% lmd = 0.0289; % adversaial computed 12 (from 12, the adversarial is hardly working)
lmd = 0.0063; % JTE optimization 



%% sampling to verify the joint bound approximated forward kinematics
% precomputed lmd as the joint bound 
xwall = xpos + xdist;
rng(4);
sample_num = 1000000;
xpos_approx_samples = zeros(sample_num,1);
violate = 0;
min_dist = 999;

for i = 1:sample_num
    ys = -1 + 2*rand(nlink,1); % sampling a y vector within [-1,1]
    ys_pert = ys*lmd + ys_ori; % perturbed y vector

    xpos_per = 0;
    for j = 1:nlink
        xpos_per = xpos_per + cos(ys_pert(j));
    end
%     xpos_per = cos(ys_pert(1)) + cos(ys_pert(2)) + cos(ys_pert(3)) + cos(ys_pert(4)) + cos(ys_pert(5)) + cos(ys_pert(6));
    xpos_approx_samples(i) = xpos_per; % x wall 
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