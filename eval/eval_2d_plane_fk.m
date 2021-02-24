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

bias = 2.8; % 2.7321 < 2.8 < 2.8284
lmd = 0.1145; % seed = 1, w/o normalization

% precomputed the positive norm vector 
anchor_point = [bias;0]; % y = 0, x = bias, due to y = -x + bias
vec_norm = [-1,-1];
vec_norm = vec_norm / norm(vec_norm); % positive direction of norm vector pependicular to plane

%% sampling to verify the joint bound approximated forward kinematics
% precomputed lmd as the joint bound 
sample_num = 30000;
xpos_approx_samples = zeros(sample_num,1);


for i = 1:sample_num
    ys = -1 + 2*rand(2,1); % sampling a y vector within [-1,1]
    ys_pert = ys*lmd + ys_ori; % perturbed y vector
    xpos_per = cos(ys_pert(1)) + cos(ys_pert(2));
    ypos_per = sin(ys_pert(1)) + sin(ys_pert(2));
%     xpos_approx_samples(i) = xpos_per; % x wall 
    % compute the distance to the plane 
    epos = [xpos_per;ypos_per];
    vec_epos = epos - anchor_point;
    dist = dot(vec_epos,vec_norm);
    xpos_approx_samples(i) = dist; % y wall 
end

figure
plot(xpos_approx_samples,'.');
hold on 
% plot the solidline to demonstrate 1.8
% yline = xwall * ones(sample_num,1); % x wall
yline = 0 * ones(sample_num,1); % x wall
plot(yline,'-','lineWidth',2);
hold on 
% limitation 
% ylim([0 xwall + 0.2]); % x wall 
ylim([0-0.2, 0 + 0.2]); % x wall 
disp(max(xpos_approx_samples));