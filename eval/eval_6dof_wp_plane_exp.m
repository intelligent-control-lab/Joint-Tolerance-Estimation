%% evaluation of the 6dof forward kinematics problem solution
% forward kinematics for original configuration
ROBOT = 'GP50';
robot=robotproperty(ROBOT);

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
lmd = 0.0302; % optimal joint bound without normalization
% lmd = 0.0364; % optimized joint bound with normalization 

sample_num = 10000;
xpos_samples = zeros(10000,1);
% sampling a y vector within [-1,1]
violate = 0;
min_dist = 999;

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
%     epos = ForKine_jointbound(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    epos = ForKine_jointbound_approx(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    proj = proj_plane(epos, plane); % get projection point on the plane
    vec = epos - proj;
    dist = dot(n,vec);
    xpos_samples(i) = dist; % y axis wall 
    
    % violation check
    if dist < 0
        violate = violate + 1;
    end
    % update optimality 
    if dist < min_dist
        min_dist = dist;
    end
end

figure
plot(xpos_samples,'.');
hold on 

yline = 0 * ones(10000,1); % constraint distance to plane > 0

plot(yline,'-','lineWidth',2);
hold on 
xlabel('sample number');
ylabel('safe distance / m'); 
ylim([-0.05 0.3]);  % distance to plane 
disp( violate);
disp( min_dist);