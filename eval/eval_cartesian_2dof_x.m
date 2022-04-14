%% evaluation of the 6dof forward kinematics problem solution
% forward kinematics for original configuration

% robot definition 
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20]';

% cpre = ForKine(theta_ini, robot.DH, robot.base, robot.cap);
nlink = 2;
cpre = get_endpose(theta_ini,robot.DH,robot.base,robot.cap,nlink);
xwall = cpre(1) + 0.02; %it is 2cm away 

%% sampling to verify the joint bound exact forward kinematics
% made sure cpre.x = 1.7421 

% precomputed lmd as the joint bound 
% for x wall 
lmd = 0.02124;


sample_num = 100000;
xpos_samples = zeros(sample_num,1);
% sampling a y vector within [-1,1]
violate = 0;
min_dist = 999;

for i = 1:sample_num
    ys = -1 + 2*rand(nlink,1);
    epos = ForKine_jointbound_casual_dof(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd,nlink);
    xpos_samples(i) = epos(1); % x axis wall
    
    % violation check
    if epos(1) > xwall
        violate = violate + 1;
    end
    % update optimality 
    dist = xwall - epos(1);
    if dist < min_dist
        min_dist = dist;
    end
end

figure
plot(xpos_samples,'.');
hold on 
% plot the solidline to demonstrate xwall

yline = xwall * ones(sample_num,1); % x axis wall

plot(yline,'-','lineWidth',4);
% xlabel('sample number');
% ylabel('x coordinate / m'); 
hold on 
% limitation 
ylim([xwall-0.04 xwall+0.01]);  % x axis wall
h=gca; 
h.XAxis.TickLength = [0 0];
h.YAxis.TickLength = [0 0];
set(gca,'XTick',[], 'YTick', [0.125, 0.145, 0.165])
a = get(gca,'YTickLabel');
set(gca,'YTickLabel',a,'FontName','Times','fontsize',24)

fig=gcf;
fig.Position(3:4)=[200,100];

disp( violate);
disp( min_dist);

