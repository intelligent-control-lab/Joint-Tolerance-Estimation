%% plot the experimental platform 
ROBOT = '2d2linkBot';
theta1 = pi/3;
theta2 = pi/6;
ys_ori = [theta1; theta2];

% forward kinematics 
xpos = cos(theta1) + cos(theta2);
ypos = sin(theta1) + sin(theta2);

point0 = [0;0];
point1 = [cos(theta1);sin(theta1)];
point2 = [xpos;ypos];

link1 = [point0 point1];
link2 = [point1 point2];

%% plot 

color1 = 1.5*[100,100,100]/255;
color2 = [150, 111, 51]/255;
r = 0.04;
figure 
hold on 

plot(link1(1,:),link1(2,:),'-','lineWidth',10,'color',color1);
plot(link2(1,:),link2(2,:),'-','lineWidth',10,'color',color1);
circle(point0(1),point0(2),r)
circle(point1(1),point1(2),r)
% circle(point2(1),point2(2),r)

%% plot the extra links 
% circle(point1(1)+0.3,point1(2));
% circle(point1(1)+0.6,point1(2));
% circle(point1(1)+0.9,point1(2));

circle(point2(1),point2(2),r);
circle(point2(1)+0.1,point2(2),0.005);
circle(point2(1)+0.15,point2(2),0.005);
circle(point2(1)+0.2,point2(2),0.005);

point3 = point2;
point3(1) = point3(1) + 0.3;
point4 = point3 + [cos(pi/4),sin(pi/4)];
link3 = [point3 point4];
plot(link3(1,:),link3(2,:),'-','lineWidth',10,'color',color1);
circle(point3(1),point3(2),r)
circle(point4(1),point4(2),r)

% plot the plane 
bias = 2.8;
% plane1 = [0, bias]';
% plane2 = [bias, 0]';
plane1 = [point4(2) + 0.6, 0]';
plane2 = [point4(2) + 0.6, 2.8]';
plane = [plane1 plane2];
plot(plane(1,:),plane(2,:),'-','lineWidth',7,'color',color2);

h=gca; 
h.XAxis.TickLength = [0 0];
h.YAxis.TickLength = [0 0];
set(gca,'XTick',[], 'YTick', [])

axis equal

function h = circle(x,y,r)
    hold on
    color = [0,0,0]/255;
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit,'lineWidth',3,'color',color);
    hold on
end