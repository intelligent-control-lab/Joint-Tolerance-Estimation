%% examine the sound function to approximate the cos and sin function 
clc
clear

%% cos function approximation, upper bound 
% x = -pi/2:0.01:pi/2;
% y = [];
% for i = 1:size(x,2)
%     tmp = x(i);
%     y = [y 1 - tmp^2/2 + 1/24*tmp^4 - cos(tmp)];
% end
% plot(x,y);

%% sin function, lower bound 
% x = -pi/2:0.01:pi/2;
% % x = 0:0.01:pi/2;
% y = [];
% for i = 1:size(x,2)
%     t = x(i);
%     y = [y sin(t) - t + 1/6*t^3 + 0.08];
% end
% plot(x,y);

%% plot other function examination 
x = -1:0.01:1;
y = [];
for i = 1:size(x,2)
    tmp = x(i);
    y = [y -2/tmp - tmp];
end
plot(x,y);

%% SIMPLE sin function 
% x = -0.1:0.001:0.1;
% y = [];
% for i = 1:size(x,2)
%     tmp = x(i);
%     % sin 
% %     y = [y sin(tmp)-tmp];
%     % cos 
%     y = [y cos(tmp)-(1 - tmp^2/2)];
% end
% plot(x,y);

%% test one link approximation 
% thres = 1;
% x = -thres:0.01:thres;
% f = []
% lmd = 0.326;
% for i = 1:size(x,2)
%     y = x(i);
%     f = [f 0.25 * lmd^2 * y^2 + sqrt(3)/2 * lmd * y + 0.25];
% %     f = [f 0.25 * y^2 + sqrt(3)/2 * y + 0.25];
% %     f = [f 3/4 - 0.5*(1 - 0.5*y^2 + 1/24*y^4) + sqrt(3)/2*(y - 1/6*y^3 - 0.08)];
% end
% plot(x,f);

