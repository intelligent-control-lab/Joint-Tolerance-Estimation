%% get the towards hole normal 
function unitnorm = normal2hole(planes,point)
if nargin < 2
    point = [1.427, 0, 1.459];
end
if nargin == 2
    point = point';
end
unit2h = [];
for i = 1:size(planes,1)
    normal = planes(i,1:3);
    unitn = normal / norm(normal);
    d = planes(i,4);
    c = planes(i,3);
    O = [0,0,-d/c];
    OP = point - O; % OP vector 
    if dot(OP,normal) > 0
        unit2h = [unit2h;unitn];
    else
        unit2h = [unit2h;-unitn];
    end
end
unitnorm = unit2h(1,:);

%% visualize the vectors toward the hole 
% figure
% pn = 5;
% p0 = [1.427, 0, 1.459];
% p1 = p0 + unit2h(pn,:);
% vectarrow(p0,p1);
% hold on 
% 
% 
% d = planes(pn,4);
% c = planes(pn,3);
% a = planes(pn,1);
% O = [0,0,-d/c];
% O1 = [1,0,-(d+a)/c];
% vecp = O1 - O;
% disp(dot(vecp,unit2h(pn,:)));
% 
% 
% vectarrow(O,O1);
% hold on
% 
% x1 = 1.2:0.1:2.3;
% z1 = 0.8:0.02:1.6;
% [x, z] = meshgrid(x1,z1);
% % xyzread()
% 
% subs = [-0.02832, -0.056102, -1, 0.289227];
% 
% planes = [0.87419, 1.00458, -1, -3.03183;
%           -0.61667, -0.79248, -1, 1.92910;
%           -0.02832, -0.056102, -1, 0.289227;
%           0.68587, 0.96291, -1, -2.24876;
%           -0.88556, -1.01449, -1, 3.06574];
% 
% planes = planes(pn,:);
% for i = 1:1
% %     z = planes(i,1)*x + planes(i,2)*y + planes(i,4);
%     y = planes(i,1)*x + planes(i,2)*z + planes(i,4);
%     surf(x,y,z);
%     hold on 
% end
% 
% 
% point = [1.427, 0, 1.459];
% plot3(point(1), point(2), point(3), '*');
% title('My title')
% xlabel('My x label')
% ylabel('My y label')
% zlabel('My z label')
% 

