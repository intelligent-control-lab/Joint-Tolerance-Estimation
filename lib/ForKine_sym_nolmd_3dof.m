%%
function [epos, rotate_new] = ForKine_sym_nolmd_3dof(x,DH,base,RoCap, ys)
%% polishing 
% new 6 to end tool 
Msix2tool = [0.71906213, -0.49627146, -0.48648154, -0.073912;
            0.485629, 0.85957094, -0.15906689, -0.074794;
            0.49710575, -0.12187057, 0.85908872, 0.46376;
            0, 0, 0, 1];
         
nlink=size(DH,1);
DH(:,1) = x;

nlink = 3;

M=cell(1,nlink+1); M{1}=eye(4);
for i=1:nlink
    if i <= nlink
        R=[cossym(DH(i,1),ys(i))    -sinsym(DH(i,1),ys(i))*cos(DH(i,4))     sinsym(DH(i,1),ys(i))*sin(DH(i,4));
            sinsym(DH(i,1),ys(i))   cossym(DH(i,1),ys(i))*cos(DH(i,4))   -cossym(DH(i,1),ys(i))*sin(DH(i,4));
            0  sin(DH(i,4)) cos(DH(i,4))];
        T=[DH(i,3)*cossym(DH(i,1),ys(i));    DH(i,3)*sinsym(DH(i,1),ys(i));   DH(i,2)];
        M{i+1}=M{i}*[R T; zeros(1,3) 1];
    end
%     if i == nlink % the end-effector tool has three capsule for GP50
%         M{i+1}=M{i+1}*Msix2tool;
%     end
end
epos = M{i+1}(1:3,4)+base;

end

function c = cossym(t, y)
    c = cos(t)*(1 - y^2/2) - sin(t)*y;
end

function s = sinsym(t, y)
    s = sin(t)*(1 - y^2/2) + cos(t)*y;
end