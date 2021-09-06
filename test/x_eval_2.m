Ni=9;
Nj=9;

lmds=zeros(Ni,Nj);
errors=zeros(Ni,Nj);
vrs=zeros(Ni,Nj);
xwalls=zeros(Ni,Nj);

for i=1:Ni
    for j=1:Nj
        theta_ini=[i*pi/10, j*pi/10]';
        wall_dist=0.2;
        [lmd, min_dist, violate_rate, xwall]=two_D_plane_plane_exp(wall_dist, theta_ini);
        lmds(i,j)=lmd;
        errors(i,j)=max(-min_dist,0);
        vrs(i,j)=violate_rate;
        xwalls(i,j)=xwall;
        disp([i,j,lmd, min_dist, violate_rate, xwall]);
    end
end

% csvwrite('lmds_2.csv',lmds);
% csvwrite('errors_2.csv',errors);
% csvwrite('vrs_2.csv',vrs);
% csvwrite('xwalls_2.csv',xwalls);