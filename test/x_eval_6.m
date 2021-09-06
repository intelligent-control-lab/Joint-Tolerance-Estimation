Ni=5;
Nj=5;

lmds=zeros(Ni,Nj);
errors=zeros(Ni,Nj);
vrs=zeros(Ni,Nj);
xwalls=zeros(Ni,Nj);
deltas=zeros(Ni,Nj);

for i=1:Ni
    for j=1:Nj
        theta_ini=[i*pi/12   -9*pi/20    i*pi/12   pi/20    1*pi/20   pi/20]';
        wall_dist=0.02*j;
        [lmd, min_dist, violate_rate, xwall, min_delta]=sixdof_plane_x_exp(theta_ini, wall_dist);
        lmds(i,j)=lmd;
        errors(i,j)=-min_dist;
        vrs(i,j)=violate_rate;
        xwalls(i,j)=xwall;
        deltas(i,j)=min_delta;
        disp([i,j,lmd, min_dist, violate_rate, xwall, min_delta]);
    end
end

% csvwrite('lmds_2.csv',lmds);
% csvwrite('errors_2.csv',errors);
% csvwrite('vrs_2.csv',vrs);
% csvwrite('xwalls_2.csv',xwalls); 