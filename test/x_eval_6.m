label='byTerm_005';

Ni=9;
Nj=9;

lmds=zeros(Ni,Nj);
errors=zeros(Ni,Nj);
vrs=zeros(Ni,Nj);
xwalls=zeros(Ni,Nj);
deltas=zeros(Ni,Nj);

x_fail=[];
global x_fail;

for i=1:Ni
    for j=1:Nj
        theta_ini=[i*pi/20   -i*pi/20    4*pi/20   pi/20    i*pi/20   pi/20]';
        wall_dist=0.05*j;
        [lmd, min_dist, violate_rate, xwall, min_delta]=sixdof_plane_x_exp_ori(theta_ini, wall_dist);
        lmds(i,j)=lmd;
        errors(i,j)=-min_dist;
        vrs(i,j)=violate_rate;
        xwalls(i,j)=xwall;
        deltas(i,j)=min_delta;
        disp([i,j,lmd, min_dist, violate_rate, xwall, min_delta]);
    end
end

csvwrite(['lmds_',label,'.csv'],lmds);
csvwrite(['errors_',label,'.csv'],errors);
csvwrite(['vrs_',label,'.csv'],vrs);
csvwrite(['xwalls_',label,'.csv'],xwalls); 
csvwrite(['deltas_',label,'.csv'],deltas); 