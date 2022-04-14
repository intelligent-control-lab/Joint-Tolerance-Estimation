%% compute the max lower bound for Cartesian example 

function maximum_delta = compute_lower_bound_shift(theta_ini, lmd, nlink)
    syms y1 y2 y3 y4 y5 y6 real;
    syms c1 c2 c3 c4 c5 c6 s1 s2 s3 s4 s5 s6;
    ys = [y1 y2 y3 y4 y5 y6];
    % syms b c d e g h k real;
    ROBOT = 'GP50';
    robot=robotproperty(ROBOT);
%     theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20]';
    
    
    % problem settings 
%     nlink = 3;
%     lmd = 0.05;
    
    % test the cos sin decomposition 
    fk_cos_sin = ForKine_cossin_sym_casual_dof(theta_ini, robot.DH, robot.base, robot.cap, ys, nlink);
    fk_cos_sin_x = fk_cos_sin(1);
    
    % decompose the term 
    [c,t] = coeffs(fk_cos_sin_x);
    deci_coe = vpa(c,3);
    
    % compute the maximum delta 
    maximum_delta = 0;
    for i = 1:size(t,2)
        term = t(i);
        coef = deci_coe(i);
    
        % substitute cos and sin using new variable 
        for j = 1:nlink
            eval(['term = subs(term, {cos(y' num2str(j) ')}, {c' num2str(j) '});']);
        end
        for j = 1:nlink
            eval(['term = subs(term, {sin(y' num2str(j) ')}, {s' num2str(j) '});']);
        end
    
        % compute how many cos
        ncos = 0;
        for j = 1:nlink
            eval(['grad = diff(term,c' num2str(j) ');'])
            if grad ~= 0 
                % there is cos exists 
                ncos = ncos + 1;
            end
        end
    
        % compute how many sin
        nsin = 0;
        for j = 1:nlink
            eval(['grad = diff(term,s' num2str(j) ');'])
            if grad ~= 0 
                % there is cos exists 
                nsin = nsin + 1;
            end
        end
        
        delta_max_term = abs(coef)*(delta_cos_max(lmd, nsin, ncos) + delta_sin_max(lmd, nsin));
        maximum_delta = maximum_delta + delta_max_term;
    end
    
    disp(maximum_delta);
end