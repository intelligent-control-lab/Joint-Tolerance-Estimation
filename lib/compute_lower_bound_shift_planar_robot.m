%% compute the max lower bound for Cartesian example 

function maximum_delta = compute_lower_bound_shift_planar_robot(theta_ini, lmd, nlink)
    for i = 1:nlink
        eval(['syms y' num2str(i) ' real']);
    end
    for i = 1:nlink
        eval(['syms c' num2str(i) ' real']);
    end
    for i = 1:nlink
        eval(['syms s' num2str(i) ' real']);
    end
    
    % compute the forward kinematics 
    xfk = 0;
    for i = 1:nlink
        eval(['xfk = xfk + cossym(theta_ini(i), y' num2str(i) ');']);
    end
    
    % decompose the term 
    [c,t] = coeffs(xfk);
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
        
        assert(nsin + ncos == 1);
        if nsin == 1
            delta_max_term = abs(coef)*(abs(sin(lmd) - lmd))^nsin;
        end
        if ncos == 1
            delta_max_term = abs(coef)*(abs(cos(lmd) - (1 - lmd^2/2)))^ncos;
        end

        maximum_delta = maximum_delta + delta_max_term;
    end
    
    disp(maximum_delta);
end

function c = cossym(t, y)
    c = cos(t)*cos(y) - sin(t)*sin(y);
end

function s = sinsym(t, y)
    s = sin(t)*cos(y) + cos(t)*sin(y);
end