function [q,row,col] = coe_assign(weight, order_seq, nlink)
    % next determine the order 
    % check 1 and 2 nubmer 
    oneses = 0;
    twoses = 0;
    for i = 1:size(order_seq,2)
        if order_seq(i) == 1
            oneses = oneses + 1;
        end
        if order_seq(i) == 2
            twoses = twoses + 1; 
        end       
    end

    % construct the decomposed matrix terms
    if oneses == 0 && twoses == 0
        % the constant term
        % q11
        q = weight;
        row = 1;
        col = 1;
    end
    if twoses == 0 && oneses ~= 0
        % determine the order 
        order = 0;
        for j = 1:nlink
            if order_seq(j) ~= 0
                order = order + 2^(j-1);
            end
        end

        % need to +1 to fit the order definition in MATLAB 
        order = order + 1;
        % q_1_order is weight/2
        q = weight / 2;
        row = 1;
        col = order;
    end
    if twoses ~= 0 && oneses == 0
        % diagnoal terms 
        order = 0;
        for j = 1:nlink
            if order_seq(j) ~= 0
                order = order + 2^(j-1);
            end
        end

        % need to +1 to fit the order definition in MATLAB 
        order = order + 1;
        % q_order_order 
        q = weight;
        row = order;
        col = order;
    end
    if twoses ~= 0 && oneses ~= 0
        % get the two order terms
        order_2nd = 0;
        for j = 1:nlink
            if order_seq(j) == 2
                order_2nd = order_2nd + 2^(j-1);
            end
        end
        order_1st = 0;
        for j = 1:nlink
            if order_seq(j) ~= 0
                order_1st = order_1st + 2^(j-1);
            end
        end
        if order_1st >= order_2nd
            % swap 
            tmp = order_2nd;
            order_2nd = order_1st;
            order_1st = tmp;
        end

        % need to +1 to fit the order definition in MATLAB 
        order_1st = order_1st + 1;
        order_2nd = order_2nd + 1;
        % q_order1st_order2nd
        q = weight/2;
        row = order_1st;
        col = order_2nd;
    end

end
