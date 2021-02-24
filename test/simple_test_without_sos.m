%% fmincon solution 
obj = @(x)-x;

[x, fval] = fmincon(obj,0,[],[],[],[],[],[],@(x)nonlcon1(x))
