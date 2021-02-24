function bi = dec2binary_6dof(num)
%% customizable toolbox for converting decimal number of binary number 
   % only work within 6 digits, corresponds to 6 dof 
   deno = [32 16 8 4 2];
   bi = [0 0 0 0 0 0];
   for i = 1:5
       if num >= deno(i)
           bi(6-i+1) = 1;
           num = num - deno(i);
       end
   end
   bi(1) = num;
end