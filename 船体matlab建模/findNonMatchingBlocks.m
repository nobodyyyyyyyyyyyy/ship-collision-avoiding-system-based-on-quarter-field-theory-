function index = findNonMatchingBlocks(str1, str2) 
        m = 1;
        syms t;
        str1_1 = simplify(subs(str1 , t , m));
        str1_2 = simplify(subs(str2 , t , m));
        if (str1_2 < 0&&str1_1 > 0)
            index = 2;
        elseif (str1_1 < 0&& str1_2 <0)
            index = 4;
        elseif (str1_1 > 0&& str1_2 >0)
            index = 3;
        elseif (str1_1 < 0&&str1_2 > 0)
            index = 1;
        end
end