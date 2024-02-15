function  [objvalue]=calobjvalue(pop)
temp1=decodechrom(pop,1,10);         %将二值化的数值转化为十进制

x=temp1*10/1023;                     %将十进制的数值放到1——10的范围之内
objvalue=x+10*sin(5*x)+7*cos(4*x);   %计算目标函数值