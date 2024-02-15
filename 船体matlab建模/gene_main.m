clear,close all,clearvars,clc;
popsize=20;             %粒子的多少
chromlength=10;         %字符串的大小
pc=0.6;                 %染色体交叉的概率
pm=0.001;               %变异的概率


pop=inipop(popsize,chromlength);          %建立每一个点的二进制编码


for i=1:2000
    [objvalue]=calobjvalue(pop);          %计算该点的函数值大小
    fitvalue=calfitvalue(objvalue)        %计算该点的适应度
                                          %前几步的目的是得到适应度以运用轮盘赌法

    [newpop]=selection(pop,fitvalue);             %复制
    [newpop1]=crossover(newpop,pc);               %交叉
    [newpop2]=mutation(newpop1,pm);               %变异

    [objvalue]=calobjvalue(newpop2);          %计算该点的函数值大小
    fitvalue=calobjvalue(objvalue)

    [bestindividual,bestfit]=best(newpop2,fitvalue);
    y(i)=bestfit;
    x(i)=decodechrom(bestindividual,1,chromlength)*10/1023;      %将自变量解码成十进制
    pop=newpop2;

end
 fplot('x+10*sin(5*x)+7*cos(4*x)',[1,10]);
 hold on
plot(x,y,'r*')                                          
hold on

[z ,index]=max(y);             %计算最大值及其位置
x5=x(index)                     %计算最大值对应的x值
ymax=z