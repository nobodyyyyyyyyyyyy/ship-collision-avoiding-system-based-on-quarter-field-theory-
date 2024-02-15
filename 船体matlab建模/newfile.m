clc
clear all
syms t
L = 100;
v_origin = 10;
theta = 30;

v_opposite(t) = t+1;

k_ad = 10^(0.3591 * log10(v_origin) + 0.0952);
k_dt = 10.^(0.5441 * log10(v_opposite(t)) - 0.0795);


r_fort = (1 + 1.34 * sqrt(k_ad^2 + k_dt^2)) * L;
r_aft = (1 + 0.67 * sqrt(k_ad^2 + k_dt^2)) * L;
r_starb = (0.2 + k_ad) * L;
r_port = (0.2 + 0.75 * k_ad) * L;

%syms fm(x,y);
syms x y;
m=2;
%x = -t;
%fm(x,y) = ((2*x/(((1+sign(x))*r_starb)-(1-sign(x))*r_port))^m)+(2*y/((1+sign(y))*r_fort-(1-sign(y))*r_aft))^m;
eqn1 = ((2*x/(((1+sign(x))*r_starb)-(1-sign(x))*r_port))^m)+(2*y/((1+sign(y))*r_fort-(1-sign(y))*r_aft))^m==1;
eqn2 = y == -x +t;
%[xsol,ysol] = solve(fm(x,y),[x,y])
[xSol ySol] = solve([eqn1,eqn2],[x,y])
%fm(1,1)