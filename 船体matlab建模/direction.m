function [ship_opposite_translate_x , intersection_D_1_x , intersection_D_0_x , distance_R_1 , distance_R_2 , distance_DCPA , ship_origin_v , distance_D , distance_D_0 , distance_D_1 , v_opposite , m] = direction
% clc;
% clear;
% clearvars , close;
syms t x

L = 100;
ship_origin_v = 10;
theta = 30;

ship_origin_th = @(t) t + 1;
ship_origin_x = @(t) 2 * t + 2;
ship_origin_y = @(t) 3 * t + 3;

t = 100;  % 为了避免未定义的 t，这里设置一个具体的值
equation0 = (2^(1/2)*((11*2^(1/2))/2 + 10)*(t + x + 2))/11 - (3^(1/2)*(4*t - 1797))/2 - t/2 - 1;
fplot(equation0);
hold on;
v_opposite = t + 1;
k_ad = 10^(0.3591 * log10(ship_origin_v) + 0.0952);
k_dt = 10^(0.5441 * log10(v_opposite) - 0.0795);
r_fort = (1 + 1.34 * sqrt(k_ad^2 + k_dt^2)) * L;
r_aft = (1 + 0.67 * sqrt(k_ad^2 + k_dt^2)) * L;
r_starb = (0.2 + k_ad) * L;
r_port = (0.2 + 0.75 * k_ad) * L;

% 分段函数定义
segmentedFunction = @(m) (m < 0) * -1 + (m >= 0) * 1;

% 参数方程
parameter_1 = @(x, y)  power(2 * x / ((1 + segmentedFunction(x)) * r_starb - (1 - segmentedFunction(x)) * r_port), 2) + ...
    power(2 * y / ((1 + segmentedFunction(y)) * r_fort - (1 - segmentedFunction(y)) * r_aft), 2) - 1;
parameter_2 = @(x, y)  power(2 * x / ((1 + segmentedFunction(x)) * 2 * r_starb - (1 - segmentedFunction(x)) * 2 * r_port), 2) + ...
    power(2 * y / ((1 + segmentedFunction(y)) * 2 * r_fort - (1 - segmentedFunction(y)) * 2 * r_aft), 2) - 1;

% 创建figure

% 在figure中绘制参数方程曲线
h1 = ezplot(parameter_1 ,[-500, 500, -500, 1000]);
hold on;
h2 = ezplot(parameter_2 ,3 .* [-500, 500, -500, 1000]);

% 设置曲线颜色和线宽
set(h1, 'Color', 'black', 'LineWidth', 1);
set(h2, 'Color', 'black', 'LineWidth', 1);

title('Ellipse');
xlabel('x');
ylabel('y');
axis equal;  % 保持坐标轴比例一致
axis on
% 去除网格线
grid off;


% trajectory_opposite = scatter_point (flag) ;
syms t x
trajectory_opposite = @(x) -x+1800;
fplot(trajectory_opposite, [-1000, 1500],LineStyle="-",Color='r');
theta_rad = deg2rad(theta);
opposite_x = t;                              %测试用的，实际换为预测的数据
opposite_y = trajectory_opposite(t);         %测试用的，实际换为预测的数据

% 计算新坐标

ship_opposite_translate_x = opposite_x - ship_origin_x(t);
ship_opposite_translate_y = opposite_y - ship_origin_y(t);

ship_opposite_rotated_x = ship_opposite_translate_x * cos(theta_rad) - ship_opposite_translate_y * sin(theta_rad);
ship_opposite_rotated_y = ship_opposite_translate_x * sin(theta_rad) + ship_opposite_translate_y * cos(theta_rad);

slope_opposite = subs(diff(trajectory_opposite, x), x, ship_opposite_rotated_x);
angle_with_x_positive = atan2(slope_opposite, 1);
dx = cos(angle_with_x_positive) * v_opposite;
dy = ship_origin_v - sin(angle_with_x_positive) * v_opposite;
slope_combine = dy/dx;

equation_combine = slope_combine*(x - ship_opposite_translate_x) + ship_opposite_rotated_y;
equation_orthographic = -1/slope_combine*x;
intersection_combine_orthographic_x = solve(equation_combine==equation_orthographic , x);

[intersection_D_0_x , intersection_D_0_y] = piecewise_equ(equation_combine , 'D_0' , intersection_combine_orthographic_x , ship_opposite_rotated_x);
[intersection_D_1_x , intersection_D_1_y] = piecewise_equ(equation_combine , 'D_1' , intersection_combine_orthographic_x , ship_opposite_rotated_x);
[intersection_R_1_x , intersection_R_1_y] = piecewise_equ(equation_orthographic , 'R_1' , intersection_combine_orthographic_x , ship_opposite_rotated_x);
[intersection_R_2_x , intersection_R_2_y] = piecewise_equ(equation_orthographic , 'R_2' , intersection_combine_orthographic_x , ship_opposite_rotated_x);



m = slope_combine;
b = -slope_combine * ship_opposite_translate_x + ship_opposite_rotated_y;
distance_DCPA = abs(m * 0 - 0 + b) / sqrt(m^2 + 1);

distance_D = norm([ship_opposite_translate_x , ship_opposite_translate_y] , 2);
distance_D_0 = norm([intersection_D_0_x , intersection_D_0_y] , 2);
distance_D_1 = norm([intersection_D_1_x , intersection_D_1_y] , 2);
distance_R_1 = norm([intersection_R_1_x , intersection_R_1_y] , 2);
distance_R_2 = norm([intersection_R_2_x , intersection_R_2_y] , 2);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nq = 1;
% ns = 1;
% if intersection_D_1_x == []
%     nq = 0;
% elseif intersection_D_0_x == []
%     ns = 0;
% end
% 
% 
% if nq > 0 && ns > 0
%     u_DCPA = 1;
% elseif distance_R_1 < distance_DCPA && distance_DCPA <= distance_R_2 && nq == 0
%     u_DCPA = 1/2 - 1/2 * sin(180 / (distance_R_2 - distance_R_1) * (distance_DCPA - (distance_R_2 + distance_R_1) / 2));
% elseif distance_DCPA > distance_R_2 || ns == 0
%     u_DCPA = 0;
% end
% 
% 
% TCPA = sqrt(distance_D_1^2 - distance_DCPA^2)/ship_origin_v;
% T0 = sqrt(distance_D_1^2 - distance_DCPA^2)/ship_origin_v;
% if ns > 0
%     u_TCPA = exp(-log(2) * (abs(TCPA) / T0^2));
% elseif ns == 0
%     u_TCPA = 0;
% end
% 
% 
% if ns > 0
%     u_D = exp(-log(2) * ((distance_D - distance_D_0) / (distance_D_1 - distance_D_0))^2);
% elseif ns == 0
%     u_D = 0;
% end


% angle_rad = atan2(m, 1); 
% angle_deg_x = rad2deg(angle_rad);
% if angle_deg_x < 0
%     angle_deg_x = angle_deg_x + 180;
%     angle_deg_y = 270 - angle_deg_x0;
% else 
%     angle_deg_x = angle_deg_x;
%     angle_deg_y = 90 - angle_deg_x;
% end
% if ship_opposite_translate_x<0
%     angle_C = - angle_deg_y;
% else 
%     angle_C = angle_deg_y;
% end
% 
% 
% if ns > 0
%     u_C = (17/44) * cos(angle_C + 161) + sqrt(cos(angle_C + 161)^2) + 440/289;
% elseif ns == 0
%     u_C = 0;
% end
% 
% 
% K = ship_origin_v/v_opposite;
% if ns > 0
%     u_K = 1 / (1 + 2 / (K * sqrt(K^2 + 1 + 2 * K * abs(sin(abs(angle_C))))));
% elseif ns == 0
%     u_K = 0;
% end
% U = 0.36*u_DCPA+0.32*u_TCPA+0.14*u_D+0.1*u_C+0.08*u_K;
% end








