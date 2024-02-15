%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function [x_output, y_output, u, v, r, result_th,sigma] = hydrodyn_1(x_position, y_position, u_init, v_init, r_init, th_init, sigma_previous, sigma_updated, np,T)
% disp([sigma_updated, np])
syms u(t) v(t) r(t)

y0 = [u_init; v_init; r_init];

% Set your timespan
tspan = linspace(0, T, 200);  % Modify as desired

% Set your initial conditions IC = [x(t=0)  y(t=0)]
IC = [0 1 1];
angular_speed = 3000;
T_max = (sigma_updated - sigma_previous) / angular_speed;
np = np + eps;
[t, y] = ode15s(@(t, y) myODEs(t, y, sigma_previous, sigma_updated, np, T_max), tspan, y0);

if T<=abs(T_max)
    sigma = sigma_previous + sign(T_max)*angular_speed*T;
else 
    sigma = sigma_updated;
end

dt = 1 / (length(t) - 1);
vx = zeros(1, length(t)-1);
vy = zeros(1, length(t)-1);
th = zeros(1, length(t)-1);

% Extract x and y
u = y(:, 1);
v = y(:, 2);
r = y(:, 3);

% Plot results
% figure(1)
% plot(t, u, t, v, t, r, "LineWidth", 2), grid
% legend('u', 'v', 'r')
% title('Linear and Angular Velocities')
% xlabel('time (s)')
% ylabel('Linear and Angular Velocities')

t_ = diff(t);
for i = 1:length(t_)
    th(i) = r(i) * double(t_(i));
end
result_th = double(cumsum(th))+ th_init;

vy = u(1:end-1)' .* cos(result_th) - v(1:end-1)' .* sin(result_th);
vx = u(1:end-1)' .* sin(result_th) + v(1:end-1)' .* cos(result_th);

% figure(2)
% plot(t, result_th, t, vx, t, vy, "LineWidth", 2), grid
% legend('\theta', 'Vx', 'Vy')
% title('x and y components of velocities')
% xlabel('time (s)')
% ylabel('Velocity (m/s)')

x = zeros(1, length(t)-1);
y = zeros(1, length(t)-1);
for j = 1:length(t)-1
    x(j) = vx(j) * t_(j);
    y(j) = vy(j) * t_(j);
end

x_output = x_position + cumsum(x);
y_output = y_position + cumsum(y);

figure(3)
plot(x_position+cumsum(x), y_position+cumsum(y), 'LineWidth', 2), grid
legend('Trajectory with starting point (0,0)')
title('Trajectory')
xlabel('x-coordinates')
ylabel('y-coordinates')
axis equal  % Ensure equal axis scaling
end


