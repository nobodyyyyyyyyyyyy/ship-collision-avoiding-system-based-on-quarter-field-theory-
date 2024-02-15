%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
% function [ optimised_parameters ] = Particle_Swarm_Optimization (Bird_in_swarm, Number_of_quality_in_Bird, MinMaxRange, Food_availability, availability_type, velocity_clamping_factor, cognitive_constant, social_constant, Min_Inertia_weight, Max_Inertia_weight, max_iteration)
% clc,clear;
tic;
Bird_in_swarm = 20;
Number_of_quality_in_Bird = 1;
MinMaxRange = [0  1000];
availability_type = 'max';
velocity_clamping_factor = 2;
cognitive_constant = 2;
social_constant = 2;
Min_Inertia_weight = 0.4;
Max_Inertia_weight = 0.9;
max_iteration = 100;

% 检查所有函数是否存在
if (exist('MinMaxCheck.m') == 0)
    clc;
    fprintf('.');
    return;
end

% 检查所有参数是否已输入
availability_type = lower(availability_type(1:3));

% 检查边界值和输入矩阵是否正确
[row, col] = size(MinMaxRange);
if row ~= Number_of_quality_in_Bird || col ~= 2
    error('不是一个正确的 MinMaxRange 矩阵')
end
for i = 1:Number_of_quality_in_Bird
    if MinMaxRange(i, 1) >= MinMaxRange(i, 2)
        error('最小值大于最大值!!!')
    end
end

N = Bird_in_swarm * max_iteration;
q = 0;

bird_min_range = MinMaxRange(:, 1);
bird_max_range = MinMaxRange(:, 2);

format long;
for i = 1:Number_of_quality_in_Bird
    bird(:, i) = bird_min_range(i) + (bird_max_range(i) - bird_min_range(i)) * rand(Bird_in_swarm, 1);
end

Vmax = bird_max_range * velocity_clamping_factor;
Vmin = -Vmax;

for i = 1:Number_of_quality_in_Bird
    Velocity(:, i) = Vmin(i) + (Vmax(i) - Vmin(i)) * rand(Bird_in_swarm, 1);
end

% 创建一个数组来存储每次迭代的最佳适应度值
best_fitness_values = zeros(1, max_iteration);

for itr = 1:max_iteration
    fprintf('已完成 %d%%...', uint8(q * 100 / N))

    for p = 1:Bird_in_swarm
        parameter = bird(p, :, itr);
        availability(p, itr) = direction_modify(parameter);

        switch availability_type
            case 'min'
                format long;
                [pBest_availability, index] = min(availability(p, :));
                pBest = bird(p, :, index);

                if (p == 1 && itr == 1)
                    gBest = pBest;
                    gBest_availability = pBest_availability;
                elseif availability(p, itr) < gBest_availability
                    gBest_availability = availability(p, itr);
                    gBest = bird(p, :, itr);
                end

            case 'max'
                format long;
                [pBest_availability, index] = max(availability(p, :));
                pBest = bird(p, :, index);

                if (p == 1 && itr == 1)
                    gBest = pBest;
                    gBest_availability = pBest_availability;
                elseif availability(p, itr) > gBest_availability
                    gBest_availability = availability(p, itr);
                    gBest = bird(p, :, itr);
                end

            otherwise
                error('availability_type 不匹配')
        end

        w(itr) = ((max_iteration - itr) * (Max_Inertia_weight - Min_Inertia_weight)) / (max_iteration - 1) + Min_Inertia_weight;
        Velocity(p, :, (itr + 1)) = w(itr) * Velocity(p, :, itr) + social_constant * rand(1, Number_of_quality_in_Bird) .* (gBest - bird(p, :, itr)) + cognitive_constant * rand(1, Number_of_quality_in_Bird) .* (pBest - bird(p, :, itr));
        Velocity(p, :, (itr + 1)) = MinMaxCheck(Vmin, Vmax, Velocity(p, :, (itr + 1)));

        bird(p, :, (itr + 1)) = bird(p, :, itr) + Velocity(p, :, (itr + 1));
        bird(p, :, (itr + 1)) = MinMaxCheck(bird_min_range, bird_max_range, bird(p, :, (itr + 1)));
        q = q + 1;
    end

    % 计算并存储每次迭代的最佳适应度值
    best_fitness_values(itr) = gBest_availability;
end

elapsedTime = toc;
disp(['程序总运行时间： ' num2str(elapsedTime) ' 秒']);

% 绘制收敛曲线
figure;
plot(1:max_iteration, best_fitness_values, 'LineWidth', 2);
title('PSO Algorithm Convergent Curve');
xlabel('Iteration Time');
ylabel('Best Fitness');
grid on;

% 创建子图，绘制鸟群位置的散点图
figure;
scatter3(bird(:, 1, end), bird(:, 1, end), best_fitness_values(end) * ones(Bird_in_swarm, 1), 50, 'filled');
title('Scatter Plot of Bird Flock Positions');
xlabel('Dimension 1');
ylabel('Dimension 2');
zlabel('Best Fitness');
grid on;

% 创建子图，绘制鸟群位置的3D曲面
figure;
[X, Y] = meshgrid(linspace(bird_min_range, bird_max_range, 20));
Z = direction_modify([X(:), Y(:)]);
Z = reshape(Z, size(X));
surf(X, Y, Z);
title('3D Surface Representation of Bird Flock Locations');
xlabel('Dimension 1');
ylabel('Dimension 2');
zlabel('Fitness');
grid on;

% 创建子图，绘制鸟群速度的直方图
figure;
histogram(Velocity(:, 1, end), 'Normalization', 'probability', 'FaceColor', [0.2 0.4 0.6]);
title('Histogram of Bird Swarm Velocities');
xlabel('Velocity ');
ylabel('Probability');
grid on;

% 显示最终的最佳适应度值
disp(['最终的最佳适应度值：' num2str(gBest_availability)]);

