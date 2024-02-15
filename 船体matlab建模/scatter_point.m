close all;
clc ,clear ,clf;
k = 1;
% 顶点坐标
vertex = [300, 350];

% 过点坐标
point = [400, 300];

% 求解 a
a = (point(2) - vertex(2)) / (point(1) - vertex(1))^2;

% 定义匿名函数
parabola = @(x) a * (x - vertex(1)).^2 + vertex(2);

% 生成 200 个均匀分布的点
x_values_all = linspace(0, 400, 200);

% 计算对应的 y 值并添加随机噪点
noise_amplitude = 5; % 噪点的振幅
y_values_all = parabola(x_values_all) + noise_amplitude * randn(size(x_values_all));

% 将点按照每组 20 个的方式储存
x_values_groups = reshape(x_values_all, 20, []);
y_values_groups = reshape(y_values_all, 20, []);

% 绘制抛物线和两个点
x_values = linspace(0, 400, 100); % 在 x 范围内生成一些点
y_values = parabola(x_values);

figure;
plot(x_values, y_values, '-r', 'LineWidth', 2);
hold on;
scatter([vertex(1), point(1)], [vertex(2), point(2)], 'bo', 'filled');
scatter(x_values_groups, y_values_groups, 'g.', 'LineWidth', 0.5);
title('trajectory prediction');
xlabel('x');
ylabel('f(x)');
grid on;
legend('抛物线', '顶点', '过点', '带噪点的抛物线');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_values = [];
y_values = [];
% 设置图形的总数和每个subplot的行列数
total_plots = size(x_values_groups, 2);
rows = 2;
cols = 2;

% 计算需要创建的subplot的总数
num_subplots = ceil(total_plots / (rows * cols));

for subplot_idx = 1:num_subplots
    % 创建一个新的figure
    figure;
    
    % 遍历当前subplot中的图形
    for i = 1:rows * cols
        % 计算当前图形在总体中的索引
        current_idx = (subplot_idx - 1) * (rows * cols) + i;
        
        % 检查是否超出总图形数量
        if current_idx <= total_plots
            % 选择当前图形的数据
            x_values = x_values_groups(:, current_idx);
            y_values = y_values_groups(:, current_idx);
            x_igbt_zth = x_values;
            y_igbt_zth = y_values;
            
            % 在当前subplot中创建子图
            subplot(rows, cols, i);
            
            % 绘制曲线
            plot(x_igbt_zth, y_igbt_zth, 'o', 'LineWidth', 2);
            hold on;
            
            % 进行曲线拟合
            [param, stat] = sigm_fit(x_igbt_zth, y_igbt_zth);
            y_val = polyval(param, x_values_all);
            y_error{k} = y_val - y_values_all;

            % 在图上添加文本框
            xlabel('x');
            ylabel('f(x)');
            box off;
            ax = gca;
            ax_pos = ax.Position;
            text_x = ax_pos(1) + ax_pos(3) * 0.7;
            text_y = ax_pos(2) + ax_pos(4) * 0;
            annotation('textbox', [text_x, text_y, 0.1, 0.1], 'String', ['Iteration: ', num2str(current_idx)], 'FitBoxToText', 'on', 'FontSize', 5, 'BackgroundColor', 'w', 'EdgeColor', 'k');
            hold off;
        end
        k = k + 1;
    end
    k = k + 1;
end
y_error = y_error(~cellfun('isempty', y_error));
figure;
hold on;
for j = 1 : numel(y_error)
    color = rand(1, 3);
    errorbar(x_values_all, y_values_all, y_error{numel(y_error) - j + 1}, 'Color', color);
    legend_labels{numel(y_error) - j + 1} = (['Iteration ', num2str(numel(y_error) - j + 1)]);
end
legend(legend_labels, 'Location', 'best','FontSize', 8);
hold off;

xlabel('X-axis');
ylabel('Y-axis');
title('Errorbar of Each Iteration');
