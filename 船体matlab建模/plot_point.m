%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function plot_point(allValues_x, allValues_y, allValues, number)
    % 示例的 allValues_x, allValues_y, allValues
    allValuesx = double(subs(allValues_x, 't', number));
    allValuesy = double(subs(allValues_y, 't', number));
    allValues = allValues;

    figure;
    scatterColor = rand(1, 3);
    scatter3(allValuesx, allValuesy, allValues, 10, [0 , 0 , 1], 'filled');

    hold on;

    % 绘制灰色半透明连续平面
    [X, Y] = meshgrid(linspace(min(allValuesx)+100, max(allValuesx)+100, 100), linspace(min(allValuesy)+100, max(allValuesy)+100, 100));
    Z_plane = zeros(size(X));
    surf(X, Y, Z_plane, 'FaceAlpha', 0.7, 'EdgeColor', 'none', 'FaceColor', [1 , 0.8549 , 0.72549]);

    % 绘制垂直线段（每隔二十个数据）
    for i = 1:5:length(allValuesx)
        plot3([allValuesx(i), allValuesx(i)], [allValuesy(i), allValuesy(i)], [allValues(i), 0], 'Color', [0 , 0.80784 , 0.81961], 'LineWidth', 1);
    end

    xlabel('X轴');
    ylabel('Y轴');
    zlabel('Z轴');

    hold off;
end
