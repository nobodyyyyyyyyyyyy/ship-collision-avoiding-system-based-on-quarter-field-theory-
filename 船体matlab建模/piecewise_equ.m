%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 参数定义
% L = 100;
% v_origin = 10;
% theta = 30;
% t = 10;  % 为了避免未定义的 t，这里设置一个具体的值
% v_opposite = t + 1;
% k_ad = 10^(0.3591 * log10(v_origin) + 0.0952);
% k_dt = 10^(0.5441 * log10(v_opposite) - 0.0795);
% r_fort = (1 + 1.34 * sqrt(k_ad^2 + k_dt^2)) * L;
% r_aft = (1 + 0.67 * sqrt(k_ad^2 + k_dt^2)) * L;
% r_starb = (0.2 + k_ad) * L;
% r_port = (0.2 + 0.75 * k_ad) * L;
% 
% % 分段函数定义
% segmentedFunction = @(m) (m < 0) * -1 + (m >= 0) * 1;
% 
% % 参数方程
% parameter_1 = @(x, y)  power(2*x / ((1 + segmentedFunction(x)) * r_starb - (1 - segmentedFunction(x)) * r_port), 2) + ...
%     power(2*y / ((1 + segmentedFunction(y)) * r_fort - (1 - segmentedFunction(y)) * r_aft), 2) - 1;
% parameter_2 = @(x, y)  power(2*x / ((1 + segmentedFunction(x)) *2* r_starb - (1 - segmentedFunction(x)) * 2*r_port), 2) + ...
%     power(2*y / ((1 + segmentedFunction(y)) *2* r_fort - (1 - segmentedFunction(y)) * 2*r_aft), 2) - 1;
% equation0 = @(x) (16*x*(471164915839089*t - 5158334618374031))/(4929357530924979*(t + 1));
% 
% ezplot(parameter_1 ,[-500, 500, -500, 1000]);
% hold on
% ezplot(parameter_2 ,3.*[-500, 500, -500, 1000]);
% fplot(equation0)
% title('Ellipse');
% xlabel('x');
% ylabel('y');
% axis equal;  % 保持坐标轴比例一致
% grid on;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Given values
function [reserve_x, reserve_y] = piecewise_equ(equation0, flag, intersection, initial_point)
%     Clear command window and workspace
% clc,clear
    % Define symbolic variables
    syms t x y 
    syms x_opposite
%     flag = 'D_0';
%     equation0 = (2^(1/2)*((11*2^(1/2))/2 + 10)*(t + x + 2))/11 - (3^(1/2)*(4*t - 1797))/2 - t/2 - 1;

%     Constants and calculations
    L = 100;
    v_origin = 10;
    theta = 30;
    v_opposite = t + 1;
    t = 10;
    k_ad = 10^(0.3591 * log10(v_origin) + 0.0952);
    k_dt = 10^(0.5441 * log10(v_opposite) - 0.0795);
    r_fort = (1 + 1.34 * sqrt(k_ad^2 + k_dt^2)) * L;
    r_aft = (1 + 0.67 * sqrt(k_ad^2 + k_dt^2)) * L;
    r_starb = (0.2 + k_ad) * L;
    r_port = (0.2 + 0.75 * k_ad) * L;

    if flag == 'R_1'
        % Define segmented function
        segmentedFunction = @(m) (m < 0) * -1 + (m >= 0) * 1;

        % Parameter equation
        parameter_1 = @(x, y) power(2 * x / ((1 + segmentedFunction(x)) * r_starb - (1 - segmentedFunction(x)) * r_port), 2) + ...
            power(2 * y / ((1 + segmentedFunction(y)) * r_fort - (1 - segmentedFunction(y)) * r_aft), 2) - 1;

        % Substitute values for x and y in the positive case
        parameter_11 = @(x, y) power(2 * x / ((1 + 1) * r_starb - (1 - 1) * r_port), 2) + ...
            power(2 * y / ((1 + 1) * r_fort - (1 - 1) * r_aft), 2) - 1;
        parameter_12 = @(x, y) power(2 * x / ((1 - 1) * r_starb - (1 + 1) * r_port), 2) + ...
            power(2 * y / ((1 + 1) * r_fort - (1 - 1) * r_aft), 2) - 1;
        parameter_13 = @(x, y) power(2 * x / ((1 - 1) * r_starb - (1 + 1) * r_port), 2) + ...
            power(2 * y / ((1 - 1) * r_fort - (1 + 1) * r_aft), 2) - 1;
        parameter_14 = @(x, y) power(2 * x / ((1 + 1) * r_starb - (1 - 1) * r_port), 2) + ...
            power(2 * y / ((1 - 1) * r_fort - (1 + 1) * r_aft), 2) - 1;
        parameter_functions = struct('param_11', parameter_11, 'param_12', parameter_12, 'param_13', parameter_13, 'param_14', parameter_14);

        % Loop over parameter functions
        field_names = fieldnames(parameter_functions);
            for i = 1:numel(field_names)
            current_field = field_names{i};
            current_function = parameter_functions.(current_field);

            % Solve for x and y
            a = simplify(subs(current_function(x, y), y, equation0));
            result_x{i} = simplify(solve(a, x));
            result_y{i} = subs(equation0 , x , result_x{i});
            result_y{i}(1) = simplify_equation (result_y{i}(1));
            result_y{i}(2) = simplify_equation (result_y{i}(2));
            result_x{i}(1) = simplify_equation (result_x{i}(1));
            result_x{i}(2) = simplify_equation (result_x{i}(2));
            % Handle NaN values based on conditions 
            if i == 1
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = result_x{i}(:);
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = result_y{i}(:);
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 2
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 3
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) =result_x{i}(:);
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = result_y{i}(:);
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            elseif i == 4
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            end
        end

        % Remove NaN values
        nonNaNValuesX = cellfun(@(x) x(~isnan(x)), result_x, 'UniformOutput', false);
        result__x = [nonNaNValuesX{:}];
        nonNaNValuesY = cellfun(@(x) x(~isnan(x)), result_y, 'UniformOutput', false);
        result__y = [nonNaNValuesY{:}];

        % Random number generation
        number = randi([1, 10]);

        % Calculate slopes
        slope_1 = (result__y(3) - result__y(1)) / (result__x(3) - result__x(1));
        slope_1 = subs(slope_1, 't', number);
        slope_2 = (result__y(4) - result__y(2)) / (result__x(4) - result__x(2));
        slope_2 = subs(slope_2, 't', number);

        % Calculate current slope
        [slope_C(t), ~] = coeffs(equation0, x);
        current_slope = slope_C(t);
        error_1 = simplify(abs(slope_1 - subs(current_slope, 't', number)));
        error_2 = simplify(abs(slope_2 - subs(current_slope, 't', number)));
        n = [1 , 3];
        % Adjust result based on slope conditions
        if  error_1  <  error_2
            result__x(2) = NaN;
            result__y(2) = NaN;
            result__x(4) = NaN;
            result__y(4) = NaN;
        else
            result__x(1) = NaN;
            result__y(1) = NaN;
            result__x(3) = NaN;
            result__y(3) = NaN;
            n = [2 , 4];
        end
        reserve_x = result__x(n);
        reserve_y = result__y(n);

        compare = simplify(subs(reserve_x(:) - intersection , 't' , 1));
        [~, minIndex] = min(compare);
        reserve_x = reserve_x(minIndex);
        reserve_y = reserve_y(minIndex);

    elseif flag == 'D_0'
          % Define segmented function
                segmentedFunction = @(m) (m < 0) * -1 + (m >= 0) * 1;

        % Parameter equation
        parameter_1 = @(x, y)  power(2 * x / ((1 + segmentedFunction(x)) * r_starb - (1 - segmentedFunction(x)) * r_port), 2) + ...
                      power(2 * y / ((1 + segmentedFunction(y)) * r_fort - (1 - segmentedFunction(y)) * r_aft), 2) - 1;


        % Substitute values for x and y in the positive case
        parameter_11 = @(x, y) power(2 * x / ((1 + 1) * r_starb - (1 - 1) * r_port), 2) + ...
            power(2 * y / ((1 + 1) * r_fort - (1 - 1) * r_aft), 2) - 1;
        parameter_12 = @(x, y) power(2 * x / ((1 - 1) * r_starb - (1 + 1) * r_port), 2) + ...
            power(2 * y / ((1 + 1) * r_fort - (1 - 1) * r_aft), 2) - 1;
        parameter_13 = @(x, y) power(2 * x / ((1 - 1) * r_starb - (1 + 1) * r_port), 2) + ...
            power(2 * y / ((1 - 1) * r_fort - (1 + 1) * r_aft), 2) - 1;
        parameter_14 = @(x, y) power(2 * x / ((1 + 1) * r_starb - (1 - 1) * r_port), 2) + ...
            power(2 * y / ((1 - 1) * r_fort - (1 + 1) * r_aft), 2) - 1;
        parameter_functions = struct('param_11', parameter_11, 'param_12', parameter_12, 'param_13', parameter_13, 'param_14', parameter_14);
        % Loop over parameter functions
        field_names = fieldnames(parameter_functions);
        for i = 1:numel(field_names)
            current_field = field_names{i};
            current_function = parameter_functions.(current_field);

            % Solve for x and y
            a = simplify(subs(current_function(x, y), y, equation0));
            result_x{i} = simplify(solve(a, x));
            result_y{i} = subs(equation0 , x , result_x{i});
            result_y{i}(1) = simplify_equation (result_y{i}(1));
            result_y{i}(2) = simplify_equation (result_y{i}(2));
            result_x{i}(1) = simplify_equation (result_x{i}(1));
            result_x{i}(2) = simplify_equation (result_x{i}(2));
            % Handle NaN values based on conditions 
            if i == 1
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = result_x{i}(:);
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = result_y{i}(:);
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 2
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 3
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) =result_x{i}(:);
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = result_y{i}(:);
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            elseif i == 4
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            end
        end

        % Remove NaN values
        nonNaNValuesX = cellfun(@(x) x(~isnan(x)), result_x, 'UniformOutput', false);
        result__x = [nonNaNValuesX{:}];
        nonNaNValuesY = cellfun(@(x) x(~isnan(x)), result_y, 'UniformOutput', false);
        result__y = [nonNaNValuesY{:}];

        % Random number generation
        number = randi([1, 10]);
        reserve_x = [];
        reserve_y = [];

        % Nested loops for slope comparison
        for i = 1:numel(result__x)
            for j = 1:numel(result__y)
                for m = 1:numel(result__x) - i
                    for n = 1:numel(result__y) - j
                        slope = (result__y(j) - result__y(numel(result__y) - n + 1)) / (result__x(i) - result__x(numel(result__x) - m + 1));
                        slope = subs(slope, 't', number);
                        [slope_C, ~] = coeffs(equation0, x);
                        slope_C = slope_C(1);
                        slope_C = subs(slope_C, 't', number);
                        diagnose = simplify(abs(slope_C - slope));
                        if diagnose < 10
                            % Store values in arrays
                            reserve_x = [reserve_x, [result__x(i); result__x(numel(result__x) - m + 1)]];
                            reserve_y = [reserve_y, [result__y(j); result__y(numel(result__y) - n + 1)]];
                        end
                    end
                end
            end
        end
        compare = simplify(subs(reserve_x(:) - initial_point , 't' , 1));
        [~, minIndex] = min(compare);
        reserve_x = reserve_x(minIndex);
        reserve_y = reserve_y(minIndex);
    elseif flag == 'D_1'
           % Define segmented function
                segmentedFunction = @(m) (m < 0) * -1 + (m >= 0) * 1;

        % Parameter equation
        parameter_2 = @(x, y)  power(2 * x / ((1 + segmentedFunction(x)) * 2 * r_starb - (1 - segmentedFunction(x)) * 2 * r_port), 2) + ...
                      power(2 * y / ((1 + segmentedFunction(y)) * 2 * r_fort - (1 - segmentedFunction(y)) * 2 * r_aft), 2) - 1;


        % Substitute values for x and y in the positive case
        parameter_21 = @(x, y) power(2 * x / ((1 + 1) * 2*r_starb - (1 - 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 + 1) *2* r_fort - (1 - 1) *2* r_aft), 2) - 1;
        parameter_22 = @(x, y) power(2 * x / ((1 - 1) *2* r_starb - (1 + 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 + 1) *2* r_fort - (1 - 1) * 2*r_aft), 2) - 1;
        parameter_23 = @(x, y) power(2 * x / ((1 - 1) * 2*r_starb - (1 + 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 - 1) *2* r_fort - (1 + 1) * 2*r_aft), 2) - 1;
        parameter_24 = @(x, y) power(2 * x / ((1 + 1) * 2*r_starb - (1 - 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 - 1) *2* r_fort - (1 + 1) *2* r_aft), 2) - 1;
        parameter_functions = struct('param_21', parameter_21, 'param_22', parameter_22, 'param_23', parameter_23, 'param_24', parameter_24);
        % Loop over parameter functions
        field_names = fieldnames(parameter_functions);
        for i = 1:numel(field_names)
            current_field = field_names{i};
            current_function = parameter_functions.(current_field);

            % Solve for x and y
            a = simplify(subs(current_function(x, y), y, equation0));
            result_x{i} = simplify(solve(a, x));
            result_y{i} = subs(equation0 , x , result_x{i});
            result_y{i}(1) = simplify_equation (result_y{i}(1));
            result_y{i}(2) = simplify_equation (result_y{i}(2));
            result_x{i}(1) = simplify_equation (result_x{i}(1));
            result_x{i}(2) = simplify_equation (result_x{i}(2));
            % Handle NaN values based on conditions 
            if i == 1
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = result_x{i}(:);
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = result_y{i}(:);
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 2
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 3
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) =result_x{i}(:);
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = result_y{i}(:);
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            elseif i == 4
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            end
        end

        % Remove NaN values
        nonNaNValuesX = cellfun(@(x) x(~isnan(x)), result_x, 'UniformOutput', false);
        result__x = [nonNaNValuesX{:}];
        nonNaNValuesY = cellfun(@(x) x(~isnan(x)), result_y, 'UniformOutput', false);
        result__y = [nonNaNValuesY{:}];

        % Random number generation
        number = randi([1, 10]);
        reserve_x = [];
        reserve_y = [];

        % Nested loops for slope comparison
        for i = 1:numel(result__x)
            for j = 1:numel(result__y)
                for m = 1:numel(result__x) - i
                    for n = 1:numel(result__y) - j
                        slope = (result__y(j) - result__y(numel(result__y) - n + 1)) / (result__x(i) - result__x(numel(result__x) - m + 1));
                        slope = subs(slope, 't', number);
                        [slope_C, ~] = coeffs(equation0, x);
                        slope_C = slope_C(1);
                        slope_C = subs(slope_C, 't', number);
                        diagnose = simplify(abs(slope_C - slope));
                        if diagnose < 10
                            % Store values in arrays
                            reserve_x = [reserve_x, [result__x(i); result__x(numel(result__x) - m + 1)]];
                            reserve_y = [reserve_y, [result__y(j); result__y(numel(result__y) - n + 1)]];
                        end
                    end
                end
            end
        end
        compare = simplify(subs(reserve_x(:) - initial_point , 't' , 1));
        [~, minIndex] = min(compare);
        reserve_x = reserve_x(minIndex);
        reserve_y = reserve_y(minIndex);

    elseif flag == 'R_2'

         % Define segmented function
        segmentedFunction = @(m) (m < 0) * -1 + (m >= 0) * 1;

        % Parameter equation
        parameter_2 = @(x, y)  power(2*x / ((1 + segmentedFunction(x)) *2* r_starb - (1 - segmentedFunction(x)) * 2*r_port), 2) + ...
            power(2*y / ((1 + segmentedFunction(y)) *2* r_fort - (1 - segmentedFunction(y)) * 2*r_aft), 2) - 1;

        % Substitute values for x and y in the positive case
        parameter_21 = @(x, y) power(2 * x / ((1 + 1) * 2*r_starb - (1 - 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 + 1) *2* r_fort - (1 - 1) *2* r_aft), 2) - 1;
        parameter_22 = @(x, y) power(2 * x / ((1 - 1) *2* r_starb - (1 + 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 + 1) *2* r_fort - (1 - 1) * 2*r_aft), 2) - 1;
        parameter_23 = @(x, y) power(2 * x / ((1 - 1) * 2*r_starb - (1 + 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 - 1) *2* r_fort - (1 + 1) * 2*r_aft), 2) - 1;
        parameter_24 = @(x, y) power(2 * x / ((1 + 1) * 2*r_starb - (1 - 1) * 2*r_port), 2) + ...
            power(2 * y / ((1 - 1) *2* r_fort - (1 + 1) *2* r_aft), 2) - 1;
        parameter_functions = struct('param_21', parameter_21, 'param_22', parameter_22, 'param_23', parameter_23, 'param_24', parameter_24);

        % Loop over parameter functions
        field_names = fieldnames(parameter_functions);
            for i = 1:numel(field_names)
            current_field = field_names{i};
            current_function = parameter_functions.(current_field);

            % Solve for x and y
            a = simplify(subs(current_function(x, y), y, equation0));
            result_x{i} = simplify(solve(a, x));
            result_y{i} = subs(equation0 , x , result_x{i});
            result_y{i}(1) = simplify_equation (result_y{i}(1));
            result_y{i}(2) = simplify_equation (result_y{i}(2));
            result_x{i}(1) = simplify_equation (result_x{i}(1));
            result_x{i}(2) = simplify_equation (result_x{i}(2));
            % Handle NaN values based on conditions 
            if i == 1
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = result_x{i}(:);
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = result_y{i}(:);
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 2
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(b) = NaN;
                end
            elseif i == 3
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) =result_x{i}(:);
                else
                    result_x{i}(setdiff([1,2], a)) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = result_y{i}(:);
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            elseif i == 4
                a = findNonMatchingBlocks(result_x{i}(1), result_x{i}(2));
                if a == 3
                    result_x{i}(:) = NaN;
                elseif a == 4
                    result_x{i}(:) = NaN;
                else
                    result_x{i}(a) = NaN;
                end
                b = findNonMatchingBlocks(result_y{i}(1), result_y{i}(2));
                if b == 3
                    result_y{i}(:) = NaN;
                elseif b == 4
                    result_y{i}(:) = NaN;
                else
                    result_y{i}(setdiff([1,2], b)) = NaN;
                end
            end
        end

        % Remove NaN values
        nonNaNValuesX = cellfun(@(x) x(~isnan(x)), result_x, 'UniformOutput', false);
        result__x = [nonNaNValuesX{:}];
        nonNaNValuesY = cellfun(@(x) x(~isnan(x)), result_y, 'UniformOutput', false);
        result__y = [nonNaNValuesY{:}];

        % Random number generation
        number = randi([1, 10]);

        % Calculate slopes
        slope_1 = (result__y(3) - result__y(1)) / (result__x(3) - result__x(1));
        slope_1 = subs(slope_1, 't', number);
        slope_2 = (result__y(4) - result__y(2)) / (result__x(4) - result__x(2));
        slope_2 = subs(slope_2, 't', number);

        % Calculate current slope
        [slope_C(t), ~] = coeffs(equation0, x);
        current_slope = slope_C(t);
        error_1 = simplify(abs(slope_1 - subs(current_slope, 't', number)));
        error_2 = simplify(abs(slope_2 - subs(current_slope, 't', number)));
        n = [1 , 3];
        % Adjust result based on slope conditions
        if  error_1  <  error_2
            result__x(2) = NaN;
            result__y(2) = NaN;
            result__x(4) = NaN;
            result__y(4) = NaN;
        else
            result__x(1) = NaN;
            result__y(1) = NaN;
            result__x(3) = NaN;
            result__y(3) = NaN;
            n = [2 , 4];
        end
        reserve_x = result__x(n);
        reserve_y = result__y(n);

        compare = simplify(subs(reserve_x(:) - intersection , 't' , 1));
        [~, minIndex] = min(compare);
        reserve_x = reserve_x(minIndex);
        reserve_y = reserve_y(minIndex);
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%
