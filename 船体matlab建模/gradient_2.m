%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
clc
clear all
close all
rng(200);

opposite_velocity_1 = 5*sqrt(2);
u_orignal_velocity = 1;
v_orignal_velocity = 0;
origin_velocity = sqrt(u_orignal_velocity^2+v_orignal_velocity^2);
theta = 0;

x_initial_opposite_1 = -50;
y_initial_opposite_1 = 100;
x_initial_opposite_2 = -50;
y_initial_opposite_2 = 0;
x_initial_opposite_3 = 50;
y_initial_opposite_3 = 50;
global max_np
max_np = 6;
x_initial_orignal = 0;
y_initial_orignal = 0;
v_orignal_velocity = 0;
r_orignal_velocity = 0;
th_orignal_velocity = 0;
sigma_previous = 0;
time_simulation = 6;
time_simulation_number = 2;
time_slice = 60;
Best_score_matrix = 0;
Best_score_store = 0;
Best_pos_np = 0;
Best_pos_angle = 0;
global time_counter u_D
time_counter = 1;
u_D = 0;
k_preferred_store_1 = [];
k_preferred_store_2 = [];
time_span_matrix = time_simulation/time_slice*time_simulation_number;
sotck = struct();

dt = time_simulation/time_slice;               % Replace with your actual time step
numSteps = 6;         % Number of steps to predict

time = linspace(0, time_simulation, time_slice);
random_noise_x = randn(size(time));
random_noise_y = randn(size(time));

x_opposite_1 = x_initial_opposite_1 + time * opposite_velocity_1 * sind(45);
y_opposite_1 = y_initial_opposite_1 - (time * opposite_velocity_1 * sind(45));
x_opposite_2 = x_initial_opposite_2 + time*5;
y_opposite_2 = -x_opposite_2.^2/50 + 50;
x_opposite_3 = -time*5+x_initial_opposite_3;
y_opposite_3 = time*0+y_initial_opposite_3;

x_origin = [eps,eps];
y_origin = [5,5];

control_index = time_simulation_number;
current_Steps = 1;
origin_Best_score = 0;
tag_num = 0;
global sigma_previous_
sigma_previous_ = 0;

while 1
x_opposite__1 = x_opposite_1(1:control_index);
y_opposite__1 = y_opposite_1(1:control_index);
x_opposite__2 = x_opposite_2(1:control_index);
y_opposite__2 = y_opposite_2(1:control_index);
x_opposite__3 = x_opposite_3(1:control_index);
y_opposite__3 = y_opposite_3(1:control_index);

global estimatedStates1 predictedStates1 predictedVelocities1
global estimatedStates2 predictedStates2 predictedVelocities2
global estimatedStates3 predictedStates3 predictedVelocities3

T = current_Steps*dt;
[estimatedStates1, ~, predictedStates1, predictedVelocities1] = Kalman_filter(x_opposite__1, y_opposite__1, dt, numSteps);
[estimatedStates2, ~, predictedStates2, predictedVelocities2] = Kalman_filter(x_opposite__2, y_opposite__2, dt, numSteps);
[estimatedStates3, ~, predictedStates3, predictedVelocities3] = Kalman_filter(x_opposite__3, y_opposite__3, dt, numSteps);

global orignalvariableMatrix updated_orignalvariableMatrix
orignalvariableMatrix = [x_origin(end), y_origin(end), u_orignal_velocity(end), v_orignal_velocity(end), r_orignal_velocity(end), th_orignal_velocity(end), sigma_previous, T, current_Steps];

SearchAgents_no=30; % Number of search agents

Max_iteration=300; % Maximum numbef of iterations

  [lb,ub,dim,fobj] = Get_Functions_details('F1' , T);
  [Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);

global U_quarter
global u_D

% if abs(x_origin(end))<10&&y_origin(end)>=50
%     break;
% end

if  u_D <= 0.3&&y_origin(end)<50
[lb,ub,dim,fobj] = Get_Functions_details('F2' , T);
[Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
end

if  y_origin(end)>=50
SearchAgents_no=20; % Number of search agents
Max_iteration=60; % Maximum numbef of iterations
[lb,ub,dim,fobj] = Get_Functions_details('F2' , T);
[Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
end

Best_pos_angle = [Best_pos_angle , sigma_previous];
Best_pos_np = [Best_pos_np , Best_pos(2)];

if abs(U_quarter - origin_Best_score)<0.05&&U_quarter >= 0.85
    current_Steps = round(current_Steps*2);
    if current_Steps > numSteps
        current_Steps = numSteps;
    elseif current_Steps < 1
        current_Steps = 1;
    end
else
    current_Steps = round(current_Steps/2);
    if current_Steps > numSteps
        current_Steps = numSteps;
    elseif current_Steps < 1
        current_Steps = 1;
    end
end
control_index = control_index + current_Steps;
x_origin(end+1) = updated_orignalvariableMatrix.a1;
y_origin(end+1) = updated_orignalvariableMatrix.a2;
u_orignal_velocity = updated_orignalvariableMatrix.a3;
v_orignal_velocity = updated_orignalvariableMatrix.a4;
r_orignal_velocity = updated_orignalvariableMatrix.a5;
th_orignal_velocity = updated_orignalvariableMatrix.a6;

global U_ship_origin_xy
sotck(time_counter).a1 = U_ship_origin_xy(1,:);
sotck(time_counter).a2 = U_ship_origin_xy(2,:);
sotck(time_counter).a3 = u_orignal_velocity;
sotck(time_counter).a4 = v_orignal_velocity;
sotck(time_counter).a5 = r_orignal_velocity;
sotck(time_counter).a6 = th_orignal_velocity;

sigma_previous = sigma_previous_;
Best_score_matrix(end+1) = Best_score;
global k_preferred_1 k_preferred_2
k_preferred_1 = -abs(origin_Best_score - U_quarter)/abs(y_origin(end)-y_origin(end-1));
k_preferred_store_1 = [k_preferred_store_1 , k_preferred_1];
k_preferred_2 = -abs(origin_Best_score - U_quarter)/abs(x_origin(end)-x_origin(end-1));
k_preferred_store_2 = [k_preferred_store_2 , k_preferred_2];
origin_Best_score = U_quarter;

if control_index >= time_slice
    break;
end

% if numel(Best_score_matrix)>10
%    if all(Best_score_matrix(end-5: end) == 0)
%       break;
%    end
% end
   time_counter = time_counter + 1;
   time_span_matrix = [time_span_matrix , control_index*dt];
   global U_ship_origin_y_combine
end

    videoFile1 = ['animation_np_' num2str(1) '.mp4'];
    writerObj1 = VideoWriter(videoFile1, 'MPEG-4');
    writerObj1.FrameRate = 1; 
    open(writerObj1);  
    figure;
    for j = 1:time_counter - 1
        subplot('Position', [0.1 0.1 0.6 0.8]);
        plot(time_span_matrix(1:j), Best_pos_np(1:j), 'LineWidth', 1,'LineStyle','-','Color','r');
        hold on;
        title('t-np');
        xlabel('time');
        ylabel('np');
        axis([min(time_span_matrix), max(time_span_matrix), min(Best_pos_np)+3, max(Best_pos_np)+3]); 
        grid on;
        legend('np', 'Location','northeast','FontSize',12);
        subplot('Position', [0.75 0.1 0.25 0.8]);
        U__1 = U_ship_origin_y_combine(j*SearchAgents_no*Max_iteration , 5);
        U__2 = U_ship_origin_y_combine(j*SearchAgents_no*Max_iteration , 6);
        U__3 = U_ship_origin_y_combine(j*SearchAgents_no*Max_iteration , 7);
        data = [U__1 , U__2 , U__3];
        colormap(autumn(length(data)));
        bar(data, 'FaceColor', 'flat');
        colorbar;
        frame = getframe(gcf);
        writeVideo(writerObj1, frame);
    end
    close(writerObj1);
    
    videoFile1 = ['animation_angle_' num2str(1) '.mp4'];
    writerObj2 = VideoWriter(videoFile1, 'MPEG-4');
    writerObj2.FrameRate = 1; 
    open(writerObj2);  
    figure;
    for j = 1:time_counter - 1
        subplot('Position', [0.1 0.1 0.6 0.8]);
        plot(time_span_matrix(1:j), Best_pos_angle(1:j), 'LineWidth', 1,'LineStyle','-','Color','b');
        hold on;
        title('t-angle');
        xlabel('time');
        ylabel('np');
        axis([min(time_span_matrix), max(time_span_matrix), min(Best_pos_angle)+3, max(Best_pos_angle)+3]); 
        grid on;
        legend('Angle', 'Location','northwest','FontSize',12);
        subplot('Position', [0.75 0.1 0.25 0.8]);
        U__1 = U_ship_origin_y_combine(j*SearchAgents_no*Max_iteration , 5);
        U__2 = U_ship_origin_y_combine(j*SearchAgents_no*Max_iteration , 6);
        U__3 = U_ship_origin_y_combine(j*SearchAgents_no*Max_iteration , 7);
        data = [U__1 , U__2 , U__3];
        colormap(autumn(length(data)));
        bar(data, 'FaceColor', 'flat');
        colorbar;
        frame = getframe(gcf);
        writeVideo(writerObj2, frame);
    end
    close(writerObj2);
    
    videoFile1 = ['curve' num2str(3) '.mp4'];
    writerObj3 = VideoWriter(videoFile1, 'MPEG-4');                 %U_ship_origin_y = [U , ship_origin_x ,ship_origin_y , U_ , U_1 , U_2 , U_3];
    writerObj3.FrameRate = 20; 
    open(writerObj3);  
    figure;
    set(gcf, 'Position', [0, 0, 1262, 1262]);
    for j = 1:time_counter - 1
        x_simutation = sotck(j).a1;
        y_simutation = sotck(j).a2;
        scale_control = round(linspace(1, 199, 20));
        if j == 1
            t_simulation_ber = 0;
            t_simulation_aft = time_span_matrix(1);
            time_simulation_ = linspace(t_simulation_ber , t_simulation_aft , size(x_simutation , 2));
        else
            [t_simulation_ber, t_simulation_aft] = deal(time_span_matrix(j - 1) , time_span_matrix(j));
            time_simulation_ = linspace(t_simulation_ber , t_simulation_aft , size(x_simutation , 2));
        end
        for h = 1 : size(scale_control , 2)

        x_opposite_1 = x_initial_opposite_1 + time_simulation_ * opposite_velocity_1 * sind(45);
        y_opposite_1 = y_initial_opposite_1 - (time_simulation_ * opposite_velocity_1 * sind(45));
        x_opposite_2 = x_initial_opposite_2 + time_simulation_*5;
        y_opposite_2 = -x_opposite_2.^2/50 + 50;
        x_opposite_3 = -time_simulation_*5+x_initial_opposite_3;
        y_opposite_3 = time_simulation_*0+y_initial_opposite_3;
        x_simutation = sotck(j).a1;
        y_simutation = sotck(j).a2;
        x_opposite_1 = x_opposite_1(scale_control);
        y_opposite_1 = y_opposite_1(scale_control);
        x_opposite_2 = x_opposite_2(scale_control);
        y_opposite_2 = y_opposite_2(scale_control);
        x_opposite_3 = x_opposite_3(scale_control);
        y_opposite_3 = y_opposite_3(scale_control);
        x_simutation = x_simutation(scale_control);
        y_simutation = y_simutation(scale_control);

        plot(x_opposite_1(1:h), y_opposite_1(1:h), 'LineWidth', 3,'LineStyle','-','Color','r');
        hold on;
        plot(x_opposite_2(1:h), y_opposite_2(1:h), 'LineWidth', 3,'LineStyle','-','Color','cyan');
        hold on;
        plot(x_opposite_3(1:h), y_opposite_3(1:h), 'LineWidth', 3,'LineStyle','-','Color','g');
        hold on;
        plot(x_simutation(1:h), y_simutation(1:h), 'LineWidth', 3,'LineStyle','-','Color','b');
        hold on;
        title('curve');
        xlabel('x');
        ylabel('y');
        axis([-100, 100, 0, 200]); 
        grid on;
        legend('Opposite 1', 'Opposite 2', 'Opposite 3', 'Simulation');
        frame = getframe(gcf);
        writeVideo(writerObj3, frame);
        end
    end
    close(writerObj3);

    videoFile1 = ['state' num2str(4) '.mp4'];
    writerObj4 = VideoWriter(videoFile1, 'MPEG-4');                 %U_ship_origin_y = [U , ship_origin_x ,ship_origin_y , U_ , U_1 , U_2 , U_3];
    writerObj4.FrameRate = 20; 
    open(writerObj4);  
    figure;
    set(gcf, 'Position', [0, 0, 1262, 946]);
    for j = 1:time_counter - 1

        if time_counter == 1
            t_simulation_ber = 0;
            t_simulation_aft = time_span_matrix(1);
            time_simulation_add = linspace(t_simulation_ber , t_simulation_aft , size(sotck(j).a1 , 2));
        else
            [t_simulation_ber, t_simulation_aft] = deal(time_span_matrix(time_counter - 1) , time_span_matrix(time_counter));
            time_simulation_add = linspace(t_simulation_ber , t_simulation_aft , size(sotck(j).a1 , 2));
        end
        for h = 1 : size(scale_control , 2)
        u_simutation = sotck(j).a3;
        v_simutation = sotck(j).a4;
        r_simutation = sotck(j).a5;
        th_simutation = sotck(j).a6;
        th_simutation = th_simutation(scale_control);
        r_simutation = r_simutation(scale_control);
        v_simutation = v_simutation(scale_control);
        u_simutation = u_simutation(scale_control);
        time_simulation_ = time_simulation_add(scale_control);

% 第一个子图：u_simutation
subplot(2, 2, 1);
plot(time_simulation_(1:h), u_simutation(1:h), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'b');
title('u\_simulation');
legend('u');


% 第二个子图：v_simutation
subplot(2, 2, 2);
plot(time_simulation_(1:h), v_simutation(1:h), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'b');
title('v\_simulation');
legend('v');


% 第三个子图：r_simutation
subplot(2, 2, 3);
plot(time_simulation_(1:h), r_simutation(1:h), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'b');
title('r\_simulation');
legend('r');


% 第四个子图：th_simutation
subplot(2, 2, 4);
plot(time_simulation_(1:h), th_simutation(1:h), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'b');
title('th\_simulation');
legend('th');

        
        frame = getframe(gcf);
        writeVideo(writerObj4, frame);
        end
    end
    close(writerObj4);
