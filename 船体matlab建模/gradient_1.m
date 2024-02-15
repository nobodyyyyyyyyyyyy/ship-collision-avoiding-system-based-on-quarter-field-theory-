% Example usage
clc
clear all
close all
opposite_velocity = 5*sqrt(2);
u_orignal_velocity = 5;
v_orignal_velocity = 0;
origin_velocity = sqrt(u_orignal_velocity^2+v_orignal_velocity^2);
theta = 0;
x_initial_opposite = -5;
y_initial_opposite = 10;
x_initial_orignal = 0;
y_initial_orignal = 0;
v_orignal_velocity = 0;
r_orignal_velocity = 0;
th_orignal_velocity = 0;
sigma_previous = 0;
time_simulation = 1.3;
time_simulation_number = 90;
time_slice = 120;
Best_score_matrix = 0;
Best_score_store = 0;
Best_pos_np = 0;
Best_pos_angle = 0;
time_counter = 0;
time_span_matrix = time_simulation/time_slice*time_simulation_number;

dt = time_simulation/time_slice;               % Replace with your actual time step
numSteps = 6;         % Number of steps to predict

time = linspace(0, time_simulation, time_slice);
random_noise_x = randn(size(time));
random_noise_y = randn(size(time));
x_opposite = x_initial_opposite + time * opposite_velocity * sind(45);
y_opposite = y_initial_opposite - (time * opposite_velocity * sind(45));
x_origin = [eps,eps];
y_origin = [5,5];

control_index = time_simulation_number;
current_Steps = 1;
origin_Best_score = 0;

while 1
x_opposite_ = x_opposite(1:control_index);
y_opposite_ = y_opposite(1:control_index);

global estimatedStates predictedStates predictedVelocities

T = current_Steps*dt;
[estimatedStates, ~, predictedStates, predictedVelocities] = Kalman_filter(x_opposite_, y_opposite_, dt, numSteps);

global orignalvariableMatrix updated_orignalvariableMatrix
orignalvariableMatrix = [x_origin(end), y_origin(end), u_orignal_velocity(end), v_orignal_velocity(end), r_orignal_velocity(end), th_orignal_velocity(end), sigma_previous, T, current_Steps];

SearchAgents_no=20; % Number of search agents

Max_iteration=30; % Maximum numbef of iterations
[lb,ub,dim,fobj] = Get_Functions_details('F1');
[Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
Best_score_store = [Best_score_store , Best_score];

if Best_score < 0.522
[lb,ub,dim,fobj] = Get_Functions_details('F2');
[Best_score,Best_pos,GWO_cg_curve]=GWO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);
end
Best_pos_angle = [Best_pos_angle , Best_pos(1)];
Best_pos_np = [Best_pos_np , Best_pos(2)];

if abs(Best_score - origin_Best_score)<0.05&&Best_score<0.9
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
control_index = time_simulation_number + current_Steps;
time_simulation_number = time_simulation_number + current_Steps;
x_origin(end+1) = updated_orignalvariableMatrix.a1;
y_origin(end+1) = updated_orignalvariableMatrix.a2;
u_orignal_velocity = updated_orignalvariableMatrix.a3;
v_orignal_velocity = updated_orignalvariableMatrix.a4;
r_orignal_velocity = updated_orignalvariableMatrix.a5;
th_orignal_velocity = updated_orignalvariableMatrix.a6;
sigma_previous = Best_pos(1);
Best_score_matrix(end+1) = Best_score;
origin_Best_score = Best_score;

if control_index == time_slice
    break;
end

if numel(Best_score_matrix)>10
   if all(Best_score_matrix(end-9: end) <= 0)
      break;
   end
end
   time_counter = time_counter + 1;
   time_span_matrix = [time_span_matrix , control_index*dt];
end

for i = 1:length(time_counter)
    videoFile1 = ['animation_np_' num2str(i) '.mp4'];
    writerObj1 = VideoWriter(videoFile1, 'MPEG-4');
    writerObj1.FrameRate = 10; 
    open(writerObj1);
    
    figure;
    plot(time_span_matrix(1:i), Best_pos_np(1:i), 'LineWidth', 2);
    title(t-np);
    xlabel('time');
    ylabel('np');
    axis([min(time_span_matrix), max(time_span_matrix), min(Best_pos_np), max(Best_pos_np)]); 
    grid on;
    
    for j = 1:i
        frame = getframe(gcf);
        writeVideo(writerObj1, frame);
    end
    
    close(writerObj1);
    
    videoFile2 = ['animation_angle_' num2str(i) '.mp4'];
    writerObj2 = VideoWriter(videoFile2, 'MPEG-4');
    writerObj2.FrameRate = 10; 
    open(writerObj2);

    figure;
    plot(time_span_matrix(1:i), Best_pos_angle(1:i), 'LineWidth', 2);
    title(t-np);
    xlabel('time');
    ylabel('angle');
    axis([min(time_span_matrix), max(time_span_matrix), min(Best_pos_angle), max(Best_pos_angle)]); 
    grid on;
    
    for j = 1:i
        frame = getframe(gcf);
        writeVideo(writerObj2, frame);
    end
    
    close(writerObj2);
    
    videoFile3 = ['animation_score_' num2str(i) '.mp4'];
    writerObj3 = VideoWriter(videoFile3, 'MPEG-4');
    writerObj3.FrameRate = 10; 
    open(writerObj3);
    
    figure;
    plot(time_span_matrix(1:i), Best_score_store(1:i), 'LineWidth', 2);
    title(t-np);
    xlabel('time');
    ylabel('score');
    axis([min(time_span_matrix), max(time_span_matrix), min(Best_score_store), max(Best_score_store)]); 
    grid on;
    
    for j = 1:i
        frame = getframe(gcf);
        writeVideo(writerObj3, frame);
    end
    close(writerObj3);
end
