%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function [lb,ub,dim,fobj] = Get_Functions_details(F , T)
global sigma_previous_ 
switch F
case 'F1'
fobj = @F1;
global max_np orignalvariableMatrix u_D
if (sigma_previous_-T*3000)>-35
    sigma_L = (sigma_previous_-T*3000);
else 
    sigma_L = -35;
end

if (sigma_previous_+T*3000)>35
    sigma_U = 35;
else 
    sigma_U = sigma_previous_+T*3000;
end
if abs(orignalvariableMatrix(1)) <= eps&&u_D<0.4
    sigma_L = 0;
    sigma_U = 0;
end
lb=[sigma_L , max_np-1];
ub=[sigma_U , max_np+1];
dim=2;

case 'F2'
fobj = @F2;
global max_np sigma_previous_ orignalvariableMatrix u_D
if (sigma_previous_-T*3000)>-35
    sigma_L = (sigma_previous_-T*3000);
else 
    sigma_L = -35;
end

if (sigma_previous_+T*3000)>35
    sigma_U = 35;
else 
    sigma_U = sigma_previous_+T*3000;
end
if abs(orignalvariableMatrix(1)) <= eps&&u_D<0.3
    sigma_L = 0;
    sigma_U = 0;
end

lb=[sigma_L , max_np-1];
ub=[sigma_U , max_np+1];
dim=2;
end

end

function [U] = F1(x) %x(1) = sigma_previous、x(2) = np
flag = 1;
flag_exit = 0;
global orignalvariableMatrix 

x_initial_orignal = orignalvariableMatrix(1);
y_initial_orignal = orignalvariableMatrix(2);
u_orignal_velocity = orignalvariableMatrix(3);
v_orignal_velocity = orignalvariableMatrix(4);
r_orignal_velocity = orignalvariableMatrix(5);
th_orignal_velocity = orignalvariableMatrix(6);
sigma_previous = orignalvariableMatrix(7);
T = orignalvariableMatrix(8);
current_Steps = orignalvariableMatrix(9);
global sigma_previous_

global updated_orignalvariableMatrix
[ship_origin_x, ship_origin_y, u, v, r, theta, sigma_previous_] = hydrodyn_1(x_initial_orignal, y_initial_orignal, u_orignal_velocity, v_orignal_velocity, r_orignal_velocity, th_orignal_velocity, sigma_previous, x(1), x(2)*60,T); %得到在该状况之下，未知为sigma与np，T时间后的状态
global U_ship_origin_xy
U_ship_origin_xy = [ship_origin_x ;ship_origin_y];


ship_origin_x = ship_origin_x(end);
ship_origin_y = ship_origin_y(end);

updated_orignalvariableMatrix = struct('a1', [], 'a2', [], 'a3', [], 'a4', [], 'a5', [], 'a6', []);

fieldNames = {'a1', 'a2', 'a3', 'a4', 'a5', 'a6'};
values = {ship_origin_x, ship_origin_y, u, v, r, theta};

for i = 1:length(fieldNames)
eval(['updated_orignalvariableMatrix.', fieldNames{i}, ' = values{i};']);
end

%空一个全局变量以传入ship_opposite_x_present, ship_opposite_y_present

global estimatedStates1 predictedStates1 predictedVelocities1
global estimatedStates2 predictedStates2 predictedVelocities2
global estimatedStates3 predictedStates3 predictedVelocities3

ship_opposite_x_present_1 = [estimatedStates1(1 , :) , predictedStates1(1 , 1:current_Steps)];
ship_opposite_y_present_1 = [estimatedStates1(2 , :) , predictedStates1(2 , 1:current_Steps)];
opposite_velocity_1 = sqrt(predictedVelocities1(1 , current_Steps)^2 + predictedVelocities1(2 , current_Steps)^2);

ship_opposite_x_present_2 = [estimatedStates2(1 , :) , predictedStates2(1 , 1:current_Steps)];
ship_opposite_y_present_2 = [estimatedStates2(2 , :) , predictedStates2(2 , 1:current_Steps)];
opposite_velocity_2 = sqrt(predictedVelocities2(1 , current_Steps)^2 + predictedVelocities2(2 , current_Steps)^2);

ship_opposite_x_present_3 = [estimatedStates3(1 , :) , predictedStates3(1 , 1:current_Steps)];
ship_opposite_y_present_3 = [estimatedStates3(2 , :) , predictedStates3(2 , 1:current_Steps)];
opposite_velocity_3 = sqrt(predictedVelocities3(1 , current_Steps)^2 + predictedVelocities3(2 , current_Steps)^2);

[ship_opposite_rotated_x_1 , ship_opposite_rotated_y_1 , slope_opposite_1] = transition_matrix(ship_opposite_x_present_1, ship_opposite_y_present_1, ship_origin_x, ship_origin_y, theta);
flag_matrix = ship_opposite_x_present_1(end) - ship_opposite_x_present_1(end-1);
if flag_matrix <0
    flag = -1;
end
global u_D
origin_velocity = sqrt(u(end)^2 + v(end)^2);
[U_1, ~, ~, u_D_1, ~, ~, distance_D_1] = direction_modify(ship_opposite_rotated_x_1, ship_opposite_rotated_y_1, origin_velocity, opposite_velocity_1, slope_opposite_1, flag);

[ship_opposite_rotated_x_2 , ship_opposite_rotated_y_2 , slope_opposite_2] = transition_matrix(ship_opposite_x_present_2, ship_opposite_y_present_2, ship_origin_x, ship_origin_y, theta);
flag_matrix = ship_opposite_x_present_2(end) - ship_opposite_x_present_2(end-1);
if flag_matrix <0
    flag = -1;
end
origin_velocity = sqrt(u(end)^2 + v(end)^2);
[U_2, ~, ~, u_D_2, ~, ~, distance_D_2] = direction_modify(ship_opposite_rotated_x_2, ship_opposite_rotated_y_2, origin_velocity, opposite_velocity_2, slope_opposite_2 , flag);

[ship_opposite_rotated_x_3 , ship_opposite_rotated_y_3 , slope_opposite_3] = transition_matrix(ship_opposite_x_present_3, ship_opposite_y_present_3, ship_origin_x, ship_origin_y, theta);
flag_matrix = ship_opposite_x_present_3(end) - ship_opposite_x_present_3(end-1);
if flag_matrix <0
    flag = -1;
end
origin_velocity = sqrt(u(end)^2 + v(end)^2);
[U_3, ~, ~, u_D_3, ~, ~, distance_D_3] = direction_modify(ship_opposite_rotated_x_3, ship_opposite_rotated_y_3, origin_velocity, opposite_velocity_3, slope_opposite_3 , flag);

[U_, index] = max([U_1, U_2, U_3], [], 2);
matrix_u_D = [u_D_1 , u_D_2 , u_D_3];
u_D = matrix_u_D(index);
U = gravity_incline(ship_origin_x , ship_origin_y , 100) + U_;

global U_ship_origin_y_combine U_quarter
U_ship_origin_y = [U , ship_origin_x ,ship_origin_y , U_ , U_1 , U_2 , U_3];
U_quarter = U_;
U_ship_origin_y_combine = [U_ship_origin_y_combine ; U_ship_origin_y];

end

function [U] = F2(x) %x(1) = sigma_previous、x(2) = np
flag_exit = 0;
global orignalvariableMatrix sigma_previous_

x_initial_orignal = orignalvariableMatrix(1);
y_initial_orignal = orignalvariableMatrix(2);
u_orignal_velocity = orignalvariableMatrix(3);
v_orignal_velocity = orignalvariableMatrix(4);
r_orignal_velocity = orignalvariableMatrix(5);
th_orignal_velocity = orignalvariableMatrix(6);
sigma_previous = orignalvariableMatrix(7);
T = orignalvariableMatrix(8);
current_Steps = orignalvariableMatrix(9);

global updated_orignalvariableMatrix

[ship_origin_x, ship_origin_y, u, v, r, theta, sigma_previous_] = hydrodyn_1(x_initial_orignal, y_initial_orignal, u_orignal_velocity, v_orignal_velocity, r_orignal_velocity, th_orignal_velocity, sigma_previous, x(1), x(2)*60,T); %得到在该状况之下，未知为sigma与np，T时间后的状态
global U_ship_origin_xy
U_ship_origin_xy = [ship_origin_x ;ship_origin_y];
ship_origin_x = ship_origin_x(end);
ship_origin_y = ship_origin_y(end);

updated_orignalvariableMatrix = struct('a1', [], 'a2', [], 'a3', [], 'a4', [], 'a5', [], 'a6', []);

fieldNames = {'a1', 'a2', 'a3', 'a4', 'a5', 'a6'};
values = {ship_origin_x, ship_origin_y, u, v, r, theta};

for i = 1:length(fieldNames)
eval(['updated_orignalvariableMatrix.', fieldNames{i}, ' = values{i};']);
end

U = gravity_incline(ship_origin_x , ship_origin_y , 100);
end
