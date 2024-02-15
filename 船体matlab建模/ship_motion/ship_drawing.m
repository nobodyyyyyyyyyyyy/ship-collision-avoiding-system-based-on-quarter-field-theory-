%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function ship_drawing(center, angle_ship)
[~, distance, angle] = ship_control;
angle_position = angle + angle_ship;
theta = deg2rad(angle_position);
x = center(1) + distance' .* sin(theta);
y = center(2) + distance' .* cos(theta);
smoothed_x = smooth(x, 0.1, 'rloess');
smoothed_y = smooth(y, 0.1, 'rloess');
figure;
n = 200;
tq = 0:1/n:1;
xyqP = interpclosed(x,y,tq,'linear');
plot(xyqP(1,:),xyqP(2,:),'linewidth',2.5,'MarkerSize',30);
axis equal;
hold off;


