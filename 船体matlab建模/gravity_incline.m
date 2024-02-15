%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function Z = gravity_incline(x , y , opposite_y)
global time_counter   u_D
global k_preferred_1 k_preferred_2 
n = k_preferred_1*2;
k = k_preferred_2*0.5;
if time_counter == 1||u_D<0.3
    n = -100;
    k = -20;
end
% opposite_y = 10;

% y = linspace(0, 20, 100);
% x = linspace(-10, 10, 100);           

% [Y, X] = meshgrid(y, x);

Z = -k*(abs(x) - abs(opposite_y)) + n*y;

% figure;
% surf(Y, X, Z, 'EdgeColor', 'none');
% 
% title('Surface Plot of the Function');
% xlabel('y');
% ylabel('x');
% zlabel('z');
% colormap('jet');
% colorbar;
end
