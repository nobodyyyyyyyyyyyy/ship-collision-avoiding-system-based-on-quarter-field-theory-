%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
clc
clear all
close all
opposite_velocity = 5*sqrt(2);
x_initial = -5;
y_initial = 10;
time = linspace(0, 1, 200);
random_noise_x = randn(size(time));
random_noise_y = randn(size(time));

x = x_initial + time * opposite_velocity * sind(45);
y= y_initial - time * opposite_velocity * sind(45) ;
dt = 1/200;               % Replace with your actual time step
numSteps = 3;         % Number of steps to predict

[estimatedStates, ~, predictedStates, predictedVelocities] = Kalman_filter(x, y, dt, numSteps);