%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function [estimatedStates, covarianceMatrix, predictedstates, predictedVelocities] = Kalman_filter(x, y, dt, numSteps)
    % Initialize Kalman filter parameters
    A = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];  % State transition matrix
    H = [1 0 0 0; 0 0 1 0];   % Measurement matrix
    Q = diag([0.5, 0.5, 0.5, 0.5]); % Process noise covariance
    R = diag([0.5, 0.5]); % Measurement noise covariance

    % Initialize state and covariance matrix
    initialVelocityX = (x(2) - x(1)) / dt;
    initialVelocityY = (y(2) - y(1)) / dt;
    initialState = [x(1); initialVelocityX; y(1); initialVelocityY];
    initialCovariance = eye(4);

    currentState = initialState;
    covarianceMatrix = initialCovariance;

    % Kalman filter loop
    estimatedStates = zeros(4, length(x));
    predictedStates = zeros(4, numSteps);
    predictedVelocities = zeros(2, numSteps); % Changed to 2D array
    velocities = zeros(2, length(x));
    
for i = 1:length(x)
    % Prediction step
    currentState = A * currentState;
    covarianceMatrix = A * covarianceMatrix * A' + Q;

    % Update step
    kalmanGain = covarianceMatrix * H' / (H * covarianceMatrix * H' + R);
    measurement = [x(i); y(i)];
    currentState = currentState + kalmanGain * (measurement - H * currentState);
    covarianceMatrix = (eye(4) - kalmanGain * H) * covarianceMatrix;

    % Store the estimated state
    estimatedStates(:, i) = currentState;

    % Calculate velocities at each time step
    velocities(:, i) = currentState(2:2:4);

    % Update A matrix based on changing velocity
end

    % Prediction for the next few steps
    predictedStates(:, 1) = currentState;
    for j = 1:numSteps
        currentState = A * currentState;
        predictedStates(:, j) = currentState;
        predictedVelocities(:, j) = currentState(2:2:4); % Extract velocities from the state
    end

    predictedstates = [predictedStates(1, :) ; predictedStates(3, :)];
    % Plot the results in 3D
%     figure;
% 
%     subplot(2, 1, 1);
%     plot3(x, y, 1:length(x), 'o-', 'DisplayName', 'Measured Positions');
%     hold on;
%     plot3(estimatedStates(1, :), estimatedStates(3, :), 1:length(x), 'x-', 'DisplayName', 'Estimated States');
%     plot3(predictedStates(1, :), predictedStates(3, :), (length(x)+1):(length(x)+numSteps), 's-', 'DisplayName', 'Predicted States');
%     legend('Location', 'best');
%     xlabel('X Position');
%     ylabel('Y Position');
%     zlabel('Time Step');
%     title('Kalman Filter for Ship Position Estimation');
%     grid on;
% 
%     subplot(2, 1, 2);
%     plot3(predictedStates(1, :), predictedStates(3, :), (length(x)+1):(length(x)+numSteps), 's-', 'DisplayName', 'Predicted Positions');
%     legend('Location', 'best');
%     xlabel('X Position');
%     ylabel('Y Position');
%     zlabel('Time Step');
%     title('Kalman Filter Predictions');
%     grid on;
end


