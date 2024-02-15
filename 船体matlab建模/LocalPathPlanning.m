function LocalPathPlanning()
    
    % Initial position and orientation 
    x = -0.5;
    y = 0.5;
    theta = 0;
    
    % Goal position
    x_goal = 3.5;
    y_goal = 2.75;
    position_accuracy = 0.1;
    
    % Sampling period
    dT = 0.1;
    
    % Generate obstacles
    Obstacle_count = 10;
    angles = linspace(0, 2*pi, 360)';
    obstacle = zeros(Obstacle_count, length(angles), 2);
    c = zeros(Obstacle_count,2);
    r = zeros(Obstacle_count,1);
    for i=1:Obstacle_count
        while 1
            c(i,:) = 4*rand(1,2) - 1;
            r(i) = 0.25*rand() + 0.15;

            if norm([x y] - c(i,:)) > (r(i) + 0.35) && norm([x_goal y_goal] - c(i,:)) > (r(i) + 0.35)
                if i == 1, break; end
                [idx, dist] = dsearchn([c(1:(i-1),1) c(1:(i-1),2)], c(i,:));
                if dist > (r(idx)+r(i)+0.1)
                    break;
                end
            end
        end
        obstacle(i,:,:) = [r(i) * cos(angles)+c(i,1) r(i)*sin(angles)+c(i,2) ];
    end
    
    % Simulation
    simTimeMax = 600;
    APF = ArtificialPotentialField(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax);     
    SAPF = SafeArtificialPotentialField(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax);    
    DWA = DynamicWindowApproach(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax);
    VAPF = VortexArtificialPotentialField(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax);   

    % Plot it
    figure(1);
    cla; hold on; grid on; box on;
    daspect([1 1 1]); 
    xlim([-1,4]);  ylim([-1 3]);
    box on; hold on;
    plot(DWA.X(1:DWA.t), DWA.Y(1:DWA.t), 'Color',[0.8500 0.3250 0.0980], 'LineWidth', 2); % Plot traveled path
    plot(APF.X(1:APF.t), APF.Y(1:APF.t), 'Color',[0 0.4470 0.7410], 'LineWidth', 2); % Plot traveled path
    plot(VAPF.X(1:VAPF.t), VAPF.Y(1:VAPF.t), 'Color',[0.4660 0.6740 0.1880], 'LineWidth', 2); % Plot traveled path
    plot(SAPF.X(1:SAPF.t), SAPF.Y(1:SAPF.t), 'Color',[0.6350 0.0780 0.1840], 'LineWidth', 2); % Plot traveled path
    plot(x_goal, y_goal, 'xg');
    for i=1:Obstacle_count
        plot(obstacle(i,:,1), obstacle(i,:,2), '-r');
    end
    legend('DWA', 'APF', 'VAPF', 'SAPF', 'Location','best');
    drawnow;
end

function APF = ArtificialPotentialField(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax)
    
    % APF parameters
    APF.zeta = 1.1547;
    APF.eta = 0.0732;
    APF.dstar = 0.3;
    APF.Qstar = 1.0;
    
    % Parameters related to kinematic model
    APF.error_theta_max = deg2rad(45);
    APF.v_max = 0.2;
    APF.Kp_omega = 1.5;
    APF.omega_max = 0.5*pi; 

    t = 1;
    APF.X = zeros(1,simTimeMax);
    APF.Y = zeros(1,simTimeMax);
    APF.Theta = zeros(1,simTimeMax);
    APF.X(1) = x;
    APF.Y(1) = y;
    APF.Theta(1) = theta;

    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;

    Obstacle_count = size(obstacle,1);

    while norm([x_goal y_goal] - [x y]) > position_accuracy && t < simTimeMax  
        tic;
    
        % Calculate Attractive Potential
        if norm([x y]-[x_goal y_goal]) <= APF.dstar
            nablaU_att =  APF.zeta*([x y]-[x_goal y_goal]);
        else 
            nablaU_att = APF.dstar/norm([x y]-[x_goal y_goal]) * APF.zeta*([x y]-[x_goal y_goal]);
        end
    
        % Find the minimum distance from the obstacle & Calculate Repulsive Potential
        obst_idx = zeros(1,Obstacle_count);
        obst_dist = zeros(1,Obstacle_count);
        nablaU_rep = [0 0];
        for i=1:Obstacle_count
            [obst_idx(i), obst_dist(i)] = dsearchn([obstacle(i,:,1)' obstacle(i,:,2)'], [x y]);
            obst_dist(i) = obst_dist(i) + 0.01*(2*rand()-1); % MEASUREMENT NOISE
            if obst_dist(i) <= APF.Qstar     
                nablaU_rep = nablaU_rep + (APF.eta*(1/APF.Qstar - 1/obst_dist(i)) * 1/obst_dist(i)^2)*([x y] - [obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)]);
            end
        end
        
        % Calculate final potential
        nablaU = nablaU_att+nablaU_rep;
    
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));
    
        error_theta = theta_ref - theta;
        if abs(error_theta) <= APF.error_theta_max
            alpha = (APF.error_theta_max - abs(error_theta)) / APF.error_theta_max;
            v_ref = min( alpha*norm(-nablaU), APF.v_max );
        else
            v_ref = 0;
        end
    
        t_i = toc;
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i;
        t_count = t_count + 1;
    
        % Simple kinematic mobile robot model
        % Omitted dynamics.
        omega_ref = APF.Kp_omega * error_theta;
        omega_ref = min( max(omega_ref, -APF.omega_max), APF.omega_max);
        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta));
        x = x + v_ref*cos(theta) * dT;
        y = y + v_ref*sin(theta) * dT;
    
        % Archive and plot it
        t = t + 1;
        APF.X(t) = x;
        APF.Y(t) = y;
        APF.Theta(t) = theta;
    end

    APF.t = t;
    APF.travelTime = (t-1)*dT;
    APF.MeanCalculationTime = t_sum/t_count;
    APF.MaxCalculationTime = t_max;
    APF.MinCalculationTime = t_min;
end

function SAPF = SafeArtificialPotentialField(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax)
    
    % SAPF parameters
    SAPF.zeta = 1.1547;
    SAPF.eta = 0.0732;
    SAPF.dstar = 0.3;
    SAPF.Qstar = 1.0;
    SAPF.dsafe = 0.2;
    SAPF.dvort = 0.35;
    SAPF.alpha_th = deg2rad(5);

    % Parameters related to kinematic model
    SAPF.error_theta_max = deg2rad(45);
    SAPF.v_max = 0.2;
    SAPF.Kp_omega = 1.5;
    SAPF.omega_max = 0.5*pi; 

    t = 1;
    SAPF.X = zeros(1,simTimeMax);
    SAPF.Y = zeros(1,simTimeMax);
    SAPF.Theta = zeros(1,simTimeMax);
    SAPF.X(1) = x;
    SAPF.Y(1) = y;
    SAPF.Theta(1) = theta;


    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;

    Obstacle_count = size(obstacle,1);

    while norm([x_goal y_goal] - [x y]) > position_accuracy && t < simTimeMax  
        tic;
    
        % Calculate Attractive Potential
        if norm([x y]-[x_goal y_goal]) <= SAPF.dstar
            nablaU_att =  SAPF.zeta*([x y]-[x_goal y_goal]);
        else 
            nablaU_att = SAPF.dstar/norm([x y]-[x_goal y_goal]) * SAPF.zeta*([x y]-[x_goal y_goal]);
        end
    
        % Find the minimum distance from the obstacle & Calculate Repulsive Potential
        obst_idx = zeros(1,Obstacle_count);
        obst_dist = zeros(1,Obstacle_count);
        nablaU_obst = [0 0];
        for i=1:Obstacle_count
            [obst_idx(i), obst_dist(i)] = dsearchn([obstacle(i,:,1)' obstacle(i,:,2)'], [x y]);
            obst_dist(i) = obst_dist(i) + 0.01*(2*rand()-1); % MEASUREMENT NOISE
            alpha = theta - atan2(obstacle(i,obst_idx(i),2)-y, obstacle(i,obst_idx(i),1)-x);
            alpha = atan2(sin(alpha), cos(alpha));
            if obst_dist(i) <= SAPF.Qstar &&  abs(alpha) < deg2rad(150)
                nablaU_rep_Oi = (SAPF.eta*(1/SAPF.Qstar - 1/obst_dist(i)) * 1/obst_dist(i)^2)*([x y] - [obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)]);
                
                if obst_dist(i) <= SAPF.dsafe 
                    drel_Oi = 0;
                elseif obst_dist(i) >= 2*SAPF.dvort-SAPF.dsafe 
                    drel_Oi = 1; 
                else
                    drel_Oi = (obst_dist(i)-SAPF.dsafe) / (2*(SAPF.dvort-SAPF.dsafe));    
                end
                
                if rad2deg(alpha) <= SAPF.alpha_th, D_alpha = +1; else, D_alpha = -1; end 

                if drel_Oi <= 0.5
                    gamma = pi * D_alpha * drel_Oi;
                else
                    gamma = pi * D_alpha * (1-drel_Oi);
                end

                R = [cos(gamma) -sin(gamma)
                     sin(gamma)  cos(gamma) ];

                nablaU_obst = nablaU_obst + (R*nablaU_rep_Oi')';
            end
        end
        
        % Calculate final potential
        nablaU = nablaU_att+nablaU_obst;
    
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));
    
        error_theta = theta_ref - theta;
        if abs(error_theta) <= SAPF.error_theta_max
            alpha = (SAPF.error_theta_max - abs(error_theta)) / SAPF.error_theta_max;
            v_ref = min( alpha*norm(-nablaU), SAPF.v_max );
        else
            v_ref = 0;
        end
    
        t_i = toc;
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i;
        t_count = t_count + 1;
    
        % Simple kinematic mobile robot model
        % Omitted dynamics.
        omega_ref = SAPF.Kp_omega * error_theta;
        omega_ref = min( max(omega_ref, -SAPF.omega_max), SAPF.omega_max);
        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta));
        x = x + v_ref*cos(theta) * dT;
        y = y + v_ref*sin(theta) * dT;
    
        % Archive and plot it
        t = t + 1;
        SAPF.X(t) = x;
        SAPF.Y(t) = y;
        SAPF.Theta(t) = theta;
    end

    SAPF.t = t;
    SAPF.travelTime = (t-1)*dT;
    SAPF.MeanCalculationTime = t_sum/t_count;
    SAPF.MaxCalculationTime = t_max;
    SAPF.MinCalculationTime = t_min;
end

function DWA = DynamicWindowApproach(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax)
    
    DWA.heading = 0.1;
    DWA.distance = 1;
    DWA.velocity = 10;
    DWA.predictionTime = 1;
    DWA.v_accuracyPrediction = 0.005;
    DWA.omega_accuracyPrediction = 0.025*pi;
    DWA.obstacleDistance = 0.25;

    DWA.v_max = 0.2;
    DWA.a_max = 0.2;
    DWA.omega_max = 0.5*pi; 
    DWA.epsilon_max = 1.5*pi; 

    t = 1;
    DWA.X = zeros(1,simTimeMax);
    DWA.Y = zeros(1,simTimeMax);
    DWA.Theta = zeros(1,simTimeMax);
    DWA.X(1) = x;
    DWA.Y(1) = y;
    DWA.Theta(1) = theta;

    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;

    V_current = 0;
    omega_current = 0;

    Obstacle_count = size(obstacle,1);

    while norm([x_goal y_goal] - [x y]) > position_accuracy && t < simTimeMax  
        tic;
    
        % Dynamic Window
        %[Vmin Vmax omega_min omega_max]
        tmp = [0                      DWA.v_max                 -DWA.omega_max                      DWA.omega_max
               V_current-DWA.a_max*dT V_current+DWA.a_max*dT     omega_current-DWA.epsilon_max*dT   omega_current+DWA.epsilon_max*dT];
        dynamicWindow = [max(tmp(:,1)) min(tmp(:,2)) max(tmp(:,3)) min(tmp(:,4))];

        % Evaluate possible movements
        v_range = dynamicWindow(1):DWA.v_accuracyPrediction:dynamicWindow(2);
        omega_range = dynamicWindow(3):DWA.omega_accuracyPrediction:dynamicWindow(4);
        eval_function = zeros(length(v_range), length(omega_range));
        t_predition_range = 0:dT:DWA.predictionTime;
        eval_trajectory = zeros(length(v_range), length(omega_range), length(t_predition_range), 3);
        opt_max_eval_function = -inf;
        opt_idx = [0;0];
        for i = 1:length(v_range)
            for j = 1:length(omega_range)
                % Prediction
                eval_v = v_range(i);
                eval_omega = omega_range(j);
                eval_trajectory(i,j,1,1) = x;
                eval_trajectory(i,j,1,2) = y;
                eval_trajectory(i,j,1,3) = theta;
                for k=2:length(t_predition_range)
                    x_prev = eval_trajectory(i,j,k-1,1);
                    y_prev = eval_trajectory(i,j,k-1,2);
                    theta_prev = eval_trajectory(i,j,k-1,3);
                    eval_trajectory(i,j,k,1) = x_prev + dT*cos(theta_prev)*eval_v;
                    eval_trajectory(i,j,k,2) = y_prev + dT*sin(theta_prev)*eval_v;
                    eval_trajectory(i,j,k,3) = theta_prev + dT*eval_omega;
                end
                
%                 X = reshape( eval_trajectory(i,j,:,1), [], size(eval_trajectory, 3));
%                 Y = reshape( eval_trajectory(i,j,:,2), [], size(eval_trajectory, 3));
%                 hold on; plot(X,Y);

                % Cost function
                robotTheta = eval_trajectory(i,j,end,3);
                goalTheta = atan2(y_goal-eval_trajectory(i,j,end,2), x_goal-eval_trajectory(i,j,end,1));
                headingReward = DWA.heading*(deg2rad(180) - abs(robotTheta - goalTheta));
             
                distanceReward = DWA.distance * 1/(1+ norm([x_goal y_goal] - [eval_trajectory(i,j,end,1) eval_trajectory(i,j,end,2)]));

                velocityReward = DWA.velocity*eval_v;

                eval_function(i,j) = headingReward+distanceReward+velocityReward;
    
                % Find the minimum distance from the obstacle
                obst_idx = zeros(1,Obstacle_count);
                obst_dist = inf*ones(1,Obstacle_count);
                for k=1:Obstacle_count
                    for l=1:2:size(eval_trajectory,3)
                        robotPose = [eval_trajectory(i,j,l,1) eval_trajectory(i,j,l,2)];
                        [tmp_min_dist_idx, tmp_minDist] = dsearchn([obstacle(k,:,1)' obstacle(k,:,2)'], robotPose);
                        tmp_minDist = tmp_minDist + 0.01*(2*rand()-1); % MEASUREMENT NOISE
                        if tmp_minDist < obst_dist(k)
                            obst_idx(k) = tmp_min_dist_idx;
                            obst_dist(k) = tmp_minDist;
                        end
                    end
                end

                if eval_function(i,j) > opt_max_eval_function && min(obst_dist) > DWA.obstacleDistance
                    opt_idx = [i;j];
                    opt_max_eval_function = eval_function(i,j);
                end
            end
        end

        % Select best candidate
        try
        v_ref = v_range(opt_idx(1));
        omega_ref = omega_range(opt_idx(2));
        catch 
            v_ref = 0;
            omega_ref = 0;
        end

        V_current = v_ref;
        omega_current = omega_ref;

        t_i = toc;
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i;
        t_count = t_count + 1;
    
        % Simple kinematic mobile robot model
        % Omitted dynamics.
        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta));
        x = x + v_ref*cos(theta) * dT;
        y = y + v_ref*sin(theta) * dT;
    
        % Archive and plot it
        t = t + 1;
        DWA.X(t) = x;
        DWA.Y(t) = y;
        DWA.Theta(t) = theta;
    end

    DWA.t = t;
    DWA.travelTime = (t-1)*dT;
    DWA.MeanCalculationTime = t_sum/t_count;
    DWA.MaxCalculationTime = t_max;
    DWA.MinCalculationTime = t_min;
end



function VAPF = VortexArtificialPotentialField(x, y, theta, x_goal, y_goal, position_accuracy, obstacle, dT, simTimeMax)
    
    % SAPF parameters
    VAPF.zeta = 1.1547;
    VAPF.eta = 0.0732;
    VAPF.dstar = 0.3;
    VAPF.Qstar = 1.0;

    % Parameters related to kinematic model
    VAPF.error_theta_max = deg2rad(45);
    VAPF.v_max = 0.2;
    VAPF.Kp_omega = 1.5;
    VAPF.omega_max = 0.5*pi; 

    t = 1;
    VAPF.X = zeros(1,simTimeMax);
    VAPF.Y = zeros(1,simTimeMax);
    VAPF.Theta = zeros(1,simTimeMax);
    VAPF.X(1) = x;
    VAPF.Y(1) = y;
    VAPF.Theta(1) = theta;


    t_max = -inf;
    t_min = inf;
    t_count = 0;
    t_sum = 0;

    Obstacle_count = size(obstacle,1);
    
    isVortex = false;
    lastSign = -1;

    while norm([x_goal y_goal] - [x y]) > position_accuracy && t < simTimeMax  
        tic;
    
        % Calculate Attractive Potential
        if norm([x y]-[x_goal y_goal]) <= VAPF.dstar
            nablaU_att =  VAPF.zeta*([x y]-[x_goal y_goal]);
        else 
            nablaU_att = VAPF.dstar/norm([x y]-[x_goal y_goal]) * VAPF.zeta*([x y]-[x_goal y_goal]);
        end
    
        % Find the minimum distance from the obstacle & Calculate Repulsive Potential
        obst_idx = zeros(1,Obstacle_count);
        obst_dist = zeros(1,Obstacle_count);
        nablaU_obst = [0 0];
        for i=1:Obstacle_count
            [obst_idx(i), obst_dist(i)] = dsearchn([obstacle(i,:,1)' obstacle(i,:,2)'], [x y]);
            obst_dist(i) = obst_dist(i) + 0.01*(2*rand()-1); % MEASUREMENT NOISE
            alpha = theta - atan2(obstacle(i,obst_idx(i),2)-y, obstacle(i,obst_idx(i),1)-x);
            alpha = atan2(sin(alpha), cos(alpha));
            if obst_dist(i) <= VAPF.Qstar &&  abs(alpha) < deg2rad(150)
                nablaU_rep_Oi = (VAPF.eta*(1/VAPF.Qstar - 1/obst_dist(i)) * 1/obst_dist(i)^2)*([x y] - [obstacle(i,obst_idx(i),1) obstacle(i,obst_idx(i),2)]);
                
                if isVortex == false 
                    if t > 10
                        if norm( [VAPF.X(t-10) VAPF.Y(t-10)] - [x y]) < 0.02
                            lastSign = lastSign*-1;
                            isVortex = true;
                        end
                    end
                else
                    goalTheta = atan2(y_goal-y, x_goal-x) - theta;
                    if abs(goalTheta) < deg2rad(10)
                        isVortex = false;
                    end
                end

                gamma = 0;
                if isVortex, gamma = lastSign*pi/2; end

                R = [cos(gamma) -sin(gamma)
                     sin(gamma)  cos(gamma) ];

                nablaU_obst = nablaU_obst + (R*nablaU_rep_Oi')';
            end
        end
        
        % Calculate final potential
        nablaU = nablaU_att+nablaU_obst;
    
        % Calculate reference value of linear velocity (v_ref) and orientation (theta_ref)
        theta_ref = atan2(-nablaU(2), -nablaU(1));
    
        error_theta = theta_ref - theta;
        if abs(error_theta) <= VAPF.error_theta_max
            alpha = (VAPF.error_theta_max - abs(error_theta)) / VAPF.error_theta_max;
            v_ref = min( alpha*norm(-nablaU), VAPF.v_max );
        else
            v_ref = 0;
        end
    
        t_i = toc;
        t_max = max(t_max, t_i);
        t_min = min(t_min, t_i);
        t_sum = t_sum + t_i;
        t_count = t_count + 1;
    
        % Simple kinematic mobile robot model
        % Omitted dynamics.
        omega_ref = VAPF.Kp_omega * error_theta;
        omega_ref = min( max(omega_ref, -VAPF.omega_max), VAPF.omega_max);
        theta = theta + omega_ref * dT;
        theta = atan2(sin(theta), cos(theta));
        x = x + v_ref*cos(theta) * dT;
        y = y + v_ref*sin(theta) * dT;
    
        % Archive and plot it
        t = t + 1;
        VAPF.X(t) = x;
        VAPF.Y(t) = y;
        VAPF.Theta(t) = theta;
    end

    VAPF.t = t;
    VAPF.travelTime = (t-1)*dT;
    VAPF.MeanCalculationTime = t_sum/t_count;
    VAPF.MaxCalculationTime = t_max;
    VAPF.MinCalculationTime = t_min;
end
