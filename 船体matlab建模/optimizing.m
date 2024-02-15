function U = optimizing(number) 
% number = 10;
persistent iteration_n;
persistent s ;
if isempty(s)
   [ship_opposite_translate_x , intersection_D_1_x , intersection_D_0_x , distance_R_1 , distance_R_2 , distance_DCPA , ship_origin_v , distance_D , distance_D_0 , distance_D_1 , v_opposite , m] = direction_modify(number);
   isFirstCall = false;
end
intersection_D_1_x_ =subs(intersection_D_1_x , 't' , number);
intersection_D_0_x_ =subs(intersection_D_0_x , 't' , number);
distance_R_1_ = subs(distance_R_1 , 't' , number);
distance_R_2_ = subs(distance_R_2 , 't' , number);
distance_DCPA_ = subs(distance_DCPA , 't' , number);
ship_origin_v_ = subs(ship_origin_v , 't' , number);
distance_D_ = subs(distance_D , 't' , number);
distance_D_0_ = subs(distance_D_0 , 't' , number);
distance_D_1_ = subs(distance_D_1 , 't' , number);
v_opposite_ = subs(v_opposite , 't' , number);
m_ = subs(m , 't' , number);
ship_opposite_translate_x_ = subs(ship_opposite_translate_x , 't' , number);


nq = 1;
ns = 1;
if intersection_D_1_x_ == []
    nq = 0;
elseif intersection_D_0_x_ == []
    ns = 0;
end


if nq > 0 && ns > 0
    u_DCPA = 1;
elseif distance_R_1_ < distance_DCPA_ && distance_DCPA_ <= distance_R_2_ && nq == 0
    u_DCPA = 1/2 - 1/2 * sin(180 / (distance_R_2_ - distance_R_1_) * (distance_DCPA_ - (distance_R_2_ + distance_R_1_) / 2));
elseif distance_DCPA_ > distance_R_2_ || ns == 0
    u_DCPA = 0;
end


TCPA = sqrt(distance_D_1_^2 - distance_DCPA_^2)/ship_origin_v_;
T0 = sqrt(distance_D_1_^2 - distance_DCPA_^2)/ship_origin_v_;
if ns > 0
    u_TCPA = exp(-log(2) * (abs(TCPA) / T0^2));
elseif ns == 0
    u_TCPA = 0;
end


if ns > 0
    u_D = exp(-log(2) * ((distance_D_ - distance_D_0_) / (distance_D_1_ - distance_D_0_))^2);
elseif ns == 0
    u_D = 0;
end


angle_rad = atan2(m_, 1); 
angle_deg_x = rad2deg(angle_rad);
if angle_deg_x < 0
    angle_deg_x = angle_deg_x + 180;
    angle_deg_y = 270 - angle_deg_x0;
else 
    angle_deg_x = angle_deg_x;
    angle_deg_y = 90 - angle_deg_x;
end
if ship_opposite_translate_x_<0
    angle_C = - angle_deg_y;
else 
    angle_C = angle_deg_y;
end


if ns > 0
    u_C = (17/44) * cos(angle_C + 161) + sqrt(cos(angle_C + 161)^2) + 440/289;
elseif ns == 0
    u_C = 0;
end


K = ship_origin_v_/v_opposite_;
if ns > 0
    u_K = 1 / (1 + 2 / (K * sqrt(K^2 + 1 + 2 * K * abs(sin(abs(angle_C))))));
elseif ns == 0
    u_K = 0;
end
U = 0.36*u_DCPA+0.32*u_TCPA+0.14*u_D+0.1*u_C+0.08*u_K;
% [ optimised_parameters ] = Particle_Swarm_Optimization (Bird_in_swarm, Number_of_quality_in_Bird, MinMaxRange, U, availability_type, 2, 2, 2, 0.4, 0.9, max_iteration);
iteration_n = iteration_n+1;
disp(iteration_n);
end