clc, clear
u_init  = 1.179;
v_init  = 0;
r_init  = 0;
y0      = [u_init; v_init; r_init];
sigma   = 35;  
np      = 11.086*60;     
tspan   = [0, 1000]; 
[t, y]  = ode15s(@(t, y) myODEs(t, y, sigma, np), tspan, y0);

figure;
plot(t, y(:, 1), 'r', t, y(:, 2), 'g', t, y(:, 3), 'b', 'LineWidth', 2);
legend('u', 'v', 'r');
xlabel('Time');
ylabel('Values');
title('Numerical Solution using ode15s with Smaller Time Steps');
