clear;
clc;
close all;

M = 8; % Mass (kg)
D = 50; % Damping coefficient (N*s/m)
K = 100; % Spring stiffness (N/m)

% X(1) = displacement, X(2) = momentum
odeSystem = @(t, X) [X(2)/M; -(D/M)*X(2) - K*X(1)];

v0 = 10; % Initial velocity (m/s)

tspan = [0 10];
initialConditions = [0; M*v0];

[t, X] = ode45(odeSystem, tspan, initialConditions);

% Plot
figure;

subplot(2, 1, 1);
plot(t, X(:, 1));
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs. Time');
legend({'Position'});
grid on;

subplot(2, 1, 2);
plot(t, X(:, 2));
xlabel('Time (s)');
ylabel('Momentum (kg*m/s)');
title('Momentum vs. Time');
legend('Momentum');
grid on;
