clear;
clc;
close all;

M = 8; % Mass (kg)
D = 50; % Damping coefficient (N*s/m)
K = 100; % Spring stiffness (N/m)

% Define displacement as symbolic function
syms x(t)

% Differential equation
Dx = diff(x, t);
eqn = M*diff(x, t, 2) + D*diff(x, t) + K*x == 0;

% Initial conditions
initialConditions = [x(0) == 0; Dx(0) == 10];

% Solve for displacement, velocity, and acceleration
xSolution = dsolve(eqn, initialConditions);
vSolution = diff(xSolution, t);
aSolution = diff(vSolution, t);

% Plot
figure;

subplot(2, 2, 1);
fplot(xSolution,[0 10]);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Displacement x(t)');
grid on;

subplot(2, 2, 2);
fplot(vSolution,[0 10]);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity v(t)');
grid on;

subplot(2, 2, 3);
fplot(aSolution,[0 10]);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration a(t)');
grid on;
