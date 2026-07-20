clear;
clc;
close all;

%% Initial joint coordinates
A = [ 1.400, 0.485, 0];
B = [ 1.670, 0.990, 0];
C = [ 0.255, 1.035, 0];
D = [ 0.285, 0.055, 0];
E = [ 0.195, 2.540, 0];
F = [-0.980, 2.570, 0];
G = [ 0.050, 0.200, 0];

B_original = B;

%% Constant link lengths
AB = norm(B - A);
BC = norm(C - B);
CD = norm(C - D);
DE = norm(E - D);
EF = norm(F - E);
FG = norm(F - G);

%% Initial angle of link AB
initialAngle = atan2(B(2) - A(2), B(1) - A(1));

if initialAngle < 0
    initialAngle = initialAngle + 2*pi;
end

%% Preallocate trajectory arrays
numberOfSteps = 360;

B_joint_x = zeros(1, numberOfSteps + 1);
B_joint_y = zeros(1, numberOfSteps + 1);

%% Store the initial position of B
B_joint_x(1) = B(1);
B_joint_y(1) = B(2);

%% Compute the trajectory of B
for theta = 1:numberOfSteps
    currentAngle = initialAngle + deg2rad(theta);

    B_new = [A(1) + AB*cos(currentAngle), A(2) + AB*sin(currentAngle), 0];

    index = theta + 1;

    B_joint_x(index) = B_new(1);
    B_joint_y(index) = B_new(2);
end

%% Plot the trajectory of B
figure();

hB = plot(B_joint_x, B_joint_y, 'b-', 'LineWidth', 2);

hold on;

hA = plot(A(1), A(2), 'o', 'LineStyle', 'none', 'MarkerSize', 10, ...
    'MarkerEdgeColor', [0.60, 0.30, 0.30]);

hB_original = plot(B_original(1), B_original(2), 'o', 'LineStyle', 'none', ...
    'MarkerSize', 10, 'MarkerEdgeColor', [0.85, 0.95, 0.65]);

title('Trajectory of Joint B');
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');

legend([hB, hA, hB_original], {'Trajectory of Joint B', 'Point A', ...
    'Original Point B'}, 'Location', 'northeast');

xlim([0.5, 2.1]);
ylim([-0.2, 1.2]);

axis equal;
grid on;
box on;
hold off;