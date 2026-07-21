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

%% Initial angles
initialAngle = atan2(B(2) - A(2), B(1) - A(1));

if initialAngle < 0
    initialAngle = initialAngle + 2*pi;
end

% C, D, and E are on the same rigid ternary link
initialDCAngle = atan2(C(2) - D(2), C(1) - D(1));
initialDEAngle = atan2(E(2) - D(2), E(1) - D(1));

CDEAngleOffset = initialDEAngle - initialDCAngle;

%% Preallocate trajectory arrays
numberOfSteps = 360;

B_joint_x = zeros(1, numberOfSteps + 1);
B_joint_y = zeros(1, numberOfSteps + 1);

C_joint_x = zeros(1, numberOfSteps + 1);
C_joint_y = zeros(1, numberOfSteps + 1);

E_joint_x = zeros(1, numberOfSteps + 1);
E_joint_y = zeros(1, numberOfSteps + 1);

F_joint_x = zeros(1, numberOfSteps + 1);
F_joint_y = zeros(1, numberOfSteps + 1);

%% Store initial joint positions
B_joint_x(1) = B(1);
B_joint_y(1) = B(2);

C_joint_x(1) = C(1);
C_joint_y(1) = C(2);

E_joint_x(1) = E(1);
E_joint_y(1) = E(2);

F_joint_x(1) = F(1);
F_joint_y(1) = F(2);

lastIdx = 1;

%% Position analysis
for theta = 1:numberOfSteps

    fprintf('theta = %d degrees\n', theta);

    %% Compute the new position of B
    currentAngle = initialAngle + deg2rad(theta);
    B_new = [A(1) + AB*cos(currentAngle), A(2) + AB*sin(currentAngle), 0];

    %% Compute the new position of C
    C_intersections = circleCircleIntersectionEquation(B_new, BC, D, CD);

    if isempty(C_intersections)
        fprintf(['The new position of joint C cannot be determined at angle %d degrees.\n'], theta);
        break;
    end

    C_new = chooseClosestIntersection(C_intersections, C);

    %% Compute the new position of E
    % C, D, and E are on the same rigid link.
    currentDCAngle = atan2(C_new(2) - D(2), C_new(1) - D(1));
    currentDEAngle = currentDCAngle + CDEAngleOffset;

    E_new = [D(1) + DE*cos(currentDEAngle), D(2) + DE*sin(currentDEAngle), 0];

    %% Compute the new position of F
    F_intersections = circleCircleIntersectionEquation(E_new, EF, G, FG);

    if isempty(F_intersections)
        fprintf(['The new position of joint F cannot be ' ...
                 'determined at angle %d degrees.\n'], theta);
        break;
    end

    F_new = chooseClosestIntersection(F_intersections, F);

    %% Save new positions
    index = theta + 1;
    
    B_joint_x(index) = B_new(1);
    B_joint_y(index) = B_new(2);

    C_joint_x(index) = C_new(1);
    C_joint_y(index) = C_new(2);

    E_joint_x(index) = E_new(1);
    E_joint_y(index) = E_new(2);

    F_joint_x(index) = F_new(1);
    F_joint_y(index) = F_new(2);

    lastIdx = index;

    %% Update the previous joint positions
    B = B_new;
    C = C_new;
    E = E_new;
    F = F_new;
end

%% Truncate unused preallocated entries
B_joint_x = B_joint_x(1:lastIdx);
B_joint_y = B_joint_y(1:lastIdx);

C_joint_x = C_joint_x(1:lastIdx);
C_joint_y = C_joint_y(1:lastIdx);

E_joint_x = E_joint_x(1:lastIdx);
E_joint_y = E_joint_y(1:lastIdx);

F_joint_x = F_joint_x(1:lastIdx);
F_joint_y = F_joint_y(1:lastIdx);

%% Plot trajectories
figure();

hB = plot(B_joint_x, B_joint_y, 'b-', 'LineWidth', 2);

hold on;

hA = plot(A(1), A(2), 'o', 'LineStyle', 'none', 'MarkerSize', 10, ...
    'MarkerEdgeColor', [0.60, 0.30, 0.30]);

hB_original = plot(B_original(1), B_original(2), 'o', 'LineStyle', 'none', ...
    'MarkerSize', 10, 'MarkerEdgeColor', [0.85, 0.95, 0.65]);

hC = plot(C_joint_x, C_joint_y, 'r-', 'LineWidth', 2);
hE = plot(E_joint_x, E_joint_y, '--', 'Color', [0.65, 1.00, 0.25], 'LineWidth', 2);
hF = plot(F_joint_x, F_joint_y, 'm:', 'LineWidth', 2);

title('Trajectory of All Joints');
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');

legend([hB, hA, hB_original, hC, hE, hF], {'Trajectory of Joint B', 'Point A', ...
     'Original Point B', 'Trajectory of Joint C', 'Trajectory of Joint E', ...
     'Trajectory of Joint F'}, 'Location', 'northeast');

xlim([-3, 2.5]);
ylim([-0.5, 3]);

axis equal;
grid on;
box on;
hold off;

function intersectionPoints = circleCircleIntersectionEquation(center1, radius1, center2, radius2)
    intersectionPoints = [];

    syms x y real

    equation1 = (x - center1(1))^2 + (y - center1(2))^2 == radius1^2;
    equation2 = (x - center2(1))^2 + (y - center2(2))^2 == radius2^2;

    try
        solution = solve([equation1, equation2], [x, y]);

        xSolutions = double(solution.x);
        ySolutions = double(solution.y);
        xSolutions = xSolutions(:);
        ySolutions = ySolutions(:);

        if isempty(xSolutions) || isempty(ySolutions)
            return;
        end

        tolerance = 1e-10;
        realIdx = abs(imag(xSolutions)) < tolerance & abs(imag(ySolutions)) < tolerance;

        if ~any(realIdx)
            return;
        end

        xSolutions = real(xSolutions(realIdx));
        ySolutions = real(ySolutions(realIdx));
        intersectionPoints = [xSolutions, ySolutions];

    catch
        intersectionPoints = [];
    end
end

function selectedPoint = chooseClosestIntersection(intersectionPoints, previousPoint)
    numberOfPoints = size(intersectionPoints, 1);
    distances = zeros(numberOfPoints, 1);

    for i = 1:numberOfPoints
        distances(i) = pointDistance(intersectionPoints(i,:), previousPoint);
    end

    [~, closestIndex] = min(distances);

    selectedPoint = [intersectionPoints(closestIndex,1), intersectionPoints(closestIndex,2), 0];
end

function distance = pointDistance(point1, point2)
    distance = sqrt((point1(1) - point2(1))^2 + (point1(2) - point2(2))^2);
end
