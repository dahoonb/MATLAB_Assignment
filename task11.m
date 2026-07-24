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

%% Constant link lengths
AB = norm(B - A);
BC = norm(C - B);
CD = norm(C - D);
DE = norm(E - D);
EF = norm(F - E);
FG = norm(F - G);

%% Initial linkage angles
initialAngle = atan2(B(2) - A(2), B(1) - A(1));

if initialAngle < 0
    initialAngle = initialAngle + 2*pi;
end

% Preserve the fixed orientation of rigid link CDE
initialDCAngle = atan2(C(2) - D(2), C(1) - D(1));
initialDEAngle = atan2(E(2) - D(2), E(1) - D(1));
CDEAngleOffset = initialDEAngle - initialDCAngle;

%% Preallocate arrays
numberOfSteps = 360;
numberOfPositions = numberOfSteps + 1;
B_joint_x = zeros(1, numberOfPositions);
B_joint_y = zeros(1, numberOfPositions);
C_joint_x = zeros(1, numberOfPositions);
C_joint_y = zeros(1, numberOfPositions);
E_joint_x = zeros(1, numberOfPositions);
E_joint_y = zeros(1, numberOfPositions);
F_joint_x = zeros(1, numberOfPositions);
F_joint_y = zeros(1, numberOfPositions);
H_joint_x = zeros(1, numberOfPositions);
H_joint_y = zeros(1, numberOfPositions);

% Rows correspond to joints A through G, columns correspond to input angles
jointForceX = zeros(7, numberOfPositions);
jointForceY = zeros(7, numberOfPositions);
inputTorque = zeros(1, numberOfPositions);

%% Store initial position and do initial static analysis
B_joint_x(1) = B(1);
B_joint_y(1) = B(2);
C_joint_x(1) = C(1);
C_joint_y(1) = C(2);
E_joint_x(1) = E(1);
E_joint_y(1) = E(2);
F_joint_x(1) = F(1);
F_joint_y(1) = F(2);

[initialForces, inputTorque(1), H] = staticAnalysis(A, B, C, D, E, F, G);
H_joint_x(1) = H(1);
H_joint_y(1) = H(2);
jointForceX(:,1) = initialForces(:,1);
jointForceY(:,1) = initialForces(:,2);

lastIdx = 1;

%% Position and static analysis for one complete input rotation
for theta = 1:numberOfSteps
    fprintf('theta = %d degrees\n', theta);

    %% Compute the new position of B
    currentAngle = initialAngle + deg2rad(theta);
    B_new = [A(1) + AB*cos(currentAngle), A(2) + AB*sin(currentAngle), 0];

    %% Compute the new position of C
    C_intersections = circleCircleIntersectionEquation(B_new, BC, D, CD);

    if isempty(C_intersections)
        fprintf('The new position of joint C cannot be determined at angle %d degrees.\n', theta);
        break;
    end

    C_new = chooseClosestIntersection(C_intersections, C);

    %% Compute the new position of E
    currentDCAngle = atan2(C_new(2) - D(2), C_new(1) - D(1));
    currentDEAngle = currentDCAngle + CDEAngleOffset;
    E_new = [D(1) + DE*cos(currentDEAngle), ...
             D(2) + DE*sin(currentDEAngle), 0];

    %% Compute the new position of F
    F_intersections = circleCircleIntersectionEquation(E_new, EF, G, FG);

    if isempty(F_intersections)
        fprintf('The new position of joint F cannot be determined at angle %d degrees.\n', theta);
        break;
    end

    F_new = chooseClosestIntersection(F_intersections, F);

    %% Static analysis at the new position
    [currentForces, currentTorque, H_new] = staticAnalysis(A, B_new, C_new, D, E_new, F_new, G);

    %% Save new positions, forces, and torque
    index = theta + 1;

    B_joint_x(index) = B_new(1);
    B_joint_y(index) = B_new(2);
    C_joint_x(index) = C_new(1);
    C_joint_y(index) = C_new(2);
    E_joint_x(index) = E_new(1);
    E_joint_y(index) = E_new(2);
    F_joint_x(index) = F_new(1);
    F_joint_y(index) = F_new(2);
    H_joint_x(index) = H_new(1);
    H_joint_y(index) = H_new(2);

    jointForceX(:,index) = currentForces(:,1);
    jointForceY(:,index) = currentForces(:,2);
    inputTorque(index) = currentTorque;

    lastIdx = index;

    %% Update previous positions for branch selection
    C = C_new;
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
H_joint_x = H_joint_x(1:lastIdx);
H_joint_y = H_joint_y(1:lastIdx);

jointForceX = jointForceX(:,1:lastIdx);
jointForceY = jointForceY(:,1:lastIdx);
inputTorque = inputTorque(1:lastIdx);

%% Plot joint trajectories
figure();

hB = plot(B_joint_x, B_joint_y, 'b-', 'LineWidth', 2);
hold on;
hC = plot(C_joint_x, C_joint_y, 'r-', 'LineWidth', 2);
hE = plot(E_joint_x, E_joint_y, '--', 'Color', [0.65, 1.00, 0.25], 'LineWidth', 2);
hF = plot(F_joint_x, F_joint_y, 'm:', 'LineWidth', 2);

plot(A(1), A(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7, 'HandleVisibility', 'off');

title('Trajectories of Joints B, C, E, and F');
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
legend([hB, hC, hE, hF], {'Joint B', 'Joint C', 'Joint E', 'Joint F'}, 'Location', 'northeast');
axis equal;
grid on;
box on;
hold off;

%% Animate the six bar linkage
figure();
hold on;

% Initial linkage lines
hAB = plot([A(1), B_joint_x(1)], [A(2), B_joint_y(1)], 'k-', 'LineWidth', 2);
hBC = plot([B_joint_x(1), C_joint_x(1)], [B_joint_y(1), C_joint_y(1)], 'k-', 'LineWidth', 2);
hCD = plot([C_joint_x(1), D(1)], [C_joint_y(1), D(2)], 'k-', 'LineWidth', 2);
hCE = plot([C_joint_x(1), E_joint_x(1)], [C_joint_y(1), E_joint_y(1)], 'k-', 'LineWidth', 2);
hDE = plot([D(1), E_joint_x(1)], [D(2), E_joint_y(1)], 'k-', 'LineWidth', 2);
hEF = plot([E_joint_x(1), F_joint_x(1)], [E_joint_y(1), F_joint_y(1)], 'k-', 'LineWidth', 2);
hFG = plot([F_joint_x(1), G(1)], [F_joint_y(1), G(2)], 'k-', 'LineWidth', 2);

% Joint markers
plot([A(1), D(1), G(1)], [A(2), D(2), G(2)], 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
hMovingJoints = plot([B_joint_x(1), C_joint_x(1), E_joint_x(1), F_joint_x(1)], ...
    [B_joint_y(1), C_joint_y(1), E_joint_y(1), F_joint_y(1)], 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 7);

% Animated trajectories
animatedB = animatedline('Color', 'b', 'LineWidth', 1.5);
animatedC = animatedline('Color', 'r', 'LineWidth', 1.5);
animatedE = animatedline('Color', [0.45, 0.70, 0.10], 'LineStyle', '--', 'LineWidth', 1.5);
animatedF = animatedline('Color', 'm', 'LineStyle', ':', 'LineWidth', 1.5);

allX = [A(1), D(1), G(1), B_joint_x, C_joint_x, E_joint_x, F_joint_x];
allY = [A(2), D(2), G(2), B_joint_y, C_joint_y, E_joint_y, F_joint_y];
xlim([min(allX) - 0.15, max(allX) + 0.15]);
ylim([min(allY) - 0.15, max(allY) + 0.15]);
axis equal;
grid on;
box on;
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
title('Pick-and-Place Six-Bar Linkage Animation');
legend([animatedB, animatedC, animatedE, animatedF], {'Joint B trajectory', 'Joint C trajectory', ...
    'Joint E trajectory', 'Joint F trajectory'}, 'Location', 'eastoutside');

for frameIndex = 1:lastIdx
    set(hAB, 'XData', [A(1), B_joint_x(frameIndex)], 'YData', [A(2), B_joint_y(frameIndex)]);
    set(hBC, 'XData', [B_joint_x(frameIndex), C_joint_x(frameIndex)], 'YData', [B_joint_y(frameIndex), C_joint_y(frameIndex)]);
    set(hCD, 'XData', [C_joint_x(frameIndex), D(1)], 'YData', [C_joint_y(frameIndex), D(2)]);
    set(hCE, 'XData', [C_joint_x(frameIndex), E_joint_x(frameIndex)], 'YData', [C_joint_y(frameIndex), E_joint_y(frameIndex)]);
    set(hDE, 'XData', [D(1), E_joint_x(frameIndex)], 'YData', [D(2), E_joint_y(frameIndex)]);
    set(hEF, 'XData', [E_joint_x(frameIndex), F_joint_x(frameIndex)], 'YData', [E_joint_y(frameIndex), F_joint_y(frameIndex)]);
    set(hFG, 'XData', [F_joint_x(frameIndex), G(1)], 'YData', [F_joint_y(frameIndex), G(2)]);

    set(hMovingJoints, 'XData', [B_joint_x(frameIndex), C_joint_x(frameIndex), E_joint_x(frameIndex), F_joint_x(frameIndex)], ...
        'YData', [B_joint_y(frameIndex), C_joint_y(frameIndex), E_joint_y(frameIndex), F_joint_y(frameIndex)]);

    addpoints(animatedB, B_joint_x(frameIndex), B_joint_y(frameIndex));
    addpoints(animatedC, C_joint_x(frameIndex), C_joint_y(frameIndex));
    addpoints(animatedE, E_joint_x(frameIndex), E_joint_y(frameIndex));
    addpoints(animatedF, F_joint_x(frameIndex), F_joint_y(frameIndex));

    drawnow;
    pause(0.05);
end

hold off;

%% Save positions, joint forces, and input torque to CSV file
trajectoryData = table(B_joint_x.', B_joint_y.', C_joint_x.', C_joint_y.', E_joint_x.', E_joint_y.', F_joint_x.', F_joint_y.', H_joint_x.', H_joint_y.', ...
    jointForceX(1,:).', jointForceY(1,:).', ...
    jointForceX(2,:).', jointForceY(2,:).', ...
    jointForceX(3,:).', jointForceY(3,:).', ...
    jointForceX(4,:).', jointForceY(4,:).', ...
    jointForceX(5,:).', jointForceY(5,:).', ...
    jointForceX(6,:).', jointForceY(6,:).', ...
    jointForceX(7,:).', jointForceY(7,:).', ...
    inputTorque.', ...
    'VariableNames', {'B_x_m', 'B_y_m', 'C_x_m', 'C_y_m', 'E_x_m', 'E_y_m', 'F_x_m', 'F_y_m', 'H_x_m', 'H_y_m', ...
    'A_force_x_N', 'A_force_y_N', ...
    'B_force_x_N', 'B_force_y_N', ...
    'C_force_x_N', 'C_force_y_N', ...
    'D_force_x_N', 'D_force_y_N', ...
    'E_force_x_N', 'E_force_y_N', ...
    'F_force_x_N', 'F_force_y_N', ...
    'G_force_x_N', 'G_force_y_N', ...
    'InputTorque_N_m'});

writetable(trajectoryData, 'trajectory_data.csv');

%% Local functions
function [jointForces, inputTorque, H] = staticAnalysis(A, B, C, D, E, F, G)
    % Artifact point and downward 100 N force
    H = F + 1.843*(F-G)/norm(F-G);

    % Unknown vector:
    % x = [FAx; FAy; FBx; FBy; FCx; FCy; FDx; FDy; FEx; FEy; FFx; FFy; FGx; FGy; Tin]
    M = zeros(15,15);
    b = zeros(15,1);

    %% Link AB
    M(1,[1 3]) = [1 1];
    M(2,[2 4]) = [1 1];

    rAB = B-A;
    M(3,[3 4 15]) = [-rAB(2) rAB(1) 1];

    %% Link BC
    M(4,[3 5]) = [-1 1];
    M(5,[4 6]) = [-1 1];

    rBC = C-B;
    M(6,[5 6]) = [-rBC(2) rBC(1)];

    %% Link CDE
    M(7,[5 7 9]) = [-1 1 1];
    M(8,[6 8 10]) = [-1 1 1];

    rCD = D-C;
    rCE = E-C;
    M(9,[7 8 9 10]) = [-rCD(2) rCD(1) -rCE(2) rCE(1)];

    %% Link EF
    M(10,[9 11]) = [-1 1];
    M(11,[10 12]) = [-1 1];

    rEF = F-E;
    M(12,[11 12]) = [-rEF(2) rEF(1)];

    %% Link FG, with the artifact force acting at H
    M(13,[11 13]) = [-1 1];
    M(14,[12 14]) = [-1 1];
    b(14) = 100;

    rGF = F-G;
    rGH = H-G;
    M(15,[11 12]) = [rGF(2) -rGF(1)];
    b(15) = 100*rGH(1);

    %% Solve equilibrium equations
    x = M\b;

    jointForces = [
        x(1)  x(2);
        x(3)  x(4);
        x(5)  x(6);
        x(7)  x(8);
        x(9)  x(10);
        x(11) x(12);
        x(13) x(14)
    ];

    inputTorque = x(15);
end

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
