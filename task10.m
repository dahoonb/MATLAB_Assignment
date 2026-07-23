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
CE = norm(E - C);
DE = norm(E - D);
EF = norm(F - E);
FG = norm(F - G);

%% Initial input link angle
initialAngle = atan2(B(2) - A(2), B(1) - A(1));

if initialAngle < 0
    initialAngle = initialAngle + 2*pi;
end

%% Preallocate arrays
numberOfSteps = 360;
numberOfPositions = numberOfSteps + 1;
inputAngleDegrees = 0:numberOfSteps;

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

% Rows correspond to joints A through G, columns correspond to input angles.
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
    E_intersections = circleCircleIntersectionEquation(C_new, CE, D, DE);

    if isempty(E_intersections)
        fprintf('The new position of joint E cannot be determined at angle %d degrees.\n', theta);
        break;
    end

    E_new = chooseClosestIntersection(E_intersections, E);

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
    B = B_new;
    C = C_new;
    E = E_new;
    F = F_new;
end

%% Truncate unused preallocated entries
inputAngleDegrees = inputAngleDegrees(1:lastIdx);

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

hA = plot(A(1), A(2), 'o', 'LineStyle', 'none', 'MarkerSize', 10, 'MarkerEdgeColor', [0.60, 0.30, 0.30]);

hB_original = plot(B_original(1), B_original(2), 'o', 'LineStyle', 'none', 'MarkerSize', 10, ...
    'MarkerEdgeColor', [0.85, 0.95, 0.65]);

hC = plot(C_joint_x, C_joint_y, 'r-', 'LineWidth', 2);
hE = plot(E_joint_x, E_joint_y, '--', 'Color', [0.65, 1.00, 0.25], 'LineWidth', 2);
hF = plot(F_joint_x, F_joint_y, 'm:', 'LineWidth', 2);
hH = plot(H_joint_x, H_joint_y, 'k-.', 'LineWidth', 2);

title('Trajectory of Moving Joints and Artifact Point H');
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');

legend([hB, hA, hB_original, hC, hE, hF, hH], {'Trajectory of Joint B', 'Point A', 'Original Point B', ...
    'Trajectory of Joint C', 'Trajectory of Joint E', 'Trajectory of Joint F', 'Trajectory of Point H'}, ...
    'Location', 'northeast');

axis equal;
grid on;
box on;
hold off;

%% Plot joint force magnitudes
jointNames = {'A','B','C','D','E','F','G'};
jointForceMagnitude = sqrt(jointForceX.^2 + jointForceY.^2);

figure();
plot(inputAngleDegrees, jointForceMagnitude.', 'LineWidth', 1.5);
title('Joint-Force Magnitudes versus Input Angle');
xlabel('Input Rotation from Initial Position (degrees)');
ylabel('Force Magnitude (N)');
legend(jointNames, 'Location', 'best');
grid on;
box on;

%% Plot input torque
figure();
plot(inputAngleDegrees, inputTorque, 'LineWidth', 2);
title('Input Torque at Joint A versus Input Angle');
xlabel('Input Rotation from Initial Position (degrees)');
ylabel('Input Torque (N*m)');
grid on;
box on;

%% Import PMKS+ data
kinematicsData = readtable('kinematics_loops7Joints6Links22-Jul 21_13_07.xlsx', 'VariableNamingRule', 'preserve');
staticsData = readtable('statics7Joints6Links13-Jul 15_27_32.xlsx', 'VariableNamingRule', 'preserve');

% The kinematics export contains the input angle, but its joint-position
% columns are NaN. The statics export contains the valid joint positions.
pmksInputAngle = double(kinematicsData.("Link AB angle degree"));
pmksInputAngle = rad2deg(unwrap(deg2rad(pmksInputAngle)));
pmksInputAngle = pmksInputAngle - pmksInputAngle(1);
numberOfComparisonPositions = length(pmksInputAngle);

%% Compare positions of joints C, E, and F
pmks_C_x = double(staticsData.("Joint C x (m)"));
pmks_C_y = double(staticsData.("Joint C y (m)"));
pmks_D_x = double(staticsData.("Joint D x (m)"));
pmks_D_y = double(staticsData.("Joint D y (m)"));
pmks_E_x = double(staticsData.("Joint E x (m)"));
pmks_E_y = double(staticsData.("Joint E y (m)"));
pmks_F_x = double(staticsData.("Joint F x (m)"));
pmks_F_y = double(staticsData.("Joint F y (m)"));
pmks_G_x = double(staticsData.("Joint G x (m)"));
pmks_G_y = double(staticsData.("Joint G y (m)"));

% Compare the distance from each moving joint to a connected ground joint
pmksDistanceC = hypot(pmks_C_x - pmks_D_x, pmks_C_y - pmks_D_y);
matlabDistanceC = hypot(C_joint_x - D(1), C_joint_y - D(2)).';

pmksDistanceE = hypot(pmks_E_x - pmks_D_x, pmks_E_y - pmks_D_y);
matlabDistanceE = hypot(E_joint_x - D(1), E_joint_y - D(2)).';

pmksDistanceF = hypot(pmks_F_x - pmks_G_x, pmks_F_y - pmks_G_y);
matlabDistanceF = hypot(F_joint_x - G(1), F_joint_y - G(2)).';

positionRMSE = [
    sqrt(mean((matlabDistanceC(1:numberOfComparisonPositions) - pmksDistanceC).^2));
    sqrt(mean((matlabDistanceE(1:numberOfComparisonPositions) - pmksDistanceE).^2));
    sqrt(mean((matlabDistanceF(1:numberOfComparisonPositions) - pmksDistanceF).^2))
];

%% Compare force magnitudes at joints A through G
pmksForceX = [
    double(staticsData.("Joint A Force x (N)")), ...
    double(staticsData.("Joint B Force x (N)")), ...
    double(staticsData.("Joint C Force x (N)")), ...
    double(staticsData.("Joint D Force x (N)")), ...
    double(staticsData.("Joint E Force x (N)")), ...
    double(staticsData.("Joint F Force x (N)")), ...
    double(staticsData.("Joint G Force x (N)"))
];

pmksForceY = [
    double(staticsData.("Joint A Force y (N)")), ...
    double(staticsData.("Joint B Force y (N)")), ...
    double(staticsData.("Joint C Force y (N)")), ...
    double(staticsData.("Joint D Force y (N)")), ...
    double(staticsData.("Joint E Force y (N)")), ...
    double(staticsData.("Joint F Force y (N)")), ...
    double(staticsData.("Joint G Force y (N)"))
];

pmksForceMagnitude = hypot(pmksForceX, pmksForceY);
matlabForceMagnitude = jointForceMagnitude(:,1:numberOfComparisonPositions).';
forceMagnitudeRMSE = zeros(7,1);

for jointIndex = 1:7
    forceMagnitudeRMSE(jointIndex) = sqrt(mean((matlabForceMagnitude(:,jointIndex) - pmksForceMagnitude(:,jointIndex)).^2));
end

%% Compare input torque
% PMKS+ uses the opposite torque sign convention from MATLAB model
pmksTorque = -double(staticsData.("Torque N*m"));
matlabTorque = inputTorque(1:numberOfComparisonPositions).';
torqueRMSE = sqrt(mean((matlabTorque - pmksTorque).^2));

%% Print PMKS+ comparison results
fprintf('\nPOSITION RMSE USING LINK-DISTANCE COMPARISON\n');
fprintf('Joint C position, using distance CD: %.8e m\n', positionRMSE(1));
fprintf('Joint E position, using distance DE: %.8e m\n', positionRMSE(2));
fprintf('Joint F position, using distance FG: %.8e m\n', positionRMSE(3));

fprintf('\nJOINT-FORCE MAGNITUDE RMSE\n');
for jointIndex = 1:7
    fprintf('Joint %s: %.8e N\n', jointNames{jointIndex}, forceMagnitudeRMSE(jointIndex));
end

fprintf('\nINPUT-TORQUE RMSE\n');
fprintf('Joint A input torque: %.8e N*m\n', torqueRMSE);
fprintf('The PMKS+ torque sign was adjusted to match MATLAB.\n');

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
