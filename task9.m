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

syms x y real

%% Position analysis
for theta = 1:numberOfSteps

    fprintf('theta = %d degrees\n', theta);

    %% Compute the new position of B
    currentAngle = initialAngle + deg2rad(theta);
    B_new = [A(1) + AB*cos(currentAngle), A(2) + AB*sin(currentAngle), 0];

    %% Compute the new position of C
    equationC1 = (x - B_new(1))^2 + (y - B_new(2))^2 == BC^2;
    equationC2 = (x - D(1))^2 + (y - D(2))^2 == CD^2;

    solutionC = solve([equationC1, equationC2], [x, y]);

    xSolutionsC = double(solutionC.x);
    ySolutionsC = double(solutionC.y);
    xSolutionsC = xSolutionsC(:);
    ySolutionsC = ySolutionsC(:);

    tolerance = 1e-10;

    realIdxC = abs(imag(xSolutionsC)) < tolerance & abs(imag(ySolutionsC)) < tolerance;

    if ~any(realIdxC)
        fprintf(['The new position of joint C cannot be determined at angle %d degrees.\n'], theta);
        break;
    end

    xSolutionsC = real(xSolutionsC(realIdxC));
    ySolutionsC = real(ySolutionsC(realIdxC));

    C_intersections = [xSolutionsC, ySolutionsC];

    numberOfCPoints = size(C_intersections, 1);
    C_distances = zeros(numberOfCPoints, 1);

    for i = 1:numberOfCPoints
        C_distances(i) = sqrt((C_intersections(i,1) - C(1))^2 + (C_intersections(i,2) - C(2))^2);
    end

    [~, closestCIndex] = min(C_distances);

    C_new = [C_intersections(closestCIndex,1), C_intersections(closestCIndex,2), 0];

    %% Compute the new position of E
    % C, D, and E are on the same rigid link.
    currentDCAngle = atan2(C_new(2) - D(2), C_new(1) - D(1));
    currentDEAngle = currentDCAngle + CDEAngleOffset;

    E_new = [D(1) + DE*cos(currentDEAngle), D(2) + DE*sin(currentDEAngle), 0];

    %% Compute the new position of F
    equationF1 = (x - E_new(1))^2 + (y - E_new(2))^2 == EF^2;
    equationF2 = (x - G(1))^2 + (y - G(2))^2 == FG^2;

    solutionF = solve([equationF1, equationF2], [x, y]);
    xSolutionsF = double(solutionF.x);
    ySolutionsF = double(solutionF.y);
    xSolutionsF = xSolutionsF(:);
    ySolutionsF = ySolutionsF(:);

    realIdxF = abs(imag(xSolutionsF)) < tolerance & abs(imag(ySolutionsF)) < tolerance;

    if ~any(realIdxF)
        fprintf(['The new position of joint F cannot be ' ...
                 'determined at angle %d degrees.\n'], theta);
        break;
    end

    xSolutionsF = real(xSolutionsF(realIdxF));
    ySolutionsF = real(ySolutionsF(realIdxF));

    F_intersections = [xSolutionsF, ySolutionsF];

    numberOfFPoints = size(F_intersections, 1);
    F_distances = zeros(numberOfFPoints, 1);

    for i = 1:numberOfFPoints
        F_distances(i) = sqrt((F_intersections(i,1) - F(1))^2 + ...
            (F_intersections(i,2) - F(2))^2);
    end

    [~, closestFIndex] = min(F_distances);

    F_new = [F_intersections(closestFIndex,1), F_intersections(closestFIndex,2), 0];

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
