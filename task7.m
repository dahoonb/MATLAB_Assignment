clear;
clc;

A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

S1 = (A + B) / 2;
S2 = (B + C + E) / 3;
S3 = (C + D) / 2;
S4 = (E + F) / 2;
S5 = (F + G) / 2;

syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

FA = [FAx FAy 0];
FB = [FBx FBy 0];
FC = [FCx FCy 0];
FD = [FDx FDy 0];
FE = [FEx FEy 0];
FF = [FFx FFy 0];
FG = [FGx FGy 0];
Torque = [0 0 Tin];

Mass_AB = 0;
Mass_BEC = 0;
Mass_CD = 0;
Mass_EF = 0;
Mass_FG = 0;

Force_Input = [50 0 0];

% Link AB

weight_AB = [0 -Mass_AB*9.81 0];

eqn1 = FA + FB + weight_AB == 0;

eqn2 = cross(A - S1, FA) + cross(B - S1, FB) + Torque == 0;

% Member BEC

weight_BEC = [0 -Mass_BEC*9.81 0];

eqn3 = -FB + FC + FE + weight_BEC == 0;

eqn4 = cross(B - S2, -FB) + cross(C - S2, FC) + cross(E - S2, FE) == 0;

% Link CD

weight_CD = [0 -Mass_CD*9.81 0];

eqn5 = -FC + FD + weight_CD == 0;

eqn6 = cross(C - S3, -FC) + cross(D - S3, FD) == 0;

% Link EF

weight_EF = [0 -Mass_EF*9.81 0];

eqn7 = -FE + FF + weight_EF == 0;

eqn8 = cross(E - S4, -FE) + cross(F - S4, FF) == 0;

% Link FG

weight_FG = [0 -Mass_FG*9.81 0];

eqn9 = -FF + FG + Force_Input + weight_FG == 0;

eqn10 = cross(F - S5, -FF) + cross(G - S5, FG) == 0;

% Equations

eqns = [eqn1; eqn2; eqn3; eqn4; eqn5; eqn6; eqn7; eqn8; eqn9; eqn10];

unknowns = [FAx; FAy; FBx; FBy; FCx; FCy; FDx; FDy; FEx; FEy; FFx; FFy; FGx; FGy; Tin];

solution = solve(eqns, unknowns);

FA_result = double([solution.FAx, solution.FAy, 0]);
FB_result = double([solution.FBx, solution.FBy, 0]);
FC_result = double([solution.FCx, solution.FCy, 0]);
FD_result = double([solution.FDx, solution.FDy, 0]);
FE_result = double([solution.FEx, solution.FEy, 0]);
FF_result = double([solution.FFx, solution.FFy, 0]);
FG_result = double([solution.FGx, solution.FGy, 0]);
StaticTorque = double(solution.Tin);

fprintf('Reaction Forces (N):\n');
fprintf('FA = [%.4f, %.4f, %.4f]\n', FA_result);
fprintf('FB = [%.4f, %.4f, %.4f]\n', FB_result);
fprintf('FC = [%.4f, %.4f, %.4f]\n', FC_result);
fprintf('FD = [%.4f, %.4f, %.4f]\n', FD_result);
fprintf('FE = [%.4f, %.4f, %.4f]\n', FE_result);
fprintf('FF = [%.4f, %.4f, %.4f]\n', FF_result);
fprintf('FG = [%.4f, %.4f, %.4f]\n', FG_result);

fprintf('Static Torque (N*m): %.4f\n', StaticTorque);
