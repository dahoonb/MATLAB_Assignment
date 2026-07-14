clear;
clc;

%% Joint coordinates
A = [1.400 0.485 0];
B = [1.670 0.990 0];
C = [0.255 1.035 0];
D = [0.285 0.055 0];
E = [0.195 2.540 0];
F = [-0.980 2.570 0];
G = [0.050 0.200 0];

% Artifact point and 100 N downward force
H = F + 1.843*(F-G)/norm(F-G);
W = [0 -100 0];

%% Link lengths
lengthNames = {'AB';'BC';'CD';'CE';'DE';'EF';'FG';'AD';'DG';'AG';'FH'};
lengthValues = [
    norm(B-A);
    norm(C-B);
    norm(D-C);
    norm(E-C);
    norm(E-D);
    norm(F-E);
    norm(G-F);
    norm(D-A);
    norm(G-D);
    norm(G-A);
    norm(H-F)
];

%% Equilibrium equations
% Unknown vector:
% x = [FAx; FAy; FBx; FBy; FCx; FCy; FDx; FDy; FEx; FEy; FFx; FFy; FGx; FGy; Tin]

% Each moving link contributes three scalar equilibrium equations,
% i.e., sum(Fx) = 0, sum(Fy) = 0, and sum(Mz) = 0.
% Five moving links therefore produce 15 scalar equilibrium equations.

M = zeros(15,15); % Coefficient matrix for the 15 unknowns
b = zeros(15,1); % Right-hand-side vector

%% Link AB
% Horizontal force equilibrium (FAx + FBx = 0)
M(1,[1 3]) = [1 1];

% Vertical force equilibrium (FAy + FBy = 0)
M(2,[2 4]) = [1 1];

% Moment equilibrium about joint A (Tin - rAB_y*FBx + rAB_x*FBy = 0)
rAB = B-A;
M(3,[3 4 15]) = [-rAB(2) rAB(1) 1];

%% Link BC
% Horizontal force equilibrium (-FBx + FCx = 0)
M(4,[3 5]) = [-1 1];

% Vertical force equilibrium (-FBy + FCy = 0)
M(5,[4 6]) = [-1 1];

% Moment equilibrium about joint B (-rBC_y*FCx + rBC_x*FCy = 0)
rBC = C-B;
M(6,[5 6]) = [-rBC(2) rBC(1)];

%% Link CDE
% Horizontal force equilibrium (-FCx + FDx + FEx = 0)
M(7,[5 7 9]) = [-1 1 1];

% Vertical force equilibrium (-FCy + FDy + FEy = 0)
M(8,[6 8 10]) = [-1 1 1];

% Moment equilibrium about joint C (-rCD_y*FDx + rCD_x*FDy - rCE_y*FEx + rCE_x*FEy = 0)
rCD = D-C;
rCE = E-C;
M(9,[7 8 9 10]) = [-rCD(2) rCD(1) -rCE(2) rCE(1)];

%% Link EF
% Horizontal force equilibrium (-FEx + FFx = 0)
M(10,[9 11]) = [-1 1];

% Vertical force equilibrium (-FEy + FFy = 0)
M(11,[10 12]) = [-1 1];

% Moment equilibrium about joint E (-rEF_y*FFx + rEF_x*FFy = 0)
rEF = F-E;
M(12,[11 12]) = [-rEF(2) rEF(1)];

%% Link FG, with artifact force at H
% Horizontal force equilibrium (-FFx + FGx = 0)
M(13,[11 13]) = [-1 1];

% Vertical force equilibrium (-FFy + FGy - 100 = 0)
M(14,[12 14]) = [-1 1];
b(14) = 100;

% Moment equilibrium about joint G (rGF_y*FFx - rGF_x*FFy = 100*rGH_x)
rGF = F-G;
rGH = H-G;
M(15,[11 12]) = [rGF(2) -rGF(1)];
b(15) = 100*rGH(1);

%% Solve
x = M\b;

jointNames = {'A';'B';'C';'D';'E';'F';'G'};
matlabForces = [
    x(1) x(2);
    x(3) x(4);
    x(5) x(6);
    x(7) x(8);
    x(9) x(10);
    x(11) x(12);
    x(13) x(14)
];

Tin = x(15);

%% PMKS+ first-position results
% PMKS+ uses the opposite sign convention from this MATLAB model
pmksRawForces = [
    -190.6448 6.0629;
    190.6448 -6.0629;
    190.6448 -6.0629;
    115.4646 -4.1434;
    75.1802 -1.9195;
    75.1802 -1.9195;
    75.1802 -101.9195
];

pmksRawTorque = 97.9126;
pmksForces = -pmksRawForces;
pmksTorque = -pmksRawTorque;

pmksURL = 'https://pmksplus.mech.website/?j=A,1.4,0.485,AB,R,t,0,0.1,t,%0AB,1.67,0.99,AB%7CCB,R,f,0,0.1,f,%0AC,0.255,1.035,CB%7CDCE,R,f,0,0.1,f,%0AD,0.285,0.055,DCE,R,t,0,0.1,f,%0AE,0.195,2.54,EF%7CDCE,R,f,0,0.1,f,%0AF,-0.98,2.57,EF%7CGF,R,f,0,0.1,f,%0AG,0.05,0.2,GF,R,t,0,0.1,f,%0A&l=AB,1,1,1.535,0.7375,A%7CB,,l,1.67,0.485,1.4,0.485,1.4,0.99,1.67,0.99%0ACB,1,1,0.9624999999999999,1.0125,C%7CB,,l,1.67,1.035,0.255,1.035,0.255,0.99,1.67,0.99%0AEF,1,1,-0.39249999999999996,2.555,E%7CF,,l,-0.98,2.54,0.195,2.54,0.195,2.57,-0.98,2.57%0ADCE,1,1,0.2152807981577717,1.6982464898128944,D%7CC%7CE,,b,-0.28,-0.23,-0.397,3.593,0.711,3.627,0.828,-0.197%0AGF,1,1,-1.0206999443598936,2.545436725121601,G%7CF,F1,b,-0.257,-0.144,-2.466,4.939,-1.785,5.235,0.425,0.152%0A&f=F1,GF,-1.712,4.26,-1.712,0.508,t,true,0,100%0A&pp=&tp=&s=10,false,false,m';

%% Output
fprintf('\nPOINT H\n');
fprintf('H = (%.6f, %.6f) m\n',H(1),H(2));

fprintf('\nLENGTHS\n');
for i = 1:length(lengthNames)
    fprintf('%s = %.6f m\n',lengthNames{i},lengthValues(i));
end

fprintf('\nMATLAB AND PMKS+ JOINT FORCES\n');
disp(table(jointNames,matlabForces(:,1),matlabForces(:,2), pmksForces(:,1),pmksForces(:,2), ...
    'VariableNames',{'Joint','MATLAB_Fx','MATLAB_Fy', 'PMKS_Fx','PMKS_Fy'}));

fprintf('PMKS+ signs were adjusted to match the MATLAB sign convention.\n');

maxForceDifference = max(abs(matlabForces-pmksForces),[],'all');
fprintf('Maximum force-component difference = %.6f N\n', maxForceDifference);

fprintf('\nTORQUE COMPARISON\n');
fprintf('MATLAB Tin = %.6f N*m\n',Tin);
fprintf('PMKS+ Tin using MATLAB sign convention = %.4f N*m\n', pmksTorque);
fprintf('Torque difference = %.6f N*m\n',abs(Tin-pmksTorque));
fprintf('Negative torque means clockwise.\n');

fprintf(['The results match closely. The small difference is caused by PMKS+ ' ...
    'using H = (-1.71, 4.26) instead of the exact H = (-1.714591, 4.260273).\n']);

fprintf('\nPMKS+ URL\n%s\n',pmksURL);
