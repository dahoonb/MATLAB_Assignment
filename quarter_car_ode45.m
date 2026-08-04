clear;
clc;
close all;

ms = 300; % Sprung mass (kg)
mu = 40; % Unsprung mass (kg)
ks = 22000; % Suspension stiffness (N/m)
bs = 1500; % Suspension damping coefficient (N*s/m)
kt = 190000; % Tire stiffness (N/m)
bt = 150; % Tire damping coefficient (N*s/m)

tspan = [0 2.5];
z0 = [0; 0; 0; 0]; % [zs; vs; zu; vu]

[t, z] = ode45(@(t, y) quarter_car_ode(t, y, ms, mu, ks, bs, kt, bt), tspan, z0);

zs = z(:, 1);
vs = z(:, 2);
zu = z(:, 3);
vu = z(:, 4);

[rVec, ~] = arrayfun(@road_profile, t);
accelS = (-ks*(zs-zu) - bs*(vs-vu))/ms;

% Plot
figure;

subplot(2, 1, 1);
plot(t, rVec, '--');
hold on;
plot(t, zu);
plot(t, zs);
title('Quarter Car Response with Tire Damping');
xlabel('Time (s)');
ylabel('Displacement (m)');
legend('Road Input r(t)', 'Unsprung Mass z_u', 'Sprung Mass z_s');
grid on;

subplot(2, 1, 2);
plot(t, accelS/9.81);
title('Chassis Vertical Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (g)');
grid on;

function dydt = quarter_car_ode(t, y, ms, mu, ks, bs, kt, bt)
    zs = y(1);
    vs = y(2);
    zu = y(3);
    vu = y(4);

    [r, drdt] = road_profile(t);

    dzsdt = vs;
    dvsdt = (-ks*(zs-zu) - bs*(vs-vu))/ms;
    dzudt = vu;
    dvudt = (ks*(zs-zu) + bs*(vs-vu) - kt*(zu-r) - bt*(vu-drdt))/mu;

    dydt = [dzsdt; dvsdt; dzudt; dvudt];
end

function [r, drdt] = road_profile(t)
    bumpHeight = 0.10;
    tStart = 0.2;
    steepness = 50;

    r = (bumpHeight/2)*(1 + tanh(steepness*(t-tStart)));
    drdt = (bumpHeight/2)*steepness*sech(steepness*(t-tStart))^2;
end
