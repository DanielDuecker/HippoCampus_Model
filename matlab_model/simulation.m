%% HippoCampus Model
% by Malte Flehmke
% adapted by Daniel Duecker

%% Load the HippoCampus parameters
param = hippocampus_parameters();

%% Define the parameterized function
f = @(t, x) hippocampus_parameterized(t, sensor_data(t, x), param); 

%% Timeframe evaluated
% tspan = [0, 10];
tspan = linspace(0,10,100);

%% Initial Conditions
% States [x, y, z, phi, theta, psi, u, v, w, p, q, r];
x0 =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

%% Solving the ODE
% for now the standard ode45 solver is used
options = odeset('Stats','on');
[t, X] = ode45(f, tspan, x0, options);

%% Input Signal
thrust = thrust_signal(t, X);

%% Plot trajectory
% figure(2)
% plot(X(:,1),X(:,2))
% title('2D Position')
% xlabel('x')
% ylabel('y')
% grid on;

%% Plot vehicle Attitude
close all
figure(1)
title('Attitude')
hold on
plot(t, X(:,4))
plot(t, X(:,5))
plot(t, X(:,6))
xlabel('time in s')
ylabel('angle in rad')
legend('roll', 'pitch', 'yaw')
box on
grid on