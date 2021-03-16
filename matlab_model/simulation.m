%% HippoCampus Model
% by Malte Flehmke
% adapted by Daniel Duecker

%% Load the HippoCampus parameters
hippo_param = hippocampus_parameters();



%% Define the parameterized function
control_param=0;
f_hippo = @(t, x) hippo_dynamics_adv(t,  x, hippo_param, control_param); 
control_param=0;
f_hippo_ref = @(t, x) hippo_dynamics_base(t, sensor_data(t, x), hippo_param, control_param); 

%% Timeframe evaluated
% tspan = [0, 10];
tspan = linspace(0,4,1000);

%% Initial Conditions
% States [x, y, z, phi, theta, psi, u, v, w, p, q, r];
x0 =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

%% Solving the ODE
% for now the standard ode45 solver is used
options = odeset('Stats','on');
[t, X] = ode45(f_hippo, tspan, x0, options);
[t_ref, X_ref] = ode45(f_hippo_ref, tspan, x0, options);

%% Input Signal
% thrust = thrust_signal(t, X);
% thrust_ff = thrust_signal(t_ff, X_ff);
for i =1:numel(t)
    [dx(:,i), u(:,i), u_mot(:,i), tau(:,i)] = hippo_dynamics_adv(t(i),X(i,:)', hippo_param, control_param);
end


%% Plot vehicle Attitude
close all
figure(1)
subplot(411)
title('Attitude')
hold on
plot(t, X(:,4),'--')
plot(t, X(:,5),'--')
plot(t, X(:,6),'--')
plot(t_ref, X_ref(:,4))
plot(t_ref, X_ref(:,5))
plot(t_ref, X_ref(:,6))
xlabel('time in s')
ylabel('angle in rad')
legend('roll', 'pitch', 'yaw','roll-ref', 'pitch-ref', 'yaw-ref')
box on
grid on


subplot(412)
title('Ang Velocities')
hold on
plot(t, X(:,10),'--')
plot(t, X(:,11),'--')
plot(t, X(:,12),'--')
plot(t_ref, X_ref(:,10))
plot(t_ref, X_ref(:,11))
plot(t_ref, X_ref(:,12))
xlabel('time in s')
ylabel('ang vel in rad/2')
legend('roll-rate', 'pitch-rate', 'yaw-rate')
box on
grid on


subplot(413)
title('motor commands')
hold on
plot(t, u_mot(1,:),'--')
plot(t, u_mot(2,:),'--')
plot(t, u_mot(3,:),'--')
plot(t, u_mot(4,:),'--')
xlabel('time in s')
ylabel('u')
legend('u1', 'u2', 'u3','u4')
box on
grid on

subplot(414)
title('Tau = [Force-x, K-roll, M-pitch, N-Yaw]')
hold on
plot(t, tau(1,:),'--')
plot(t, tau(4,:),'--')
plot(t, tau(5,:),'--')
plot(t, tau(6,:),'--')
xlabel('time in s')
ylabel('Tau in N or Nm')
legend('Fx-tau1', 'Kroll-tau4', 'Mpitch-tau5','Nyaw-tau6')
box on
grid on

% %% Plot trajectory
% figure(2)
% plot(X(:,1),X(:,2))
% title('2D Position')
% xlabel('x')
% ylabel('y')
% grid on;