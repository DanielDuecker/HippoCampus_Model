function x_des = example_traj(t)
%% This function denotes a simple example trajectory for trajectoty tracking

%% Velocity setpoints
% thrust speed setpoint
u = 1;

% yaw rot speed setpoint
r = 1;

x_des = [u; r];