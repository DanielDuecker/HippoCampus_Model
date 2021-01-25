function p_out = controller(t, x)
%% This function denotes a plain proportional velocity setpoint controller

r = example_traj(t(1));

% state vector for example trajectory
state = zeros(2,1);
state(1) = x(7);
state(2) = x(12);

% Proportional gain
K_p = [2, 0; ...
       0, 0.1];

% Trajetory error
traj_error = r - state;

p_out = K_p * traj_error;

end
