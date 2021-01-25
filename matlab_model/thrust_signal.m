function tau = thrust_signal(u, param)
%THRUST_SIGNAL input signal that projects thruster inputs onto the states
%thrust, roll, pitch and yaw. This is achieved by a mixer matrix
% h = 0.055; % in [m]

%% Actuate the thrusters
% Mixer: thruster signal to thrust and moments, inverted to calculate
% actuator commands from force commands
% 
% F_thrust = sum(propeller_thrust*(param(20)));
% K_roll = sum(mixer_roll*propeller_thrust*(param(21)));
% M_pitch = sum(mixer_pitch*propeller_thrust*(param(20)*h));
% N_yaw = sum(mixer_yaw*propeller_thrust*(param(20)*h));

%% Actuate the forces directly
% Thrust signal
p_out = u; % [u1, u2, u3, u4]

F_thrust = p_out(1);
K_roll = 0;
M_pitch = 0;
N_yaw = p_out(2);

%% Outputs
% System is underactuated and non-holonomic, tau2 = tau3 = 0
tau = zeros(6,1);
tau(1) = F_thrust;
tau(4) = K_roll;
tau(5) = M_pitch;
tau(6) = N_yaw;

end

