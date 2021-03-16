function [result] = thrust_model_adv(u, param)
%THRUST_SIGNAL input signal that projects thruster inputs onto the states
%thrust, roll, pitch and yaw. This is achieved by a mixer matrix



%% Control inputs --> single motor signals

% scale all commands by 0.4
u = 0.4*u;
% limit control inputs between -1 and 1
u(u>1)=1;
u(u<-1)=-1;

% u[thrust; roll; pitch; yaw] -> u_mot[u1, u2, u3, u4]
% Assumptions all motors spin in same direction
% M_Mixer = [1,1,-1, 1;
%            1,1,-1,-1;
%            1,1, 1,-1;
%            1,1, 1, 1];
M_Mixer = [ 1, 1,-1, 1;
           -1, 1, 1, 1;
            1, 1, 1,-1;
           -1, 1,-1,-1];
       



u_mot = M_Mixer * u;


%% Mixer Control commands->Real-World forces / Torques
kf = ones(4,1) * param(20);
km = ones(4,1) * param(21);
cw = [1,-1,1,-1];

% limit motor signals between -1 and 1
u_mot(u_mot>1) = 1;
u_mot(u_mot<-1) = -1;
% Motor modeling
U = [motor_model(u_mot(1), cw(1), kf(1), km(1));
     motor_model(u_mot(2), cw(2), kf(2), km(2));
     motor_model(u_mot(3), cw(3), kf(3), km(3));
     motor_model(u_mot(4), cw(4), kf(4), km(4))];

 
% Mapping matrix:
h = 0.055; % in [m]
r = [0,0,0,0;
     -h,h,h,-h;
     -h,-h,h,h];

T_map_thrust = [1,0,1,0,1,0,1,0;
                0,0,0,0,0,0,0,0;
                0,0,0,0,0,0,0,0];
T_map_torque = [0,1,      0,1,      0,1,      0,1;
             r(3,1),0, r(3,2),0, r(3,3),0, r(3,4),0;
            -r(2,1),0,-r(2,2),0,-r(2,3),0,-r(2,4),0];            
T_map = [T_map_thrust; T_map_torque];      
               
               
tau = T_map*U;



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
% p_out = u; % [u1, u2, u3, u4]
% 
% F_thrust = p_out(1);
% K_roll = p_out(2);
% M_pitch = p_out(3);
% N_yaw = p_out(4);

%% Outputs
% System is underactuated and non-holonomic, tau2 = tau3 = 0
% tau = zeros(6,1);
% tau(1) = F_thrust;
% tau(4) = K_roll;
% tau(5) = M_pitch;
% tau(6) = N_yaw;
result = [tau; u_mot];
end


function out = motor_model(u, cw, kf, km) 
    f = kf*u*abs(u)*cw;
    m = km*u*abs(u);
    out = [f;m];
end