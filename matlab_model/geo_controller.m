function u = geo_controller(x_des, x)
%% Geo Controller following
% D. Mellinger and V. Kumar. Minimum Snap Trajectory Generation and Control
% for Quadrotors. ICRA, 2011
% D.A. Duecker,  A. Hackbarth, T. Johannink, E. Kreuzer, and E. Solowjow. 
% Micro Underwater Vehicle Hydrobatics: A Submerged Furuta Pendulum. ICRA,
% 2018.
%

% States [x, y, z, phi, theta, psi, u, v, w, p, q, r];

%
phi = x(4,1);
theta = x(5,1);
psi = x(6,1);
R = R_euler(psi, theta, phi); % Z-Y-X Euler angles to Rotation Matrix

phi_des = x_des(4,1);
theta_des = x_des(5,1);
psi_des = x_des(6,1);
R_des = R_euler(psi_des, theta_des, phi_des); % Z-Y-X Euler angles to Rotation Matrix


%% compute control errors
e_R = 0.5*(R_des'*R - R'*R_des);   %Compute attitude error

e_R_vec = [e_R(3,2); e_R(1,3); e_R(2,1)];   %vee-map the error to get a vector instead of matrix


omega = x(10:12,1);
omega_des = x_des(10:12,1);

e_omega = omega - omega_des;

% Gains
p_roll = 2;
p_pitch = 2;
p_yaw = 2;

d_roll = 1;
d_pitch = 1;
d_yaw = 1;

K_p = diag([p_roll, p_pitch, p_yaw]);
K_d = diag([d_roll, d_pitch, d_yaw]);

torques = - K_p * e_R_vec - K_d * e_omega;

thrust = 0;

u =[thrust; torques]; %[4x1]


end

function R = R_euler(psi, theta, phi)
% Z-Y-X Euler angles to Rotation Matrix
R = [cos(psi)*cos(theta),	(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)),	(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta));
     sin(psi)*cos(theta),	(cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi)),	cos(phi)*sin(theta)*sin(psi)-cos(psi)*sin(phi); 
         -sin(theta),                        cos(theta)*sin(phi),                                cos(phi)*cos(theta)];   
end    