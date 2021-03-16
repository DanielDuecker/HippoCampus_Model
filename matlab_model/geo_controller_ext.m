function u = geo_controller_ext(x_des, x, param)
%% Geo Controller following
% D. Mellinger and V. Kumar. Minimum Snap Trajectory Generation and Control
% for Quadrotors. ICRA, 2011
% D.A. Duecker,  A. Hackbarth, T. Johannink, E. Kreuzer, and E. Solowjow. 
% Micro Underwater Vehicle Hydrobatics: A Submerged Furuta Pendulum. ICRA,
% 2018.
%

%% SNAME States
% States [x, y, z, phi, theta, psi, u, v, w, p, q, r];

eta = x(1:6); % in world-frame
nu = x(7:12); % in body-frame

phi = eta(4);
theta = eta(5);
psi = eta(6);

vb = nu(1:3);
wb = nu(4:6);
%%
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


%% include system dynamics
M_RB_22 = diag([param(5), param(6), param(7)]);
M_A_22 = -diag([param(11), param(12), param(13)]);
% Only decoupled motions are performed
D = -diag([param(14)*abs(nu(1)), param(15)*abs(nu(2)), param(16)*abs(nu(3)),...
    param(17)*abs(nu(4)), param(18)*abs(nu(5)), param(19)*abs(nu(6))]);

D_w = D(4:6,4:6);

fl = cross(wb,(M_RB_22 + M_A_22)*wb) + D_w * wb

%%
fb = - K_p * e_R_vec - K_d * e_omega
torques = fb + fl;

thrust = 0;

u =[thrust; torques]; %[4x1]


end

function R = R_euler(psi, theta, phi)
% Z-Y-X Euler angles to Rotation Matrix
R = [cos(psi)*cos(theta),	(cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)),	(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta));
     sin(psi)*cos(theta),	(cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi)),	cos(phi)*sin(theta)*sin(psi)-cos(psi)*sin(phi); 
         -sin(theta),                        cos(theta)*sin(phi),                                cos(phi)*cos(theta)];   
end    