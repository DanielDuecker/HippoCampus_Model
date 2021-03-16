function dx = hippo_dynamics_base(t, x, param, control_param)
%% Parametervector
% Parameters [l, d, m, B, Ix, Iy, Iz, X_Du, Y_Dv, Z_Dw, K_Dp, M_Dq, N_Dr,
% X_uu, Y_vv, Z_ww, K_pp, M_qq, N_rr, C_T, C_D];
% l = param(1);
% d = param(2);
% m = param(3);
% B = param(4);
% Ix = param(5);
% Iy = param(6);
% Iz = param(7);
% 
% Added Mass Parameters
% X_Du = param(8);
% Y_Dv = param(9);
% Z_Dw = param(10;
% K_Dp = param(11);
% M_Dq = param(12);
% N_Dr = param(13);
% 
% Damping Parameters
% X_uu = param(14);
% Y_vv = param(15);
% Z_ww = param(16);
% K_pp = param(17);
% M_qq = param(18); 
% N_rr = param(19);
% 
% Propeller Thrust and Drag Coefficient
% C_T = param(20);
% C_D = param(21);

%% SNAME States
% States [x, y, z, phi, theta, psi, u, v, w, p, q, r];

eta = x(1:6,1); % in world-frame
nu = x(7:12,1); % in body-frame

phi = eta(4,1);
theta = eta(5,1);
psi = eta(6,1);

vb = nu(1:3,1);
wb = nu(4:6,1);

%% Transformation matrices
T_Theta = [1 sin(phi)*tan(theta)  cos(phi)*tan(theta);...
           0 cos(phi)            -sin(phi);...
           0 sin(phi)/cos(theta)  cos(phi)/cos(theta)];
       
R_nb = [ cos(psi)*cos(theta) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);...
         sin(psi)*cos(theta)  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi) -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);...
        -sin(theta)           cos(theta)*sin(phi)                             cos(theta)*cos(phi)];

J_Theta = [R_nb zeros(3); zeros(3) T_Theta];

%% System matrices with simplification:
% Symmetry, center of gravity equals origin of the bodyframe
M_RB_11 = param(3)*eye(3);
M_RB_12 = zeros(3);
M_RB_21 = zeros(3);
M_RB_22 = diag([param(5), param(6), param(7)]);

M_RB = [M_RB_11 M_RB_12; M_RB_21 M_RB_22];

% UUV moves at low speed
M_A_11 = -diag([param(8), param(9), param(10)]);
M_A_12 = zeros(3);
M_A_21 = zeros(3);
M_A_22 = -diag([param(11), param(12), param(13)]);

M_A = [M_A_11 M_A_12; M_A_21 M_A_22];

% With above simplifications
C_RB = [zeros(3) -param(3)*Skew(vb); -param(3)*Skew(vb) Skew(M_RB_22*wb)];

C_A = [zeros(3) -param(3)*Skew(M_A_11*vb); -param(3)*Skew(M_A_11*vb) Skew(M_A_22*wb)];

% Only decoupled motions are performed
D = -diag([param(14)*abs(nu(1)), param(15)*abs(nu(2)), param(16)*abs(nu(3)),...
    param(17)*abs(nu(4)), param(18)*abs(nu(5)), param(19)*abs(nu(6))]);

% The buoyancy force and gravitational force are assumed equal, center of mass and
% center of buoyancy coincide, static restoring forces are equal to zero.

g = 0; % buoyancy force and gravitational force
g_0 = 0; % static restoring forces

%% Input values tau
x_des = sp_generator(t, x);
if control_param == 0% 'geo'
    u = geo_controller(x_des, x);
elseif control_param == 1 % 'geo_ext'
    u = geo_controller_ext(x_des, x, param);
end
tau = thrust_signal(u);

%% Differential Equation
dx = zeros(12,1);
dx(1:6,1) = J_Theta*nu;
dx(7:12,1) = (M_RB + M_A)\(tau - D*nu - C_RB*nu - C_A*nu + g + g_0);



end

