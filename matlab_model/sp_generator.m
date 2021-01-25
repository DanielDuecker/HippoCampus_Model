function [x_des] = sp_generator(t,x)
%SP_GENERATOR simmple setpoint generator for attitude control
% States [x, y, z, phi, theta, psi, u, v, w, p, q, r];
x_des = [0,0,0, pi/8,-pi/4,pi/4, 0,0,0,0,0,0]';
end

