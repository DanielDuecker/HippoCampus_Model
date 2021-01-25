function param = hippocampus_parameters()
%% PARAMETERS FROM PRO-034
% HippoCampus paramters
param(1) = 0.205;          % [m]
param(2) = 0.235;          % [m]
param(3) = 1.47;           % [kg]
param(4) = 14.42;          % [N]
param(5) = 2.408e-3;      % [kg m^2]
param(6) = 10.717e-3;     % [kg m^2]
param(7) = 10.717e-3;     % [kg m^2]

% Added Mass Parameters
param(8) = -1.11;       % [kg]
param(9) = -2.8;        % [kg]
param(10) = -2.8;        % [kg]
param(11) = -4.51e-3;    % [kg m^2]
param(12) = -16.3e-3;    % [kg m^2]
param(13) = -16.3e-3;    % [kg m^2]

% Damping Parameters
param(14) = -5.39;       % [kg/m]
param(15) = -17.36;      % [kg/m]
param(16) = -17.36;      % [kg/m]
param(17) = -1.14e-3;    % [kg m^2]
param(18) = -7e-3;       % [kg m^2]
param(19) = -7e-3;       % [kg m^2]

% Propeller Thrust and Drag Coefficient
param(20) = 3.633e-5;     % [N/us^2]
param(21) = 2.469e-9;     % [N/us^2]

end

