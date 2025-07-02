function varargout = vehicleDynamics(functionName, varargin)
%% Vehicle Dynamics Functions
% Collection of vehicle dynamics functions for GP-MPC Tutorial
% 
% Usage:
%   xDot = vehicleDynamics('continuousDynamics', x, u, model_params)
%   xDot = vehicleDynamics('plantDynamics', x, u, plant_params)
%   x_next = vehicleDynamics('RK4', x, u, dynamics, dt)
%   x_next = vehicleDynamics('simulatePlant', x, u, p)

switch functionName
    case 'continuousDynamics'
        varargout{1} = continuousDynamics(varargin{:});
    case 'plantDynamics'
        varargout{1} = plantDynamics(varargin{:});
    case 'RK4'
        varargout{1} = RK4(varargin{:});
    case 'simulatePlant'
        varargout{1} = simulatePlant(varargin{:});
    otherwise
        error('Unknown function name: %s', functionName);
end

end

%% Dynamics model used for MPC prediction (ideal)
function xDot = continuousDynamics(x, u, model_params)
    % Kinematic Bicycle Model
    l_r = model_params.l_r;
    l_f = model_params.l_f;
    m = model_params.m;
    
    % State x: [xPos, yPos, vel, heading, steeringAngle]'
    v = x(3);
    psi = x(4);
    delta = x(5);
    
    % Input u: [Accel, steeringRate]'
    FLon = u(1);
    steeringRate = u(2);

    beta = atan(l_r / (l_r + l_f) * tan(delta));
    
    xDot = zeros(5,1);
    xDot(1) = v * cos(psi + beta);
    xDot(2) = v * sin(psi + beta);
    xDot(3) = FLon / m;
    xDot(4) = v / l_r * sin(beta);
    xDot(5) = steeringRate;
end

%% Actual plant dynamics model 
function xDot = plantDynamics(x, u, plant_params)
    % This model is intentionally set different from MPC's prediction model
    l_r = plant_params.l_r;
    l_f = plant_params.l_f;
    m = plant_params.m;
    
    v = x(3);
    psi = x(4);
    delta = x(5);
    
    FLon = u(1);
    steeringRate = u(2);

    beta = atan(l_r / (l_r + l_f) * tan(delta));
    
    xDot = zeros(5,1);
    xDot(1) = v * cos(psi + beta)*1.1;
    xDot(2) = v * sin(psi + beta)*0.8;
    xDot(3) = FLon / m - 0.01 * v^2; % Add nonlinear terms like air resistance
    % Add nonlinearity where steering response changes with higher velocity
    xDot(4) = (v / l_r * sin(beta)) * (1 - 0.1 * v / 5); 
    xDot(5) = steeringRate;
    % ================== [ADD NOISE] ==================
    noise_level = 0.05;

    noise = noise_level * (2 * rand(1, 1) - 1);
    noise_vec=[noise*0.1;noise*0.1;noise;noise*0.1];

    xDot(1:4) = xDot(1:4) + noise_vec;
end

%% RK4 numerical integrator
function x_next = RK4(x, u, dynamics, dt)
    k1 = dynamics(x, u);
    k2 = dynamics(x + dt/2 * k1, u);
    k3 = dynamics(x + dt/2 * k2, u);
    k4 = dynamics(x + dt * k3, u);
    x_next = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

%% Actual plant simulation function
function x_next = simulatePlant(x, u, p)
    % Calculate next state of actual plant using RK4 integrator
    x_next = RK4(x, u, @(x_p, u_p) plantDynamics(x_p, u_p, p.Plant), p.timeStep);
end
