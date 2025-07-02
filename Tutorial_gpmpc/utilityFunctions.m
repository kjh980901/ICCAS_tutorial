function varargout = utilityFunctions(functionName, varargin)
%% Utility Functions
% Collection of utility functions for GP-MPC Tutorial
%
% Usage:
%   [p, I] = utilityFunctions('defineSystemAndMPCParams')
%   [train_data, val_data] = utilityFunctions('prepareTrainingData', data, I, train_ratio)

switch functionName
    case 'defineSystemAndMPCParams'
        [varargout{1}, varargout{2}] = defineSystemAndMPCParams();
    case 'prepareTrainingData'
        [varargout{1}, varargout{2}] = prepareTrainingData(varargin{:});
    otherwise
        error('Unknown function name: %s', functionName);
end

end

%% System and MPC parameters definition function
function [p, I] = defineSystemAndMPCParams()
    % Variable index definition (I: Index)
    % One step variable z = [u; x] -> 7-dimensional vector
    % Input u (2 variables)
    I.FLon = 1;         % Longitudinal acceleration/deceleration force [N]
    I.steeringRate = 2; % Steering angular velocity [rad/s]
    I.inputs = [I.FLon, I.steeringRate];
    
    % State x (5 variables)
    I.xPos = 3;         % x coordinate [m]
    I.yPos = 4;         % y coordinate [m]
    I.velocity = 5;     % Longitudinal velocity [m/s]
    I.heading = 6;      % Vehicle heading angle [rad]
    I.steeringAngle = 7;% Current steering angle [rad]
    I.states = [I.xPos, I.yPos, I.velocity, I.heading, I.steeringAngle];

    % --- Model Mismatch Generation ---
    % MPC prediction model (ideal)
    p.Model.l_r = 0.5;      % Rear wheel-center of mass distance [m]
    p.Model.l_f = 0.5;      % Front wheel-center of mass distance [m]
    p.Model.m = 1.0;        % Vehicle mass [kg]

    % Actual plant model (including nonlinearities)
    p.Plant.l_r = 0.5;      % Different parameter from prediction model
    p.Plant.l_f = 0.5;      % Different parameter from prediction model
    p.Plant.m = 1.0;        % Different parameter from prediction model
    
    % General simulation/MPC parameters
    p.timeStep = 0.1;       % Sampling time [s]
    p.horizonLength = 5;   % Prediction horizon length
    p.simLength = 200;      % Number of simulation steps
    p.nvar = length(I.inputs) + length(I.states); % 7
    
    % MPC weights
    p.pos_weight = 20;
    p.FLon_weight = 0.2;
    p.steerrate_weight = 0.2;
    p.fpos_weight = 40;     % Terminal position weight
    p.fFLon_weight = 0.2;
    p.fsteerrate_weight = 0.2;
end

%% 학습 데이터 준비 함수
function [train_data, val_data] = prepareTrainingData(data, ~, train_ratio)
    n_samples = size(data.X, 1);
    idx = randperm(n_samples);
    n_train = floor(n_samples * train_ratio);
    train_idx = idx(1:n_train);
    val_idx = idx(n_train+1:end);
    
    train_data.X = data.X(train_idx, :);
    train_data.Y = data.Y(train_idx, :);
    
    [train_data.X_norm, train_data.X_mean, train_data.X_std] = gpFunctions('normalize', train_data.X);
    
    val_data.X = data.X(val_idx, :);
    val_data.Y = data.Y(val_idx, :);
    val_data.X_norm = gpFunctions('normalize', val_data.X, train_data.X_mean, train_data.X_std);
    
    fprintf('Data ready:\n');
    fprintf('  - Total samples: %d\n', n_samples);
    fprintf('  - Training samples (pre-split): %d\n', n_train);
    fprintf('  - Validation samples: %d\n', length(val_idx));
end
