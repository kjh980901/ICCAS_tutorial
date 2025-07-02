function varargout = mpcFunctions(functionName, varargin)
%% MPC Functions
% Collection of MPC-related functions for GP-MPC Tutorial
%
% Usage:
%   model = mpcFunctions('generatePathTrackingProblem_modified', p, I)
%   J = mpcFunctions('mpcObjective', z, p_vec, p, I)
%   [c, ceq] = mpcFunctions('mpcConstraints', z, p_vec, p, I)
%   model = mpcFunctions('generateMPCProblem', p, I, gp_models, type)
%   J = mpcFunctions('baseMpcObjective', z, p_vec, p, I)
%   [c, ceq] = mpcFunctions('baseMpcConstraints', z, p_vec, p, I)
%   J = mpcFunctions('gpMpcObjective', z, p_vec, p, I, gp_models)
%   [c, ceq] = mpcFunctions('gpMpcConstraints', z, p_vec, p, I, gp_models)
%   [mu_corr, var_corr] = mpcFunctions('predictGPError', mu_k, u_k, p, I, gp_models)

switch functionName
    case 'generatePathTrackingProblem_modified'
        varargout{1} = generatePathTrackingProblem_modified(varargin{:});
    case 'mpcObjective'
        varargout{1} = mpcObjective(varargin{:});
    case 'mpcConstraints'
        [varargout{1}, varargout{2}] = mpcConstraints(varargin{:});
    case 'generateMPCProblem'
        varargout{1} = generateMPCProblem(varargin{:});
    case 'baseMpcObjective'
        varargout{1} = baseMpcObjective(varargin{:});
    case 'baseMpcConstraints'
        [varargout{1}, varargout{2}] = baseMpcConstraints(varargin{:});
    case 'gpMpcObjective'
        varargout{1} = gpMpcObjective(varargin{:});
    case 'gpMpcConstraints'
        [varargout{1}, varargout{2}] = gpMpcConstraints(varargin{:});
    case 'predictGPError'
        [varargout{1}, varargout{2}] = predictGPError(varargin{:});
    otherwise
        error('Unknown function name: %s', functionName);
end

end

%% MPC problem generation function (for Data Generation)
function model = generatePathTrackingProblem_modified(p, I)
    
    % Set bounds for one stage variable z
    lb_stage = [-5, deg2rad(-90), -100, -100, 0, -1000, deg2rad(-50)]';
    ub_stage = [ 5, deg2rad(90),   100,  100, 5,  1000, deg2rad(50)]';
    
    model.N = p.horizonLength;
    model.nvar = p.nvar;
    model.lb = repmat(lb_stage, model.N, 1);
    model.ub = repmat(ub_stage, model.N, 1);
    
    % Define objective function and constraint function handles
    model.objective = @(z, p_vec) mpcObjective(z, p_vec, p, I);
    model.nonlcon = @(z, p_vec) mpcConstraints(z, p_vec, p, I);
    
    % State transition function (using RK4)
    % MPC predicts future using 'prediction model' with p.Model parameters
    model.eq = @(z) vehicleDynamics('RK4', z(I.states), z(I.inputs), @(x,u) vehicleDynamics('continuousDynamics', x, u, p.Model), p.timeStep);
end

%% MPC objective function (for Data Generation)
function J = mpcObjective(z, p_vec, p, I)
    J = 0;
    n_states = length(I.states);
    x_refs = p_vec(n_states+1 : n_states+p.horizonLength);
    y_refs = p_vec(n_states+p.horizonLength+1 : end);

    for k = 1:p.horizonLength
        z_k = z((k-1)*p.nvar + 1 : k*p.nvar);
        xRef = x_refs(k);
        yRef = y_refs(k);
        
        pos_error = (z_k(I.xPos) - xRef)^2 + (z_k(I.yPos) - yRef)^2;
        FLon_cost = z_k(I.FLon)^2;
        steering_cost = z_k(I.steeringRate)^2;
        
        if k == p.horizonLength
            J = J + p.fpos_weight * pos_error + p.fFLon_weight * FLon_cost + p.fsteerrate_weight * steering_cost;
        else
            J = J + p.pos_weight * pos_error + p.FLon_weight * FLon_cost + p.steerrate_weight * steering_cost;
        end
    end
end

%% MPC constraint function (for Data Generation)
function [c, ceq] = mpcConstraints(z, p_vec, p, I)
    c = []; % No inequality constraints
    
    x_init = p_vec(1:length(I.states));
    ceq = z(I.states) - x_init; % First state must match current state
    
    for k = 1:p.horizonLength-1
        z_k = z((k-1)*p.nvar + 1 : k*p.nvar);
        z_kp1 = z(k*p.nvar + 1 : (k+1)*p.nvar);
        
        % Next state calculated through prediction model must match optimization variable z_{k+1} state
        x_next_pred = vehicleDynamics('RK4', z_k(I.states), z_k(I.inputs), @(x,u) vehicleDynamics('continuousDynamics', x,u,p.Model), p.timeStep);
        ceq = [ceq; z_kp1(I.states) - x_next_pred];
    end
end

%% MPC problem generation function (for GP-MPC)
function model = generateMPCProblem(p, I, gp_models, type)
    lb_stage = [-10, deg2rad(-90), -inf, -inf, 0, -inf, deg2rad(-50)]';
    ub_stage = [ 10,  deg2rad(90),  inf,  inf, 5,  inf, deg2rad(50)]';
    
    model.N = p.horizonLength;
    model.nvar = p.nvar;
    model.lb = repmat(lb_stage, model.N, 1);
    model.ub = repmat(ub_stage, model.N, 1);
    
    if strcmp(type, 'GP-MPC')
        model.objective = @(z, p_vec) gpMpcObjective(z, p_vec, p, I, gp_models);
        model.nonlcon = @(z, p_vec) gpMpcConstraints(z, p_vec, p, I, gp_models);
    else 
        model.objective = @(z, p_vec) baseMpcObjective(z, p_vec, p, I);
        model.nonlcon = @(z, p_vec) baseMpcConstraints(z, p_vec, p, I);
    end
end

%% Baseline MPC Functions (structure modified)
function J = baseMpcObjective(z, p_vec, p, I)
    J = 0;
    n_states = length(I.states);
    x_refs = p_vec(n_states+1 : n_states+p.horizonLength);
    y_refs = p_vec(n_states+p.horizonLength+1 : end);

    for k = 1:p.horizonLength
        % Get k-th state and input directly from optimization variable z
        z_k = z((k-1)*p.nvar + 1 : k*p.nvar);
        x_k = z_k(I.states);
        u_k = z_k(I.inputs);
        
        % Calculate cost
        pos_error = (x_k(1) - x_refs(k))^2 + (x_k(2) - y_refs(k))^2;
        FLon_cost = u_k(1)^2;
        steering_cost = u_k(2)^2;
        
        if k == p.horizonLength
            J = J + p.fpos_weight * pos_error + p.fFLon_weight * FLon_cost + p.fsteerrate_weight * steering_cost;
        else
            J = J + p.pos_weight * pos_error + p.FLon_weight * FLon_cost + p.steerrate_weight * steering_cost;
        end
    end
end

function [c, ceq] = baseMpcConstraints(z, p_vec, p, I)
    c = []; % Inequality constraints handled by variable bounds (lb, ub)
    
    x_init = p_vec(1:length(I.states));
    % First state must match current vehicle state
    ceq = z(I.states) - x_init;
    
    for k = 1:p.horizonLength-1
        z_k = z((k-1)*p.nvar + 1 : k*p.nvar);
        z_kp1 = z(k*p.nvar + 1 : (k+1)*p.nvar);
        
        x_k = z_k(I.states);
        u_k = z_k(I.inputs);
        
        % Predict next state through basic prediction model (continuousDynamics)
        x_next_pred = vehicleDynamics('RK4', x_k, u_k, @(x,u) vehicleDynamics('continuousDynamics', x,u,p.Model), p.timeStep);
        
        % Force predicted next state to match state of optimization variable z_{k+1}
        ceq = [ceq; z_kp1(I.states) - x_next_pred];
    end
end

%% GP-MPC Functions (structure modified)
function J = gpMpcObjective(z, p_vec, p, I, gp_models)
    J = 0;
    n_states = length(I.states);
    x_refs = p_vec(n_states+1 : n_states+p.horizonLength);
    y_refs = p_vec(n_states+p.horizonLength+1 : end);
    
    % Variable for propagating variance (uncertainty) within cost function
    current_var = zeros(n_states, 1);
    
    % Weight matrix for uncertainty cost
    Q_var = diag([p.pos_weight, p.pos_weight, 0, 0, 0]);

    for k = 1:p.horizonLength
        % Get k-th mean state (mu) and input from optimization variable z
        z_k = z((k-1)*p.nvar + 1 : k*p.nvar);
        mu_k = z_k(I.states);
        u_k = z_k(I.inputs);
        
        % 1. Path following cost (based on mean state)
        pos_error = (mu_k(1) - x_refs(k))^2 + (mu_k(2) - y_refs(k))^2;
        
        % 2. Control input cost
        FLon_cost = u_k(1)^2;
        steering_cost = u_k(2)^2;
        
        % 3. Uncertainty cost (based on variance)
        [~, var_corr] = predictGPError(mu_k, u_k, p, I, gp_models);
        current_var = current_var + var_corr; % Variance propagation
        variance_cost = trace(Q_var * diag(current_var));
        
        % Final cost summation
        if k == p.horizonLength
            J = J + p.fpos_weight * pos_error ...
                  + p.fFLon_weight * FLon_cost ...
                  + p.fsteerrate_weight * steering_cost ...
                  + p.variance_weight * variance_cost;
        else
            J = J + p.pos_weight * pos_error ...
                  + p.FLon_weight * FLon_cost ...
                  + p.steerrate_weight * steering_cost ...
                  + p.variance_weight * variance_cost;
        end
    end
end

function [c, ceq] = gpMpcConstraints(z, p_vec, p, I, gp_models)
    x_init = p_vec(1:length(I.states));
    
    % First state must match current vehicle state
    ceq = z(I.states) - x_init;
    
    % Variable for propagating variance within constraints
    current_var = zeros(length(I.states), 1);
    
    % Array to hold inequality constraints (Chance Constraints)
    c = [];
    
    % Physical limits of state variables
    state_max = [inf; inf; 5; inf; deg2rad(50)];
    state_min = [-inf; -inf; 0; -inf; deg2rad(-50)];

    for k = 1:p.horizonLength-1
        z_k = z((k-1)*p.nvar + 1 : k*p.nvar);
        z_kp1 = z(k*p.nvar + 1 : (k+1)*p.nvar);
        
        mu_k = z_k(I.states);
        u_k = z_k(I.inputs);
        
        % --- Equality Constraints ---
        % Basic model prediction + GP error correction
        x_base_next = vehicleDynamics('RK4', mu_k, u_k, @(x,u) vehicleDynamics('continuousDynamics', x,u,p.Model), p.timeStep);
        [mu_corr, var_corr] = predictGPError(mu_k, u_k, p, I, gp_models);
        mu_next_pred = x_base_next + mu_corr;
        
        % Force GP-corrected predicted mean state to match state of z_{k+1}
        ceq = [ceq; z_kp1(I.states) - mu_next_pred];
        
        % --- Inequality Constraints ---
        current_var = current_var + var_corr; % Variance propagation
        std_dev = sqrt(current_var);
        
        % Chance Constraints: Set mu +/- beta*std to be within limits
        % fmincon follows c(x) <= 0 format
        c_upper = (mu_next_pred + p.beta * std_dev) - state_max;
        c_lower = state_min - (mu_next_pred - p.beta * std_dev);
        
        % Add inequality constraints only for constrained states (velocity, steering angle)
        c = [c; c_upper(3); c_upper(5); c_lower(3); c_lower(5)];
    end
end

function [mu_corr, var_corr] = predictGPError(mu_k, u_k, p, I, gp_models)
    % Calculate mean and variance of error predicted by GP for given state (mu_k) and input (u_k)
    gp_input = [mu_k; u_k]';
    mu_corr = zeros(length(I.states), 1);
    var_corr = zeros(length(I.states), 1);
    
    for i = 1:length(gp_models)
        % Apply GP compensation selectively according to p.gp_enable_mask
        if p.gp_enable_mask(i) == 1
            gp_input_norm = gpFunctions('normalize', gp_input, gp_models{i}.X_mean, gp_models{i}.X_std);
            [mu_i, var_i] = gpFunctions('gpPredict', gp_models{i}, gp_input_norm);
            mu_corr(i) = mu_i;
            var_corr(i) = var_i;
        end
    end
end
