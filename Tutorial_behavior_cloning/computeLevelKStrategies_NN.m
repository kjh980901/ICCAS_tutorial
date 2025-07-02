function [action_ids, Q_values] = computeLevelKStrategies_NN(X_old, num_cars, NN_models)
% Compute level-k strategies using neural networks (1-step only)
% Inputs:
%   X_old - Current state matrix (5 x num_cars)
%   num_cars - Number of vehicles
%   NN_models - Struct containing 8 neural networks:
%               {NN_V1_0, NN_V2_0, NN_V1_1, NN_V2_1, NN_V1_2, NN_V2_2, NN_V1_3, NN_V2_3}
% Outputs:
%   action_ids - Cell array of single-step actions for each level
%   Q_values - Cell array of Q-values (empty for NN version)

% Validate NN_models structure
required_fields = {'NN_V1_0', 'NN_V2_0', 'NN_V1_1', 'NN_V2_1', ...
                  'NN_V1_2', 'NN_V2_2', 'NN_V1_3', 'NN_V2_3'};
for i = 1:length(required_fields)
    if ~isfield(NN_models, required_fields{i})
        error('Missing required neural network: %s', required_fields{i});
    end
end

% Initialize storage (4 levels: L0, L1, L2, L3)
action_ids = cell(4, 1);
Q_values = cell(4, 1);  % Empty for NN version

% Each level stores actions for all cars
for level = 1:4
    action_ids{level} = cell(num_cars, 1);
    Q_values{level} = cell(num_cars, 1);
end

% Prepare state vector for NN (8D: [x1,y1,θ1,v1,x2,y2,θ2,v2])
State = [X_old(1:4,1); X_old(1:4,2)];

% Get actions from neural networks for current state only
% Level-0
action_ids{1}{1} = vec2ind(NN_models.NN_V1_0(State));
action_ids{1}{2} = vec2ind(NN_models.NN_V2_0(State));

% Level-1
action_ids{2}{1} = vec2ind(NN_models.NN_V1_1(State));
action_ids{2}{2} = vec2ind(NN_models.NN_V2_1(State));

% Level-2
action_ids{3}{1} = vec2ind(NN_models.NN_V1_2(State));
action_ids{3}{2} = vec2ind(NN_models.NN_V2_2(State));

% Level-3
action_ids{4}{1} = vec2ind(NN_models.NN_V1_3(State));
action_ids{4}{2} = vec2ind(NN_models.NN_V2_3(State));

end