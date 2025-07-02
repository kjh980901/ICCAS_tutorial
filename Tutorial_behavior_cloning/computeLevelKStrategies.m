function [action_ids, Q_values, X_pseudos] = computeLevelKStrategies(X_old, num_cars, action_space, t_step_DT, max_level)
% Compute all level-k strategies from L0 to max_level
% Returns cell arrays containing actions, Q-values, and pseudo states for each level

% Initialize storage for all levels
action_ids = cell(max_level+1, 1);  % L0 to L[max_level]
Q_values = cell(max_level+1, 1);
X_pseudos = cell(max_level+1, 1);

% Each level stores data for all cars
for level = 0:max_level
    action_ids{level+1} = cell(num_cars, 1);
    Q_values{level+1} = cell(num_cars, 1);
end

%% Level-0 computation
for car_id = 1:num_cars
    [Q_values{1}{car_id}, action_ids{1}{car_id}] = DecisionTree_L0(X_old, car_id, action_space, t_step_DT);
end

% Simulate L0 actions
X_temp = X_old;
X_pseudo_L0 = zeros(size(X_old,1), size(X_old,2), length(action_ids{1}{1}));
for step_idx = 1:length(action_ids{1}{1})
    for car_id = 1:num_cars
        X_temp = Motion_Update(X_temp, car_id, action_ids{1}{car_id}(step_idx), t_step_DT);
    end
    X_pseudo_L0(:,:,step_idx) = X_temp;
end
X_pseudos{1} = X_pseudo_L0;

%% Level-k computation (k >= 1)
for level = 1:max_level
    % Prepare X_pseudo_Id for each car based on previous level
    X_pseudo_prev_Id = cell(num_cars, 1);
    for car_id = 1:num_cars
        X_pseudo_prev_Id{car_id} = X_pseudos{level};
        % Reset ego vehicle position to current state
        for pre_step = 1:size(X_pseudos{level}, 3)
            X_pseudo_prev_Id{car_id}(:, car_id, pre_step) = X_old(:, car_id);
        end
    end
    
    % Compute level-k actions for all cars
    for car_id = 1:num_cars
        [Q_values{level+1}{car_id}, action_ids{level+1}{car_id}] = ...
            DecisionTree_L1(X_pseudo_prev_Id{car_id}, car_id, action_space, t_step_DT);
    end
    
    % Simulate level-k actions
    X_temp = X_old;
    X_pseudo_Lk = zeros(size(X_old,1), size(X_old,2), length(action_ids{level+1}{1}));
    for step_idx = 1:length(action_ids{level+1}{1})
        for car_id = 1:num_cars
            X_temp = Motion_Update(X_temp, car_id, action_ids{level+1}{car_id}(step_idx), t_step_DT);
        end
        X_pseudo_Lk(:,:,step_idx) = X_temp;
    end
    X_pseudos{level+1} = X_pseudo_Lk;
end

end