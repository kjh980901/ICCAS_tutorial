function Level_ratio = updateLevelEstimation(Level_ratio, Action_id, L0_action_id, L1_action_id, L2_action_id)
%% Update level estimation based on observed actions
% Inputs:
%   Level_ratio - Current level estimation matrix (num_cars x 3)
%   Action_id - Actual actions taken by each car (cell array)
%   L0_action_id, L1_action_id, L2_action_id - Actions suggested by each level
% Output:
%   Level_ratio - Updated level estimation

num_cars = 2;
for car_id = 1:num_cars
    opponent_id = 3 - car_id;
    
    % Skip if all levels suggest same action
    if L0_action_id{opponent_id}(1) == L1_action_id{opponent_id}(1) && ...
       L1_action_id{opponent_id}(1) == L2_action_id{opponent_id}(1)
        continue;
    end
    
    % Update belief based on observed action
    if Action_id{opponent_id} == L0_action_id{opponent_id}(1)
        Level_ratio(car_id,1) = Level_ratio(car_id,1) + 0.5;
    end
    if Action_id{opponent_id} == L1_action_id{opponent_id}(1)
        Level_ratio(car_id,2) = Level_ratio(car_id,2) + 0.5;
    end
    if Action_id{opponent_id} == L2_action_id{opponent_id}(1)
        Level_ratio(car_id,3) = Level_ratio(car_id,3) + 0.5;
    end
    
    % Normalize
    Level_ratio(car_id,:) = Level_ratio(car_id,:) / sum(Level_ratio(car_id,:));
end
end