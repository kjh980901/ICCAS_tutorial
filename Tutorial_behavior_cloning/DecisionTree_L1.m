%% Decision Tree for Level-1 (considers opponent's predicted behavior)
function [Q_value_opt, Action_id] = DecisionTree_L1(X_pseudo, car_id, action_space, t_step_DT)

% Discount factor for future rewards
discount = 0.8;

% Number of actions and lookahead steps
num_actions = length(action_space);
num_steps = 4;

% Initialize arrays to store best Q-value and action sequence
best_Q = -inf;
best_actions = zeros(num_steps, 1);

% Exhaustively search all possible 4-step action sequences
for id_1 = 1:num_actions
    % Step 1: Start with predicted state at time 1
    X1 = X_pseudo(:,:,1);
    % Update only ego vehicle
    X1 = Motion_Update(X1, car_id, action_space(id_1), t_step_DT);
    R1 = Reward(X1, car_id, action_space(id_1));
    
    for id_2 = 1:num_actions
        % Step 2: Use predicted state at time 2, but keep ego's position from step 1
        X2 = X_pseudo(:,:,2);
        X2(:,car_id) = X1(:,car_id);  % Keep ego vehicle's state from previous step
        % Update ego vehicle
        X2 = Motion_Update(X2, car_id, action_space(id_2), t_step_DT);
        R2 = Reward(X2, car_id, action_space(id_2));
        
        for id_3 = 1:num_actions
            % Step 3: Use predicted state at time 3, but keep ego's position from step 2
            X3 = X_pseudo(:,:,3);
            X3(:,car_id) = X2(:,car_id);  % Keep ego vehicle's state from previous step
            % Update ego vehicle
            X3 = Motion_Update(X3, car_id, action_space(id_3), t_step_DT);
            R3 = Reward(X3, car_id, action_space(id_3));
            
            for id_4 = 1:num_actions
                % Step 4: Use predicted state at time 4, but keep ego's position from step 3
                X4 = X_pseudo(:,:,4);
                X4(:,car_id) = X3(:,car_id);  % Keep ego vehicle's state from previous step
                % Update ego vehicle
                X4 = Motion_Update(X4, car_id, action_space(id_4), t_step_DT);
                R4 = Reward(X4, car_id, action_space(id_4));
                
                % Calculate total Q-value with discount factor
                Q_total = R1 + R2*discount + R3*discount^2 + R4*discount^3;
                
                % Update best if this sequence is better
                if Q_total > best_Q
                    best_Q = Q_total;
                    best_actions = [action_space(id_1); action_space(id_2); 
                                   action_space(id_3); action_space(id_4)];
                end
            end
        end
    end
end

% Return results
Q_value_opt = best_Q;
Action_id = best_actions;

end