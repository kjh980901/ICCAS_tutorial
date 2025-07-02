function [X_history, R_history, Level_ratio_history, Action_history, avg_compute_time] = runLevelKSimulation(num_episodes, human_level, random_init, use_nn, NN_models)
%% Run Level-k Game-Theoretic Simulation
% Inputs:
%   num_episodes - Number of episodes to simulate
%   human_level - Level for Car 1 (0, 1, or 2, default: 2)
%   random_init - Boolean flag for random initial positions (default: false)
%   use_nn - Boolean flag to use neural networks (0: game theory, 1: NN, default: 0)
%   NN_models - Struct with trained NN models (required if use_nn=1)
% Outputs:
%   X_history - State history for all episodes
%   R_history - Reward history
%   Level_ratio_history - Level estimation history
%   Action_history - Action history [state(8), all_actions(8)] for NN training
%   avg_compute_time - Average computation time per step (ms)

% Default arguments
if nargin < 2, human_level = 2; end
if nargin < 3, random_init = false; end
if nargin < 4, use_nn = false; end
if nargin < 5 && use_nn
    error('NN_models required when use_nn=true');
end

% Parameters
global w_lane l_car w_car v_nominal v_max v_min discount
w_lane = 4;          % lane width
l_car = 5;           % car length
w_car = 2;           % car width
v_nominal = 2.5;     % [m/s]
v_max = 5;
v_min = 0;
t_step_DT = 0.5;     % Decision-making sample time [s], 4step for 2s prediction
t_step_Sim = 0.25;   % simulation sample time [s]
discount = 0.8;

num_cars = 2;
max_steps = 50;

% Initialize history storage
X_history = [];
R_history = [];
Level_ratio_history = [];
Action_history = [];
compute_times = [];  % Store computation times

% Action space
action_space = [1,2,3,4,5,6];
% 1:maintain  2:turn left  3:turn right  4:accelerate  5:decelerate  6:hard brake

for ep = 1:num_episodes
    fprintf('\n Episode = %i\n', ep);
    
    % Initialize vehicles
    initial_state = initializeVehicles(random_init);
    
    % Initial level estimation
    Level_ratio = [0.1 0.6 0.3;  % Car 1's estimate of Car 2
                   0.1 0.6 0.3]; % Car 2's estimate of Car 1
    
    X_old = initial_state;
    episode_actions = [];
    
    for step = 1:max_steps
        % Print progress every 5 steps
        if mod(step, 5) == 0
            fprintf(' Steps completed: %i/%i\n', step, max_steps);
        end
        
        % Record state
        X_history(:,:,step,ep) = X_old;
        
        % Compute all level-k strategies (with timing)
        tic;
        if use_nn
            % Use neural networks for fast computation
            [action_ids, Q_values] = computeLevelKStrategies_NN(X_old, num_cars, NN_models);
        else
            % Use game-theoretic decision trees
            [action_ids, Q_values, ~] = computeLevelKStrategies(X_old, num_cars, action_space, t_step_DT, 3);
        end
        compute_time = toc;
        compute_times = [compute_times, compute_time];
        
        % Extract actions for each level
        L0_action_id = action_ids{1};
        L1_action_id = action_ids{2};
        L2_action_id = action_ids{3};
        L3_action_id = action_ids{4};
        
        % Decide actions based on level estimation
        Action_id = cell(num_cars,1);
        
        % Car 1 (Blue) - Fixed strategy based on human_level parameter
        switch human_level
            case 0
                Action_id{1} = L0_action_id{1}(1);
            case 1
                Action_id{1} = L1_action_id{1}(1);
            case 2
                Action_id{1} = L2_action_id{1}(1);
            case 3
                Action_id{1} = L3_action_id{1}(1);
            otherwise
                Action_id{1} = L2_action_id{1}(1);  % Default to L2
        end
        
        % Car 2 (Red) - Adaptive based on Car 1's estimated level
        [~,level_hv] = max(Level_ratio(2,:));
        switch level_hv
            case 1 % Opponent is L0
                Action_id{2} = L1_action_id{2}(1);
            case 2 % Opponent is L1
                Action_id{2} = L2_action_id{2}(1);
            case 3 % Opponent is L2
                Action_id{2} = L3_action_id{2}(1);
        end
        
        % Record state and actions
        state_8d = [X_old(1:4,1); X_old(1:4,2)]';  % 1x8 vector
        
        % Record all level actions for NN training
            all_actions = [L0_action_id{1}(1), L0_action_id{2}(1), ...
                          L1_action_id{1}(1), L1_action_id{2}(1), ...
                          L2_action_id{1}(1), L2_action_id{2}(1), ...
                          L3_action_id{1}(1), L3_action_id{2}(1)];
        
        episode_actions(step,:) = [state_8d, all_actions];
        
        
        % Update level estimation
        Level_ratio = updateLevelEstimation(Level_ratio, Action_id, L0_action_id, L1_action_id, L2_action_id);
        Level_ratio_history(:,:,step,ep) = Level_ratio;
        
        % Update states
        X_new = X_old;
        for car_id = 1:num_cars
            X_new = Motion_Update(X_new, car_id, Action_id{car_id}, t_step_Sim);
        end
        
        % Calculate rewards
        R = zeros(num_cars, 1);
        for car_id = 1:num_cars
            R(car_id) = Reward(X_new, car_id, Action_id{car_id});
        end
        
        X_old = X_new;
        R_history(:,step,ep) = R;
        
        % Visualization (removed - now handled externally)
        
        % Check stopping conditions
        if R(1) < -5000 || R(2) < -5000
            fprintf('Collision detected!\n');
            break;
        end
        
        % Check if both vehicles reached their destinations
        % Car 1 (Blue) goes north, Car 2 (Red) goes east
        car1_y = X_old(2, 1);  % Car 1's y position
        car2_x = X_old(1, 2);  % Car 2's x position
        
        if car1_y > 24 && car2_x > 24
            fprintf('Both vehicles reached destinations!\n');
            break;
        end
    end
    
    Action_history = [Action_history; episode_actions];
end

% Calculate average computation time in milliseconds
avg_compute_time = mean(compute_times) * 1000;  % Convert to ms

end

