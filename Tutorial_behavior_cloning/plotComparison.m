function plotComparison(X_GT, R_GT, Level_GT, Action_GT, X_NN, R_NN, Level_NN, Action_NN)
%% Plot comparison between Game Theory and Neural Network results
% Inputs:
%   X_GT, R_GT, Level_GT, Action_GT - Game Theory results
%   X_NN, R_NN, Level_NN, Action_NN - Neural Network results

%% Trajectory Comparison
figure('Name', 'Trajectory Comparison'); clf;
hold on;
plot(squeeze(X_GT(1,1,:)), squeeze(X_GT(2,1,:)), 'b-', 'LineWidth', 2);
plot(squeeze(X_GT(1,2,:)), squeeze(X_GT(2,2,:)), 'r-', 'LineWidth', 2);
plot(squeeze(X_NN(1,1,:)), squeeze(X_NN(2,1,:)), 'b--', 'LineWidth', 2);
plot(squeeze(X_NN(1,2,:)), squeeze(X_NN(2,2,:)), 'r--', 'LineWidth', 2);
xlabel('X [m]');
ylabel('Y [m]');
title('Vehicle Trajectories Comparison');
legend('Car 1 (GT)', 'Car 2 (GT)', 'Car 1 (NN)', 'Car 2 (NN)', 'Location', 'best');
grid on;
axis equal;
xlim([-24 24]);
ylim([-24 24]);

%% Reward Comparison
figure('Name', 'Reward Comparison');
hold on;
plot(R_GT(1,:), 'b-', 'LineWidth', 2);
plot(R_GT(2,:), 'r-', 'LineWidth', 2);
plot(R_NN(1,:), 'b--', 'LineWidth', 2);
plot(R_NN(2,:), 'r--', 'LineWidth', 2);
xlabel('Time Step');
ylabel('Reward');
title('Reward Comparison');
legend('Car 1 (GT)', 'Car 2 (GT)', 'Car 1 (NN)', 'Car 2 (NN)', 'Location', 'best');
grid on;

%% Action Comparison for Red Car (Car 2)
figure('Name', 'Action Comparison'); clf;

% Get the time steps
num_steps_GT = size(X_GT, 3);
num_steps_NN = size(X_NN, 3);

% Extract Red car's actual actions from history
% Red car adapts based on Blue car's estimated level
red_car_actions_GT = zeros(num_steps_GT, 1);
red_car_actions_NN = zeros(num_steps_NN, 1);

% For GT: Red car uses adaptive strategy based on level estimation
for step = 1:num_steps_GT
    [~, estimated_level] = max(Level_GT(2,:,step));  % Red's estimate of Blue
    switch estimated_level
        case 1  % Blue is L0, Red uses L1
            red_car_actions_GT(step) = Action_GT(step, 12);  % Column 12: V2_L1
        case 2  % Blue is L1, Red uses L2
            red_car_actions_GT(step) = Action_GT(step, 14);  % Column 14: V2_L2
        case 3  % Blue is L2, Red uses L3
            red_car_actions_GT(step) = Action_GT(step, 16);  % Column 16: V2_L3
    end
end

% For NN: Same adaptive strategy
for step = 1:num_steps_NN
    [~, estimated_level] = max(Level_NN(2,:,step));  % Red's estimate of Blue
    switch estimated_level
        case 1  % Blue is L0, Red uses L1
            red_car_actions_NN(step) = Action_NN(step, 12);  % Column 12: V2_L1
        case 2  % Blue is L1, Red uses L2
            red_car_actions_NN(step) = Action_NN(step, 14);  % Column 14: V2_L2
        case 3  % Blue is L2, Red uses L3
            red_car_actions_NN(step) = Action_NN(step, 16);  % Column 16: V2_L3
    end
end

% Create time series plot
subplot(2,1,1);
stairs(1:num_steps_GT, red_car_actions_GT, 'r-', 'LineWidth', 2);
hold on;
stairs(1:num_steps_NN, red_car_actions_NN, 'b--', 'LineWidth', 2);
xlabel('Time Step');
ylabel('Action ID');
title('Red Car (Adaptive) Actions Comparison');
legend('Game Theory', 'Neural Network', 'Location', 'best');
grid on;
ylim([0.5 6.5]);
yticks(1:6);
yticklabels({'Maintain', 'Turn Left', 'Turn Right', 'Accelerate', 'Decelerate', 'Hard Brake'});

% Plot action difference
subplot(2,1,2);
min_steps = min(num_steps_GT, num_steps_NN);
action_diff = red_car_actions_GT(1:min_steps) - red_car_actions_NN(1:min_steps);
stem(1:min_steps, action_diff, 'k', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Action Difference (GT - NN)');
title('Red Car Action Difference');
grid on;
ylim([-6 6]);

end