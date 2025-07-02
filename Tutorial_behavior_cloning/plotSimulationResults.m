function plotSimulationResults(R_history, Level_ratio_history, t_step_Sim)
% Create new figure for simulation results
figure('Name', 'Simulation Results'); clf;

% Reward plot - using subplot to display in Live Script
subplot(2,1,1); hold on
plot([1:size(R_history,2)]*t_step_Sim, R_history(1,:,end), 'b-', 'LineWidth', 3)
plot([1:size(R_history,2)]*t_step_Sim, R_history(2,:,end), 'r-', 'LineWidth', 3)
xlabel('t [s]')
ylabel('Reward')
legend('Car 1 (Blue)', 'Car 2 (Red)', 'Location', 'best')
title('Vehicle Rewards Over Time')
grid on

% Level estimation plots
subplot(2,1,2); hold on
time = [1:size(Level_ratio_history,3)]*t_step_Sim;
plot(time, squeeze(Level_ratio_history(2,1,:,end)), 'b-', 'LineWidth', 3)
plot(time, squeeze(Level_ratio_history(2,2,:,end)), 'r-', 'LineWidth', 3)
plot(time, squeeze(Level_ratio_history(2,3,:,end)), 'g-', 'LineWidth', 3)
xlabel('t [s]')
ylabel('Probability')
legend('Level 0', 'Level 1', 'Level 2', 'Location', 'best')
title('Estimation of Human Car''s Level (Blue Car)')
ylim([0 1])
grid on
end

