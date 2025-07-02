function plotSimulationLive(X_history, episode)
% Plot simulation animation for Live Script using plotSimulation style
% Inputs:
%   X_history - State history (5 x num_cars x steps x episodes)
%   episode - Episode number to visualize
%
global w_lane l_car w_car

% Extract data for this episode
X_episode = X_history(:,:,:,episode);
num_steps = size(X_episode, 3);
color = ['b' 'r' 'm' 'g'];

% Clear any existing plots and force new figure
figure;      % Create new figure
clf;         % Clear it completely

% Animation loop
for step = 1:num_steps
    % Complete reset of current axes
    cla reset;
    
    % Set up fresh axes properties
    hold on; 
    box on;
    set(gca, 'fontsize', 14);
    
    % Set figure size for Live Script - make it square
    set(gca, 'Position', [0.1 0.1 0.7 0.8]);  % Narrower width to maintain square aspect
    
    % Pre-set axis limits
    xlim([-24 24]);
    ylim([-24 24]);
    
    % Note: Annotations don't work well in Live Scripts, will use text instead
    
    %% Draw intersection roads
    RoadBound = [w_lane,-6*w_lane;
        w_lane,-2*w_lane;
        2*w_lane,-w_lane;
        6*w_lane,-w_lane;
        6*w_lane,-6*w_lane;
        w_lane,6*w_lane;
        w_lane,2*w_lane;
        2*w_lane,w_lane;
        6*w_lane,w_lane;
        6*w_lane,6*w_lane;
        -w_lane,-6*w_lane;
        -w_lane,-2*w_lane;
        -2*w_lane,-w_lane;
        -6*w_lane,-w_lane;
        -6*w_lane,-6*w_lane;
        -w_lane,6*w_lane;
        -w_lane,2*w_lane;
        -2*w_lane,w_lane;
        -6*w_lane,w_lane;
        -6*w_lane,6*w_lane];

    % Fill road areas
    fill(RoadBound(1:5,1), RoadBound(1:5,2), [0.8 0.8 0.8], 'LineWidth', 2)
    fill(RoadBound(6:10,1), RoadBound(6:10,2), [0.8 0.8 0.8], 'LineWidth', 2)
    fill(RoadBound(11:15,1), RoadBound(11:15,2), [0.8 0.8 0.8], 'LineWidth', 2)
    fill(RoadBound(16:20,1), RoadBound(16:20,2), [0.8 0.8 0.8], 'LineWidth', 2)

    %% Draw center lines
    RoadMid = [2*w_lane 0;
        10*w_lane 0;
        -2*w_lane 0;
        -10*w_lane 0;
        0 -2*w_lane;
        0 -10*w_lane;
        0 2*w_lane;
        0 10*w_lane];

    plot([RoadMid(1,1) RoadMid(2,1)], [RoadMid(1,2) RoadMid(2,2)], '--', 'LineWidth', 3, 'Color', [1 0.5 0]);
    plot([RoadMid(3,1) RoadMid(4,1)], [RoadMid(3,2) RoadMid(4,2)], '--', 'LineWidth', 3, 'Color', [1 0.5 0]);
    plot([RoadMid(5,1) RoadMid(6,1)], [RoadMid(5,2) RoadMid(6,2)], '--', 'LineWidth', 3, 'Color', [1 0.5 0]);
    plot([RoadMid(7,1) RoadMid(8,1)], [RoadMid(7,2) RoadMid(8,2)], '--', 'LineWidth', 3, 'Color', [1 0.5 0]);

    %% Draw all vehicles
    X_old = X_episode(:,:,step);
    for id = 1:length(X_old(1,:))
        % Get vehicle state
        x = X_old(1, id);
        y = X_old(2, id);
        theta = X_old(3, id);
        
        % Calculate vehicle rectangle corners
        Vehicle_rectangle = [
            x - l_car/2*cos(theta) - w_car/2*sin(theta), y - l_car/2*sin(theta) + w_car/2*cos(theta);
            x - l_car/2*cos(theta) + w_car/2*sin(theta), y - l_car/2*sin(theta) - w_car/2*cos(theta);
            x + l_car/2*cos(theta) - w_car/2*sin(theta), y + l_car/2*sin(theta) + w_car/2*cos(theta);
            x + l_car/2*cos(theta) + w_car/2*sin(theta), y + l_car/2*sin(theta) - w_car/2*cos(theta);
            x + (l_car/2-1)*cos(theta) - w_car/2*sin(theta), y + (l_car/2-1)*sin(theta) + w_car/2*cos(theta);
            x + (l_car/2-1)*cos(theta) + w_car/2*sin(theta), y + (l_car/2-1)*sin(theta) - w_car/2*cos(theta)];
        
        % Draw vehicle body
        plot([Vehicle_rectangle(1,1) Vehicle_rectangle(2,1)], [Vehicle_rectangle(1,2) Vehicle_rectangle(2,2)], '-', 'LineWidth', 3, 'Color', color(id), 'HandleVisibility', 'off');
        plot([Vehicle_rectangle(1,1) Vehicle_rectangle(3,1)], [Vehicle_rectangle(1,2) Vehicle_rectangle(3,2)], '-', 'LineWidth', 3, 'Color', color(id), 'HandleVisibility', 'off');
        plot([Vehicle_rectangle(3,1) Vehicle_rectangle(4,1)], [Vehicle_rectangle(3,2) Vehicle_rectangle(4,2)], '-', 'LineWidth', 3, 'Color', color(id), 'HandleVisibility', 'off');
        plot([Vehicle_rectangle(2,1) Vehicle_rectangle(4,1)], [Vehicle_rectangle(2,2) Vehicle_rectangle(4,2)], '-', 'LineWidth', 3, 'Color', color(id), 'HandleVisibility', 'off');
        plot([Vehicle_rectangle(5,1) Vehicle_rectangle(6,1)], [Vehicle_rectangle(5,2) Vehicle_rectangle(6,2)], '-', 'LineWidth', 3, 'Color', color(id), 'HandleVisibility', 'off');
    end

    %% Add velocity text (instead of annotations for Live Script)
    % Vehicle 1 velocity
    text(12, -20, ['v_1 = ' num2str(X_old(4,1), '%.1f') ' m/s'], ...
        'FontSize', 14, ...
        'Color', color(1), ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.95 0.95 0.95], ...
        'EdgeColor', color(1), ...
        'LineWidth', 1);

    % Vehicle 2 velocity
    if size(X_old, 2) >= 2
        text(12, 20, ['v_2 = ' num2str(X_old(4,2), '%.1f') ' m/s'], ...
            'FontSize', 14, ...
            'Color', color(2), ...
            'FontWeight', 'bold', ...
            'BackgroundColor', [0.95 0.95 0.95], ...
            'EdgeColor', color(2), ...
            'LineWidth', 1);
    end

    %% Set axis limits and title
    % Set limits
    xlim([-24 24]);
    ylim([-24 24]);
    
    % Use plot box aspect ratio instead of axis equal
    pbaspect([1 1 1]);  % Square plot box
    daspect([1 1 1]);   % Equal data scaling
    
    xlabel('X [m]', 'FontSize', 12)
    ylabel('Y [m]', 'FontSize', 12)
    grid on
    title(sprintf('Episode %d, Step %d', episode, step), 'FontSize', 16)
    
    % Force update display
    drawnow limitrate;  % Use limitrate to prevent display issues
    
    % Pause for animation effect
    pause(0.05); % Faster animation
end

% After animation ends, set limits one more time
xlim([-24 24]);
ylim([-24 24]);

end