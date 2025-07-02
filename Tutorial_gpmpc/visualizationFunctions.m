function varargout = visualizationFunctions(functionName, varargin)
%% Visualization Functions
% Collection of visualization functions for GP-MPC Tutorial
%
% Usage:
%   handles = visualizationFunctions('plotSimData', k, p, I, track, simStates, simInputs, predTraj, nextPath, handles, Y_data)
%   handles = visualizationFunctions('plotRealtimeSimData', k, p, I, reference_path, simStates, simInputs, predictedTraj, nextPathPoints, handles, controller_type, compTimes)
%   visualizationFunctions('validateGPModels', gp_models, val_data, I)
%   visualizationFunctions('visualizeGPPredictions', gp_models, data, I, p)
%   visualizationFunctions('visualizeAndCompareResults', p, I, reference_path, results_gp, results_base)
%   lat_err_vec = visualizationFunctions('calculateLateralError', states, reference_path)

switch functionName
    case 'plotSimData'
        varargout{1} = plotSimData(varargin{:});
    case 'plotRealtimeSimData'
        varargout{1} = plotRealtimeSimData(varargin{:});
    case 'validateGPModels'
        validateGPModels(varargin{:});
    case 'visualizeGPPredictions'
        visualizeGPPredictions(varargin{:});
    case 'visualizeAndCompareResults'
        visualizeAndCompareResults(varargin{:});
    case 'calculateLateralError'
        varargout{1} = calculateLateralError(varargin{:});
    otherwise
        error('Unknown function name: %s', functionName);
end

end

%% Simulation result visualization function (for Data Generation)
function handles = plotSimData(k, p, I, track, simStates, simInputs, predTraj, nextPath, handles, Y_data)
    if isempty(handles)
        handles.fig = figure('Name', 'GP Data Generation', 'Position', [100, 100, 1600, 800]);
        
        % XY trajectory plot
        handles.ax1 = subplot(2,2,1);
        handles.p1(1) = plot(track.x, track.y, 'k--', 'DisplayName', 'Reference'); hold on;
        handles.p1(2) = plot(simStates(1,1:k), simStates(2,1:k), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Traj.');
        handles.p1(3) = plot(predTraj(I.xPos,:), predTraj(I.yPos,:), 'g-o', 'DisplayName', 'Predicted Traj.');
        handles.p1(4) = plot(nextPath(1,:), nextPath(2,:), 'rx', 'DisplayName', 'Horizon Refs');
        title(handles.ax1, 'Vehicle Trajectory'); xlabel('X [m]'); ylabel('Y [m]'); axis equal; grid on; legend;
        
        % Velocity plot
        handles.ax2 = subplot(2,2,2);
        handles.p2(1) = plot(p.timeStep*(0:k-1), simStates(I.velocity-2,1:k), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Vel.'); hold on;
        
        % ======[Error Fix]======
        % Fix x-axis data length to p.horizonLength (10)
        time_pred = p.timeStep*(k:k+p.horizonLength-1);
        handles.p2(2) = plot(time_pred, predTraj(I.velocity,:), 'g-o', 'DisplayName', 'Predicted Vel.');
        % =======================

        title(handles.ax2, 'Velocity Profile'); xlabel('Time [s]'); ylabel('Velocity [m/s]'); grid on; legend;
        
        % Steering angle/steering rate plot
        handles.ax3 = subplot(2,2,3);
        yyaxis left;
        handles.p3(1) = plot(p.timeStep*(0:k-1), rad2deg(simStates(I.steeringAngle-2,1:k)), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Steer Angle');
        ylabel('Angle [deg]');
        hold on;
        yyaxis right;
        handles.p3(2) = stairs(p.timeStep*(0:k-1), rad2deg(simInputs(I.steeringRate,1:k)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Steer Rate');
        ylabel('Rate [deg/s]');
        title(handles.ax3, 'Steering'); xlabel('Time [s]'); grid on; legend;
        
        % Error data plot (Y_data)
        handles.ax4 = subplot(2,2,4);
        handles.p4 = plot(p.timeStep*(0:k-1), Y_data(1:k, :), 'LineWidth', 1.5);
        title(handles.ax4, 'Collected Model Error (GP Target)'); xlabel('Time Step'); ylabel('Error'); grid on;
        legend(handles.ax4, 'e_x', 'e_y', 'e_v', 'e_{\psi}', 'e_{\delta}');
        
    else
        % Data update
        set(handles.p1(2), 'XData', simStates(1,1:k), 'YData', simStates(2,1:k));
        set(handles.p1(3), 'XData', predTraj(I.xPos,:), 'YData', predTraj(I.yPos,:));
        set(handles.p1(4), 'XData', nextPath(1,:), 'YData', nextPath(2,:));
        
        set(handles.p2(1), 'XData', p.timeStep*(0:k-1), 'YData', simStates(I.velocity-2,1:k));
        
        % ======[Error Fix]======
        time_pred = p.timeStep*(k:k+p.horizonLength-1);
        set(handles.p2(2), 'XData', time_pred, 'YData', predTraj(I.velocity,:));
        % =======================
        
        set(handles.p3(1), 'XData', p.timeStep*(0:k-1), 'YData', rad2deg(simStates(I.steeringAngle-2,1:k)));
        set(handles.p3(2), 'XData', p.timeStep*(0:k-1), 'YData', rad2deg(simInputs(I.steeringRate,1:k)));
        
        % Error plot update
        for i = 1:size(Y_data,2)
           set(handles.p4(i), 'XData', p.timeStep*(0:k-1), 'YData', Y_data(1:k,i));
        end
    end
    
end

%% Real-time visualization function (for GP-MPC)
function handles = plotRealtimeSimData(k, p, I, reference_path, simStates, simInputs, predictedTraj, nextPathPoints, handles, controller_type, compTimes)
    
    createPlots = true;
    updatePlots = false;
    
    % Check if handles exist and are valid
    if all(isfield(handles, {'figHandle','xyPlotHandles','velocityPlotHandles',...
            'steeringPlotHandles','accelPlotHandles','compTimeHandles'}))
        createPlots = false;
        updatePlots = true;
        figHandle = handles.figHandle;
        xyPlotHandles = handles.xyPlotHandles;
        velocityPlotHandles = handles.velocityPlotHandles;
        steeringPlotHandles = handles.steeringPlotHandles;
        accelPlotHandles = handles.accelPlotHandles;
        compTimeHandles = handles.compTimeHandles;
    end
    
    if createPlots
        figHandle = figure('Name', sprintf('%s - Real-time Simulation', controller_type), ...
                          'Position', [100, 100, 1600, 800]);
        clf;
        handles.figHandle = figHandle;
    else
        try
            figure(figHandle);
        catch
            warning('Figure was closed. Stopping visualization.');
            return;
        end
    end
    
    %% XY trajectory plot
    if createPlots
        subplot(2,3,[1,4]); hold all;
        xyPlotHandles(1) = plot(reference_path.x, reference_path.y,'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
        xyPlotHandles(2) = plot(nextPathPoints(1,:), nextPathPoints(2,:),'rx', 'MarkerSize', 8, 'DisplayName', 'Horizon Refs');
        xyPlotHandles(3) = plot(simStates(I.xPos-2,1),simStates(I.yPos-2,1),'bo','MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'Start');
        xyPlotHandles(4) = plot(simStates(I.xPos-2,1:k),simStates(I.yPos-2,1:k),'b-','LineWidth', 2, 'DisplayName', 'Actual Traj.');
        xyPlotHandles(5) = plot(predictedTraj(I.xPos,:),predictedTraj(I.yPos,:),'-g','LineWidth', 2, 'DisplayName', 'Predicted Traj.');
        
        legend('Location','southeast');
        title(sprintf('%s - Vehicle Trajectory', controller_type));
        xlabel('X [m]'); ylabel('Y [m]');
        xlim([min(reference_path.x)-1, max(reference_path.x)+1]); 
        ylim([min(reference_path.y)-1, max(reference_path.y)+1]);
        axis equal; grid on;
        
    elseif updatePlots
        set(xyPlotHandles(2), 'XData', nextPathPoints(1,:), 'YData', nextPathPoints(2,:));
        set(xyPlotHandles(4), 'XData', simStates(I.xPos-2,1:k), 'YData', simStates(I.yPos-2,1:k));
        set(xyPlotHandles(5), 'XData', predictedTraj(I.xPos,:), 'YData', predictedTraj(I.yPos,:));
    end
    
    %% Velocity profile
    if createPlots
        subplot(2,3,2); hold all;
        title('Velocity Profile'); grid on;
        velocityPlotHandles(1) = plot(p.timeStep*(0:k-1), simStates(I.velocity-2,1:k),'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
        time_pred = p.timeStep*(k:k+p.horizonLength-1);
        velocityPlotHandles(2) = plot(time_pred, predictedTraj(I.velocity,:),'g-o', 'DisplayName', 'Predicted');
        velocityPlotHandles(3) = plot([0 p.simLength*p.timeStep], [5 5], 'r:', 'DisplayName', 'Max Vel');
        xlim([0, p.simLength*p.timeStep]); ylabel('Velocity [m/s]'); xlabel('Time [s]');
        legend;
        
    elseif updatePlots
        set(velocityPlotHandles(1), 'XData', p.timeStep*(0:k-1), 'YData', simStates(I.velocity-2,1:k));
        time_pred = p.timeStep*(k:k+p.horizonLength-1);
        set(velocityPlotHandles(2), 'XData', time_pred, 'YData', predictedTraj(I.velocity,:));
    end
    
    %% Steering angle/steering rate
    if createPlots
        subplot(2,3,3); hold all;
        title('Steering'); grid on;
        yyaxis left;
        steeringPlotHandles(1) = plot(p.timeStep*(0:k-1), rad2deg(simStates(I.steeringAngle-2,1:k)),'b-', 'LineWidth', 2, 'DisplayName', 'Angle');
        ylabel('Steering Angle [deg]');
        yyaxis right;
        steeringPlotHandles(2) = stairs(p.timeStep*(0:k-1), rad2deg(simInputs(I.steeringRate,1:k)),'r-', 'LineWidth', 1.5, 'DisplayName', 'Rate');
        ylabel('Steering Rate [deg/s]');
        xlim([0, p.simLength*p.timeStep]); xlabel('Time [s]');
        legend;
        
    elseif updatePlots
        yyaxis left;
        set(steeringPlotHandles(1), 'XData', p.timeStep*(0:k-1), 'YData', rad2deg(simStates(I.steeringAngle-2,1:k)));
        yyaxis right;
        set(steeringPlotHandles(2), 'XData', p.timeStep*(0:k-1), 'YData', rad2deg(simInputs(I.steeringRate,1:k)));
    end
    
    %% Acceleration force
    if createPlots
        subplot(2,3,5); hold all;
        title('Acceleration Force'); grid on;
        accelPlotHandles(1) = stairs(p.timeStep*(0:k-1), simInputs(I.FLon,1:k),'b-', 'LineWidth', 2, 'DisplayName', 'Applied');
        accelPlotHandles(2) = stairs(k*p.timeStep:p.timeStep:(k+p.horizonLength-1)*p.timeStep, predictedTraj(I.FLon,:),'g-', 'DisplayName', 'Predicted');
        accelPlotHandles(3) = plot([0 p.simLength*p.timeStep], [5 5], 'r:', 'DisplayName', 'Max');
        accelPlotHandles(4) = plot([0 p.simLength*p.timeStep], [-5 -5], 'r:', 'DisplayName', 'Min');
        xlim([0, p.simLength*p.timeStep]); ylabel('Force [N]'); xlabel('Time [s]');
        legend;
        
    elseif updatePlots
        set(accelPlotHandles(1), 'XData', p.timeStep*(0:k-1), 'YData', simInputs(I.FLon,1:k));
        set(accelPlotHandles(2), 'XData', k*p.timeStep:p.timeStep:(k+p.horizonLength-1)*p.timeStep, 'YData', predictedTraj(I.FLon,:));
    end
    
    %% Computation time
    if createPlots
        subplot(2,3,6); hold all;
        title('Computation Time'); grid on;
        compTimeHandles(1) = plot(p.timeStep*(1:k), compTimes(1:k)*1000,'b-', 'LineWidth', 2, 'DisplayName', 'Comp. Time');
        compTimeHandles(2) = plot([0 p.simLength*p.timeStep], [mean(compTimes(1:k))*1000 mean(compTimes(1:k))*1000], 'r--', 'DisplayName', sprintf('Avg: %.1f ms', mean(compTimes(1:k))*1000));
        xlim([0, p.simLength*p.timeStep]); ylabel('Time [ms]'); xlabel('Time [s]');
        legend;
        
    elseif updatePlots
        set(compTimeHandles(1), 'XData', p.timeStep*(1:k), 'YData', compTimes(1:k)*1000);
        avg_time = mean(compTimes(1:k))*1000;
        set(compTimeHandles(2), 'YData', [avg_time avg_time]);
        % Update legend
        legend_entries = get(legend(gca), 'String');
        legend_entries{2} = sprintf('Avg: %.1f ms', avg_time);
        legend(legend_entries);
    end
    
    % Update overall title
    sgtitle(sprintf('%s - Step %d/%d (%.1f%%)', controller_type, k, p.simLength, k/p.simLength*100));
    
    %% Save handles
    handles.xyPlotHandles = xyPlotHandles;
    handles.velocityPlotHandles = velocityPlotHandles;
    handles.steeringPlotHandles = steeringPlotHandles;
    handles.accelPlotHandles = accelPlotHandles;
    handles.compTimeHandles = compTimeHandles;
end

%% GP model validation visualization
function validateGPModels(gp_models, val_data, I)
    n_outputs=length(I.states); state_names={'x_{pos}','y_{pos}','velocity','heading','steeringAngle'};
    figure('Name','GP Model Validation (Predicted vs. Actual)','Position',[100,100,1500,600]);
    for i = 1:n_outputs
        [mu, sigma2] = gpFunctions('gpPredict', gp_models{i}, val_data.X_norm);
        errors = val_data.Y(:, i) - mu; rmse = sqrt(mean(errors.^2));
        conf_bound = 2*sqrt(sigma2); in_bounds_percent=sum(abs(errors)<=conf_bound)/length(errors)*100;
        subplot(2, 3, i); hold on; grid on;
        [mu_sorted,sort_idx]=sort(mu); conf_sorted=conf_bound(sort_idx);
        fill_x=[mu_sorted;flipud(mu_sorted)]; fill_y=[mu_sorted-conf_sorted;flipud(mu_sorted+conf_sorted)];
        fill(fill_x,fill_y,[0.8 0.8 0.8],'EdgeColor','none','FaceAlpha',0.5);
        scatter(mu, val_data.Y(:, i), 15, 'b', 'filled', 'MarkerFaceAlpha', 0.4);
        lim_min=min([min(mu),min(val_data.Y(:,i))]); lim_max=max([max(mu),max(val_data.Y(:,i))]);
        plot([lim_min,lim_max],[lim_min,lim_max],'r--','LineWidth',2);
        xlabel('GP Prediction'); ylabel('True Value');
        title(sprintf('Error: %s\nRMSE: %.4f | In 95%% CI: %.1f%%',state_names{i},rmse,in_bounds_percent));
        legend('95% Confidence','Data','Perfect Prediction','Location','best'); axis tight;
    end
    sgtitle('GP Validation: Predicted vs. Actual on Unseen Data');
end

%% GP prediction performance visualization
function visualizeGPPredictions(gp_models, data, I, p)
    n_outputs=length(I.states); state_names={'e_{x}','e_{y}','e_{v}','e_{\psi}','e_{\delta}'};
    num_vis_steps=min(200,size(data.X,1)); vis_idx=1:num_vis_steps;
    X_vis=data.X(vis_idx,:); Y_true=data.Y(vis_idx,:);
    Y_pred=zeros(size(Y_true)); Y_std=zeros(size(Y_true));
    for i=1:n_outputs
        X_vis_norm = gpFunctions('normalize', X_vis, gp_models{i}.X_mean, gp_models{i}.X_std);
        [mu,sigma2]=gpFunctions('gpPredict', gp_models{i}, X_vis_norm);
        Y_pred(:,i)=mu; Y_std(:,i)=sqrt(sigma2);
    end
    figure('Name','GP Predictions Over Time','Position',[150,150,1500,600]);
    time_vec=(0:num_vis_steps-1)*p.timeStep;
    for i=1:n_outputs
        subplot(2,3,i); hold on; grid on;
        fill([time_vec,fliplr(time_vec)],[Y_pred(:,i)'-2*Y_std(:,i)',fliplr(Y_pred(:,i)'+2*Y_std(:,i)')],...
            [0.8 0.8 0.8],'EdgeColor','none','FaceAlpha',0.5);
        plot(time_vec,Y_true(:,i),'r-','LineWidth',2);
        plot(time_vec,Y_pred(:,i),'b--','LineWidth',2);
        xlabel('Time [s]'); ylabel('State Error'); title(sprintf('Prediction for %s', state_names{i}));
        legend('95% Confidence','True Error','GP Prediction','Location','best');
        xlim([time_vec(1), time_vec(end)]);
    end
    sgtitle('GP Prediction of Model Error vs. True Error Over Time');
end

%% Results comparison visualization
function visualizeAndCompareResults(p, I, reference_path, results_gp, results_base)
    figure('Name', 'GP-MPC vs Baseline-MPC Performance Comparison', 'Position', [100, 100, 1600, 800]);
    
    % 1. Trajectory Plot
    subplot(2,2,1);
    plot(reference_path.x,reference_path.y,'k--','DisplayName','Reference'); hold on; grid on;
    plot(results_base.states(1,:),results_base.states(2,:),'r-','LineWidth',2,'DisplayName',results_base.name);
    plot(results_gp.states(1,:),results_gp.states(2,:),'b-','LineWidth',2,'DisplayName',results_gp.name);
    title('Trajectory Comparison'); xlabel('X [m]'); ylabel('Y [m]'); axis equal; legend('Location','southeast');
    
    % 2. Lateral Error Plot
    ax2 = subplot(2,2,2);
    lat_err_base = calculateLateralError(results_base.states, reference_path);
    lat_err_gp = calculateLateralError(results_gp.states, reference_path);
    time_vec = (0:p.simLength) * p.timeStep;
    plot(time_vec, lat_err_base, 'r-', 'LineWidth', 2, 'DisplayName', sprintf('Baseline (RMSE: %.3fm)', rms(lat_err_base))); hold on; grid on;
    plot(time_vec, lat_err_gp, 'b-', 'LineWidth', 2, 'DisplayName', sprintf('GP-MPC (RMSE: %.3fm)', rms(lat_err_gp)));
    title('Lateral Error Comparison');
    xlabel('Time [s]'); ylabel('Lateral Error [m]'); legend;

    % 3. Velocity Plot
    ax3 = subplot(2,2,3);
    plot(time_vec, results_base.states(3,:),'r-','LineWidth',2,'DisplayName',results_base.name); hold on; grid on;
    plot(time_vec, results_gp.states(3,:),'b-','LineWidth',2,'DisplayName',results_gp.name);
    title('Velocity Profile'); xlabel('Time [s]'); ylabel('Velocity [m/s]'); legend;

    % 4. Steering Input Plot
    ax4 = subplot(2,2,4);
    time_vec_input = (0:p.simLength-1) * p.timeStep;
    yyaxis left;
    plot(time_vec_input,rad2deg(results_base.inputs(I.steeringRate,:)),'r-','DisplayName','Base Rate'); hold on;
    plot(time_vec_input,rad2deg(results_gp.inputs(I.steeringRate,:)),'b-','DisplayName','GP Rate');
    ylabel('Steering Rate [deg/s]');
    
    yyaxis right;
    plot(time_vec,rad2deg(results_base.states(I.steeringAngle-2,:)),'r:','DisplayName','Base Angle');
    plot(time_vec,rad2deg(results_gp.states(I.steeringAngle-2,:)),'b:','DisplayName','GP Angle');
    ylabel('Steering Angle [deg]');
    
    title('Steering Input Comparison'); xlabel('Time [s]'); grid on; legend('Location','best');

    linkaxes([ax2, ax3, ax4], 'x');
    xlim(ax2, [0, p.simLength * p.timeStep]);
end

%% Lateral error calculation
function lat_err_vec = calculateLateralError(states, reference_path)
    lat_err_vec = zeros(1, size(states, 2));
    for i=1:size(states,2)
        [~,idx]=min((reference_path.x-states(1,i)).^2+(reference_path.y-states(2,i)).^2);
        vec_track_to_car=[states(1,i)-reference_path.x(idx); states(2,i)-reference_path.y(idx)];
        track_angle=reference_path.heading(idx);
        normal_vec=[-sin(track_angle); cos(track_angle)];
        lat_err_vec(i)=dot(vec_track_to_car,normal_vec);
    end
end
