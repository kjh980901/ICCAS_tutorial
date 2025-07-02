function X_new = Motion_Update(X_old, car_id, action_id, t_step)
    global v_min v_max
    
    % Copy input to avoid modifying original
    X_new = X_old;
    
    % Extract current state for readability
    x = X_old(1, car_id);
    y = X_old(2, car_id);
    theta = X_old(3, car_id);
    v = X_old(4, car_id);
    
    % Define action parameters: [velocity_change, angular_velocity_change]
    action_params = [
        0,    0;      % 1: maintain
        0,    pi/4;   % 2: turn left
        0,   -pi/4;   % 3: turn right
        2.5,  0;      % 4: accelerate
       -2.5,  0;      % 5: decelerate
       -5,    0;      % 6: hard brake
    ];
    
    % Validate action_id
    if action_id < 1 || action_id > size(action_params, 1)
        warning('Invalid action_id: %d. No action taken.', action_id);
        return;
    end
    
    % Get action parameters
    dv = action_params(action_id, 1);
    dtheta = action_params(action_id, 2);
    
    % Update velocity and heading
    v_new = v + dv * t_step;
    theta_new = theta + dtheta * t_step;
    
    % Apply velocity constraints
    v_new = max(v_min, min(v_max, v_new));
    
    % Update position using new velocity and heading
    x_new = x + v_new * cos(theta_new) * t_step;
    y_new = y + v_new * sin(theta_new) * t_step;
    
    % Update state vector
    X_new(1, car_id) = x_new;
    X_new(2, car_id) = y_new;
    X_new(3, car_id) = theta_new;
    X_new(4, car_id) = v_new;
end