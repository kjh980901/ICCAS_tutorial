function R = Reward(X_old, car_id, action_id)

global w_lane l_car w_car

% Initialize penalties
Off_road = 0;
Colli = 0;
Safe = 0;
Complete = 0;

% Penalty values
Colli_Penalty = -10000;
Safe_Penalty = -1000;
Complete_Penalty = -50;
Off_road_Penalty = -300;

% Get car position and orientation
car_pos = X_old(1:2, car_id);
car_theta = X_old(3, car_id);

%% 1. Off-road penalty
% RoadBoundary violation penalty
Off_road = RoadBoundaryCheck(car_pos, car_theta, w_lane);

% Midline violation penalty
Off_road = Off_road + MidlineViolationCheck(car_pos, car_theta, w_lane);

% Target point violation penalty
if car_id == 1 && car_pos(1) < 0 && abs(car_pos(2)) < 6*w_lane
    Off_road = Off_road + Off_road_Penalty;
elseif car_id == 2
    % Check if in prohibited zone
    if (car_pos(1) > -2*w_lane && car_pos(1) < w_lane && ...
        car_pos(2) > -2*w_lane && car_pos(2) < w_lane) || ...
       (car_pos(1) > -6*w_lane && car_pos(1) < w_lane && ...
        car_pos(2) < -2*w_lane)
        Off_road = Off_road + Off_road_Penalty;
    end
end

%% 2. Collision penalty
% Build ego car rectangle
l_car_safe = 1.1*l_car;
w_car_safe = 1.1*w_car;
Ego_rectangle = getCarRectangle(car_pos, car_theta, l_car_safe, w_car_safe);

% Check collision with other cars
for id = 1:length(X_old(1,:))
    if id ~= car_id
        other_pos = X_old(1:2, id);
        other_theta = X_old(3, id);
        Other_rectangle = getCarRectangle(other_pos, other_theta, l_car_safe, w_car_safe);
        
        if CollisionDetection(Ego_rectangle, Other_rectangle)
            Colli = Colli + Colli_Penalty;
        end
    end
end

%% 3. Safe zone violation penalty
% Use larger safety margin
l_car_safe_zone = 2*l_car;
w_car_safe = 1.2*w_car;

% Create asymmetric safety rectangle
Ego_safe_rect = getCarRectangle(car_pos, car_theta, l_car_safe_zone, w_car_safe);

% Check safety violations with other cars
for id = 1:length(X_old(1,:))
    if id ~= car_id
        other_pos = X_old(1:2, id);
        other_theta = X_old(3, id);
        Other_safe_rect = getCarRectangle(other_pos, other_theta, l_car_safe_zone, w_car_safe);

        
        if CollisionDetection(Ego_safe_rect, Other_safe_rect)
            Safe = Safe + Safe_Penalty;
        end
    end
end

%% 4. Completion reward
target = X_old(5, car_id);
switch target
    case 1 % Going east
        if car_pos(1) < 6*w_lane
            Complete = Complete + Complete_Penalty * abs(6*w_lane - car_pos(1));
            Complete = Complete + 2*Complete_Penalty * abs(-w_lane/2 - car_pos(2));
        end
        
    case 2 % Going north
        if car_pos(2) < 6*w_lane
            Complete = Complete + Complete_Penalty * abs(6*w_lane - car_pos(2));
            Complete = Complete + 2*Complete_Penalty * abs(w_lane/2 - car_pos(1));
        end
        
    case 3 % Going west
        if car_pos(1) > -6*w_lane
            Complete = Complete + Complete_Penalty * abs(-6*w_lane - car_pos(1));
            Complete = Complete + 2*Complete_Penalty * abs(w_lane/2 - car_pos(2));
        end
        
    case 4 % Going south
        if car_pos(2) > -6*w_lane
            Complete = Complete + Complete_Penalty * abs(-6*w_lane - car_pos(2));
            Complete = Complete + 2*Complete_Penalty * abs(-w_lane/2 - car_pos(1));
        end
end

% Total reward
R = Off_road + Colli + Safe + Complete;

end
