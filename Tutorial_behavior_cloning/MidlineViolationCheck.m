function penalty = MidlineViolationCheck(car_pos, car_theta, w_lane)
% Fast midline violation check based on car position and direction
% Input: car_pos - [x, y] position
%        car_theta - orientation angle
%        w_lane - lane width
% Output: penalty - penalty value (0 if no violation, negative if violation)

x = car_pos(1);
y = car_pos(2);
penalty = 0;
Off_Mid_Penalty = -500;

% Get direction vector
dir_vec = [cos(car_theta); sin(car_theta)];

% Check which lane the car is in and its direction
para = 2; % Parameter for midline zone

% Vertical lanes
if abs(x) < w_lane && abs(y) > para*w_lane
    % Check if car is in north-south lane
    if y > 0 % Northern part
        % Should be going north (positive y direction)
        if dir_vec(2) < 0 % Going south - wrong way
            penalty = Off_Mid_Penalty;
        end
    else % Southern part
        % Should be going south (negative y direction)
        if dir_vec(2) > 0 % Going north - wrong way
            penalty = Off_Mid_Penalty;
        end
    end
end

% Horizontal lanes
if abs(y) < w_lane && abs(x) > para*w_lane
    % Check if car is in east-west lane
    if x > 0 % Eastern part
        % Should be going east (positive x direction)
        if dir_vec(1) < 0 % Going west - wrong way
            penalty = Off_Mid_Penalty;
        end
    else % Western part
        % Should be going west (negative x direction)
        if dir_vec(1) > 0 % Going east - wrong way
            penalty = Off_Mid_Penalty;
        end
    end
end

end