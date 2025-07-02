function offroad = RoadBoundaryCheck(car_pos, car_theta, w_lane)
% Fast road boundary check for intersection scenario
% Input: car_pos - [x, y] position
%        car_theta - orientation angle
%        w_lane - lane width
% Output: offroad - penalty value (0 if on road, negative if off road)

x = car_pos(1);
y = car_pos(2);
offroad = 0;
Off_road_Penalty = -5000;

% Check if car is in valid road area
% The road has a cross shape: vertical and horizontal lanes

% Check vertical lanes (north-south)
in_vertical_lane = abs(x) <= w_lane && (y >= -8*w_lane && y <= 8*w_lane);

% Check horizontal lanes (east-west)
in_horizontal_lane = abs(y) <= w_lane && (x >= -8*w_lane && x <= 8*w_lane);

% Check intersection area
in_intersection = abs(x) <= w_lane && abs(y) <= w_lane;

% If not in any valid area, apply penalty
if ~(in_vertical_lane || in_horizontal_lane || in_intersection)
    offroad = Off_road_Penalty;
end

end