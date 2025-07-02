function initial_state = initializeVehicles(random_init)
global w_lane v_nominal

% Blue car (Car 1) - starts at bottom, goes north
x1 = 0.5*w_lane;
if random_init
    % Random y position between -5*w_lane and -3*w_lane
    y1 = -w_lane * (3 + 2*rand());
else
    y1 = -4*w_lane;
end
theta1 = pi/2;
v1 = v_nominal;
target1 = 2;  % north

% Red car (Car 2) - starts at top, goes east  
x2 = -0.5*w_lane;
if random_init
    % Random y position between 3*w_lane and 5*w_lane
    y2 = w_lane * (3 + 2*rand());
else
    y2 = 4*w_lane;
end
theta2 = -pi/2;
v2 = v_nominal;
target2 = 1;  % east

initial_state = [x1 x2; y1 y2; theta1 theta2; v1 v2; target1 target2];
end