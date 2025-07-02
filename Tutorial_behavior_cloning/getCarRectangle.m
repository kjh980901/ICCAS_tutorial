
%% Helper functions
function rect = getCarRectangle(pos, theta, l_safe, w_safe)
% Get 4 corners of car rectangle
x = pos(1);
y = pos(2);

% Half dimensions
hl = l_safe / 2;
hw = w_safe / 2;

% Rotation matrix components
c = cos(theta);
s = sin(theta);

% Four corners (counterclockwise from rear-left)
rect = [x - hl*c + hw*s, y - hl*s - hw*c;   % Rear-left
        x - hl*c - hw*s, y - hl*s + hw*c;   % Rear-right
        x + hl*c - hw*s, y + hl*s + hw*c;   % Front-right
        x + hl*c + hw*s, y + hl*s - hw*c];  % Front-left
end
