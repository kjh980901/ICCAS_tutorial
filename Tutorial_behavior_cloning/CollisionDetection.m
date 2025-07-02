function collision = CollisionDetection(rect1, rect2)
% Fast collision detection using Separating Axis Theorem (SAT)
% Input: rect1, rect2 - 4x2 matrices containing corner coordinates
% Output: collision - true if rectangles intersect, false otherwise

% Quick bounding box check first
min1 = min(rect1);
max1 = max(rect1);
min2 = min(rect2);
max2 = max(rect2);

% If bounding boxes don't overlap, no collision
if max1(1) < min2(1) || min1(1) > max2(1) || ...
   max1(2) < min2(2) || min1(2) > max2(2)
    collision = false;
    return;
end

% SAT algorithm for precise collision detection
% Get edges of both rectangles
edges1 = [rect1(2,:) - rect1(1,:);
          rect1(4,:) - rect1(1,:)];
edges2 = [rect2(2,:) - rect2(1,:);
          rect2(4,:) - rect2(1,:)];

% Test all 4 axes (2 from each rectangle)
axes = [edges1; edges2];

for i = 1:4
    % Get perpendicular axis
    axis = [-axes(i,2), axes(i,1)];
    
    % Skip zero-length axis
    if norm(axis) == 0
        continue;
    end
    
    % Normalize axis
    axis = axis / norm(axis);
    
    % Project all vertices onto this axis
    proj1 = rect1 * axis';
    proj2 = rect2 * axis';
    
    % Check if projections overlap
    if max(proj1) < min(proj2) || max(proj2) < min(proj1)
        % Found a separating axis - no collision
        collision = false;
        return;
    end
end

% No separating axis found - collision detected
collision = true;
end