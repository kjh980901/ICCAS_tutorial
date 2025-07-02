function varargout = pathFunctions(functionName, varargin)
%% Path and Track Functions
% Collection of path planning and track definition functions for GP-MPC Tutorial
%
% Usage:
%   map = pathFunctions('defineRacingTrack', mapName)
%   map = pathFunctions('getEllipsePath')
%   map = pathFunctions('getIfARacetrack')
%   map = pathFunctions('planVelocity', map, accelLatMax, velMax)
%   arclengthLocalization = pathFunctions('localizeVehicleOnPath', xVehicle, yVehicle, map)
%   pointsToTrack = pathFunctions('resamplePathForTracker', stageVar, I, map, timeStep, horizonLength)

switch functionName
    case 'defineRacingTrack'
        varargout{1} = defineRacingTrack(varargin{:});
    case 'getEllipsePath'
        varargout{1} = getEllipsePath();
    case 'getIfARacetrack'
        varargout{1} = getIfARacetrack();
    case 'planVelocity'
        varargout{1} = planVelocity(varargin{:});
    case 'localizeVehicleOnPath'
        varargout{1} = localizeVehicleOnPath(varargin{:});
    case 'resamplePathForTracker'
        varargout{1} = resamplePathForTracker(varargin{:});
    otherwise
        error('Unknown function name: %s', functionName);
end

end

%% Racing path definition function
function map = defineRacingTrack(mapName)
    if strcmp(mapName,'ellipse')
        map = getEllipsePath();
    elseif strcmp(mapName,'racetrack')
        map = getIfARacetrack();
    else
        error('Specified map type does not exist!');
    end
    
    % Plan target velocity profile for the path
    accelLatMax = 3; % Maximum lateral acceleration [m/s^2]
    velMax = 5;      % Maximum velocity [m/s]
    map = planVelocity(map, accelLatMax, velMax);
end

function [ map ] = getEllipsePath()
    numPoints = 200;
    dAngle = 2 * pi / numPoints;
    angleWaypoints = 0:dAngle:numPoints*dAngle;
    xWaypoints = 3.0*cos(angleWaypoints); % Adjust track size
    yWaypoints = 5.0*sin(angleWaypoints);
    dArclength = hypot(diff(xWaypoints), diff(yWaypoints));
    arclengthWaypoints = [0,cumsum(dArclength)];
    headingWaypoints = atan2(gradient(yWaypoints), gradient(xWaypoints));
    curvatureWaypoints = wrapToPi(diff(headingWaypoints))./dArclength;
    map.x = xWaypoints; map.y = yWaypoints; map.arclength = arclengthWaypoints;
    map.heading = headingWaypoints; map.curvature = [curvatureWaypoints, curvatureWaypoints(1)];
end

function [ map ] = getIfARacetrack()
    data = load( fullfile('data','track2_IfA.mat') );
    scalingFactor = 7;
    xWaypoints = scalingFactor * data.track2.center(1,:);
    yWaypoints = scalingFactor * data.track2.center(2,:);
    dArclength = hypot(diff(xWaypoints), diff(yWaypoints));
    arclengthWaypoints = [0,cumsum(dArclength)];
    headingWaypoints = atan2(gradient(yWaypoints), gradient(xWaypoints));
    curvatureWaypoints = wrapToPi(diff(headingWaypoints))./dArclength;
    map.x = xWaypoints; map.y = yWaypoints; map.arclength = arclengthWaypoints;
    map.heading = headingWaypoints; map.curvature = [curvatureWaypoints, curvatureWaypoints(1)];
end

function [ map ] = planVelocity(map, accelLatMax, velMax)
    velFromLatAccel = sqrt(accelLatMax./abs(map.curvature));
    map.velocity = min(velFromLatAccel, velMax);
    velAvg = movmean(map.velocity,2);
    dt = diff(map.arclength)./velAvg(1:end-1);
    map.time = [0,cumsum(dt)];
end

function [ arclengthLocalization ] = localizeVehicleOnPath(xVehicle, yVehicle, map)
    distanceFromWaypoints = sqrt((xVehicle - map.x).^2 + (yVehicle - map.y).^2);
    [~, idxClosest] = min(distanceFromWaypoints);
    nElements = length(map.x);
    if idxClosest == 1
        idx1 = nElements-1; idx2 = 1;
    elseif idxClosest == nElements
        idx1 = nElements-1; idx2 = nElements;
    else
        if distanceFromWaypoints(mod(nElements + idxClosest - 2,nElements)+1) < distanceFromWaypoints(mod(nElements + idxClosest,nElements)+1)
            idx1 = mod(nElements + idxClosest - 2,nElements)+1 ;
            idx2 = idxClosest;
        else
            idx1 = idxClosest;
            idx2 = mod(nElements + idxClosest,nElements)+1;
        end
    end
    x1 = map.x(idx1); y1 = map.y(idx1); x2 = map.x(idx2); y2 = map.y(idx2);
    dx = x2-x1; dy = y2-y1; ds = hypot(dx,dy);
    if ds < 1e-4; lambda = 0;
    else; lambda = -(dx*(x1-xVehicle)+dy*(y1-yVehicle))/(dx^2+dy^2);
    end
    arclengthLocalization = mod(map.arclength(idx1) + lambda*ds,map.arclength(end));
end

function [ pointsToTrack ] = resamplePathForTracker(stageVar, ~, map, timeStep, horizonLength)
    arclengthLocalization = localizeVehicleOnPath(stageVar(1), stageVar(2), map);
    timeReference = interp1(map.arclength, map.time, arclengthLocalization, 'linear', 'extrap');
    timeHorizon = timeReference : timeStep : (timeReference + (horizonLength-1)*timeStep);
    timeHorizon = mod(timeHorizon, map.time(end));
    pointsToTrack(1,:) = interp1(map.time, map.x, timeHorizon, 'linear', 'extrap');
    pointsToTrack(2,:) = interp1(map.time, map.y, timeHorizon, 'linear', 'extrap');
end
