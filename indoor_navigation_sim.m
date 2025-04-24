clc; clear; close all;

%% Setup
gridSize = [10, 10];
map = zeros(gridSize);
enableDynamic = true;

%% Random Start and Goal in opposite corners
cornerOptions = {
    [10, 1; 1, 10];
    [1, 1; 10, 10];
    [10, 10; 1, 1];
    [1, 10; 10, 1];
};
pair = cornerOptions{randi(4)};
startPos = pair(1, :);
goalPos  = pair(2, :);

%% Generate obstacles
map = zeros(gridSize);
numObstacles = randi([2, 4]);
obstacleShapes = {};
occupiedMask = zeros(gridSize);
attemptLimit = 20;

for i = 1:numObstacles
    placed = false; attempts = 0;
    while ~placed && attempts < attemptLimit
        shapeType = randi([1, 3]);
        x = randi([1, 7]); y = randi([1, 7]);
        shape = struct(); shape.type = shapeType; shape.origin = [x, y];
        tempMap = zeros(gridSize);

        if shapeType == 1
            yR = y:y+2; xR = x:x+2;
            if max(yR) > gridSize(1) || max(xR) > gridSize(2), attempts = attempts + 1; continue; end
            tempMap(yR, xR) = 1; shape.size = [3, 3];

        elseif shapeType == 2
            if rand < 0.5
                yR = y:y+1; xR = x:x+3; shape.size = [2, 4];
            else
                yR = y:y+3; xR = x:x+1; shape.size = [4, 2];
            end
            if max(yR) > gridSize(1) || max(xR) > gridSize(2), attempts = attempts + 1; continue; end
            tempMap(yR, xR) = 1;

        elseif shapeType == 3
            [X, Y] = meshgrid(1:gridSize(2), 1:gridSize(1));
            cx = x + 1; cy = y + 1; r = 2;
            mask = (X - cx).^2 + (Y - cy).^2 <= r^2;
            tempMap(mask) = 1; shape.center = [cx, cy]; shape.radius = r;
        end

        if ~any(tempMap(:) & occupiedMask(:))
            map = map | tempMap;
            occupiedMask = occupiedMask | tempMap;
            obstacleShapes{end+1} = shape;
            placed = true;
        end
        attempts = attempts + 1;
    end
end

map(startPos(1), startPos(2)) = 0;
map(goalPos(1), goalPos(2)) = 0;

%% Initial Path
[path, success] = aStarPathPlanner(map, startPos, goalPos);
if ~success, error('No valid path found!'); end

% Place dynamic obstacle
midIndex = round(size(path, 1) / 2);
dynamicObstacle = path(midIndex, :);

%% State variables
moveState = 1;                
trail = [];                   
i = 1;                        
maxRetries = 1;
retryCounter = 0;

%% Simulation loop
while i <= size(path, 1)
    currentPos = path(i, :);

    if i < size(path, 1)
        nextPos = path(i+1, :);
    else
        nextPos = currentPos;
    end

    %% Pre-move collision prediction
    isBlocked = isequal(dynamicObstacle, currentPos) || isequal(dynamicObstacle, nextPos);

    if isBlocked
        fprintf('Replanning triggered BEFORE moving: robot [%d,%d] wants [%d,%d], obstacle at [%d,%d]\n', ...
            currentPos(1), currentPos(2), nextPos(1), nextPos(2), dynamicObstacle(1), dynamicObstacle(2));

        % Temporarily mark obstacle position as blocked
        map(dynamicObstacle(1), dynamicObstacle(2)) = 1;
        [newPath, success] = aStarPathPlanner(map, currentPos, goalPos);
        map(dynamicObstacle(1), dynamicObstacle(2)) = 0;

        if success && ~any(ismember(newPath, dynamicObstacle, 'rows'))
            fprintf('New valid path found — switching path.\n');
            path = newPath;
            i = 1;
            retryCounter = 0;
            continue;
        else
            retryCounter = retryCounter + 1;

            if retryCounter >= maxRetries
                retryCounter = 0;
                i = max(i - 1, 1);  % backtrack or stay
            end

            drawGrid(gridSize, map, startPos, goalPos, trail, dynamicObstacle, currentPos, obstacleShapes);
            pause(0.2);
            continue;
        end
    end

    %% Safe move
    trail = [trail; currentPos];
    i = i + 1;
    retryCounter = 0; 

    %% Dynamic Obstacle Patrol
    switch moveState
        case 1, step = [1, 0];   
        case 2, step = [-1, 0];  
        case 3, step = [0, 1];   
        case 4, step = [0, -1];  
    end

    newPos = dynamicObstacle + step;
    if all(newPos > 0) && all(newPos <= gridSize) && ...
       map(newPos(1), newPos(2)) == 0 && ~isequal(newPos, currentPos)
        dynamicObstacle = newPos;
    else
        moveState = mod(moveState, 4) + 1;
    end

    %% Update Visualization
    drawGrid(gridSize, map, startPos, goalPos, trail, dynamicObstacle, currentPos, obstacleShapes);
    pause(0.2);
end

disp("Robot reached the goal!");
