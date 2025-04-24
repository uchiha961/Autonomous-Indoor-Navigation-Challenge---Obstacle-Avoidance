function drawGrid(gridSize, map, startPos, goalPos, path, dynamicObstacle, robotPos, obstacleShapes)
    cla;
    hold on;
    axis equal;
    xlim([0 gridSize(2)]); ylim([0 gridSize(1)]);
    title('Autonomous Navigation - Dynamic World');
    xlabel('X'); ylabel('Y');
    set(gca, 'XTick', 0:gridSize(2), 'YTick', 0:gridSize(1));
    set(gca, 'YDir', 'reverse');

    %% Draw grid background
    for x = 1:gridSize(2)
        for y = 1:gridSize(1)
            rectangle('Position', [x-1 y-1 1 1], 'EdgeColor', [0.9 0.9 0.9]);
        end
    end

    %% Draw obstacles 
    shapeHandles = gobjects(length(obstacleShapes), 1);
    for i = 1:length(obstacleShapes)
        shape = obstacleShapes{i};

        if shape.type == 1  % square
            x = shape.origin(1)-1;
            y = shape.origin(2)-1;
            shapeHandles(i) = fill([x x+3 x+3 x], [y y y+3 y+3], 'r');

        elseif shape.type == 2  % rectangle
            x = shape.origin(1)-1;
            y = shape.origin(2)-1;
            w = shape.size(2);
            h = shape.size(1);
            shapeHandles(i) = fill([x x+w x+w x], [y y y+h y+h], 'r');

        elseif shape.type == 3  % circle
            theta = linspace(0, 2*pi, 40);
            cx = shape.center(1)-0.5;
            cy = shape.center(2)-0.5;
            r = shape.radius;
            shapeHandles(i) = fill(cx + r*cos(theta), cy + r*sin(theta), 'r');
        end
    end

    %% Start & Goal
    hStart = plot(startPos(2)-0.5, startPos(1)-0.5, 'go', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'g');
    hGoal = plot(goalPos(2)-0.5, goalPos(1)-0.5, 'bo', ...
                 'MarkerSize', 10, 'MarkerFaceColor', 'b');

    %% Path trail
    if nargin >= 5 && ~isempty(path)
        px = path(:,2) - 0.5;
        py = path(:,1) - 0.5;
        hTrail = plot(px, py, 'm--', 'LineWidth', 1.5);
    else
        hTrail = NaN;
    end

    %% Dynamic Obstacle
    if nargin >= 6 && ~isempty(dynamicObstacle)
        hDyn = plot(dynamicObstacle(2)-0.5, dynamicObstacle(1)-0.5, 'cs', ...
                    'MarkerSize', 12, 'MarkerFaceColor', 'c');
    else
        hDyn = NaN;
    end

    %% Robot
    if nargin >= 7 && ~isempty(robotPos)
        hRobot = plot(robotPos(2)-0.5, robotPos(1)-0.5, 'ko', ...
                      'MarkerSize', 10, 'MarkerFaceColor', 'k');
    else
        hRobot = NaN;
    end

    %% Legend
    legend([shapeHandles(1), hStart, hGoal, hTrail, hDyn, hRobot], ...
           {'Obstacle', 'Start', 'Goal', 'Trail', 'Dynamic', 'Robot'}, ...
           'Location', 'northeastoutside');
end
