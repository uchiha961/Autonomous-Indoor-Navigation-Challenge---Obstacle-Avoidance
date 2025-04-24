function [path, success] = aStarPathPlanner(map, startPos, goalPos)
    [rows, cols] = size(map);
    openList = [];
    closedList = zeros(rows, cols);
    parent = zeros(rows, cols, 2);

    g = inf(rows, cols);
    g(startPos(1), startPos(2)) = 0;
    h = @(x,y) abs(goalPos(1)-x) + abs(goalPos(2)-y);
    f = g;
    f(startPos(1), startPos(2)) = h(startPos(1), startPos(2));

    openList = [f(startPos(1), startPos(2)), 0, startPos];

    directions = [0 1; 1 0; 0 -1; -1 0];  % Right, Down, Left, Up

    while ~isempty(openList)
        [~, idx] = min(openList(:,1));
        current = openList(idx, 3:4);
        openList(idx,:) = [];

        if isequal(current, goalPos)
            path = reconstructPath(parent, current);
            success = true;
            return;
        end

        closedList(current(1), current(2)) = 1;

        for i = 1:size(directions,1)
            neighbor = current + directions(i,:);
            x = neighbor(1); y = neighbor(2);

            isStart = (x == startPos(1) && y == startPos(2));

            if x < 1 || x > rows || y < 1 || y > cols || (map(x,y)==1 && ~isStart) || closedList(x,y)
                continue;
            end

            newG = g(current(1), current(2)) + 1;

            if newG < g(x,y)
                parent(x,y,:) = current;
                g(x,y) = newG;
                f(x,y) = newG + h(x,y);
                openList = [openList; f(x,y), newG, x, y];
            end
        end
    end

    path = [];
    success = false;
end

function path = reconstructPath(parent, current)
    path = current;
    while any(parent(current(1), current(2), :))
        current = squeeze(parent(current(1), current(2), :))';
        path = [current; path];
    end
end
