clc; clear;

%% PARAMETERS
numGS = 5;
initialDronesPerGS = 3;
requestInterval = 2;
chargingTime = 10;
simDuration = 120;
droneSpeed = 1.0;
boxSize = 30;
GS_zLevel = 0;
flightZ = 3;
minDist = 8;

%% GENERATE CONNECTED GS POSITIONS WITH MIN DISTANCE
GS_positions = zeros(numGS, 3);
GS_positions(1,1:2) = rand(1,2) * boxSize;  % First GS at random

for i = 2:numGS
    placed = false;
    attempt = 0;
    while ~placed && attempt < 1000
        candidate = rand(1,2) * boxSize;
        dists = vecnorm(GS_positions(1:i-1,1:2) - candidate, 2, 2);

        % Check minDist to all existing GS and connectivity to at least one GS
        if all(dists >= minDist) && any(dists <= minDist * 2)
            GS_positions(i,1:2) = candidate;
            placed = true;
        end
        attempt = attempt + 1;
    end
    if ~placed
        error('Failed to place GS %d with required minDist and connectivity.', i);
    end
end
GS_positions(:,3) = GS_zLevel;  % All Z levels same

%% CREATE ADJACENCY MATRIX TO RESTRICT EDGES
adjacency = zeros(numGS);

% Example: Connect each GS only to its next two neighbors (circularly)
for i = 1:numGS
    adjacency(i, mod(i, numGS) + 1) = 1;
    adjacency(i, mod(i+1, numGS) + 1) = 1;
end

%% BUILD DISTANCE GRAPH BASED ON ADJACENCY
distGraph = inf(numGS);

for i = 1:numGS
    for j = 1:numGS
        if adjacency(i,j) == 1
            distGraph(i,j) = norm(GS_positions(i,1:2) - GS_positions(j,1:2));
        end
    end
end

%% LOAD STL MODELS
droneStl = stlread('new_dronev_whole.stl');
F_drone = droneStl.ConnectivityList;
V_drone0 = (droneStl.Points - mean(droneStl.Points)) * 0.005;

gsStl = stlread('Monpoc_Apartments.stl');
F_gs = gsStl.ConnectivityList;
V_gs0 = (gsStl.Points - mean(gsStl.Points)) * 0.05;

%% INITIALIZE DRONES
dronePool = cell(numGS,1);
for g=1:numGS
    for d=1:initialDronesPerGS
        dronePool{g}(end+1) = struct( ...
            'pos', GS_positions(g,:), ...
            'state', "idle", ...
            'path', [], ...
            'hop', 1, ...
            'timeInState', 0, ...
            'startPos', [], ...
            'endPos', [], ...
            'travelTime', 0 ...
        );
    end
end
activeDrones = struct('pos', {}, 'state', {}, 'path', {}, 'hop', {}, ...
                      'timeInState', {}, 'startPos', {}, 'endPos', {}, 'travelTime', {});

% Store all drones that flew during simulation (to log paths at end)
allDrones = struct('path', {});

%% FIGURE SETUP
figure;
axis([0 boxSize 0 boxSize 0 boxSize]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on; hold on; camlight; lighting gouraud;

%% SIMULATION LOOP
for t=0:simDuration
    cla;
    title(['Time: ' num2str(t) ' sec']);
    axis([0 boxSize 0 boxSize 0 boxSize]);

    for g = 1:numGS
        V_shift = V_gs0 + GS_positions(g,:);
        patch('Faces', F_gs, 'Vertices', V_shift, ...
              'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);

        % Add GS label below the GS
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3) - 1.5, ...
             sprintf('GS-%d', g), 'FontSize', 10, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', 'b');
    end

    if mod(t, requestInterval) == 0
        src = randi(numGS);
        dest = randi(numGS);
        while dest == src
            dest = randi(numGS);
        end
        if ~isempty(dronePool{src})
            drone = dronePool{src}(end);
            dronePool{src}(end) = [];

            % Shortest path using Dijkstra on adjacency-restricted graph
            path = dijkstra(distGraph, src, dest);
            if isempty(path)
                fprintf('Time %d: No valid path from GS %d to GS %d\n', t, src, dest);
                continue;  % Skip if no path found
            end
            drone.path = path;

            drone.state = "charging";
            drone.hop = 1;
            drone.timeInState = 0;
            drone.pos = GS_positions(src,:);
            drone.startPos = [];
            drone.endPos = [];
            drone.travelTime = 0;

            activeDrones(end+1) = drone;

            fprintf('Time %d: New request generated from GS %d to GS %d with path: %s\n', ...
                    t, src, dest, mat2str(drone.path));
        end
    end

    toRemove = [];
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        drone.timeInState = drone.timeInState + 1;

        switch drone.state
            case "charging"
                if drone.timeInState >= chargingTime
                    currentGS = drone.path(drone.hop);
                    nextGS = drone.path(drone.hop + 1);
                    startPos = GS_positions(currentGS,:); startPos(3) = flightZ;
                    endPos = GS_positions(nextGS,:); endPos(3) = flightZ;
                    drone.startPos = startPos;
                    drone.endPos = endPos;
                    dist = norm(endPos(1:2) - startPos(1:2));
                    drone.travelTime = dist / droneSpeed;
                    drone.state = "moving"; drone.timeInState = 0;
                end

            case "moving"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                drone.pos(3) = flightZ;
                if frac >= 1
                    drone.hop = drone.hop + 1;
                    if drone.hop == length(drone.path)
                        drone.state = "done"; drone.timeInState = 0;
                    else
                        drone.state = "charging"; drone.timeInState = 0;
                        drone.pos = GS_positions(drone.path(drone.hop), :);
                    end
                end

            case "done"
                % Save drone path in allDrones for final logging
                allDrones(end+1).path = drone.path;
                toRemove(end+1) = i;
        end

        activeDrones(i) = drone;
        V_shift = V_drone0 + drone.pos;
        patch('Faces', F_drone, 'Vertices', V_shift, ...
              'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        text(drone.pos(1), drone.pos(2), drone.pos(3) + 2.0, ...
             sprintf('DR-%d', i), 'FontSize', 8, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', 'r');
    end
    activeDrones(toRemove) = [];

    % Display info on the side
    infoX = boxSize + 2;  
    infoY = boxSize - 1;  
    infoZ = boxSize - 2;

    text(infoX, infoY, infoZ, 'Active Drones:', ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'Color', 'k');
    infoZ = infoZ - 2;

    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        srcGS = drone.path(drone.hop);
        if drone.hop < length(drone.path)
            destGS = drone.path(drone.hop + 1);
        else
            destGS = srcGS;
        end
        str = sprintf('DR-%d: GS %d -> GS %d', i, srcGS, destGS);
        text(infoX, infoY, infoZ, str, 'FontSize', 9, ...
            'HorizontalAlignment', 'left', 'Color', 'k');
        infoZ = infoZ - 1.2;
    end

    drawnow;
    pause(0.05);
end

%% PRINT FINAL PATHS OF ALL DRONES THAT FLEW
fprintf('\n--- Final Paths of All Completed Drones ---\n');
for i = 1:length(allDrones)
    fprintf('Drone %d path: %s\n', i, mat2str(allDrones(i).path));
end

%% DIJKSTRA FUNCTION FOR SHORTEST PATH
function path = dijkstra(graph, startNode, endNode)
    numNodes = size(graph, 1);
    visited = false(1, numNodes);
    dist = inf(1, numNodes);
    prev = zeros(1, numNodes);
    dist(startNode) = 0;

    while true
        unvisited = find(~visited);
        if isempty(unvisited)
            break;
        end
        [~, idx] = min(dist(unvisited));
        u = unvisited(idx);
        if isinf(dist(u)) || u == endNode
            break;
        end
        visited(u) = true;

        neighbors = find(graph(u,:) < inf);
        for v = neighbors
            alt = dist(u) + graph(u,v);
            if alt < dist(v)
                dist(v) = alt;
                prev(v) = u;
            end
        end
    end

    if isinf(dist(endNode))
        path = [];  % No path found
    else
        path = endNode;
        while path(1) ~= startNode
            if prev(path(1)) == 0
                path = []; return;
            end
            path = [prev(path(1)), path];
        end
    end
end
