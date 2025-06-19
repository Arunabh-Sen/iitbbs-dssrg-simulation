clc; clear;

%% PARAMETERS
numGS = 5;
initialDronesPerGS = 2;
requestInterval = 2;
chargingTime = 1;
simDuration = 120;
droneSpeed = 1;
boxSize = 30;
GS_zLevel = 0;         % base GS Z
slotCount = 3;         % number of z-axis slots per GS
slotHeightStep = 2;  % vertical gap between slots
slotReservationTime = 5; % seconds slot is reserved

%% GENERATE CONNECTED GS POSITIONS WITH MIN DISTANCE
GS_positions = zeros(numGS, 3);
GS_positions(1,1:2) = rand(1,2) * boxSize;  % First GS at random

for i = 2:numGS
    placed = false;
    attempt = 0;
    while ~placed && attempt < 1000
        candidate = rand(1,2) * boxSize;
        dists = vecnorm(GS_positions(1:i-1,1:2) - candidate, 2, 2);

        if all(dists >= 10) && any(dists <= 20)
            GS_positions(i,1:2) = candidate;
            placed = true;
        end
        attempt = attempt + 1;
    end
    if ~placed
        error('Failed to place GS %d with required minDist and connectivity.', i);
    end
end
GS_positions(:,3) = GS_zLevel;  % All base Z levels same

%% CREATE ADJACENCY MATRIX TO RESTRICT EDGES
adjacency = zeros(numGS);

% Connect each GS to its next two neighbors (circularly)
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

%% SLOT RESERVATION TRACKING
% For each GS, keep track of the end reservation time of each slot
GS_slotReservationEndTime = zeros(numGS, slotCount);

%% LOAD STL MODELS
droneStl = stlread('new_dronev_whole.stl');
F_drone = droneStl.ConnectivityList;
V_drone0 = (droneStl.Points - mean(droneStl.Points)) * 0.005;

gsStl = stlread('Monpoc_Apartments.stl');
F_gs = gsStl.ConnectivityList;
V_gs0 = (gsStl.Points - mean(gsStl.Points)) * 0.06;

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
            'travelTime', 0, ...
            'slotAssigned', 0, ...       % Assigned z-slot index
            'flightZ', GS_zLevel ...     % Current flight Z coordinate
        );
    end
end
activeDrones = struct('pos', {}, 'state', {}, 'path', {}, 'hop', {}, ...
                      'timeInState', {}, 'startPos', {}, 'endPos', {}, 'travelTime', {}, ...
                      'slotAssigned', {}, 'flightZ', {});

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

    % Draw GS with labels below them
    for g = 1:numGS
        V_shift = V_gs0 + GS_positions(g,:);
        patch('Faces', F_gs, 'Vertices', V_shift, ...
              'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        % Label GS number below
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3)-1, ...
            sprintf('GS-%d', g), 'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', 'b');
    end

    % Generate new requests every requestInterval seconds
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
                continue;
            end
            drone.path = path;

            drone.state = "charging";
            drone.hop = 1;
            drone.timeInState = 0;
            drone.pos = GS_positions(src,:);
            drone.startPos = [];
            drone.endPos = [];
            drone.travelTime = 0;
            drone.slotAssigned = 0;   % No slot assigned yet
            drone.flightZ = GS_zLevel; % default base Z

            activeDrones(end+1) = drone;

            fprintf('Time %d: New request from GS %d to GS %d, path: %s\n', ...
                    t, src, dest, mat2str(drone.path));
        end
    end

    toRemove = [];
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        drone.timeInState = drone.timeInState + 1;

        switch drone.state
            case "charging"
                % Check if charging complete and try assign slot
                if drone.timeInState >= chargingTime
                    currentGS = drone.path(drone.hop);
                    nextGS = drone.path(drone.hop + 1);

                    % Check for free slot at currentGS
                    slotsFree = find(GS_slotReservationEndTime(currentGS,:) <= t);

                    if isempty(slotsFree)
                        % No free slot, drone must wait longer charging
                        % Just keep charging and retry next iteration
                    else
                        % Assign the first free slot
                        slotID = slotsFree(1);
                        drone.slotAssigned = slotID;

                        % Reserve the slot for slotReservationTime seconds from now
                        GS_slotReservationEndTime(currentGS, slotID) = t + slotReservationTime;

                        % Calculate start and end positions for drone flight
                        startPos = GS_positions(currentGS,:); startPos(3) = GS_zLevel + (slotID-1)*slotHeightStep;
                        endPos = GS_positions(nextGS,:); endPos(3) = GS_zLevel + (slotID-1)*slotHeightStep;

                        drone.startPos = startPos;
                        drone.endPos = endPos;
                        drone.flightZ = startPos(3);

                        dist = norm(endPos(1:2) - startPos(1:2));
                        drone.travelTime = dist / droneSpeed;

                        drone.state = "moving";
                        drone.timeInState = 0;

                        fprintf('Time %d: Drone %d assigned slot %d at GS %d and started moving\n', ...
                            t, i, slotID, currentGS);
                    end

                end

            case "moving"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                % Fix Z to assigned flightZ slot height
                drone.pos(3) = drone.flightZ;
                if frac >= 1
                    drone.hop = drone.hop + 1;
                    if drone.hop == length(drone.path)
                        drone.state = "done";
                        drone.timeInState = 0;
                    else
                        drone.state = "charging";
                        drone.timeInState = 0;
                        % Position drone at GS for charging (at assigned slot height)
                        currentGS = drone.path(drone.hop);
                        drone.pos = GS_positions(currentGS,:);
                        drone.pos(3) = GS_zLevel + (drone.slotAssigned-1)*slotHeightStep;
                        drone.flightZ = drone.pos(3); % keep same slot height throughout
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
        str = sprintf('DR-%d: GS %d -> GS %d, Slot %d', i, srcGS, destGS, drone.slotAssigned);
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