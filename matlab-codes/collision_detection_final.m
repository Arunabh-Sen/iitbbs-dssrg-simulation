clc; clear;

%% PARAMETERS
numGS = 5;
initialDronesPerGS = 2;
requestInterval = 2;
chargingTime = 10;
simDuration = 120;
droneSpeed = 1;
verticalSpeed = 1;
boxSize = 30;
GS_zLevel = 3;
slotCount = 3;
slotHeightStep = 2;
slotReservationTime = 5;

%% Counters
totalRequests = 0;
collisionCount = 0;

%% GS positions
GS_positions = zeros(numGS, 3);
GS_positions(1,1:2) = rand(1,2) * boxSize;
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
    if ~placed, error('Failed to place GS %d.', i); end
end
GS_positions(:,3) = 0;

%% Adjacency and Graph (Proximity-Based Sparse Connections)
connThreshold = 20; % maximum distance to connect GS nodes
adjacency = zeros(numGS);
for i = 1:numGS
    for j = i+1:numGS
        dist = norm(GS_positions(i,1:2) - GS_positions(j,1:2));
        if dist <= connThreshold
            adjacency(i,j) = 1;
            adjacency(j,i) = 1; % undirected
        end
    end
end

distGraph = inf(numGS);
for i = 1:numGS
    for j = 1:numGS
        if adjacency(i,j) == 1
            distGraph(i,j) = norm(GS_positions(i,1:2) - GS_positions(j,1:2));
        end
    end
end
GS_slotReservationEndTime = zeros(numGS, slotCount);

%% Load Models
droneStl = stlread('new_dronev_whole.stl');
F_drone = droneStl.ConnectivityList;
V_drone0 = (droneStl.Points - mean(droneStl.Points)) * 0.005;
gsStl = stlread('Monpoc_Apartments.stl');
F_gs = gsStl.ConnectivityList;
V_gs0 = (gsStl.Points - mean(gsStl.Points)) * 0.06;

%% Init drones
dronePool = cell(numGS,1);
globalDroneID = 0;
for g=1:numGS
    for d=1:initialDronesPerGS
        globalDroneID = globalDroneID + 1;
        dronePool{g}(end+1) = struct( ...
            'id', globalDroneID, ...
            'pos', [GS_positions(g,1), GS_positions(g,2), GS_zLevel], ...
            'state', "idle", ...
            'path', [], ...
            'hop', 1, ...
            'timeInState', 0, ...
            'startPos', [], ...
            'endPos', [], ...
            'travelTime', 0, ...
            'slotAssigned', 0, ...
            'flightZ', GS_zLevel ...
        );
    end
end
activeDrones = struct('id', {}, 'pos', {}, 'state', {}, 'path', {}, 'hop', {}, ...
                      'timeInState', {}, 'startPos', {}, 'endPos', {}, 'travelTime', {}, ...
                      'slotAssigned', {}, 'flightZ', {}, 'startTime', {});
allDrones = struct('path', {}, 'totalTime', {});

%% Initialize collisionPairs matrix
maxDronesEstimate = 100; % adjust if needed
collisionPairs = false(maxDronesEstimate);

%% Simulation
figure;
axis([0 boxSize 0 boxSize 0 boxSize]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on; hold on; camlight; lighting gouraud;

for t=0:simDuration
    cla;
    title(['Time: ' num2str(t) ' sec']);
    axis([0 boxSize 0 boxSize 0 boxSize]);

    for g = 1:numGS
        V_shift = V_gs0 + GS_positions(g,:);
        patch('Faces', F_gs, 'Vertices', V_shift, ...
              'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3)-1, ...
            sprintf('GS-%d', g), 'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', 'b');
    end

    % Draw green lines on the ground between connected GS nodes
for i = 1:numGS
    for j = i+1:numGS
        if adjacency(i,j) == 1
            pt1 = GS_positions(i,1:2); % only X and Y
            pt2 = GS_positions(j,1:2);
            plot3([pt1(1), pt2(1)], [pt1(2), pt2(2)], [0, 0], ... % Z = 0 for both
                  'g-', 'LineWidth', 1);
        end
    end
end


    if mod(t, requestInterval) == 0
        src = randi(numGS); dest = randi(numGS);
        while dest == src, dest = randi(numGS); end
        if ~isempty(dronePool{src})
            drone = dronePool{src}(end); dronePool{src}(end) = [];
            path = dijkstra(distGraph, src, dest);
            if isempty(path), continue; end
            totalRequests = totalRequests + 1;
            drone.path = path;
            drone.state = "charging"; drone.hop = 1; drone.timeInState = 0;
            drone.pos = [GS_positions(src,1), GS_positions(src,2), GS_zLevel];
            drone.flightZ = GS_zLevel;
            drone.startTime = t;
            activeDrones(end+1) = drone;
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
                    freeSlots = find(GS_slotReservationEndTime(currentGS,:) <= t);
                    if ~isempty(freeSlots)
                        slotID = freeSlots(1);
                        drone.slotAssigned = slotID;
                        GS_slotReservationEndTime(currentGS, slotID) = t + slotReservationTime;
                        if slotID == 1
                            drone.flightZ = GS_zLevel;
                            drone.pos(3) = GS_zLevel;
                            drone.state = "moving"; drone.timeInState = 0;
                            drone.startPos = drone.pos;
                            drone.endPos = GS_positions(nextGS,:); drone.endPos(3) = drone.flightZ;
                            drone.travelTime = norm(drone.endPos(1:2) - drone.startPos(1:2)) / droneSpeed;
                        else
                            targetZ = GS_zLevel + (slotID - 1) * slotHeightStep;
                            drone.startPos = drone.pos;
                            drone.endPos = drone.pos; drone.endPos(3) = targetZ;
                            drone.travelTime = abs(targetZ - drone.pos(3)) / verticalSpeed;
                            drone.flightZ = targetZ;
                            drone.state = "ascending"; drone.timeInState = 0;
                        end
                    end
                end

            case "ascending"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                if frac >= 1
                    currentGS = drone.path(drone.hop);
                    nextGS = drone.path(drone.hop + 1);
                    drone.startPos = GS_positions(currentGS,:); drone.startPos(3) = drone.flightZ;
                    drone.endPos = GS_positions(nextGS,:); drone.endPos(3) = drone.flightZ;
                    drone.travelTime = norm(drone.endPos(1:2) - drone.startPos(1:2)) / droneSpeed;
                    drone.state = "moving"; drone.timeInState = 0;
                end

            case "moving"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                drone.pos(3) = drone.flightZ;
                if frac >= 1
                    drone.hop = drone.hop + 1;
                    if drone.hop == length(drone.path)
                        drone.startPos = drone.pos;
                        drone.endPos = drone.pos; drone.endPos(3) = GS_zLevel;
                        drone.travelTime = abs(drone.flightZ - GS_zLevel) / verticalSpeed;
                        drone.state = "descending"; drone.timeInState = 0;
                    else
                        drone.state = "descend_for_charge"; drone.timeInState = 0;
                        drone.startPos = drone.pos;
                        drone.endPos = drone.pos; drone.endPos(3) = GS_zLevel;
                        drone.travelTime = abs(drone.pos(3) - GS_zLevel) / verticalSpeed;
                    end
                end

            case "descend_for_charge"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                if frac >= 1
                    drone.state = "charging"; drone.timeInState = 0;
                    drone.flightZ = GS_zLevel;
                    drone.pos(3) = GS_zLevel;
                end

            case "descending"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                if frac >= 1
                    drone.state = "done"; drone.timeInState = 0;
                    totalTime = t - drone.startTime;
                    allDrones(end+1).path = drone.path;
                    allDrones(end).totalTime = totalTime;
                    toRemove(end+1) = i;
                end
        end

        activeDrones(i) = drone;
        V_shift = V_drone0 + drone.pos;
        patch('Faces', F_drone, 'Vertices', V_shift, ...
              'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        text(drone.pos(1), drone.pos(2), drone.pos(3)+2, ...
             sprintf('DR-%d', drone.id), 'FontSize', 8, 'Color', 'r');
    end

    activeDrones(toRemove) = [];

    %% Collision Warning - Improved counting (Only while drone is flying)
collisionThreshold = 2;
currentCollisions = false(length(activeDrones));

for i = 1:length(activeDrones)
    state_i = activeDrones(i).state;
    if ~ismember(state_i, ["moving", "ascending", "descending", "descend_for_charge"])
        continue;  % Skip drones that are not in the air
    end

    for j = i+1:length(activeDrones)
        state_j = activeDrones(j).state;
        if ~ismember(state_j, ["moving", "ascending", "descending", "descend_for_charge"])
            continue;  % Skip drones that are not in the air
        end

        pos1 = activeDrones(i).pos;
        pos2 = activeDrones(j).pos;
        dist = norm(pos1 - pos2);

        if dist < collisionThreshold
            currentCollisions(i,j) = true;

            if ~collisionPairs(i,j)
                % New collision event - count once
                collisionCount = collisionCount + 1;

                % Calculate midpoint
                midPoint = (pos1 + pos2) / 2;

                % Display exclamation mark
                text(midPoint(1), midPoint(2), midPoint(3)+3, '!', ...
                    'FontSize', 20, 'FontWeight', 'bold', 'Color', 'r');

                % Display drone IDs involved in collision
                msg = sprintf('Collision: DR-%d & DR-%d', activeDrones(i).id, activeDrones(j).id);
                text(midPoint(1), midPoint(2), midPoint(3)+5, msg, ...
                    'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r', ...
                    'HorizontalAlignment', 'center');
            end
        end
    end
end

collisionPairs = false(maxDronesEstimate); % reset matrix size in case drones removed/added
collisionPairs(1:length(activeDrones),1:length(activeDrones)) = currentCollisions;


    drawnow;
    pause(0.3);
end

%% Final Output
fprintf('\n--- Final Paths of Completed Drones ---\n');
for i = 1:length(allDrones)
    fprintf('Drone %d path: %s\n', i, mat2str(allDrones(i).path));
end
fprintf('\n--- Total Time Taken by Each Drone (from src to dest) ---\n');
for i = 1:length(allDrones)
    fprintf('Drone %d total travel time: %d seconds\n', i, allDrones(i).totalTime);
end

%% Simulation Summary
fprintf('\n--- Simulation Summary ---\n');
fprintf('Total drone requests made: %d\n', totalRequests);
fprintf('Total collisions detected (unique events): %d\n', collisionCount);

%% Dijkstra Function (unchanged)
function path = dijkstra(graph, startNode, endNode)
    numNodes = size(graph, 1);
    visited = false(1, numNodes); dist = inf(1, numNodes); prev = zeros(1, numNodes);
    dist(startNode) = 0;
    while true
        unvisited = find(~visited);
        if isempty(unvisited), break; end
        [~, idx] = min(dist(unvisited)); u = unvisited(idx);
        if isinf(dist(u)) || u == endNode, break; end
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
        path = [];
    else
        path = endNode;
        while path(1) ~= startNode
            if prev(path(1)) == 0, path = []; return; end
            path = [prev(path(1)), path];
        end
    end
end
