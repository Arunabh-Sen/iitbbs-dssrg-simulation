clc; clear;

%% PARAMETERS
numGS = 5;
initialDronesPerGS = 2;
requestInterval = 2;
chargingTime = 10;
simDuration = 120;
droneSpeed = 1;
verticalSpeed = 1; % Speed for vertical up/down
boxSize = 30;
GS_zLevel = 3;         % Rooftop height where drones fly
slotCount = 3;
slotHeightStep = 2;
slotReservationTime = 5;

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
GS_positions(:,3) = 0; % Buildings start at ground level (z=0)

%% Adjacency
adjacency = zeros(numGS);
for i = 1:numGS
    adjacency(i, mod(i, numGS) + 1) = 1;
    adjacency(i, mod(i+1, numGS) + 1) = 1;
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

%% Load models
droneStl = stlread('new_dronev_whole.stl');
F_drone = droneStl.ConnectivityList;
V_drone0 = (droneStl.Points - mean(droneStl.Points)) * 0.005;

gsStl = stlread('Monpoc_Apartments.stl');
F_gs = gsStl.ConnectivityList;
V_gs0 = (gsStl.Points - mean(gsStl.Points)) * 0.06;

%% Init drones
dronePool = cell(numGS,1);
for g=1:numGS
    for d=1:initialDronesPerGS
        dronePool{g}(end+1) = struct( ...
            'pos', [GS_positions(g,1), GS_positions(g,2), GS_zLevel], ... % Drone starts at rooftop height
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
activeDrones = struct('pos', {}, 'state', {}, 'path', {}, 'hop', {}, ...
                      'timeInState', {}, 'startPos', {}, 'endPos', {}, 'travelTime', {}, ...
                      'slotAssigned', {}, 'flightZ', {}, 'startTime', {});
allDrones = struct('path', {}, 'totalTime', {});

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
        V_shift = V_gs0 + GS_positions(g,:); % Building base at z=0
        patch('Faces', F_gs, 'Vertices', V_shift, ...
              'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3)-1, ...
            sprintf('GS-%d', g), 'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', 'b');
    end

    % Generate request
    if mod(t, requestInterval) == 0
        src = randi(numGS); dest = randi(numGS);
        while dest == src, dest = randi(numGS); end
        if ~isempty(dronePool{src})
            drone = dronePool{src}(end); dronePool{src}(end) = [];
            path = dijkstra(distGraph, src, dest);
            if isempty(path)
                fprintf('Time %d: No path from %d to %d\n', t, src, dest);
                continue;
            end
            drone.path = path;
            drone.state = "charging"; drone.hop = 1; drone.timeInState = 0;
            drone.pos = [GS_positions(src,1), GS_positions(src,2), GS_zLevel]; % Drone starts at rooftop
            drone.flightZ = GS_zLevel;
            drone.startTime = t;  % Record start time here
            activeDrones(end+1) = drone;
            fprintf('Time %d: Request from GS %d to %d, path: %s\n', ...
                    t, src, dest, mat2str(path));
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

                    % Find free slots in order, prioritize slot 1 (z=3)
                    freeSlots = find(GS_slotReservationEndTime(currentGS,:) <= t);

                    if isempty(freeSlots)
                        % wait (no slot free)
                    else
                        slotID = freeSlots(1); % pick lowest available slot
                        drone.slotAssigned = slotID;
                        GS_slotReservationEndTime(currentGS, slotID) = t + slotReservationTime;

                        if slotID == 1
                            % Slot 1 at rooftop z=3, fly horizontally at z=3 directly
                            drone.flightZ = GS_zLevel;
                            drone.pos(3) = drone.flightZ; % ensure z=3
                            drone.state = "moving";
                            drone.timeInState = 0;

                            startPos = drone.pos;
                            endPos = GS_positions(nextGS,:); endPos(3) = drone.flightZ;
                            drone.startPos = startPos;
                            drone.endPos = endPos;
                            dist = norm(endPos(1:2) - startPos(1:2));
                            drone.travelTime = dist / droneSpeed;

                            fprintf('Time %d: Drone %d uses slot 1 at z=3, moving horizontally\n', t, i);

                        else
                            % Higher slot: ascend from z=3 to slot height
                            targetZ = GS_zLevel + (slotID - 1) * slotHeightStep;
                            drone.startPos = drone.pos;
                            drone.endPos = drone.pos; drone.endPos(3) = targetZ;
                            drone.travelTime = abs(targetZ - drone.pos(3)) / verticalSpeed;
                            drone.flightZ = targetZ;
                            drone.state = "ascending";
                            drone.timeInState = 0;

                            fprintf('Time %d: Drone %d ascending to Z=%d (slot %d)\n', t, i, targetZ, slotID);
                        end
                    end
                end

            case "ascending"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                if frac >= 1
                    currentGS = drone.path(drone.hop);
                    nextGS = drone.path(drone.hop + 1);
                    startPos = GS_positions(currentGS,:); startPos(3) = drone.flightZ;
                    endPos = GS_positions(nextGS,:); endPos(3) = drone.flightZ;
                    dist = norm(endPos(1:2) - startPos(1:2));
                    drone.startPos = startPos; drone.endPos = endPos;
                    drone.travelTime = dist / droneSpeed;
                    drone.state = "moving"; drone.timeInState = 0;
                    fprintf('Time %d: Drone %d starts horizontal flight\n', t, i);
                end

            case "moving"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                drone.pos(3) = drone.flightZ;
                if frac >= 1
                    drone.hop = drone.hop + 1;
                    if drone.hop == length(drone.path)
                        % Descend to rooftop (z=GS_zLevel)
                        drone.startPos = drone.pos;
                        drone.endPos = drone.pos; drone.endPos(3) = GS_zLevel;
                        drone.travelTime = abs(drone.flightZ - GS_zLevel) / verticalSpeed;
                        drone.state = "descending";
                        drone.timeInState = 0;
                    else
                        drone.state = "charging"; drone.timeInState = 0;
                        currentGS = drone.path(drone.hop);
                        drone.pos = GS_positions(currentGS,:);
                        drone.pos(3) = GS_zLevel + (drone.slotAssigned - 1) * slotHeightStep;
                        drone.flightZ = drone.pos(3);
                    end
                end

            case "descending"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                if frac >= 1
                    drone.state = "done"; drone.timeInState = 0;
                    fprintf('Time %d: Drone %d landed at final GS\n', t, i);
                end

            case "done"
                totalTime = t - drone.startTime; % Calculate total travel time
                allDrones(end+1).path = drone.path;
                allDrones(end).totalTime = totalTime;
                toRemove(end+1) = i;
        end

        activeDrones(i) = drone;

        V_shift = V_drone0 + drone.pos;
        patch('Faces', F_drone, 'Vertices', V_shift, ...
              'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        text(drone.pos(1), drone.pos(2), drone.pos(3)+2, ...
             sprintf('DR-%d', i), 'FontSize', 8, 'Color', 'r');
    end

    activeDrones(toRemove) = [];

    % Info panel
    infoX = boxSize + 2; infoY = boxSize - 1; infoZ = boxSize - 2;
    text(infoX, infoY, infoZ, 'Active Drones:', ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    infoZ = infoZ - 2;
    for i = 1:length(activeDrones)
        d = activeDrones(i);
        srcGS = d.path(d.hop);
        destGS = srcGS;
        if d.hop < length(d.path)
            destGS = d.path(d.hop+1);
        end
        str = sprintf('DR-%d: GS %d → %d, Z=%d', i, srcGS, destGS, d.flightZ);
        text(infoX, infoY, infoZ, str, 'FontSize', 9);
        infoZ = infoZ - 1.2;
    end

    drawnow;
    pause(0.05);
end

%% Final paths
fprintf('\n--- Final Paths of Completed Drones ---\n');
for i = 1:length(allDrones)
    fprintf('Drone %d path: %s\n', i, mat2str(allDrones(i).path));
end

%% Print total time taken by each drone
fprintf('\n--- Total Time Taken by Each Drone (from src to dest) ---\n');
for i = 1:length(allDrones)
    fprintf('Drone %d total travel time: %d seconds\n', i, allDrones(i).totalTime);
end

%% Dijkstra
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