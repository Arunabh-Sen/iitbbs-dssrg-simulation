clc; clear;

%% PARAMETERS
numGS = 10; % Changed to 10 GS (0-9)
initialDronesPerGS = 2;
requestInterval = 2;
chargingTime = 10;
simDuration = 180;
droneSpeed = 1;
verticalSpeed = 10;
boxSize = 50; % Increased box size for 10 GS
GS_zLevel = 3;
slotCount = 3;
slotHeightStep = 2;
slotReservationTime = 5;
minGSDistance = 10; % Minimum distance between GS

%% Counters
totalRequests = 0;
collisionCount = 0;
collisionLog = {}; % Initialize as empty cell array

%% Fixed Graph Structure - Define edges and outbound slots
% Edges as specified: (0,1), (0,2), (0,3), (0,4), (1,5), (1,6), (2,7), (2,8), (3,9), (3,5), (5,7), (6,8), (7,9)
edges = [
    1, 2;   % (0,1) - MATLAB uses 1-based indexing
    1, 3;   % (0,2)
    1, 4;   % (0,3)
    1, 5;   % (0,4)
    2, 6;   % (1,5)
    2, 7;   % (1,6)
    3, 8;   % (2,7)
    3, 9;   % (2,8)
    4, 10;  % (3,9)
    4, 6;   % (3,5)
    6, 8;   % (5,7)
    7, 9;   % (6,8)
    8, 10   % (7,9)
];

% Outbound z-slots for each GS (0-based node numbering mapped to 1-based indexing)
outboundSlots = [1, 2, 2, 2, 2, 1, 1, 3, 3, 1]; % Node 0-9 outbound slots

%% GS Positioning with minimum distance constraint
GS_positions = zeros(numGS, 3);
GS_positions(1,1:2) = rand(1,2) * boxSize; % Place first GS randomly

for i = 2:numGS
    placed = false;
    attempt = 0;
    while ~placed && attempt < 2000
        candidate = rand(1,2) * boxSize;
        dists = vecnorm(GS_positions(1:i-1,1:2) - candidate, 2, 2);
        if all(dists >= minGSDistance) % Ensure minimum distance from all existing GS
            GS_positions(i,1:2) = candidate;
            placed = true;
        end
        attempt = attempt + 1;
    end
    if ~placed
        error('Failed to place GS %d after %d attempts. Try reducing minGSDistance or increasing boxSize.', i-1, attempt);
    end
end
GS_positions(:,3) = 0;

%% Build Adjacency Matrix from Fixed Edges
adjacency = zeros(numGS);
for i = 1:size(edges, 1)
    node1 = edges(i, 1);
    node2 = edges(i, 2);
    adjacency(node1, node2) = 1;
    adjacency(node2, node1) = 1; % undirected graph
end

%% Distance Graph for Pathfinding
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
maxDronesEstimate = 100;
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

    % Draw Ground Stations
    for g = 1:numGS
        V_shift = V_gs0 + GS_positions(g,:);
        patch('Faces', F_gs, 'Vertices', V_shift, ...
              'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3)-1, ...
            sprintf('GS-%d', g-1), 'FontSize', 15, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', 'r');
        
        % Display outbound slot info
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3)+8, ...
            sprintf('Out: z%d', outboundSlots(g)), 'FontSize', 8, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', 'g');
    end

    % Draw edges based on fixed graph structure
    for i = 1:size(edges, 1)
        node1 = edges(i, 1);
        node2 = edges(i, 2);
        pt1 = GS_positions(node1, 1:2);
        pt2 = GS_positions(node2, 1:2);
        plot3([pt1(1), pt2(1)], [pt1(2), pt2(2)], [0, 0], ...
              'g-', 'LineWidth', 1.5);
    end

    % Generate drone requests
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

    % Process active drones
    toRemove = [];
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        drone.timeInState = drone.timeInState + 1;

        switch drone.state
            case "charging"
                if drone.timeInState >= chargingTime
                    currentGS = drone.path(drone.hop);
                    
                    % Check if this is the final destination
                    if drone.hop == length(drone.path)
                        % Already at destination, mark as done
                        drone.state = "done"; 
                        drone.timeInState = 0;
                        totalTime = t - drone.startTime;
                        allDrones(end+1).path = drone.path;
                        allDrones(end).totalTime = totalTime;
                        toRemove(end+1) = i;
                    else
                        % Need to move to next GS - apply outbound slot logic
                        nextGS = drone.path(drone.hop + 1);
                        
                        % Use the predefined outbound slot for the current GS
                        requiredSlot = outboundSlots(currentGS);
                        
                        % Check if the required outbound slot is available
                        if GS_slotReservationEndTime(currentGS, requiredSlot) <= t
                            drone.slotAssigned = requiredSlot;
                            GS_slotReservationEndTime(currentGS, requiredSlot) = t + slotReservationTime;
                            
                            if requiredSlot == 1
                                % Slot 1 is at GS_zLevel, can move directly
                                drone.flightZ = GS_zLevel;
                                drone.pos(3) = GS_zLevel;
                                drone.state = "moving"; 
                                drone.timeInState = 0;
                                drone.startPos = drone.pos;
                                drone.endPos = GS_positions(nextGS,:); 
                                drone.endPos(3) = drone.flightZ;
                                drone.travelTime = norm(drone.endPos(1:2) - drone.startPos(1:2)) / droneSpeed;
                            else
                                % Need to ascend to higher slot first
                                targetZ = GS_zLevel + (requiredSlot - 1) * slotHeightStep;
                                drone.startPos = drone.pos;
                                drone.endPos = drone.pos; 
                                drone.endPos(3) = targetZ;
                                drone.travelTime = abs(targetZ - drone.pos(3)) / verticalSpeed;
                                drone.flightZ = targetZ;
                                drone.state = "ascending"; 
                                drone.timeInState = 0;
                            end
                        end
                        % If slot not available, drone stays in charging state until slot is free
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
                        % Reached destination - descend to GS level
                        drone.startPos = drone.pos;
                        drone.endPos = drone.pos; drone.endPos(3) = GS_zLevel;
                        drone.travelTime = abs(drone.flightZ - GS_zLevel) / verticalSpeed;
                        drone.state = "descending"; drone.timeInState = 0;
                    else
                        % Need to stop at intermediate GS for charging
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
        
        % Draw drone
        V_shift = V_drone0 + drone.pos;
        patch('Faces', F_drone, 'Vertices', V_shift, ...
              'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
        text(drone.pos(1), drone.pos(2), drone.pos(3)+2, ...
             sprintf('DR-%d', drone.id), 'FontSize', 8, 'Color', 'r');
    end

    activeDrones(toRemove) = [];

    %% Collision Detection with Enhanced Logging
    collisionThreshold = 2;
    currentCollisions = false(length(activeDrones));

    for i = 1:length(activeDrones)
        state_i = activeDrones(i).state;
        if ~ismember(state_i, ["moving", "ascending", "descending", "descend_for_charge"])
            continue;
        end

        for j = i+1:length(activeDrones)
            state_j = activeDrones(j).state;
            if ~ismember(state_j, ["moving", "ascending", "descending", "descend_for_charge"])
                continue;
            end

            pos1 = activeDrones(i).pos;
            pos2 = activeDrones(j).pos;
            dist = norm(pos1 - pos2);

            if dist < collisionThreshold
                currentCollisions(i,j) = true;

                if ~collisionPairs(i,j)
                    collisionCount = collisionCount + 1;
                    midPoint = (pos1 + pos2) / 2;
                    
                    % Determine collision phase for both drones
                    phase_i = determineCollisionPhase(activeDrones(i), GS_zLevel);
                    phase_j = determineCollisionPhase(activeDrones(j), GS_zLevel);
                    
                    % Log collision details - fix the cell array assignment
                    collisionLog{end+1, 1} = collisionCount;
                    collisionLog{end, 2} = activeDrones(i).id;
                    collisionLog{end, 3} = activeDrones(j).id;
                    collisionLog{end, 4} = phase_i;
                    collisionLog{end, 5} = phase_j;
                    collisionLog{end, 6} = midPoint(1);
                    collisionLog{end, 7} = midPoint(2);
                    collisionLog{end, 8} = midPoint(3);
                    collisionLog{end, 9} = t;
                    
                    text(midPoint(1), midPoint(2), midPoint(3)+3, '!', ...
                        'FontSize', 20, 'FontWeight', 'bold', 'Color', 'r');
                    msg = sprintf('Collision: DR-%d & DR-%d', activeDrones(i).id, activeDrones(j).id);
                    text(midPoint(1), midPoint(2), midPoint(3)+5, msg, ...
                        'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r', ...
                        'HorizontalAlignment', 'center');
                end
            end
        end
    end

    collisionPairs = false(maxDronesEstimate);
    collisionPairs(1:length(activeDrones),1:length(activeDrones)) = currentCollisions;

    drawnow;
    pause(0.05);
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

%% Detailed Collision Analysis
if ~isempty(collisionLog)
    fprintf('\n--- Detailed Collision Analysis ---\n');
    fprintf('%-8s %-8s %-8s %-12s %-12s %-8s %-8s %-8s %-8s\n', ...
        'Coll#', 'Drone1', 'Drone2', 'Phase1', 'Phase2', 'X', 'Y', 'Z', 'Time');
    fprintf('%-8s %-8s %-8s %-12s %-12s %-8s %-8s %-8s %-8s\n', ...
        '-----', '------', '------', '------', '------', '---', '---', '---', '----');
    
    for i = 1:size(collisionLog, 1)
        fprintf('%-8d %-8d %-8d %-12s %-12s %-8.2f %-8.2f %-8.2f %-8d\n', ...
            collisionLog{i,1}, collisionLog{i,2}, collisionLog{i,3}, ...
            collisionLog{i,4}, collisionLog{i,5}, ...
            collisionLog{i,6}, collisionLog{i,7}, collisionLog{i,8}, collisionLog{i,9});
    end
    
    % Collision Phase Statistics
    fprintf('\n--- Collision Phase Statistics ---\n');
    takeoffCount = 0;
    landingCount = 0;
    pathCount = 0;
    mixedCount = 0;
    
    for i = 1:size(collisionLog, 1)
        phase1 = collisionLog{i,4};
        phase2 = collisionLog{i,5};
        
        if strcmp(phase1, 'Takeoff') && strcmp(phase2, 'Takeoff')
            takeoffCount = takeoffCount + 1;
        elseif strcmp(phase1, 'Landing') && strcmp(phase2, 'Landing')
            landingCount = landingCount + 1;
        elseif strcmp(phase1, 'Path') && strcmp(phase2, 'Path')
            pathCount = pathCount + 1;
        else
            mixedCount = mixedCount + 1;
        end
    end
    
    fprintf('Takeoff-Takeoff collisions: %d (%.1f%%)\n', takeoffCount, takeoffCount/collisionCount*100);
    fprintf('Landing-Landing collisions: %d (%.1f%%)\n', landingCount, landingCount/collisionCount*100);
    fprintf('Path-Path collisions: %d (%.1f%%)\n', pathCount, pathCount/collisionCount*100);
    fprintf('Mixed phase collisions: %d (%.1f%%)\n', mixedCount, mixedCount/collisionCount*100);
else
    fprintf('\nNo collisions detected during simulation.\n');
end

%% Display Graph Structure
fprintf('\n--- Graph Structure ---\n');
fprintf('Edges: ');
for i = 1:size(edges, 1)
    fprintf('(%d,%d) ', edges(i,1)-1, edges(i,2)-1); % Display as 0-based
end
fprintf('\n');
fprintf('Outbound Slots: ');
for i = 1:numGS
    fprintf('GS-%d:z%d ', i-1, outboundSlots(i));
end
fprintf('\n');

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

%% Function to determine collision phase
function phase = determineCollisionPhase(drone, GS_zLevel)
    switch drone.state
        case "ascending"
            phase = 'Takeoff';
        case "descending"
            phase = 'Landing';
        case "descend_for_charge"
            phase = 'Landing';
        case "moving"
            if abs(drone.pos(3) - GS_zLevel) < 1
                phase = 'Takeoff'; % Still close to GS level
            else
                phase = 'Path';
            end
        otherwise
            phase = 'Unknown';
    end
end