clc; clear;
%% PARAMETERS
numGS = 3;                    % Number of ground stations (parameter)
areaSize = 30;              % Area size (1000x1000) - parameter
gsRange = 40;                % Communication range for each GS (parameter)
totalDR = 10;                 % Total number of drones available (parameter)
minDistGS = 10;               % Minimum distance between ground stations (parameter)
requestInterval = 2;
% chargingTime = 15;            % Charging time = 15 sec (parameter) - REMOVED
simDuration = 60;
droneSpeed = 1.0;

GS_zLevel = 0; % Base Z level for ground stations
% New Z-levels for docking floors
D1_z = 1; % Z-level for Docking Floor D1
D2_z = 2; % Z-level for Docking Floor D2
D3_z = 3; % Z-level for Docking Floor D3
dockingZLevels = [D1_z, D2_z, D3_z]; % Array of available docking Z-levels
maxGS_height = 10; % Visual height for GS base

%% GENERATE RANDOM GS POSITIONS WITH MINIMUM DISTANCE
GS_positions = zeros(numGS, 3);
GS_positions(:,3) = GS_zLevel;  % All Z levels same base level
% Generate first GS position randomly
GS_positions(1,1:2) = rand(1, 2) * areaSize;
% Generate remaining GS positions with minimum distance constraint
for i = 2:numGS
    attempts = 0;
    maxAttempts = 1000;  % Prevent infinite loop
    
    while attempts < maxAttempts
        % Generate random position
        candidate = rand(1, 2) * areaSize;
        
        % Check distance to all existing GS
        validPosition = true;
        for j = 1:i-1
            dist = norm(candidate - GS_positions(j,1:2));
            if dist < minDistGS
                validPosition = false;
                break;
            end
        end
        
        if validPosition
            GS_positions(i,1:2) = candidate;
            break;
        end
        
        attempts = attempts + 1;
    end
    
    if attempts >= maxAttempts
        fprintf('Warning: Could not place GS-%d with minimum distance constraint after %d attempts\n', i, maxAttempts);
        fprintf('Placing GS-%d at random position (may violate minimum distance)\n', i);
        GS_positions(i,1:2) = rand(1, 2) * areaSize;
    end
end
fprintf('Ground Station Positions (with minimum distance %.2f):\n', minDistGS);
for i = 1:numGS
    fprintf('GS-%d: (%.2f, %.2f)\n', i, GS_positions(i,1), GS_positions(i,2));
end
% Verify minimum distances
fprintf('\nVerifying minimum distances between ground stations:\n');
minDistFound = inf;
for i = 1:numGS
    for j = i+1:numGS
        dist = norm(GS_positions(i,1:2) - GS_positions(j,1:2));
        if dist < minDistFound
            minDistFound = dist;
        end
    end
end
fprintf('Actual minimum distance between any two GS: %.2f\n', minDistFound);
%% CREATE RANGE-BASED ADJACENCY MATRIX
adjacency = zeros(numGS);
% Generate connections based on communication range
fprintf('\nGenerating Range-Based Graph Structure (Range: %.2f):\n', gsRange);
totalConnections = 0;
for i = 1:numGS
    connectedNeighbors = [];
    for j = 1:numGS
        if i ~= j
            dist = norm(GS_positions(i,1:2) - GS_positions(j,1:2));
            if dist <= gsRange
                adjacency(i,j) = 1;
                adjacency(j,i) = 1;  % Make it symmetric (undirected graph)
                connectedNeighbors = [connectedNeighbors, j];
                if j > i  % Count each connection only once
                    totalConnections = totalConnections + 1;
                end
            end
        end
    end
    
    if ~isempty(connectedNeighbors)
        distances = [];
        for k = connectedNeighbors
            distances(end+1) = norm(GS_positions(i,1:2) - GS_positions(k,1:2));
        end
        fprintf('GS-%d connects to %d neighbors: %s (distances: %s)\n', ...
                i, length(connectedNeighbors), mat2str(connectedNeighbors), ...
                mat2str(round(distances, 2)));
    else
        fprintf('GS-%d has no neighbors within range %.2f\n', i, gsRange);
    end
end
fprintf('Total connections in network: %d\n', totalConnections);
% Check graph connectivity
components = findConnectedComponents(adjacency);
if length(components) > 1
    fprintf('\nWarning: Graph has %d disconnected components:\n', length(components));
    for i = 1:length(components)
        fprintf('Component %d: GS %s\n', i, mat2str(components{i}));
    end
    fprintf('Consider increasing gsRange (currently %.2f) for better connectivity\n', gsRange);
    
    % Optionally connect components (uncomment next line if desired)
    % adjacency = ensureConnectivity(adjacency, GS_positions);
else
    fprintf('\nGraph is fully connected with range %.2f\n', gsRange);
end
%% BUILD DISTANCE GRAPH BASED ON RANGE CONNECTIVITY
distGraph = inf(numGS);
for i = 1:numGS
    for j = 1:numGS
        if adjacency(i,j) == 1
            % Use actual Euclidean distance for connected nodes
            distGraph(i,j) = norm(GS_positions(i,1:2) - GS_positions(j,1:2));
        end
    end
end
% Display some statistics about the network
avgConnections = sum(sum(adjacency)) / (2 * numGS);  % Divide by 2 since matrix is symmetric
fprintf('Average connections per GS: %.2f\n', avgConnections);
% Find GS with most and least connections
connections = sum(adjacency, 2);
[maxConn, maxIdx] = max(connections);
[minConn, minIdx] = min(connections);
fprintf('GS with most connections: GS-%d (%d connections)\n', maxIdx, maxConn);
fprintf('GS with least connections: GS-%d (%d connections)\n', minIdx, minConn);
%% LOAD STL MODELS
try
    droneStl = stlread('new_dronev_whole.stl');
    F_drone = droneStl.ConnectivityList;
    V_drone0 = (droneStl.Points - mean(droneStl.Points)) * 0.003;  % Tiny drone scaling like reference
    
    gsStl = stlread('Body431.stl');
    F_gs = gsStl.ConnectivityList;
    % FIXED: Much smaller scaling for ground stations
    V_gs0 = (gsStl.Points - mean(gsStl.Points)) * 0.2;  % Same as reference code
    
    useSTL = true;
    fprintf('STL models loaded successfully\n');
catch
    fprintf('STL models not found, using simple geometric shapes\n');
    useSTL = false;
end
%% INITIALIZE DRONE TRACKING
activeDrones = struct('pos', {}, 'state', {}, 'path', {}, 'hop', {}, ...
                      'timeInState', {}, 'startPos', {}, 'endPos', {}, ...
                      'travelTime', {}, 'dockingZ', {}); % Added 'dockingZ'
% Store all drones that flew during simulation (to log paths at end)
allDrones = struct('path', {});
% Counter for staggered requests and drone usage
requestCounter = 0;
dronesUsed = 0;  % Track how many drones have been used
%% FIGURE SETUP
figure;
axis([0 areaSize 0 areaSize 0 D3_z + 10]);  % Adjusted Z-axis for better visibility (accommodate D3_z)
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30); grid on; hold on; camlight; lighting gouraud;  % Better viewing angle
%% SIMULATION LOOP
for t=0:simDuration
    cla;
    title(['Time: ' num2str(t) ' sec']);
    axis([0 areaSize 0 areaSize 0 D3_z + 10]);  % Consistent axis limits
    % Display ground stations and their range circles
    for g = 1:numGS
        % Draw range circle at ground level to show communication range
        theta = linspace(0, 2*pi, 50);
        rangeX = gsRange * cos(theta) + GS_positions(g,1);
        rangeY = gsRange * sin(theta) + GS_positions(g,2);
        rangeZ = ones(size(rangeX)) * (GS_zLevel + 1);  % Slightly above ground
        plot3(rangeX, rangeY, rangeZ, 'c--', 'LineWidth', 0.5, 'Color', [0.7 0.7 0.7]);
        
        % Draw edges in green lines (only to neighbors within range)
        neighbors = find(adjacency(g,:));
        for neighbor = neighbors
            if neighbor > g  % Avoid drawing edges twice
                plot3([GS_positions(g,1), GS_positions(neighbor,1)], ...
                      [GS_positions(g,2), GS_positions(neighbor,2)], ...
                      [GS_positions(g,3), GS_positions(neighbor,3)], ...
                      'g-', 'LineWidth', 1.0);
            end
        end
        
        if useSTL
            V_shift = V_gs0 + GS_positions(g,:);
            patch('Faces', F_gs, 'Vertices', V_shift, ...
                  'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', ...
                  'FaceAlpha', 0.9);
        else
            % FIXED: Smaller cylinder for ground stations, adjusted height
            [X,Y,Z] = cylinder(5, 20);  % Reduced radius from 10 to 5
            Z = Z * maxGS_height; % Height adjusted to accommodate docking levels
            X = X + GS_positions(g,1);
            Y = Y + GS_positions(g,2);
            Z = Z + GS_positions(g,3);
            surf(X, Y, Z, 'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
            
            % Draw horizontal lines for docking floors
            plot3([GS_positions(g,1)-5, GS_positions(g,1)+5], [GS_positions(g,2), GS_positions(g,2)], [D1_z, D1_z], 'k--', 'LineWidth', 0.5);
            plot3([GS_positions(g,1)-5, GS_positions(g,1)+5], [GS_positions(g,2), GS_positions(g,2)], [D2_z, D2_z], 'k--', 'LineWidth', 0.5);
            plot3([GS_positions(g,1)-5, GS_positions(g,1)+5], [GS_positions(g,2), GS_positions(g,2)], [D3_z, D3_z], 'k--', 'LineWidth', 0.5);
        end
        % Add GS label below the GS
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3) - 5, ...
             sprintf('GS-%d', g), 'FontSize', 10, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', 'b');
    end
    % Generate staggered requests every 2 seconds
    if mod(t, requestInterval) == 0 && dronesUsed < totalDR
        % Generate one request at a time in staggered form
        src = randi(numGS);  % Random source GS for drone spawning
        dest = randi(numGS);
        while dest == src
            dest = randi(numGS);
        end

        % Randomly select a docking Z-level for the drone
        dockingZ = dockingZLevels(randi(length(dockingZLevels)));
        
        % Create a new drone at the randomly selected source GS
        drone = struct( ...
            'pos', GS_positions(src,:), ...
            'state', "idle", ... % Drones start idle, then move directly to 'moving'
            'path', [], ...
            'hop', 1, ...
            'timeInState', 0, ...
            'startPos', [], ...
            'endPos', [], ...
            'travelTime', 0, ...
            'dockingZ', dockingZ ... % Store the chosen docking Z-level
        );
        % Find shortest path using Dijkstra on communication range graph
        path = dijkstra(distGraph, src, dest);
        if isempty(path)
            fprintf('Time %d: No valid path from GS %d to GS %d (not connected)\n', t, src, dest);
            continue;  % Skip this request if no path exists
        end
        
        drone.path = path;
        % Drone immediately prepares to move, no charging time
        drone.state = "moving";
        drone.hop = 1;
        drone.timeInState = 0;
        
        % Set initial position to the source GS at its chosen docking Z-level
        drone.pos = GS_positions(src,:);
        drone.pos(3) = drone.dockingZ;

        currentGS = drone.path(drone.hop);
        nextGS = drone.path(drone.hop + 1);
        
        drone.startPos = GS_positions(currentGS,:); drone.startPos(3) = drone.dockingZ; % Start at docking Z
        drone.endPos = GS_positions(nextGS,:); drone.endPos(3) = drone.dockingZ;     % End at docking Z
        
        dist = norm(drone.endPos(1:2) - drone.startPos(1:2)); % Calculate 2D distance
        drone.travelTime = dist / droneSpeed;
        
        activeDrones(end+1) = drone;
        requestCounter = requestCounter + 1;
        dronesUsed = dronesUsed + 1;
        fprintf('Time %d: Request #%d - Drone spawned at GS %d (Z=%.1f) to GS %d with path: %s (Drones used: %d/%d)\n', ...
                t, requestCounter, src, drone.dockingZ, dest, mat2str(drone.path), dronesUsed, totalDR);
    elseif mod(t, requestInterval) == 0 && dronesUsed >= totalDR
        fprintf('Time %d: No more drones available (used %d/%d)\n', t, dronesUsed, totalDR);
    end
    % Update active drones
    toRemove = [];
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        drone.timeInState = drone.timeInState + 1;
        switch drone.state
            case "idle" % Drones now go directly to moving
                 % This state might be skipped for first hop.
                 % If a drone lands at an intermediate GS, it transitions
                 % from "moving" to "idle" before preparing for next "moving" state.
                 % No charging time, so it immediately prepares for next hop
                 if drone.hop < length(drone.path) % If not the final destination
                    currentGS = drone.path(drone.hop);
                    nextGS = drone.path(drone.hop + 1);
                    
                    drone.startPos = GS_positions(currentGS,:); drone.startPos(3) = drone.dockingZ;
                    drone.endPos = GS_positions(nextGS,:); drone.endPos(3) = drone.dockingZ;
                    
                    dist = norm(drone.endPos(1:2) - drone.startPos(1:2));
                    drone.travelTime = dist / droneSpeed;
                    drone.state = "moving"; 
                    drone.timeInState = 0; % Reset time for new state
                 else % Final destination reached
                    drone.state = "done";
                    drone.timeInState = 0;
                 end
            case "moving"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                drone.pos(3) = drone.dockingZ; % Maintain drone's assigned docking Z-level
                if frac >= 1
                    drone.hop = drone.hop + 1;
                    if drone.hop == length(drone.path) % Destination reached
                        drone.state = "done"; 
                        drone.timeInState = 0;
                        drone.pos = GS_positions(drone.path(end), :); % Snap to final GS
                        drone.pos(3) = drone.dockingZ; % At the chosen docking Z
                    else % Intermediate GS reached
                        drone.state = "idle"; % Drone arrives at GS, ready for next hop
                        drone.timeInState = 0;
                        drone.pos = GS_positions(drone.path(drone.hop), :); % Snap to current GS
                        drone.pos(3) = drone.dockingZ; % At the chosen docking Z
                    end
                end
            case "done"
                % Save drone path in allDrones for final logging
                allDrones(end+1).path = drone.path;
                toRemove(end+1) = i;
        end
        activeDrones(i) = drone;
        
        % Display drone
        if useSTL
            V_shift = V_drone0 + drone.pos;
            patch('Faces', F_drone, 'Vertices', V_shift, ...
                  'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', ...
                  'FaceAlpha', 0.9);
        else
            % Use simple sphere if STL not available
            [X,Y,Z] = sphere(10);
            X = X * 0.8 + drone.pos(1);  % Much smaller drone sphere
            Y = Y * 0.8 + drone.pos(2);
            Z = Z * 0.8 + drone.pos(3);
            surf(X, Y, Z, 'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        end
        
        text(drone.pos(1), drone.pos(2), drone.pos(3) + 2.0, ...
             sprintf('DR-%d', i), 'FontSize', 8, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', 'r');
    end
    activeDrones(toRemove) = [];
    % Display info on the side
    infoX = areaSize + 20;  
    infoY = areaSize - 10;  
    infoZ = D3_z + 10; % Adjusted info Z to be above the highest docking floor
    text(infoX, infoY, infoZ, sprintf('Drones Used: %d/%d', dronesUsed, totalDR), ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'Color', 'k');
    infoZ = infoZ - 15;
    text(infoX, infoY, infoZ, 'Active Drones:', ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'Color', 'k');
    infoZ = infoZ - 15;
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        currentGS_idx = drone.path(drone.hop); % Get the index of the current GS
        
        % Determine current GS text for display
        currentGS_text = sprintf('GS %d (Z=%.1f)', currentGS_idx, drone.dockingZ);

        if drone.hop < length(drone.path)
            nextGS_idx = drone.path(drone.hop + 1);
            str = sprintf('DR-%d: %s at %s -> GS %d', i, drone.state, currentGS_text, nextGS_idx);
        else
            str = sprintf('DR-%d: %s at %s (destination)', i, drone.state, currentGS_text);
        end
        text(infoX, infoY, infoZ, str, 'FontSize', 9, ...
            'HorizontalAlignment', 'left', 'Color', 'k');
        infoZ = infoZ - 12;
    end
    drawnow;
    pause(0.01);
end
%% PRINT FINAL PATHS OF ALL DRONES THAT FLEW
fprintf('\n--- Final Paths of All Completed Drones ---\n');
for i = 1:length(allDrones)
    fprintf('Drone %d path: %s\n', i, mat2str(allDrones(i).path));
end
fprintf('\nTotal completed deliveries: %d out of %d drones used\n', length(allDrones), dronesUsed);
%% HELPER FUNCTIONS
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
%% FIND CONNECTED COMPONENTS
function components = findConnectedComponents(adjacency)
    n = size(adjacency, 1);
    visited = false(1, n);
    components = {};
    
    for i = 1:n
        if ~visited(i)
            component = [];
            stack = i;
            while ~isempty(stack)
                node = stack(end);
                stack(end) = [];
                if ~visited(node)
                    visited(node) = true;
                    component = [component, node];
                    neighbors = find(adjacency(node, :));
                    stack = [stack, neighbors(~visited(neighbors))];
                end
            end
            components{end+1} = component;
        end
    end
end
%% ENSURE GRAPH CONNECTIVITY
function adjacency = ensureConnectivity(adjacency, positions)
    components = findConnectedComponents(adjacency);
    
    % Connect components by finding closest pairs
    for i = 1:length(components)-1
        comp1 = components{i};
        comp2 = components{i+1};
        
        minDist = inf;
        bestPair = [0, 0];
        
        for node1 = comp1
            for node2 = comp2
                dist = norm(positions(node1, 1:2) - positions(node2, 1:2));
                if dist < minDist
                    minDist = dist;
                    bestPair = [node1, node2];
                end
            end
        end
        
        % Add edge between closest nodes
        adjacency(bestPair(1), bestPair(2)) = 1;
        adjacency(bestPair(2), bestPair(1)) = 1;
        fprintf('Added connection: GS-%d <-> GS-%d (distance: %.2f)\n', ...
                bestPair(1), bestPair(2), minDist);
    end
end