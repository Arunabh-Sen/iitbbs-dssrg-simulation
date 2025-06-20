clc; clear;
%% PARAMETERS
numGS = 5;                    % Number of ground stations (parameter)
areaSize = 50;              % Area size (1000x1000) - parameter
gsRange = 60;                % Communication range for each GS (parameter)
totalDR = 10;                 % Total number of drones available (parameter)
minDistGS = 30;               % Minimum distance between ground stations (parameter)
requestInterval = 2;
chargingTime = 10;            % Charging time = 10 sec (parameter)
droneSpeed = 1.0;
GS_zLevel = 0;
flightZ = 2;
%% ADD QUEUING SYSTEM FOR GROUND STATIONS
GS_queues = cell(numGS, 1);           % Queue for each GS (stores drone IDs)
GS_charging = zeros(numGS, 1);        % Which drone is currently charging at each GS (0 = empty)
GS_chargingTimeLeft = zeros(numGS, 1); % Time left for current charging drone
%% GENERATE RANDOM GS POSITIONS WITH MINIMUM DISTANCE
GS_positions = zeros(numGS, 3);
GS_positions(:,3) = GS_zLevel;  % All Z levels same
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
    V_drone0 = (droneStl.Points - mean(droneStl.Points)) * 0.005;  % Tiny drone scaling like reference

    gsStl = stlread('Monpoc_Apartments.stl');
    F_gs = gsStl.ConnectivityList;
    % FIXED: Much smaller scaling for ground stations
    V_gs0 = (gsStl.Points - mean(gsStl.Points)) * 0.05;  % Same as reference code

    useSTL = true;
    fprintf('STL models loaded successfully\n');
catch
    fprintf('STL models not found, using simple geometric shapes\n');
    useSTL = false;
end
%% INITIALIZE DRONE TRACKING
activeDrones = struct('id', {}, 'pos', {}, 'state', {}, 'path', {}, 'hop', {}, ...
                      'timeInState', {}, 'startPos', {}, 'endPos', {}, 'travelTime', {}, ...
                      'totalWaitTime', {}, 'journeyStartTime', {});
% Store all drones that flew during simulation (to log paths at end)
allDrones = struct('path', {}, 'totalTime', {}, 'totalWaitTime', {});
% Counter for staggered requests and drone usage
requestCounter = 0;
dronesUsed = 0;  % Track how many drones have been used
droneIDCounter = 0; % Unique ID for each drone
%% FIGURE SETUP
figure;
axis([0 areaSize 0 areaSize 0 20]);  % Adjusted Z-axis for better visibility
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30); grid on; hold on; camlight; lighting gouraud;  % Better viewing angle
%% SIMULATION LOOP
t = 0;
% The simulation will now run as long as there are active drones or new drones can still be requested.
while (length(allDrones) < totalDR || ~isempty(activeDrones))
    cla;
    title(['Time: ' num2str(t) ' sec']);
    axis([0 areaSize 0 areaSize 0 20]);  % Consistent axis limits
    %% UPDATE GS CHARGING STATUS
    for gs = 1:numGS
        if GS_charging(gs) > 0
            GS_chargingTimeLeft(gs) = GS_chargingTimeLeft(gs) - 1;
            if GS_chargingTimeLeft(gs) <= 0
                % Current drone finished charging
                chargingDroneID = GS_charging(gs);
                GS_charging(gs) = 0;

                % Start next drone in queue if any
                if ~isempty(GS_queues{gs})
                    nextDroneID = GS_queues{gs}(1);
                    GS_queues{gs}(1) = [];  % Remove from queue
                    GS_charging(gs) = nextDroneID;
                    GS_chargingTimeLeft(gs) = chargingTime;

                    % Update drone state from waiting to charging
                    for i = 1:length(activeDrones)
                        if activeDrones(i).id == nextDroneID
                            activeDrones(i).state = "charging";
                            activeDrones(i).timeInState = 0;
                            fprintf('Time %d: Drone-%d started charging at GS-%d (was waiting)\n', t, nextDroneID, gs);
                            break;
                        end
                    end
                end
            end
        end
    end
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
            % FIXED: Smaller cylinder for ground stations
            [X,Y,Z] = cylinder(5, 20);  % Reduced radius from 10 to 5
            Z = Z * 10; % Reduced height from 15 to 10
            X = X + GS_positions(g,1);
            Y = Y + GS_positions(g,2);
            Z = Z + GS_positions(g,3);
            surf(X, Y, Z, 'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        end
        % Add GS label and queue info
        queueSize = length(GS_queues{g});
        if GS_charging(g) > 0
            queueInfo = sprintf('GS-%d\n[Charging: DR-%d]\n[Queue: %d]', g, GS_charging(g), queueSize);
        else
            queueInfo = sprintf('GS-%d\n[Available]\n[Queue: %d]', g, queueSize);
        end

        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3) - 8, ...
             queueInfo, 'FontSize', 8, 'FontWeight', 'bold', ...
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

        droneIDCounter = droneIDCounter + 1;

        % Create a new drone at the randomly selected source GS
        drone = struct( ...
            'id', droneIDCounter, ...
            'pos', GS_positions(src,:), ...
            'state', "waiting", ...
            'path', [], ...
            'hop', 1, ...
            'timeInState', 0, ...
            'startPos', [], ...
            'endPos', [], ...
            'travelTime', 0, ...
            'totalWaitTime', 0, ...
            'journeyStartTime', t ...
        );
        % Find shortest path using Dijkstra on communication range graph
        path = dijkstra(distGraph, src, dest);
        if isempty(path)
            fprintf('Time %d: No valid path from GS %d to GS %d (not connected)\n', t, src, dest);
            % Decrement droneIDCounter and dronesUsed if a drone wasn't actually created
            droneIDCounter = droneIDCounter - 1;
            continue;  % Skip this request if no path exists
        end

        drone.path = path;
        drone.hop = 1;
        drone.pos = GS_positions(src,:);
        % Add drone to GS queue or start charging immediately
        if GS_charging(src) == 0
            % Charging port is available
            drone.state = "charging";
            GS_charging(src) = droneIDCounter;
            GS_chargingTimeLeft(src) = chargingTime;
            fprintf('Time %d: Drone-%d started charging immediately at GS-%d\n', t, droneIDCounter, src);
        else
            % Charging port is occupied, add to queue
            drone.state = "waiting";
            GS_queues{src} = [GS_queues{src}, droneIDCounter];
            fprintf('Time %d: Drone-%d added to queue at GS-%d (position %d)\n', t, droneIDCounter, src, length(GS_queues{src}));
        end
        activeDrones(end+1) = drone;
        requestCounter = requestCounter + 1;
        dronesUsed = dronesUsed + 1;
        fprintf('Time %d: Request #%d - Drone-%d spawned at GS %d to GS %d with path: %s (Drones used: %d/%d)\n', ...
                t, requestCounter, droneIDCounter, src, dest, mat2str(drone.path), dronesUsed, totalDR);
    elseif mod(t, requestInterval) == 0 && dronesUsed >= totalDR
        % This message is now less critical as the loop continues until all drones are done
        % fprintf('Time %d: No more drones available (used %d/%d)\n', t, dronesUsed, totalDR);
    end
    % Update active drones
    toRemove = [];
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        drone.timeInState = drone.timeInState + 1;
        switch drone.state
            case "waiting"
                % Drone is waiting in queue, track wait time
                drone.totalWaitTime = drone.totalWaitTime + 1;
                % State change handled by GS charging system above

            case "charging"
                % FIXED: Correct charging state logic
                currentGS = drone.path(drone.hop);
                if GS_charging(currentGS) ~= drone.id
                    % This drone finished charging (no longer the charging drone)
                    if drone.hop == length(drone.path)
                        % Reached destination
                        drone.state = "done";
                        drone.timeInState = 0;
                        fprintf('Time %d: Drone-%d completed journey at destination GS-%d\n', t, drone.id, currentGS);
                    else
                        % Move to next hop after charging
                        drone.hop = drone.hop + 1;  % CRITICAL FIX: Increment hop
                        nextGS = drone.path(drone.hop);
                        startPos = GS_positions(currentGS,:);
                        startPos(3) = flightZ;
                        endPos = GS_positions(nextGS,:);
                        endPos(3) = flightZ;
                        drone.startPos = startPos;
                        drone.endPos = endPos;
                        dist = norm(endPos(1:2) - startPos(1:2));
                        drone.travelTime = dist / droneSpeed;
                        drone.state = "moving";
                        drone.timeInState = 0;
                        fprintf('Time %d: Drone-%d finished charging, moving from GS-%d to GS-%d\n', t, drone.id, currentGS, nextGS);
                    end
                end
                % If still charging (GS_charging(currentGS) == drone.id), do nothing
            case "moving"
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                drone.pos(3) = flightZ;
                if frac >= 1
                    % Arrived at next GS
                    nextGS = drone.path(drone.hop);
                    drone.pos = GS_positions(nextGS, :);

                    if drone.hop == length(drone.path)
                        % Reached destination
                        drone.state = "done";
                        drone.timeInState = 0;
                        fprintf('Time %d: Drone-%d reached destination GS-%d\n', t, drone.id, nextGS);
                    else
                        % Need to charge at intermediate GS
                        if GS_charging(nextGS) == 0
                            % Charging port available
                            drone.state = "charging";
                            drone.timeInState = 0;
                            GS_charging(nextGS) = drone.id;
                            GS_chargingTimeLeft(nextGS) = chargingTime;
                            fprintf('Time %d: Drone-%d started charging at GS-%d\n', t, drone.id, nextGS);
                        else
                            % Add to queue
                            drone.state = "waiting";
                            drone.timeInState = 0;
                            GS_queues{nextGS} = [GS_queues{nextGS}, drone.id];
                            fprintf('Time %d: Drone-%d added to queue at GS-%d (position %d)\n', t, drone.id, nextGS, length(GS_queues{nextGS}));
                        end
                    end
                end
            case "done"
                % Save drone path and timing info
                totalJourneyTime = t - drone.journeyStartTime;
                allDrones(end+1) = struct('path', drone.path, 'totalTime', totalJourneyTime, 'totalWaitTime', drone.totalWaitTime);
                fprintf('Time %d: Drone-%d completed journey in %d seconds (waited %d seconds)\n', t, drone.id, totalJourneyTime, drone.totalWaitTime);
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

        % Color code drone labels based on state
        switch drone.state
            case "waiting"
                labelColor = 'r';  % Red for waiting
            case "charging"
                labelColor = 'm';  % Magenta for charging
            case "moving"
                labelColor = 'g';  % Green for moving
            otherwise
                labelColor = 'k';  % Black for others
        end

        text(drone.pos(1), drone.pos(2), drone.pos(3) + 2.0, ...
             sprintf('DR-%d\n[%s]', drone.id, drone.state), 'FontSize', 8, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', labelColor);
    end
    activeDrones(toRemove) = [];
    % Display info on the side
    infoX = areaSize + 20;
    infoY = areaSize - 10;
    infoZ = areaSize - 20;
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
        currentGS = drone.path(drone.hop);
        if drone.hop < length(drone.path)
            nextGS = drone.path(drone.hop + 1);
            str = sprintf('DR-%d: %s at GS %d -> GS %d (Wait: %ds)', drone.id, drone.state, currentGS, nextGS, drone.totalWaitTime);
        else
            str = sprintf('DR-%d: %s at GS %d (destination) (Wait: %ds)', drone.id, drone.state, currentGS, drone.totalWaitTime);
        end
        text(infoX, infoY, infoZ, str, 'FontSize', 9, ...
            'HorizontalAlignment', 'left', 'Color', 'k');
        infoZ = infoZ - 12;
    end
    drawnow;
    pause(0.01); % Pause for animation
    t = t + 1; % Increment time
end
%% PRINT FINAL RESULTS
fprintf('\n--- Final Results of All Completed Drones ---\n');
fprintf('DRno\tqueue time (seconds)\tcharging time (seconds)\tfly time (seconds)\ttotal time (seconds)\n');
fprintf('----\t--------------------\t-----------------------\t------------------\t--------------------\n');
totalQueueTime = 0;
totalChargingTime = 0;
totalFlyTime = 0;
totalJourneyTimeOverall = 0; % Renamed to avoid conflict with `totalJourneyTime` for individual drones

for i = 1:length(allDrones)
    % The `totalWaitTime` stored in `allDrones` currently only accounts for queue time.
    % We need to separate queue time and charging time.

    queueTime = allDrones(i).totalWaitTime;

    % Calculate total charging time: (number of hops in path) * chargingTime
    % Each drone charges at the source GS and at every intermediate GS it lands on.
    % The number of 'stops' a drone makes for charging is equal to the number of GSs in its path.
    numChargingStops = length(allDrones(i).path);
    chargingTimeForDrone = numChargingStops * chargingTime;

    % Total journey time is already recorded as 'totalTime' for each drone
    totalTimeForDrone = allDrones(i).totalTime;

    % Fly time is total journey time minus all non-flying time (queue + charging)
    flyTimeForDrone = totalTimeForDrone - (queueTime + chargingTimeForDrone);

    fprintf('DR-%d\t\t%d\t\t\t\t\t\t%d\t\t\t\t\t\t\t%d\t\t\t\t\t\t%d\n', ...
        i, queueTime, chargingTimeForDrone, flyTimeForDrone, totalTimeForDrone);

    totalQueueTime = totalQueueTime + queueTime;
    totalChargingTime = totalChargingTime + chargingTimeForDrone;
    totalFlyTime = totalFlyTime + flyTimeForDrone;
    totalJourneyTimeOverall = totalJourneyTimeOverall + totalTimeForDrone;
end

fprintf('\nSummary:\n');
fprintf('Total completed deliveries: %d out of %d drones used\n', length(allDrones), dronesUsed);
if length(allDrones) > 0
    fprintf('Average queue time: %.1f seconds\n', totalQueueTime / length(allDrones));
    fprintf('Average charging time: %.1f seconds\n', totalChargingTime / length(allDrones));
    fprintf('Average fly time: %.1f seconds\n', totalFlyTime / length(allDrones));
    fprintf('Average total journey time: %.1f seconds\n', totalJourneyTimeOverall / length(allDrones));
    fprintf('Queue time as %% of journey: %.1f%%\n', (totalQueueTime / totalJourneyTimeOverall) * 100);
    fprintf('Charging time as %% of journey: %.1f%%\n', (totalChargingTime / totalJourneyTimeOverall) * 100);
    fprintf('Fly time as %% of journey: %.1f%%\n', (totalFlyTime / totalJourneyTimeOverall) * 100);
end
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