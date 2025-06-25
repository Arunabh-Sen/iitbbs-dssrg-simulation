clc; clear;
%% PARAMETERS
numGS = 5;                  
areaSize = 50;              
gsRange = 40;                
minDistGS = 20;               
chargingTimePlaceholder = 7.5; % Average for reference
droneSpeed = 1.0;
GS_zLevel = 0; % Base Z level for ground stations
D1_z = 1; % Z-level for Docking Floor D1
D2_z = 2; % Z-level for Docking Floor D2
D3_z = 3; % Z-level for Docking Floor D3
dockingZLevels = [D1_z, D2_z, D3_z]; % Array of available docking Z-levels
maxGS_height = 10; % Visual height for GS base
poissonNumIntervals = 2;    % Number of intervals for requests
poissonIntervalLength = 5;  % Length of each interval in seconds (e.g., 5 seconds)
poissonAvgRatePerInterval = 5; % Average number of requests per interval (e.g., 10 requests per 5-second interval)
totalDR_expected = poissonNumIntervals * poissonAvgRatePerInterval;
% Total simulation time over which requests will be generated
totalRequestDuration = poissonNumIntervals * poissonIntervalLength;

%% Collision Detection Parameters (NEW)
collisionThreshold = 1; % Distance (units) at which two drones are considered to collide
zTolerance = 0.5;         % Tolerance for Z-level check. Drones must be within this Z-range to collide.
% NEW: A buffer for un-colliding. Drones must be this far apart (and different Z) to no longer be considered "collided".
collisionExitBuffer = 2.0; % Should be greater than collisionThreshold
totalCollisions = 0;      % Counter for total unique collisions

%% ADD QUEUING SYSTEM FOR GROUND STATIONS (Re-introduced)
GS_queues = cell(numGS, 1);           % Queue for each GS (stores drone IDs)
GS_charging = zeros(numGS, 1);        % Which drone is currently charging at each GS (0 = empty)
GS_chargingTimeLeft = zeros(numGS, 1); % Time left for current charging drone

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
    V_drone0 = (droneStl.Points - mean(stlread('new_dronev_whole.stl').Points)) * 0.003;  % Tiny drone scaling like reference
    gsStl = stlread('Body431.stl');
    F_gs = gsStl.ConnectivityList;
    % FIXED: Much smaller scaling for ground stations
    V_gs0 = (gsStl.Points - mean(stlread('Body431.stl').Points)) * 0.2;  % Same as reference code
    useSTL = true;
    fprintf('STL models loaded successfully\n');
catch
    fprintf('STL models not found, using simple geometric shapes\n');
    useSTL = false;
end
%% INITIALIZE DRONE TRACKING
% Define a template struct for drones to ensure consistency
droneTemplate = struct(...
    'id', 0, ...
    'pos', [0,0,0], ...
    'state', "idle", ... % Initial state can be "idle" before request
    'path', [], ...
    'hop', 1, ...
    'timeInState', 0, ...
    'startPos', [], ...
    'endPos', [], ...
    'travelTime', 0, ...
    'dockingZ', 0, ...
    'totalWaitTime', 0, ...
    'actualChargingTime', 0, ...
    'journeyStartTime', 0, ...
    'actualFlyTime', 0, ...
    'actualTotalTime', 0, ...
    'collisions', struct('time', {}, 'otherDroneID', {}) ... % Collision log for each drone
);
activeDrones = droneTemplate([]); % Initialize as empty array of droneTemplate structs
% Store all drones that flew during simulation (to log paths at end)
allDrones = struct('id', {}, 'path', {}, 'totalWaitTime', {}, 'actualChargingTime', {}, ...
                   'actualFlyTime', {}, 'actualTotalTime', {}, 'collisions', {}); % NEW: added collisions here too
% Counter for staggered requests and drone usage
requestCounter = 0;
dronesUsed = 0;  % Track how many drones have been used
droneIDCounter = 0; % Unique ID for each drone
% --- Poisson Distribution Request Schedule ---
% Calculate the average rate per second
lambdaPerSecond = poissonAvgRatePerInterval / poissonIntervalLength;
% This array will store how many drones to generate at the start of each time step (second)
% Generate numbers of arrivals for each second within the total request duration
poissonSchedule = zeros(1, totalRequestDuration + 1); % +1 for time 0
for t_step = 0:totalRequestDuration % Iterate through each second where requests might arrive
    numRequestsAtThisSecond = poissrnd(lambdaPerSecond);
    poissonSchedule(t_step + 1) = numRequestsAtThisSecond; % +1 because array index starts at 1
end
actualScheduledDrones = sum(poissonSchedule);
fprintf('\n--- Drone Request Schedule (Poisson Distribution) ---\n');
fprintf('Expected total drones: %d (based on parameters)\n', totalDR_expected);
fprintf('Actual total drones scheduled based on Poisson distribution: %d (This varies due to randomness)\n', actualScheduledDrones);
fprintf('Requests will be generated over %d seconds.\n', totalRequestDuration);
%% FIGURE SETUP
figure;
axis([0 areaSize 0 areaSize 0 D3_z + 10]);  % Adjusted Z-axis for better visibility (accommodate D3_z)
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30); grid on; hold on; camlight; lighting gouraud;  % Better viewing angle

%% Simulation-wide tracking for unique collisions (NEW)
% Stores pairs of drone IDs currently in a collision state.
% Example: {'1_2', '3_5'} means drone 1 is colliding with 2, and drone 3 with 5.
collidedPairs = {}; 

%% SIMULATION LOOP
t = 0; % Initialize time
% The loop now continues as long as there are still drones to spawn OR active drones that haven't finished.
while dronesUsed < actualScheduledDrones || ~isempty(activeDrones)
    cla;
    title(['Time: ' num2str(t) ' sec']);
    axis([0 areaSize 0 areaSize 0 D3_z + 10]);  % Consistent axis limits
    %% UPDATE GS CHARGING STATUS
    for gs = 1:numGS
        if GS_charging(gs) > 0 % If a drone is currently charging at this GS
            chargingDroneID = GS_charging(gs);
            droneIdx = find([activeDrones.id] == chargingDroneID);
            if ~isempty(droneIdx) % Ensure the drone exists in activeDrones
                % FIX 2: Increment actualChargingTime for the current second of charging
                activeDrones(droneIdx).actualChargingTime = activeDrones(droneIdx).actualChargingTime + 1;
            end
            GS_chargingTimeLeft(gs) = GS_chargingTimeLeft(gs) - 1;
            if GS_chargingTimeLeft(gs) <= 0
                % Current drone finished charging
                GS_charging(gs) = 0; % Clear the charging spot
                if ~isempty(droneIdx) % Double-check droneIdx
                    currentDrone = activeDrones(droneIdx);
                    if strcmp(currentDrone.state, "charging") % Only proceed if it was actually charging
                        % Use 'gs' directly as the current ground station where charging finished.
                        currentGS_idx_for_transition = gs; 
                        
                        % If the drone has completed all hops, it reached its final destination.
                        % 'hop' points to the *next* segment. So if hop is already beyond path length,
                        % it means it was the last segment.
                        if currentDrone.hop > length(currentDrone.path) 
                            currentDrone.state = "done";
                            currentDrone.timeInState = 0;
                            currentDrone.actualTotalTime = t - currentDrone.journeyStartTime;
                            fprintf('Time %d: Drone-%d completed journey at final destination GS-%d (after final charge).\n', t, currentDrone.id, currentGS_idx_for_transition);
                        else % It's an intermediate GS or the initial charge finished, proceed to next hop
                            % Determine start and end points for the next movement segment
                            startGS_idx = currentGS_idx_for_transition; % Drone is at this GS
                            endGS_idx = currentDrone.path(currentDrone.hop); % Next GS on its path
                            
                            currentDrone.startPos = GS_positions(startGS_idx,:);
                            currentDrone.startPos(3) = currentDrone.dockingZ; % Start at docking Z
                            currentDrone.endPos = GS_positions(endGS_idx,:);
                            currentDrone.endPos(3) = currentDrone.dockingZ; % End at docking Z
                            
                            dist = norm(currentDrone.endPos(1:2) - currentDrone.startPos(1:2));
                            currentDrone.travelTime = dist / droneSpeed;
                            currentDrone.state = "moving";
                            currentDrone.timeInState = 0; % Reset time for new state
                            fprintf('Time %d: Drone-%d finished charging, now moving from GS-%d to GS-%d\n', t, currentDrone.id, startGS_idx, endGS_idx);
                        end
                    end
                    activeDrones(droneIdx) = currentDrone; % Update the struct in the array
                end
                
                % Start next drone in queue if any
                if ~isempty(GS_queues{gs})
                    nextDroneInQueueID = GS_queues{gs}(1);
                    GS_queues{gs}(1) = [];  % Remove from queue
                    GS_charging(gs) = nextDroneInQueueID;
                    % Assign random charging time (5-10s)
                    randomChargingDuration = randi([5, 10]);
                    GS_chargingTimeLeft(gs) = randomChargingDuration;
                    % Update drone state from waiting to charging
                    droneIdx = find([activeDrones.id] == nextDroneInQueueID);
                    if ~isempty(droneIdx)
                        activeDrones(droneIdx).state = "charging";
                        activeDrones(droneIdx).timeInState = 0;
                        % actualChargingTime will be incremented in the next iteration by FIX 2
                        fprintf('Time %d: Drone-%d started charging at GS-%d (was waiting) for %d seconds\n', t, nextDroneInQueueID, gs, randomChargingDuration);
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
        % Add GS label and queue info
        queueSize = length(GS_queues{g});
        if GS_charging(g) > 0
            chargeTimeRemaining = GS_chargingTimeLeft(g);
            chargingDroneID = GS_charging(g);
            queueInfo = sprintf('GS-%d\n[Charging: DR-%d (%ds)]\n[Queue: %d]', g, chargingDroneID, chargeTimeRemaining, queueSize);
        else
            queueInfo = sprintf('GS-%d\n[Available]\n[Queue: %d]', g, queueSize);
        end
        text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3) - 8, ...
             queueInfo, 'FontSize', 8, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', 'b');
    end
    % --- Generate new requests based on Poisson schedule ---
    if t <= totalRequestDuration && (t + 1) <= length(poissonSchedule) && poissonSchedule(t + 1) > 0
        numDronesToSpawn = poissonSchedule(t + 1);
        for k = 1:numDronesToSpawn
            if dronesUsed < actualScheduledDrones
                src = randi(numGS);
                dest = randi(numGS);
                while dest == src
                    dest = randi(numGS);
                end
                droneIDCounter = droneIDCounter + 1;
                dockingZ = dockingZLevels(randi(length(dockingZLevels)));
                
                % Create a new drone struct by copying the template
                drone = droneTemplate; 
                drone.id = droneIDCounter;
                drone.pos = GS_positions(src,:);
                drone.state = "waiting"; % Will immediately try to charge or queue
                drone.hop = 1; % Start at the first hop (source GS)
                drone.journeyStartTime = t;
                drone.dockingZ = dockingZ;
                % Find shortest path
                path = dijkstra(distGraph, src, dest);
                if isempty(path)
                    fprintf('Time %d: No valid path from GS %d to GS %d (not connected) for Drone-%d. Drone not spawned.\n', t, src, dest, droneIDCounter);
                    droneIDCounter = droneIDCounter - 1;
                    continue;
                end
                drone.path = path;
                drone.pos(3) = drone.dockingZ; % Set Z-level for starting position
                % Initial charging at source GS
                if GS_charging(src) == 0
                    drone.state = "charging";
                    GS_charging(src) = droneIDCounter;
                    randomChargingDuration = randi([5, 10]);
                    GS_chargingTimeLeft(src) = randomChargingDuration;
                    % actualChargingTime will be incremented in the next iteration by FIX 2
                    fprintf('Time %d: Drone-%d started charging immediately at GS-%d (Z=%.1f) for %d seconds\n', t, droneIDCounter, src, drone.dockingZ, randomChargingDuration);
                else
                    drone.state = "waiting";
                    GS_queues{src} = [GS_queues{src}, drone.id];
                    fprintf('Time %d: Drone-%d added to queue at GS-%d (Z=%.1f, position %d)\n', t, drone.id, src, drone.dockingZ, length(GS_queues{src}));
                end
                activeDrones(end+1) = drone;
                requestCounter = requestCounter + 1;
                dronesUsed = dronesUsed + 1;
                fprintf('Time %d: Request #%d - Drone-%d spawned at GS %d (Z=%.1f) to GS %d with path: %s (Drones spawned: %d/%d)\n', ...
                        t, requestCounter, droneIDCounter, src, drone.dockingZ, dest, mat2str(drone.path), dronesUsed, actualScheduledDrones);
            end
        end
    end
    % Update active drones
    toRemove = [];
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        if ~strcmp(drone.state, "done")
            drone.timeInState = drone.timeInState + 1;
        end
        switch drone.state
            case "waiting"
                drone.totalWaitTime = drone.totalWaitTime + 1;
            case "charging"
                % actualChargingTime is now handled in the GS_charging status update loop (FIX 2)
            case "moving"
                drone.actualFlyTime = drone.actualFlyTime + 1;
                
                % Ensure startPos and endPos are correctly set for the current hop
                if isempty(drone.startPos) || isempty(drone.endPos) || drone.timeInState == 1 % Re-initialize for the start of movement
                    currentGS_idx = drone.path(drone.hop); % The GS the drone is currently moving *from*
                    nextGS_idx = drone.path(drone.hop + 1); % The GS the drone is moving *to*
                    drone.startPos = GS_positions(currentGS_idx, :);
                    drone.startPos(3) = drone.dockingZ;
                    drone.endPos = GS_positions(nextGS_idx, :);
                    drone.endPos(3) = drone.dockingZ;
                    dist = norm(drone.endPos(1:2) - drone.startPos(1:2));
                    drone.travelTime = dist / droneSpeed;
                    if drone.travelTime == 0 % Handle case where start and end are same (shouldn't happen with path logic but for safety)
                         drone.travelTime = 1; % Avoid division by zero
                    end
                end
                frac = min(drone.timeInState / drone.travelTime, 1);
                drone.pos = drone.startPos + frac * (drone.endPos - drone.startPos);
                
                % Arrival check
                if frac >= 1
                    % Arrived at next GS
                    currentGS_arrived_at_idx = drone.path(drone.hop + 1);
                    drone.pos = GS_positions(currentGS_arrived_at_idx, :); % Ensure it snaps to the exact GS position
                    drone.pos(3) = drone.dockingZ; % And the correct docking Z
                    % FIX 1: Check if this is the final destination BEFORE deciding to charge
                    if currentGS_arrived_at_idx == drone.path(end) % If the drone arrived at its final destination
                        drone.state = "done";
                        drone.timeInState = 0;
                        drone.actualTotalTime = t - drone.journeyStartTime;
                        fprintf('Time %d: Drone-%d reached FINAL destination GS-%d. Final total time: %d, Wait: %d, Charge: %d, Fly: %d\n', ...
                                t, drone.id, currentGS_arrived_at_idx, drone.actualTotalTime, drone.totalWaitTime, drone.actualChargingTime, drone.actualFlyTime);
                        % No charging or queuing needed if it's the end
                    else % It's an intermediate GS, so it needs to charge (or wait)
                        drone.hop = drone.hop + 1; % Increment hop for the *next* segment after this intermediate stop
                        
                        % Reset startPos/endPos for next segment to force recalculation
                        drone.startPos = [];
                        drone.endPos = [];
                        if GS_charging(currentGS_arrived_at_idx) == 0
                            drone.state = "charging";
                            drone.timeInState = 0;
                            GS_charging(currentGS_arrived_at_idx) = drone.id;
                            randomChargingDuration = randi([5, 10]);
                            GS_chargingTimeLeft(currentGS_arrived_at_idx) = randomChargingDuration;
                            % actualChargingTime will be incremented in the next iteration by FIX 2
                            fprintf('Time %d: Drone-%d started charging at GS-%d for %d seconds\n', t, drone.id, currentGS_arrived_at_idx, randomChargingDuration);
                        else
                            drone.state = "waiting";
                            drone.timeInState = 0;
                            GS_queues{currentGS_arrived_at_idx} = [GS_queues{currentGS_arrived_at_idx}, drone.id];
                            fprintf('Time %d: Drone-%d added to queue at GS-%d (position %d)\n', t, drone.id, currentGS_arrived_at_idx, length(GS_queues{currentGS_arrived_at_idx}));
                        end
                    end
                end
            case "done"
                if isempty(find([allDrones.id] == drone.id, 1))
                    allDrones(end+1) = struct('id', drone.id, 'path', drone.path, ...
                                             'totalWaitTime', drone.totalWaitTime, ...
                                             'actualChargingTime', drone.actualChargingTime, ...
                                             'actualFlyTime', drone.actualFlyTime, ...
                                             'actualTotalTime', drone.actualTotalTime, ...
                                             'collisions', drone.collisions); % NEW: Save collisions
                    toRemove(end+1) = i;
                end
        end
        if ~ismember(i, toRemove)
            activeDrones(i) = drone;
        end
        % Display drone (only if not removed in this iteration)
        if ~ismember(i, toRemove)
            if useSTL
                V_shift = V_drone0 + drone.pos;
                patch('Faces', F_drone, 'Vertices', V_shift, ...
                      'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', ...
                      'FaceAlpha', 0.9);
            else
                [X,Y,Z] = sphere(10);
                X = X * 0.8 + drone.pos(1);
                Y = Y * 0.8 + drone.pos(2);
                Z = Z * 0.8 + drone.pos(3);
                surf(X, Y, Z, 'FaceColor', [0.1 0.7 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
            end
            switch drone.state
                case "waiting"
                    labelColor = 'r';
                case "charging"
                    labelColor = 'm';
                case "moving"
                    labelColor = 'g';
                otherwise
                    labelColor = 'k';
            end
            text(drone.pos(1), drone.pos(2), drone.pos(3) + 2.0, ...
                 sprintf('DR-%d\n[%s]', drone.id, drone.state), 'FontSize', 8, 'FontWeight', 'bold', ...
                 'HorizontalAlignment', 'center', 'Color', labelColor);
        end
    end

    %% COLLISION DETECTION LOGIC (UNIQUE DETECTION)
    numActiveDrones = length(activeDrones);
    currentCollidingThisFrame = {}; % To store pairs colliding in this frame

    if numActiveDrones > 1
        for i = 1:numActiveDrones
            for j = i+1:numActiveDrones % Check each unique pair
                drone1 = activeDrones(i);
                drone2 = activeDrones(j);

                % Skip if either drone is not moving, or is done
                if ~strcmp(drone1.state, "moving") || ~strcmp(drone2.state, "moving")
                    continue;
                end
                
                % Form a unique key for the pair (e.g., '1_2' or '2_1' becomes '1_2')
                pairKey = sprintf('%d_%d', min(drone1.id, drone2.id), max(drone1.id, drone2.id));

                % Check for current collision conditions
                dist2D = norm(drone1.pos(1:2) - drone2.pos(1:2));
                isCollidingNow = (dist2D < collisionThreshold) && (abs(drone1.pos(3) - drone2.pos(3)) < zTolerance);

                if isCollidingNow
                    % Add to current frame's colliding pairs
                    currentCollidingThisFrame{end+1} = pairKey;

                    % Check if this pair was NOT previously in a collision state
                    if ~ismember(pairKey, collidedPairs)
                        % This is a NEW, unique collision event
                        totalCollisions = totalCollisions + 1;
                        fprintf('Time %d: NEW UNIQUE COLLISION DETECTED between Drone-%d and Drone-%d at (%.2f,%.2f,%.2f) and (%.2f,%.2f,%.2f)!\n', ...
                                t, drone1.id, drone2.id, drone1.pos(1), drone1.pos(2), drone1.pos(3), ...
                                drone2.pos(1), drone2.pos(2), drone2.pos(3));
                        
                        % Log collision in both drones' records for analytics
                        activeDrones(i).collisions(end+1) = struct('time', t, 'otherDroneID', drone2.id);
                        activeDrones(j).collisions(end+1) = struct('time', t, 'otherDroneID', drone1.id);

                        % Display red exclamation mark at collision point
                        collisionX = (drone1.pos(1) + drone2.pos(1)) / 2;
                        collisionY = (drone1.pos(2) + drone2.pos(2)) / 2;
                        collisionZ = (drone1.pos(3) + drone2.pos(3)) / 2 + 1; % Slightly above drones
                        text(collisionX, collisionY, collisionZ, '!', 'Color', 'r', 'FontSize', 20, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
                    end
                end
            end
        end
    end
    
    % Update the 'collidedPairs' for the next frame
    % First, remove pairs that have now exited the collision state
    % (meaning they were colliding but aren't anymore based on exit buffer)
    newCollidedPairs = {};
    for k = 1:length(collidedPairs)
        pairKey = collidedPairs{k};
        % Check if this pair is still actively colliding in *this* frame
        % Use a larger 'exit buffer' to ensure they truly separate
        
        % Extract drone IDs from pairKey
        parts = strsplit(pairKey, '_');
        id1 = str2double(parts{1});
        id2 = str2double(parts{2});

        % Find current drone objects by ID
        drone1Idx = find([activeDrones.id] == id1, 1);
        drone2Idx = find([activeDrones.id] == id2, 1);

        if isempty(drone1Idx) || isempty(drone2Idx)
            % One or both drones are no longer active, so they can't be colliding
            continue; 
        end
        drone1 = activeDrones(drone1Idx);
        drone2 = activeDrones(drone2Idx);

        dist2D = norm(drone1.pos(1:2) - drone2.pos(1:2));
        % Check for both distance AND Z-level difference for "un-colliding"
        stillColliding = (dist2D < collisionExitBuffer) && (abs(drone1.pos(3) - drone2.pos(3)) < zTolerance);
        
        if stillColliding
            % If they are still within the 'collisionExitBuffer', keep them in the collided list
            newCollidedPairs{end+1} = pairKey;
        end
    end
    % Now add any newly detected collisions from this frame to the tracking list
    for k = 1:length(currentCollidingThisFrame)
        if ~ismember(currentCollidingThisFrame{k}, newCollidedPairs)
            newCollidedPairs{end+1} = currentCollidingThisFrame{k};
        end
    end
    collidedPairs = newCollidedPairs;

    % END COLLISION DETECTION LOGIC

    activeDrones(toRemove) = [];
    % Display info on the side
    infoX = areaSize + 20;
    infoY = areaSize - 10;
    infoZ = D3_z + 10;
    text(infoX, infoY, infoZ, sprintf('Drones Spawning: %d/%d (total scheduled)', dronesUsed, actualScheduledDrones), ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'Color', 'k');
    infoZ = infoZ - 15;
    text(infoX, infoY, infoZ, 'Active Drones:', ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'Color', 'k');
    infoZ = infoZ - 15;
    for i = 1:length(activeDrones)
        drone = activeDrones(i);
        % Adjust currentGS_idx_in_path to always point to the GS the drone is at or just left/is going to
        if strcmp(drone.state, 'moving')
            currentGS_idx_in_path = drone.path(drone.hop); % GS it's moving *from*
            nextGS_idx_for_display = drone.path(drone.hop + 1); % GS it's moving *to*
            str = sprintf('DR-%d: %s GS-%d -> GS-%d (Wait: %ds)', drone.id, drone.state, currentGS_idx_in_path, nextGS_idx_for_display, drone.totalWaitTime);
        elseif strcmp(drone.state, 'waiting') || strcmp(drone.state, 'charging')
            currentGS_idx_in_path = drone.path(drone.hop); % GS it is currently at
            str = sprintf('DR-%d: %s at GS-%d (Wait: %ds, Charge: %ds)', drone.id, drone.state, currentGS_idx_in_path, drone.totalWaitTime, drone.actualChargingTime);
        else
            str = sprintf('DR-%d: %s (status unknown)', drone.id, drone.state);
        end
        text(infoX, infoY, infoZ, str, 'FontSize', 9, ...
            'HorizontalAlignment', 'left', 'Color', 'k');
        infoZ = infoZ - 12;
    end
    drawnow;
    pause(0.001);
    t = t + 1; % Increment time
end
%% PRINT FINAL RESULTS
fprintf('\n--- Final Results of All Completed Drones ---\n');
fprintf('DR ID\tWait Time (s)\tCharge Time (s)\tFly Time (s)\tTotal Time (s)\tUnique Collisions for DR\n'); % Updated column header
fprintf('-----\t-------------\t---------------\t------------\t--------------\t------------------------\n');
totalQueueTime = 0;
totalChargingTime = 0;
totalFlyTime = 0;
totalJourneyTimeOverall = 0;
totalCollisionsLogged = 0; % Sum of unique collisions per drone (will be double the global count)
for i = 1:length(allDrones)
    droneID = allDrones(i).id;
    queueTime = allDrones(i).totalWaitTime;
    chargingTimeForDrone = allDrones(i).actualChargingTime;
    flyTimeForDrone = allDrones(i).actualFlyTime;
    totalTimeForDrone = allDrones(i).actualTotalTime;
    numCollisionsForDrone = length(allDrones(i).collisions); % Number of unique collision events for this drone
    totalCollisionsLogged = totalCollisionsLogged + numCollisionsForDrone;

    % Optional: Print detailed collision log for each drone
    collisionDetails = '';
    if numCollisionsForDrone > 0
        collisionDetails = sprintf('(%d: ', numCollisionsForDrone);
        for k = 1:numCollisionsForDrone
            collisionDetails = [collisionDetails, sprintf('t%d-DR%d, ', allDrones(i).collisions(k).time, allDrones(i).collisions(k).otherDroneID)];
        end
        collisionDetails = [collisionDetails(1:end-2), ')']; % Remove trailing ', '
    else
        collisionDetails = '0';
    end

    fprintf('DR-%d\t\t%d\t\t\t\t%d\t\t\t\t%d\t\t\t\t%d\t\t\t\t%s\n', ...
        droneID, queueTime, chargingTimeForDrone, flyTimeForDrone, totalTimeForDrone, collisionDetails);
    totalQueueTime = totalQueueTime + queueTime;
    totalChargingTime = totalChargingTime + chargingTimeForDrone;
    totalFlyTime = totalFlyTime + flyTimeForDrone;
    totalJourneyTimeOverall = totalJourneyTimeOverall + totalTimeForDrone;
end
fprintf('\nSummary:\n');
fprintf('Total completed deliveries: %d out of %d drones spawned\n', length(allDrones), dronesUsed);
if length(allDrones) > 0
    fprintf('Average queue time: %.1f seconds\n', totalQueueTime / length(allDrones));
    fprintf('Average charging time: %.1f seconds\n', totalChargingTime / length(allDrones));
    fprintf('Average fly time: %.1f seconds\n', totalFlyTime / length(allDrones));
    fprintf('Average total journey time: %.1f seconds\n', totalJourneyTimeOverall / length(allDrones));
    if totalJourneyTimeOverall > 0
        fprintf('Queue time as %% of journey: %.1f%%\n', (totalQueueTime / totalJourneyTimeOverall) * 100);
        fprintf('Charging time as %% of journey: %.1f%%\n', (totalChargingTime / totalJourneyTimeOverall) * 100);
        fprintf('Fly time as %% of journey: %.1f%%\n', (totalFlyTime / totalJourneyTimeOverall) * 100);
    else
        fprintf('Cannot calculate percentages: Total journey time is zero.\n');
    end
end

% Print total number of UNIQUE collisions from the simulation-wide counter
fprintf('\nTotal number of UNIQUE collisions recorded during simulation: %d\n', totalCollisions);

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
end % End of dijkstra function
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
end % End of findConnectedComponents function
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