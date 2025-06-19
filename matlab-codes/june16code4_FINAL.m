clc; clear;
%% Real-Time Multi-Threaded Simulation with Precise Timing
fprintf('=== REAL-TIME MULTI-THREADED GS-DRONE SIMULATION ===\n');
fprintf('Implementing thread-like behavior with 6-decimal precision timing\n\n');
%% Use lightweight geometric models only
useSTL = false;
useGSModel = false;
num_GS = 10; % Fixed to 10 Ground Stations (corresponding to 0-9 in graph image)
GS_zLevel_for_graph_display = 0; % Graph nodes are drawn at Z=0
scale_factor = 2.0; % Adjust this to make the graph larger/smaller on screen
GS_positions_raw = [
    22.42, 26.07, 0;  % Node 0 (center)
    23.71, 40.70, 0;  % Node 1 (right of center)
    9.26, 28.56, 0;   % Node 2 (bottom center)
    20.09, 12.03, 0;  % Node 3 (left of center)
    35.72, 24.62, 0;  % Node 4 (top center)
    11.59, 14.79, 0;  % Node 5 (left-center)
    16.26, 50.00, 0;  % Node 6 (bottom right)
    1.00, 6.89, 0;   % Node 7 (bottom left)
    7.73, 42.53, 0;   % Node 8 (bottom right area)
    9.65, 1.00, 0    % Node 9 (top left)
];
% Apply scaling and offset to ensure positions are positive and well-distributed
GS_positions_scaled = GS_positions_raw * scale_factor;
min_x = min(GS_positions_scaled(:,1));
min_y = min(GS_positions_scaled(:,2));
offset_x = 5 - min_x; % Ensure minimum X is at least 5 for padding
offset_y = 5 - min_y;
GS_positions = GS_positions_scaled; % This will be the one used throughout the code
GS_positions(:,1) = GS_positions(:,1) + offset_x;
GS_positions(:,2) = GS_positions(:,2) + offset_y;
GS_positions(:,3) = GS_zLevel_for_graph_display; % Ensure GS are drawn at Z=0 for graph
% Edges as specified (0-based from your input, converted to 1-based for MATLAB)
edges_0based = [
    0, 1;
    0, 2;
    0, 3;
    0, 4;
    1, 5;
    1, 6;
    2, 7;
    2, 8;
    3, 9;
    3, 5;
    5, 7;
    6, 8;
    7, 9
];
edges = edges_0based + 1; % Convert to 1-based indexing for MATLAB

% --- END: NEW GRAPH DEFINITION ---
% Original altitude slots (THESE ARE STILL USED FOR DRONE MOVEMENT LOGIC)
altitude_slots = [1, 2, 3]; % Available height levels: Slot 1=3m, Slot 2=5m, Slot 3=7m
%% Build Adjacency Matrix from Fixed Edges (for Dijkstra)
adjacency = zeros(num_GS);
for i = 1:size(edges, 1)
    node1 = edges(i, 1);
    node2 = edges(i, 2);
    adjacency(node1, node2) = 1;
    adjacency(node2, node1) = 1; % undirected graph
end
%% Distance Graph for Pathfinding (using new adjacency)
distGraph = inf(num_GS);
for i = 1:num_GS
    for j = 1:num_GS
        if adjacency(i,j) == 1
            distGraph(i,j) = norm(GS_positions(i,1:2) - GS_positions(j,1:2));
        end
    end
end
%% Initialize Ground Station Structure with Thread-like Properties
for gs_id = 1:num_GS
    GS(gs_id).id = gs_id;
    GS(gs_id).position = GS_positions(gs_id, :);
    GS(gs_id).slots = altitude_slots; % All GS have access to same slot heights
    GS(gs_id).thread_id = sprintf('GS_Thread_%02d', gs_id);
    GS(gs_id).last_update_time = 0;
    GS(gs_id).processing_queue = [];
        % Status table for each slot (All slots are available at each GS, but specific outbound is preferred)
    for slot_idx = 1:length(altitude_slots)
        GS(gs_id).status_table(slot_idx).slot_altitude = altitude_slots(slot_idx);
        GS(gs_id).status_table(slot_idx).is_reserved = false;
        GS(gs_id).status_table(slot_idx).reserved_by = 0;
        GS(gs_id).status_table(slot_idx).lock_start_time = 0;
        GS(gs_id).status_table(slot_idx).lock_duration = 0; % Duration of reservation
        GS(gs_id).status_table(slot_idx).pending_grant_to_drone = 0; % NEW: Store which drone a grant is pending for
    end
        % Thread-like message queues
    GS(gs_id).incoming_requests = [];
    GS(gs_id).pending_grants = []; % Renamed from pending_acks to be clearer: these are grants sent, awaiting ACK
    GS(gs_id).incoming_acks = []; % NEW: Queue for incoming acknowledgments from drones
    GS(gs_id).outgoing_messages = [];
        fprintf('[%012.6f] [%s] INITIALIZED at position [%.1f, %.1f, %.1f]\n', ...
            0.000000, GS(gs_id).thread_id, GS(gs_id).position(1), GS(gs_id).position(2), GS(gs_id).position(3));
end
%% Initialize drones with thread-like properties for Dijkstra
% Mission waypoints (using 1-based indexing for GS IDs 1-10)
drones(1).mission_waypoints = [1 7]; % From GS1 (Node 0) to GS8 (Node 7)
drones(2).mission_waypoints = [3 6]; % From GS3 (Node 2) to GS6 (Node 5)
drones(3).mission_waypoints = [7 2]; % From GS7 (Node 6) to GS2 (Node 1)
drones(4).mission_waypoints = [6 3]; % From GS6 (Node 5) to GS3 (Node 2)
drones(5).mission_waypoints = [9 8]; % From GS9 (Node 8) to GS8 (Node 7)
drones(6).mission_waypoints = [2 3]; % From GS2 (Node 1) to GS3 (Node 2)
colors = [
    0.8 0.2 0.2;    % Red
    0.2 0.8 0.2;    % Green
    0.2 0.2 0.8;    % Blue
    0.8 0.8 0.2;    % Yellow
    0.8 0.2 0.8;    % Magenta
    0.2 0.8 0.8;    % Cyan
];
numDrones = length(drones);
%% Initialize drone properties with thread identifiers
for i = 1:numDrones
    drones(i).id = i;
    drones(i).thread_id = sprintf('DR_Thread_%02d', i);
    drones(i).current_gs = drones(i).mission_waypoints(1); % Start at first mission waypoint
    drones(i).final_destination_gs = drones(i).mission_waypoints(end); % Target is last mission waypoint
    drones(i).path_index = 0; % Initialize to 0 to trigger initial path calculation
    drones(i).full_calculated_path = []; % Stores the path calculated by Dijkstra
    drones(i).active = false;
    drones(i).color = colors(i,:);
    drones(i).startTime = 2.0 * (i-1); % Stagger start times
    drones(i).last_update_time = 0;
        % Algorithm-specific states
    drones(i).state = 'IDLE'; % States: 'IDLE', 'PLANNING', 'REQUESTING_SLOT', 'SLOT_GRANTED', 'SLOT_DENIED', 'TAKING_OFF', 'FLYING_HORIZONTAL', 'LANDING', 'MISSION_COMPLETE'
    drones(i).target_gs = 0; % Immediate next GS in the path
    drones(i).assigned_slot = 0; % Actual Z-level assigned by GS
    drones(i).request_time = 0;
    drones(i).grant_received_time = 0; % NEW: Time when SLOT_GRANTED was received
    drones(i).flight_start_time = 0;
        % Position and movement
    drones(i).pos = GS_positions(drones(i).current_gs, :);
    drones(i).pos(3) = 0; % Ensure initial drone position is at Z=0 (ground level)
    drones(i).target_pos = drones(i).pos; % This will be dynamic based on flight phase
        % Graphics handles
    drones(i).patchHandle = [];
    drones(i).textHandle = [];
        fprintf('[%012.6f] [%s] INITIALIZED at GS-%d, Mission: [%s] to GS-%d\n', ...
            0.000000, drones(i).thread_id, drones(i).current_gs-1, ... % Display as 0-based for user
            num2str(drones(i).mission_waypoints-1, '%d '), drones(i).final_destination_gs-1); % Display as 0-based for user
end
%% Simulation parameters with high precision timing
simDuration = 120; % Increased duration for longer paths in new graph
frameRate = 100; % High frame rate for precise timing
totalFrames = simDuration * frameRate;
dt = 1/frameRate; % 0.01 second precision
% --- DRONE SPEED PARAMETER ---
% Set this value to control the drone's speed in meters per simulation second.
% If droneSpeed = 1.0, 1 meter of movement takes 1 second of simulation time.
% Increase for faster visual movement, decrease for slower.
droneSpeed = 1.0; % 1 meter per second (both horizontal and vertical)
request_timeout = 2.0; % Timeout for slot requests
ack_timeout = 1.0; % NEW: Timeout for drone to send ACK after receiving grant
grant_timeout = 2.0; % NEW: Timeout for GS to consider a grant offer valid (before reservation)
% Note: Your original code had a fixed 3s buffer for lock_duration calculation.
% We'll keep that for consistency with your existing logic.
% High precision timing
start_time = tic;
%% Setup visualization
% Calculate plot boundaries based on new GS positions
max_x = max(GS_positions(:,1)) + 5;
max_y = max(GS_positions(:,2)) + 5;
max_z = max(altitude_slots) + 5; % Max slot height + buffer (from original logic)
figure('Position', [100, 100, 1400, 900]);
set(gcf, 'Color', 'white');
axis([0 max_x 0 max_y 0 max_z]); % Dynamic axis limits
xlabel('X Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Altitude (m)', 'FontSize', 12, 'FontWeight', 'bold');
view(-30, 25);
grid on;
hold on;
camlight('headlight');
lighting gouraud;
material([0.3 0.6 0.9 20]);
%% Draw Ground Stations
gs_handles = gobjects(num_GS, 1);
for g = 1:num_GS
    gs_handles(g) = plot3(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3), ...
                         'ks', 'MarkerSize', 12, 'MarkerFaceColor', [0.3 0.3 0.9], ...
                         'LineWidth', 2);
    text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3)+0.5, ...
         sprintf('GS%d', g-1), 'FontSize', 9, 'FontWeight', 'bold', ... % Display as 0-based
         'Color', [0 0 0.8], 'HorizontalAlignment', 'center');
    
    % --- REMOVED: Display outbound slot ID (from graph, not used for logic) ---
    % text(GS_positions(g,1), GS_positions(g,2), GS_positions(g,3)+1.5, ...
    %      sprintf('Out: Slot%d', outboundSlots_graph_id(g)), 'FontSize', 8, 'FontWeight', 'bold', ...
    %      'Color', [0.8 0.4 0], 'HorizontalAlignment', 'center'); % Orange color
    % ----------------------------------------------------------------------
end
% --- NEW: Draw Edges based on Fixed Graph Structure ---
for i = 1:size(edges, 1)
    node1_idx = edges(i, 1);
    node2_idx = edges(i, 2);
    pt1 = GS_positions(node1_idx, 1:2);
    pt2 = GS_positions(node2_idx, 1:2);
    plot3([pt1(1), pt2(1)], [pt1(2), pt2(2)], [GS_zLevel_for_graph_display, GS_zLevel_for_graph_display], ...
          'Color', [0.1 0.7 0.1], 'LineWidth', 1.5); % Green lines for connections
end
% -----------------------------------------------------------------
% --- Draw conceptual mission paths (start to final destination) ---
% These lines show the overall high-level objective for each drone.
for i = 1:numDrones
    start_gs_pos = GS_positions(drones(i).mission_waypoints(1),:);
    end_gs_pos = GS_positions(drones(i).mission_waypoints(end),:);
    plot3([start_gs_pos(1), end_gs_pos(1)], [start_gs_pos(2), end_gs_pos(2)], [start_gs_pos(3), end_gs_pos(3)], ...
          'g--', 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d Mission', i));
end
% -----------------------------------------------------------------
titleHandle = title('');
fprintf('\n=== STARTING REAL-TIME SIMULATION ===\n');
fprintf('Precision: 6 decimal places | Frame Rate: %d Hz | dt: %.6f s\n\n', frameRate, dt);
%% Main Simulation Loop with Real-Time Thread Behavior
for frame = 1:totalFrames
    % High precision current time
    currTime = toc(start_time);
    frame_time = frame * dt;
    %% THREAD 1-10: Ground Station Processing Threads
    for gs_id = 1:num_GS
        gs_thread_start = tic;
                % Check if GS thread needs to process
        if currTime - GS(gs_id).last_update_time >= dt * 0.1 % GS processes 10x faster
                        %% GS Thread: Release expired slot locks
            for slot_idx = 1:length(GS(gs_id).status_table)
                if GS(gs_id).status_table(slot_idx).is_reserved
                    lock_end_time = GS(gs_id).status_table(slot_idx).lock_start_time + ...
                                   GS(gs_id).status_table(slot_idx).lock_duration;
                    if currTime >= lock_end_time
                        GS(gs_id).status_table(slot_idx).is_reserved = false;
                        GS(gs_id).status_table(slot_idx).reserved_by = 0;
                        GS(gs_id).status_table(slot_idx).pending_grant_to_drone = 0; % Clear any pending grant for this slot
                                                fprintf('[%012.6f] [%s] SLOT_RELEASED: Altitude %.1fm freed (lock expired)\n', ...
                                currTime, GS(gs_id).thread_id, GS(gs_id).status_table(slot_idx).slot_altitude);
                    end
                end
            end

            % NEW: GS Thread: Process incoming acknowledgments from drones
            for ack_idx = length(GS(gs_id).incoming_acks):-1:1
                ack_msg = GS(gs_id).incoming_acks(ack_idx);
                drone_id = ack_msg.drone_id;
                acknowledged_slot_alt = ack_msg.slot_altitude;

                fprintf('[%012.6f] [%s] RECEIVED_ACK: From Drone %d for slot %.1fm\n', ...
                    currTime, GS(gs_id).thread_id, drone_id, acknowledged_slot_alt);

                % Find the corresponding pending grant and reserve the slot
                grant_found_and_reserved = false;
                for grant_pend_idx = length(GS(gs_id).pending_grants):-1:1
                    if GS(gs_id).pending_grants(grant_pend_idx).drone_id == drone_id && ...
                       GS(gs_id).pending_grants(grant_pend_idx).slot_altitude == acknowledged_slot_alt
                        
                        % Find the actual slot index in status_table
                        slot_idx_to_reserve = -1;
                        for s = 1:length(GS(gs_id).status_table)
                            if GS(gs_id).status_table(s).slot_altitude == acknowledged_slot_alt && ...
                               ~GS(gs_id).status_table(s).is_reserved && ...
                                GS(gs_id).status_table(s).pending_grant_to_drone == drone_id % Ensure this slot was pending for THIS drone
                                slot_idx_to_reserve = s;
                                break;
                            end
                        end

                        if slot_idx_to_reserve ~= -1
                            % Calculate travel time for locking duration (consider 3 phases)
                            from_pos_xy = GS_positions(ack_msg.from_gs, 1:2); % Use from_gs from the ACK
                            to_pos_xy = GS_positions(gs_id, 1:2);
                            horizontal_distance = norm(to_pos_xy - from_pos_xy);
                            max_alt_diff = max(altitude_slots);
                            travel_time = (max_alt_diff / droneSpeed) + ... % Takeoff
                                          (horizontal_distance / droneSpeed) + ... % Horizontal
                                          (max_alt_diff / droneSpeed) + 3; % Landing + Buffer
                            
                            % Reserve the slot NOW that ACK is received
                            GS(gs_id).status_table(slot_idx_to_reserve).is_reserved = true;
                            GS(gs_id).status_table(slot_idx_to_reserve).reserved_by = drone_id;
                            GS(gs_id).status_table(slot_idx_to_reserve).lock_start_time = currTime;
                            GS(gs_id).status_table(slot_idx_to_reserve).lock_duration = travel_time;
                            GS(gs_id).status_table(slot_idx_to_reserve).pending_grant_to_drone = 0; % Clear pending grant

                            fprintf('[%012.6f] [%s] SLOT_RESERVED: Altitude %.1fm for Drone %d (ACK received)\n', ...
                                currTime, GS(gs_id).thread_id, acknowledged_slot_alt, drone_id);
                            grant_found_and_reserved = true;
                        else
                            fprintf('[%012.6f] [%s] ACK_ERROR: Could not find available slot (%.1fm) for Drone %d to reserve despite ACK.\n', ...
                                currTime, GS(gs_id).thread_id, acknowledged_slot_alt, drone_id);
                        end

                        % Remove the processed pending grant
                        GS(gs_id).pending_grants(grant_pend_idx) = [];
                        break;
                    end
                end
                % Remove processed ACK
                GS(gs_id).incoming_acks(ack_idx) = [];
            end
                        %% GS Thread: Process incoming slot requests (OFFER phase)
            for req_idx = length(GS(gs_id).incoming_requests):-1:1
                request = GS(gs_id).incoming_requests(req_idx);
                drone_id = request.drone_id;
                                fprintf('[%012.6f] [%s] PROCESSING_REQUEST: From Drone %d (origin: GS-%d)\n', ...
                        currTime, GS(gs_id).thread_id, drone_id, request.from_gs-1); % Display 0-based
                                % Find available slot that is NOT already reserved and NOT pending a grant to another drone
                slot_found = false;
                for slot_idx = 1:length(GS(gs_id).status_table)
                    if ~GS(gs_id).status_table(slot_idx).is_reserved && GS(gs_id).status_table(slot_idx).pending_grant_to_drone == 0
                        % Mark this slot as 'pending grant' to this drone, but don't reserve yet
                        GS(gs_id).status_table(slot_idx).pending_grant_to_drone = drone_id;

                        % Send slot info to requesting drone (this is an OFFER)
                        drones(drone_id).assigned_slot = GS(gs_id).status_table(slot_idx).slot_altitude;
                        drones(drone_id).state = 'SLOT_GRANTED'; % Drone will interpret this as an offer
                        drones(drone_id).grant_received_time = currTime; % Record time of offer reception

                        % Add to pending grants list (awaiting ACK from drone)
                        grant_offer.drone_id = drone_id;
                        grant_offer.slot_altitude = GS(gs_id).status_table(slot_idx).slot_altitude;
                        grant_offer.grant_time = currTime;
                        grant_offer.slot_idx = slot_idx; % Store slot index for later reservation
                        GS(gs_id).pending_grants = [GS(gs_id).pending_grants, grant_offer];
                                                
                        slot_found = true;
                        fprintf('[%012.6f] [%s] SLOT_OFFERED: Altitude %.1fm to Drone %d. Awaiting ACK.\n', ...
                                currTime, GS(gs_id).thread_id, GS(gs_id).status_table(slot_idx).slot_altitude, drone_id);
                        break;
                    end
                end
                                if ~slot_found
                    % Send FAILURE response
                    drones(drone_id).state = 'SLOT_DENIED';
                    fprintf('[%012.6f] [%s] SLOT_DENIED: All slots occupied → Drone %d\n', ...
                            currTime, GS(gs_id).thread_id, drone_id);
                end
                                % Remove processed request
                GS(gs_id).incoming_requests(req_idx) = [];
            end
                        %% GS Thread: Check for pending grant timeouts (if no ACK received)
            for grant_idx = length(GS(gs_id).pending_grants):-1:1
                grant_offer = GS(gs_id).pending_grants(grant_idx);
                if currTime - grant_offer.grant_time > grant_timeout % Timeout for drone to ACK
                    % The drone failed to ACK in time, clear the pending grant status for this slot
                    GS(gs_id).status_table(grant_offer.slot_idx).pending_grant_to_drone = 0; % Clear the pending flag
                    fprintf('[%012.6f] [%s] GRANT_TIMEOUT: Offer for slot %.1fm to Drone %d expired (no ACK).\n', ...
                            currTime, GS(gs_id).thread_id, grant_offer.slot_altitude, grant_offer.drone_id);
                    GS(gs_id).pending_grants(grant_idx) = [];
                end
            end
                        GS(gs_id).last_update_time = currTime;
            gs_processing_time = toc(gs_thread_start);
                        % Log GS thread processing time if significant
            if gs_processing_time > 0.001
                fprintf('[%012.6f] [%s] THREAD_STATS: Processing time %.6fs\n', ...
                        currTime, GS(gs_id).thread_id, gs_processing_time);
            end
        end
    end
        %% THREAD 11-16: Drone Processing Threads
    for d = 1:numDrones
        if drones(d).active || currTime >= drones(d).startTime
            drone_thread_start = tic;
                        %% Drone Thread: Activation
            if ~drones(d).active && currTime >= drones(d).startTime
                drones(d).active = true;
                drones(d).state = 'PLANNING';
                fprintf('[%012.6f] [%s] THREAD_ACTIVATED: Starting at GS-%d, Mission to GS-%d\n', ...
                        currTime, drones(d).thread_id, drones(d).current_gs-1, drones(d).final_destination_gs-1); % Display 0-based
            end
                        if drones(d).active
                prev_state = drones(d).state;
                current_drone_speed = droneSpeed; % Always droneSpeed
                switch drones(d).state
                    case 'PLANNING'
                        % Check if final destination reached
                        if drones(d).current_gs == drones(d).final_destination_gs
                            drones(d).state = 'MISSION_COMPLETE';
                            fprintf('[%012.6f] [%s] MISSION_COMPLETE: Reached final destination GS-%d\n', ...
                                    currTime, drones(d).thread_id, drones(d).final_destination_gs-1); % Display 0-based
                        else
                            % Check if a new path needs to be calculated or if current segment is done
                            if isempty(drones(d).full_calculated_path) || ...
                               (drones(d).path_index >= length(drones(d).full_calculated_path)) || ...
                               (drones(d).full_calculated_path(drones(d).path_index) ~= drones(d).current_gs) % Recalculate if somehow off path
                                
                                % Recalculate path from current_gs to final_destination_gs
                                [path_nodes, ~] = findShortestPath(distGraph, drones(d).id, drones(d).final_destination_gs); % Fix: Should be current_gs as first argument
                                % Correction: The first argument to findShortestPath should be the starting GS, not drone ID.
                                % It seems you were passing drones(d).id, but it should be drones(d).current_gs.
                                % Assuming findShortestPath is a function you've defined elsewhere:
                                [path_nodes, ~] = findShortestPath(distGraph, drones(d).current_gs, drones(d).final_destination_gs);

                                if isempty(path_nodes) || length(path_nodes) < 2 % No path or only start node
                                    fprintf('[%012.6f] [%s] PATHFINDING_ERROR: No path found from GS-%d to GS-%d. Drone Stuck.\n', ...
                                            currTime, drones(d).thread_id, drones(d).current_gs-1, drones(d).final_destination_gs-1); % Display 0-based
                                    drones(d).state = 'STUCK'; % Introduce a stuck state
                                    continue; % Skip further processing for this drone this frame
                                end
                                drones(d).full_calculated_path = path_nodes;
                                drones(d).path_index = 1; % Start at the first node of the new path (which is current_gs)
                                fprintf('[%012.6f] [%s] PATH_CALCULATED: New path from GS-%d to GS-%d: [%s]\n', ...
                                        currTime, drones(d).thread_id, drones(d).current_gs-1, drones(d).final_destination_gs-1, num2str(drones(d).full_calculated_path-1)); % Display 0-based
                            end
            
                            % Determine the immediate next hop in the calculated path
                            if drones(d).path_index + 1 <= length(drones(d).full_calculated_path)
                                drones(d).target_gs = drones(d).full_calculated_path(drones(d).path_index + 1);
                                drones(d).state = 'REQUESTING_SLOT';
                                drones(d).request_time = currTime;
            
                                % Send slot request to target GS
                                request.drone_id = drones(d).id;
                                request.from_gs = drones(d).current_gs; % Pass current GS to calculate travel time accurately
                                request.request_time = currTime;
                                GS(drones(d).target_gs).incoming_requests = ...
                                    [GS(drones(d).target_gs).incoming_requests, request];
            
                                fprintf('[%012.6f] [%s] SLOT_REQUEST_SENT: Target GS-%d (from GS-%d, via Dijkstra path)\n', ...
                                    currTime, drones(d).thread_id, drones(d).target_gs-1, drones(d).current_gs-1); % Display 0-based
                            else
                                % This case should ideally not be reached if final_destination_gs check is correct,
                                % but good for robustness if path_index logic gets out of sync.
                                fprintf('[%012.6f] [%s] PATH_LOGIC_ERROR: No next hop in calculated path from GS-%d. Marking mission complete.\n', ...
                                        currTime, drones(d).thread_id, drones(d).current_gs-1); % Display 0-based
                                drones(d).state = 'MISSION_COMPLETE'; % Or stuck
                            end
                        end
                                            case 'REQUESTING_SLOT'
                        % Wait for slot response or timeout
                        if currTime - drones(d).request_time > request_timeout
                            drones(d).state = 'SLOT_DENIED';
                            fprintf('[%012.6f] [%s] REQUEST_TIMEOUT: No response from GS-%d (%.3fs elapsed)\n', ...
                                    currTime, drones(d).thread_id, drones(d).target_gs-1, currTime - drones(d).request_time); % Display 0-based
                        end
                                            case 'SLOT_GRANTED'
                        % NEW: Drone receives grant, now send ACK to GS
                        if currTime - drones(d).grant_received_time < ack_timeout % Give the drone a chance to ACK
                            ack_msg.drone_id = drones(d).id;
                            ack_msg.slot_altitude = drones(d).assigned_slot;
                            ack_msg.from_gs = drones(d).current_gs; % Include where the drone is currently
                            GS(drones(d).target_gs).incoming_acks = [GS(drones(d).target_gs).incoming_acks, ack_msg];
                            
                            drones(d).state = 'TAKING_OFF'; % Transition to takeoff immediately after sending ACK
                            drones(d).flight_start_time = currTime;
                            % Target for takeoff is current X/Y, assigned Z
                            drones(d).target_pos = [drones(d).pos(1:2), drones(d).assigned_slot];
                            fprintf('[%012.6f] [%s] ACK_SENT_INIT_TAKEOFF: From GS-%d → Altitude %.1fm. Sent ACK to GS-%d.\n', ...
                                    currTime, drones(d).thread_id, drones(d).current_gs-1, drones(d).assigned_slot, drones(d).target_gs-1); % Display 0-based
                        else
                            % If drone somehow stays in SLOT_GRANTED too long without sending ACK
                            fprintf('[%012.6f] [%s] ACK_FAILED: Did not send ACK in time for grant from GS-%d. Retrying planning.\n', ...
                                currTime, drones(d).thread_id, drones(d).target_gs-1);
                            drones(d).state = 'PLANNING'; % Go back to planning
                        end

                    case 'SLOT_DENIED'
                        % Retry after delay. If the path from Dijkstra has many hops, it's possible this happens mid-path.
                        % For simplicity, we just go back to PLANNING to recalculate or re-request.
                        if currTime - drones(d).request_time > 2.0
                            drones(d).state = 'PLANNING';
                            fprintf('[%012.6f] [%s] RETRY_PLANNING: Retrying slot request after denial for GS-%d.\n', ...
                                    currTime, drones(d).thread_id, drones(d).target_gs-1); % Display 0-based
                        end
                    case 'STUCK'
                        % Drone is stuck, no path found or other unrecoverable error.
                        % No action, remains stuck.
                        if mod(frame, 100) == 0 % Log periodically
                            fprintf('[%012.6f] [%s] STUCK: Drone is unable to find a path or proceed.\n', ...
                                currTime, drones(d).thread_id);
                        end
                                            case 'TAKING_OFF'
                        % Move vertically toward assigned altitude
                        distance_to_target_z = abs(drones(d).target_pos(3) - drones(d).pos(3));
                        if distance_to_target_z > 0.05 % Threshold for "reached"
                            direction_z = sign(drones(d).target_pos(3) - drones(d).pos(3));
                            move_distance_z = min(current_drone_speed * dt, distance_to_target_z);
                            drones(d).pos(3) = drones(d).pos(3) + direction_z * move_distance_z;
                            % Keep X/Y fixed during takeoff
                            drones(d).pos(1:2) = GS_positions(drones(d).current_gs, 1:2);
                            if mod(frame, 50) == 0
                                fprintf('[%012.6f] [%s] TAKING_OFF: [%.2f,%.2f,%.2f] → (Target Z: %.1f, Remaining Z: %.2fm)\n', ...
                                        currTime, drones(d).thread_id, drones(d).pos(1), drones(d).pos(2), drones(d).pos(3), ...
                                        drones(d).assigned_slot, distance_to_target_z);
                            end
                        else
                            drones(d).pos(3) = drones(d).assigned_slot; % Snap to exact altitude
                            % Now, set target for horizontal flight
                            drones(d).target_pos = [GS_positions(drones(d).target_gs, 1:2), drones(d).assigned_slot];
                            drones(d).state = 'FLYING_HORIZONTAL';
                            fprintf('[%012.6f] [%s] TAKEOFF_COMPLETE: At altitude %.1fm. Starting horizontal flight to GS-%d\n', ...
                                    currTime, drones(d).thread_id, drones(d).assigned_slot, drones(d).target_gs-1); % Display 0-based
                        end
                    case 'FLYING_HORIZONTAL'
                        % Move horizontally toward target GS X/Y, maintain altitude
                        distance_to_target_xy = norm(drones(d).target_pos(1:2) - drones(d).pos(1:2));
                        if distance_to_target_xy > 0.1 % Threshold for "reached" X/Y
                            direction_xy = (drones(d).target_pos(1:2) - drones(d).pos(1:2)) / distance_to_target_xy;
                            move_distance_xy = min(current_drone_speed * dt, distance_to_target_xy);
                            drones(d).pos(1:2) = drones(d).pos(1:2) + direction_xy * move_distance_xy;
                            % Maintain Z during horizontal flight
                            drones(d).pos(3) = drones(d).assigned_slot;
                            if mod(frame, 50) == 0
                                fprintf('[%012.6f] [%s] FLYING_HORIZONTAL: [%.2f,%.2f,%.2f] (remaining XY: %.2fm)\n', ...
                                        currTime, drones(d).thread_id, drones(d).pos(1), drones(d).pos(2), drones(d).pos(3), ...
                                        distance_to_target_xy);
                            end
                        else
                            drones(d).pos(1:2) = GS_positions(drones(d).target_gs, 1:2); % Snap to exact target X/Y
                            % Now, set target for landing
                            drones(d).target_pos = [GS_positions(drones(d).target_gs, 1:2), 0]; % Target Z is ground
                            drones(d).state = 'LANDING';
                            fprintf('[%012.6f] [%s] HORIZONTAL_FLIGHT_COMPLETE: Above GS-%d. Initiating landing.\n', ...
                                    currTime, drones(d).thread_id, drones(d).target_gs-1); % Display 0-based
                        end
                    case 'LANDING'
                        % Move vertically down to ground (z=0)
                        distance_to_target_z = abs(drones(d).target_pos(3) - drones(d).pos(3));
                        if distance_to_target_z > 0.05 % Threshold for "reached"
                            direction_z = sign(drones(d).target_pos(3) - drones(d).pos(3)); % Should be negative
                            move_distance_z = min(current_drone_speed * dt, distance_to_target_z);
                            drones(d).pos(3) = drones(d).pos(3) + direction_z * move_distance_z;
                            % Keep X/Y fixed during landing
                            drones(d).pos(1:2) = GS_positions(drones(d).target_gs, 1:2);
                            if mod(frame, 50) == 0
                                fprintf('[%012.6f] [%s] LANDING: [%.2f,%.2f,%.2f] → (Target Z: %.1f, Remaining Z: %.2fm)\n', ...
                                        currTime, drones(d).thread_id, drones(d).pos(1), drones(d).pos(2), drones(d).pos(3), ...
                                        drones(d).target_pos(3), distance_to_target_z);
                            end
                        else
                            drones(d).pos(3) = 0; % Snap to ground
                            % Arrived at target GS
                            drones(d).current_gs = drones(d).target_gs;
                            drones(d).path_index = drones(d).path_index + 1; % Advance in Dijkstra-calculated path
                            drones(d).state = 'PLANNING'; % Go back to planning for the next leg (or final destination)
                            flight_duration = currTime - drones(d).flight_start_time;
                            fprintf('[%012.6f] [%s] LANDING_COMPLETE: @ GS-%d (total flight_duration: %.3fs)\n', ...
                                    currTime, drones(d).thread_id, drones(d).current_gs-1, flight_duration); % Display 0-based
                        end
                    case 'MISSION_COMPLETE'
                        % Stay at current position - no action needed
                end
                                % Log state changes
                if ~strcmp(prev_state, drones(d).state)
                    fprintf('[%012.6f] [%s] STATE_CHANGE: %s → %s\n', ...
                            currTime, drones(d).thread_id, prev_state, drones(d).state);
                end
                                drones(d).last_update_time = currTime;
                drone_processing_time = toc(drone_thread_start);
                                % Log drone thread processing time if significant
                if drone_processing_time > 0.001
                    fprintf('[%012.6f] [%s] THREAD_STATS: Processing time %.6fs\n', ...
                            currTime, drones(d).thread_id, drone_processing_time);
                end
            end
                        %% Update drone visualization
            if drones(d).active && ~strcmp(drones(d).state, 'MISSION_COMPLETE') && ~strcmp(drones(d).state, 'STUCK')
                if isempty(drones(d).patchHandle) || ~isvalid(drones(d).patchHandle)
                    % Use plot3 for a filled circle
                    drones(d).patchHandle = plot3(drones(d).pos(1), drones(d).pos(2), drones(d).pos(3), ...
                                                'o', 'MarkerSize', 8, 'MarkerFaceColor', drones(d).color, ...
                                                'MarkerEdgeColor', 'k', 'LineWidth', 1); % 'o' for circle
                else
                    % Update position for the existing circle
                    set(drones(d).patchHandle, 'XData', drones(d).pos(1), ...
                                              'YData', drones(d).pos(2), ...
                                              'ZData', drones(d).pos(3));
                end
                % Update text label for drone
                if isempty(drones(d).textHandle) || ~isvalid(drones(d).textHandle)
                    drones(d).textHandle = text(drones(d).pos(1), drones(d).pos(2), drones(d).pos(3)+1.0, ...
                                               sprintf('DR%d\n%s', drones(d).id, drones(d).state), ...
                                               'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', 'k');
                else
                    set(drones(d).textHandle, 'Position', [drones(d).pos(1), drones(d).pos(2), drones(d).pos(3)+1.0], ...
                                               'String', sprintf('DR%d\n%s', drones(d).id, drones(d).state));
                end
            elseif drones(d).active && (strcmp(drones(d).state, 'MISSION_COMPLETE') || strcmp(drones(d).state, 'STUCK'))
                % If mission complete or stuck, ensure visuals are correct (e.g., at ground)
                if isvalid(drones(d).patchHandle)
                    set(drones(d).patchHandle, 'XData', drones(d).pos(1), ...
                                              'YData', drones(d).pos(2), ...
                                              'ZData', 0, ... % Snap to ground if completed/stuck
                                              'MarkerFaceColor', [0.5 0.5 0.5]); % Grey out if completed/stuck
                end
                if isvalid(drones(d).textHandle)
                     set(drones(d).textHandle, 'Position', [drones(d).pos(1), drones(d).pos(2), 0.5], ...
                                               'String', sprintf('DR%d\n%s', drones(d).id, drones(d).state));
                end
            end
        end
    end
    %% Update simulation title with time
    set(titleHandle, 'String', sprintf('Time: %.6f s / %.0f s (Frame: %d/%d)', currTime, simDuration, frame, totalFrames));
    drawnow limitrate; % Only update plot when MATLAB is ready for it
    % Enforce real-time (or near real-time) simulation speed
    elapsed_since_frame_start = toc(start_time) - currTime; % Time elapsed since the start of *this* frame's computation
    
    % Calculate time to wait until next frame's scheduled start
    % This ensures the simulation tries to run in real-time, even if a frame takes longer.
    % We account for `dt` (target frame duration) and the time already spent in this frame.
    time_to_wait = dt - elapsed_since_frame_start;
    
    if time_to_wait > 0
        pause(time_to_wait);
    end
end
fprintf('\n=== SIMULATION ENDED ===\n');

%% Helper Function: Dijkstra's Algorithm (Assuming this function is defined elsewhere)
function [path, total_distance] = findShortestPath(graph, startNode, endNode)
    % graph is an adjacency matrix where graph(i,j) is the distance from i to j
    % or inf if no direct connection.
    numNodes = size(graph, 1);
    
    dist = inf(1, numNodes);
    prev = zeros(1, numNodes);
    visited = false(1, numNodes);
    
    dist(startNode) = 0;
    
    for i = 1:numNodes
        % Find unvisited node with the smallest distance
        [min_dist, u] = min(dist(~visited));
        if isinf(min_dist)
            break; % All remaining unvisited nodes are unreachable
        end
        
        % Map u back to its original index if using a filtered array
        % A more robust way:
        temp_dist = dist;
        temp_dist(visited) = inf; % Ignore visited nodes
        [~, u] = min(temp_dist); % Find the actual index of the minimum unvisited distance
        
        visited(u) = true;
        
        % Explore neighbors
        for v = 1:numNodes
            if graph(u,v) > 0 && ~visited(v) % Check for direct connection and if not visited
                alt = dist(u) + graph(u,v);
                if alt < dist(v)
                    dist(v) = alt;
                    prev(v) = u;
                end
            end
        end
    end
    
    % Reconstruct path
    path = [];
    total_distance = dist(endNode);
    if prev(endNode) ~= 0 || startNode == endNode % Path exists or start=end
        curr = endNode;
        while curr ~= 0
            path = [curr, path];
            curr = prev(curr);
        end
    end
    
    % If startNode and endNode are the same and path is empty, initialize it
    if isempty(path) && startNode == endNode
        path = startNode;
    end
end