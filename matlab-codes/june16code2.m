clc; clear;
%% Real-Time Multi-Threaded Simulation with Precise Timing
fprintf('=== REAL-TIME MULTI-THREADED GS-DRONE SIMULATION ===\n');
fprintf('Implementing thread-like behavior with 6-decimal precision timing\n\n');

%% Use lightweight geometric models only
[V_drone, F_drone] = createSimpleDrone();
useSTL = false;
useGSModel = false;

%% Define GS positions (all at z=0)
GS_positions = [5 5 0;      % GS 1 - Central hub
                15 5 0;     % GS 2 - East
                5 15 0;     % GS 3 - North
                15 15 0;    % GS 4 - Northeast
                10 10 0;    % GS 5 - Center-north
                20 10 0;    % GS 6 - Far east
                10 20 0;    % GS 7 - Far north
                20 20 0;    % GS 8 - Far northeast
                0 10 0;     % GS 9 - West
                25 15 0];   % GS 10 - Extended east

% Altitude slots for different heights
altitude_slots = [3, 5, 7]; % Available height levels
num_GS = size(GS_positions, 1);

%% Initialize Ground Station Structure with Thread-like Properties
for gs_id = 1:num_GS
    GS(gs_id).id = gs_id;
    GS(gs_id).position = GS_positions(gs_id, :);
    GS(gs_id).slots = altitude_slots;
    GS(gs_id).thread_id = sprintf('GS_Thread_%02d', gs_id);
    GS(gs_id).last_update_time = 0;
    GS(gs_id).processing_queue = [];
        % Status table for each slot
    for slot_idx = 1:length(altitude_slots)
        GS(gs_id).status_table(slot_idx).slot_altitude = altitude_slots(slot_idx);
        GS(gs_id).status_table(slot_idx).is_reserved = false;
        GS(gs_id).status_table(slot_idx).reserved_by = 0;
        GS(gs_id).status_table(slot_idx).lock_start_time = 0;
        GS(gs_id).status_table(slot_idx).lock_duration = 0;
    end
        % Thread-like message queues
    GS(gs_id).incoming_requests = [];
    GS(gs_id).pending_acks = [];
    GS(gs_id).outgoing_messages = [];
        fprintf('[%012.6f] [%s] INITIALIZED at position [%.1f, %.1f, %.1f]\n', ...
            0.000000, GS(gs_id).thread_id, GS(gs_id).position(1), GS(gs_id).position(2), GS(gs_id).position(3));
end

%% Initialize drones with thread-like properties
drones(1).path = [1 2 4 8];
drones(2).path = [3 1 5 6];
drones(3).path = [7 5 1 2];
drones(4).path = [6 8 4 3];
drones(5).path = [9 1 10 8];
drones(6).path = [2 5 7 3];

colors = [
    0.8 0.2 0.2;    % Red
    0.2 0.8 0.2;    % Green
    0.2 0.2 0.8;    % Blue
    0.8 0.8 0.2;    % Yellow
    0.8 0.2 0.8;    % Magenta
    0.2 0.8 0.8     % Cyan
];
numDrones = length(drones);

%% Initialize drone properties with thread identifiers
for i = 1:numDrones
    drones(i).id = i;
    drones(i).thread_id = sprintf('DR_Thread_%02d', i);
    drones(i).current_gs = drones(i).path(1);
    drones(i).path_index = 1;
    drones(i).active = false;
    drones(i).color = colors(i,:);
    drones(i).startTime = 2.0 * (i-1);
    drones(i).last_update_time = 0;
        % Algorithm-specific states
    drones(i).state = 'IDLE'; % States: 'IDLE', 'PLANNING', 'REQUESTING_SLOT', 'SLOT_GRANTED', 'SLOT_DENIED', 'TAKING_OFF', 'FLYING_HORIZONTAL', 'LANDING', 'MISSION_COMPLETE'
    drones(i).target_gs = 0;
    drones(i).assigned_slot = 0;
    drones(i).request_time = 0;
    drones(i).flight_start_time = 0;
        % Position and movement
    drones(i).pos = GS_positions(drones(i).current_gs, :);
    drones(i).target_pos = drones(i).pos; % This will be dynamic based on flight phase
        % Graphics handles
    drones(i).patchHandle = [];
    drones(i).textHandle = [];
        fprintf('[%012.6f] [%s] INITIALIZED at GS-%d, Path: [%s]\n', ...
            0.000000, drones(i).thread_id, drones(i).current_gs, num2str(drones(i).path, '%d '));
end

%% Simulation parameters with high precision timing
simDuration = 30;
frameRate = 100; % High frame rate for precise timing
totalFrames = simDuration * frameRate;
dt = 1/frameRate; % 0.01 second precision
droneSpeed = 2; % Overall speed, will be applied to vertical and horizontal segments
verticalSpeedFactor = 1; % Make vertical speed slightly slower if desired (e.g., 0.5 * droneSpeed)
request_timeout = 10.0;

% High precision timing
start_time = tic;

%% Setup visualization
figure('Position', [100, 100, 1400, 900]);
set(gcf, 'Color', 'white');
axis([0 30 0 25 0 12]);
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
         sprintf('GS%d', g), 'FontSize', 9, 'FontWeight', 'bold', ...
         'Color', [0 0 0.8], 'HorizontalAlignment', 'center');
end

% --- Draw lines connecting GS positions (unchanged) ---
for i = 1:numDrones
    path = drones(i).path;
    for j = 1:(length(path)-1)
        start_gs_index = path(j);
        end_gs_index = path(j+1);
        start_pos = GS_positions(start_gs_index,:);
        end_pos = GS_positions(end_gs_index,:);
        plot3([start_pos(1), end_pos(1)], [start_pos(2), end_pos(2)], [start_pos(3), end_pos(3)], 'g-', 'LineWidth', 1);
    end
end
% -----------------------------------------

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
                                                fprintf('[%012.6f] [%s] SLOT_RELEASED: Altitude %.1fm freed (lock expired)\n', ...
                                currTime, GS(gs_id).thread_id, GS(gs_id).status_table(slot_idx).slot_altitude);
                    end
                end
            end
                        %% GS Thread: Process incoming slot requests
            for req_idx = length(GS(gs_id).incoming_requests):-1:1
                request = GS(gs_id).incoming_requests(req_idx);
                drone_id = request.drone_id;
                                fprintf('[%012.6f] [%s] PROCESSING_REQUEST: From Drone %d (origin: GS-%d)\n', ...
                        currTime, GS(gs_id).thread_id, drone_id, request.from_gs);
                                % Find available slot
                slot_found = false;
                for slot_idx = 1:length(GS(gs_id).status_table)
                    if ~GS(gs_id).status_table(slot_idx).is_reserved
                        % Calculate travel time for locking duration (consider 3 phases)
                        from_pos_xy = GS_positions(request.from_gs, 1:2);
                        to_pos_xy = GS_positions(gs_id, 1:2);
                        horizontal_distance = norm(to_pos_xy - from_pos_xy);
                        % Estimate max altitude diff
                        max_alt_diff = max(altitude_slots);
                        
                        % Rough estimate for travel time: vertical_up + horizontal + vertical_down + buffer
                        travel_time = (max_alt_diff / (droneSpeed * verticalSpeedFactor)) + ... % Takeoff
                                      (horizontal_distance / droneSpeed) + ... % Horizontal
                                      (max_alt_diff / (droneSpeed * verticalSpeedFactor)) + 3; % Landing + Buffer
                                                
                        % Reserve the slot
                        GS(gs_id).status_table(slot_idx).is_reserved = true;
                        GS(gs_id).status_table(slot_idx).reserved_by = drone_id;
                        GS(gs_id).status_table(slot_idx).lock_start_time = currTime;
                        GS(gs_id).status_table(slot_idx).lock_duration = travel_time;
                                                % Send slot info to requesting drone
                        drones(drone_id).assigned_slot = GS(gs_id).status_table(slot_idx).slot_altitude;
                        drones(drone_id).state = 'SLOT_GRANTED'; % Drone will handle transition to TAKING_OFF
                                                % Add to pending acknowledgments
                        ack_request.drone_id = drone_id;
                        ack_request.slot_idx = slot_idx;
                        ack_request.grant_time = currTime;
                        GS(gs_id).pending_acks = [GS(gs_id).pending_acks, ack_request];
                                                slot_found = true;
                        fprintf('[%012.6f] [%s] SLOT_GRANTED: Altitude %.1fm → Drone %d (estimated total travel_time: %.3fs)\n', ...
                                currTime, GS(gs_id).thread_id, GS(gs_id).status_table(slot_idx).slot_altitude, ...
                                drone_id, travel_time);
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
                        %% GS Thread: Check for acknowledgment timeouts
            for ack_idx = length(GS(gs_id).pending_acks):-1:1
                ack_req = GS(gs_id).pending_acks(ack_idx);
                if currTime - ack_req.grant_time > 2.0 % 2 second timeout for ACK
                    % Release the slot if no acknowledgment received
                    GS(gs_id).status_table(ack_req.slot_idx).is_reserved = false;
                    GS(gs_id).status_table(ack_req.slot_idx).reserved_by = 0;
                    fprintf('[%012.6f] [%s] TIMEOUT_RELEASE: Slot freed (no ACK from Drone %d)\n', ...
                            currTime, GS(gs_id).thread_id, ack_req.drone_id);
                    GS(gs_id).pending_acks(ack_idx) = [];
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
                fprintf('[%012.6f] [%s] THREAD_ACTIVATED: Starting at GS-%d\n', ...
                        currTime, drones(d).thread_id, drones(d).current_gs);
            end
                        if drones(d).active
                prev_state = drones(d).state;
                                % Determine current drone speed for this frame
                current_drone_speed = droneSpeed;
                if strcmp(drones(d).state, 'TAKING_OFF') || strcmp(drones(d).state, 'LANDING')
                    current_drone_speed = droneSpeed * verticalSpeedFactor;
                end

                switch drones(d).state
                    case 'PLANNING'
                        % Extract next destination and compute next hop
                        if drones(d).path_index < length(drones(d).path)
                            drones(d).target_gs = drones(d).path(drones(d).path_index + 1);
                            drones(d).state = 'REQUESTING_SLOT';
                            drones(d).request_time = currTime;
                                                        % Send slot request to target GS
                            request.drone_id = drones(d).id;
                            request.from_gs = drones(d).current_gs;
                            request.request_time = currTime;
                            GS(drones(d).target_gs).incoming_requests = ...
                                [GS(drones(d).target_gs).incoming_requests, request];
                                                        fprintf('[%012.6f] [%s] SLOT_REQUEST_SENT: Target GS-%d (from GS-%d)\n', ...
                                    currTime, drones(d).thread_id, drones(d).target_gs, drones(d).current_gs);
                        else
                            drones(d).state = 'MISSION_COMPLETE';
                            fprintf('[%012.6f] [%s] MISSION_COMPLETE: All waypoints visited\n', ...
                                    currTime, drones(d).thread_id);
                        end
                                            case 'REQUESTING_SLOT'
                        % Wait for slot response or timeout
                        if currTime - drones(d).request_time > request_timeout
                            drones(d).state = 'SLOT_DENIED';
                            fprintf('[%012.6f] [%s] REQUEST_TIMEOUT: No response from GS-%d (%.3fs elapsed)\n', ...
                                    currTime, drones(d).thread_id, drones(d).target_gs, currTime - drones(d).request_time);
                        end
                                            case 'SLOT_GRANTED'
                        % Send acknowledgment and initiate takeoff
                        gs_id = drones(d).target_gs;
                        for ack_idx = length(GS(gs_id).pending_acks):-1:1
                            if GS(gs_id).pending_acks(ack_idx).drone_id == drones(d).id
                                GS(gs_id).pending_acks(ack_idx) = [];
                                break;
                            end
                        end
                                                drones(d).state = 'TAKING_OFF';
                        drones(d).flight_start_time = currTime;
                        % Target for takeoff is current X/Y, assigned Z
                        drones(d).target_pos = [drones(d).pos(1:2), drones(d).assigned_slot];
                        fprintf('[%012.6f] [%s] ACK_SENT_INIT_TAKEOFF: From GS-%d → Altitude %.1fm\n', ...
                                currTime, drones(d).thread_id, drones(d).current_gs, drones(d).assigned_slot);

                    case 'SLOT_DENIED'
                        % Retry after delay
                        if currTime - drones(d).request_time > 2.0
                            drones(d).state = 'PLANNING';
                            fprintf('[%012.6f] [%s] RETRY_PLANNING: Retrying slot request after denial\n', ...
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
                                    currTime, drones(d).thread_id, drones(d).assigned_slot, drones(d).target_gs);
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
                                    currTime, drones(d).thread_id, drones(d).target_gs);
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
                            drones(d).path_index = drones(d).path_index + 1;
                            drones(d).state = 'PLANNING'; % Go back to planning for the next leg
                            flight_duration = currTime - drones(d).flight_start_time;
                            fprintf('[%012.6f] [%s] LANDING_COMPLETE: @ GS-%d (total flight_duration: %.3fs)\n', ...
                                    currTime, drones(d).thread_id, drones(d).current_gs, flight_duration);
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
            if drones(d).active && ~strcmp(drones(d).state, 'MISSION_COMPLETE')
                if isempty(drones(d).patchHandle) || ~isvalid(drones(d).patchHandle)
                    [sx, sy, sz] = sphere(8);
                    sx = sx * 0.3 + drones(d).pos(1);
                    sy = sy * 0.3 + drones(d).pos(2);
                    sz = sz * 0.3 + drones(d).pos(3);
                    drones(d).patchHandle = surf(sx, sy, sz, 'FaceColor', drones(d).color, ...
                                                'EdgeColor', 'none', 'FaceAlpha', 0.8);
                else
                    [sx, sy, sz] = sphere(8);
                    sx = sx * 0.3 + drones(d).pos(1);
                    sy = sy * 0.3 + drones(d).pos(2);
                    sz = sz * 0.3 + drones(d).pos(3);
                    set(drones(d).patchHandle, 'XData', sx, 'YData', sy, 'ZData', sz);
                end
                                if mod(frame, 10) == 0
                    if ~isempty(drones(d).textHandle) && isvalid(drones(d).textHandle)
                        set(drones(d).textHandle, 'Position', drones(d).pos + [0 0 0.8], ...
                                                   'String', sprintf('D%d', drones(d).id));
                    else
                        drones(d).textHandle = text(drones(d).pos(1), drones(d).pos(2), drones(d).pos(3)+0.8, ...
                                                     sprintf('D%d', drones(d).id), ...
                                                     'FontSize', 8, 'Color', 'black', 'FontWeight', 'bold', ...
                                                     'HorizontalAlignment', 'center');
                    end
                end
            end
        end
    end
        %% Update display
    active_drones = sum([drones.active]);
    completed_drones = sum(strcmp({drones.state}, 'MISSION_COMPLETE'));
        if mod(frame, 20) == 0
        set(titleHandle, 'String', sprintf('Real-Time Multi-Threaded Simulation | T: %.6fs | Active: %d | Complete: %d/%d', ...
                          currTime, active_drones, completed_drones, numDrones));
    end
        % System status every 2 seconds
    if mod(frame, 200) == 0
        total_reserved_slots = 0;
        for gs_id = 1:num_GS
            total_reserved_slots = total_reserved_slots + sum([GS(gs_id).status_table.is_reserved]);
        end
        fprintf('[%012.6f] [SYSTEM] STATUS: Active=%d, Complete=%d, Reserved_Slots=%d\n', ...
                currTime, active_drones, completed_drones, total_reserved_slots);
    end
        drawnow;
        % Maintain real-time execution
    while toc(start_time) < frame * dt
        % Busy wait to maintain precise timing
    end
end

%% Final simulation report
total_time = toc(start_time);
fprintf('\n=== REAL-TIME SIMULATION COMPLETE ===\n');
fprintf('Total Runtime: %.6f seconds\n', total_time);
fprintf('Simulation Duration: %.6f seconds\n', simDuration);
fprintf('Frame Rate Achieved: %.2f Hz\n', totalFrames / total_time);

fprintf('\n--- FINAL THREAD STATUS ---\n');
for d = 1:numDrones
    if strcmp(drones(d).state, 'MISSION_COMPLETE')
        status = '✓ COMPLETE';
    elseif drones(d).active
        status = sprintf('◐ %s', drones(d).state);
    else
        status = '✗ NOT STARTED';
    end
    fprintf('[%s] %s | Current GS: %d | Progress: %d/%d\n', ...
            drones(d).thread_id, status, drones(d).current_gs, drones(d).path_index, length(drones(d).path));
end

fprintf('\n--- GROUND STATION THREAD STATUS ---\n');
for gs_id = 1:num_GS
    reserved_count = sum([GS(gs_id).status_table.is_reserved]);
    total_requests = length(GS(gs_id).incoming_requests);
    fprintf('[%s] Slots: %d/%d reserved, %d pending requests\n', ...
            GS(gs_id).thread_id, reserved_count, length(altitude_slots), total_requests);
end
fprintf('\n=====================================\n');

%% Helper function to create simple geometric drone
function [V, F] = createSimpleDrone()
    bodyLength = 1.0; bodyWidth = 0.4; bodyHeight = 0.15;
        V = [
        -bodyLength/2, -bodyWidth/2, -bodyHeight/2;
         bodyLength/2, -bodyWidth/2, -bodyHeight/2;
         bodyLength/2,  bodyWidth/2, -bodyHeight/2;
        -bodyLength/2,  bodyWidth/2, -bodyHeight/2;
        -bodyLength/2, -bodyWidth/2,  bodyHeight/2;
         bodyLength/2, -bodyWidth/2,  bodyHeight/2;
         bodyLength/2,  bodyWidth/2,  bodyHeight/2;
        -bodyLength/2,  bodyWidth/2,  bodyHeight/2
    ];
        armLength = 0.6;
    propPositions = [
        -armLength, -armLength, bodyHeight/2;
         armLength, -armLength, bodyHeight/2;
         armLength,  armLength, bodyHeight/2;
        -armLength,  armLength, bodyHeight/2
    ];
        V = [V; propPositions];
        F = [
        1, 2, 3, 4;
        5, 8, 7, 6;
        1, 5, 6, 2;
        3, 7, 8, 4;
        1, 4, 8, 5;
        2, 6, 7, 3
    ];
end