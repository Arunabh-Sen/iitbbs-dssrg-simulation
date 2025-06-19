clc; clear;
%% Load STL drone model or create simple geometric drone
try
    stlData = stlread('new_dronev_whole.stl');
    F_drone = stlData.ConnectivityList;
    V_drone = stlData.Points;
    scaleFactor = 0.005;
    V_drone = V_drone - mean(V_drone, 1);
    V_drone = V_drone * scaleFactor;
    useSTL = true;
catch
    % Create simple geometric drone if STL not available
    warning('STL file not found. Using geometric drone model.');
    [V_drone, F_drone] = createSimpleDrone();
    useSTL = false;
end

%% Load Ground Station model or create simple representation
try
    gsStl = stlread('Monpoc_Apartments.stl');
    F_gs = gsStl.ConnectivityList;
    V_gs = (gsStl.Points - mean(gsStl.Points)) * 0.02;
    useGSModel = true;
catch
    warning('GS STL file not found. Using simple markers.');
    useGSModel = false;
end

%% Define GS positions (all at z=0)
GS = [5 5 0;      % GS 1 - Central hub
      15 5 0;     % GS 2 - East
      5 15 0;     % GS 3 - North
      15 15 0;    % GS 4 - Northeast
      10 10 0;    % GS 5 - Center-north
      20 10 0;    % GS 6 - Far east
      10 20 0;    % GS 7 - Far north
      20 20 0;    % GS 8 - Far northeast
      0 10 0;     % GS 9 - West
      25 15 0];   % GS 10 - Extended east

% Altitude slots for different drones
GS_slots = [3, 5, 7]; % Drone altitude levels

%% Initialize drones with enhanced properties
drones(1).path = [1 2 4 8];
drones(2).path = [3 1 5 6];
drones(3).path = [7 5 1 2];
drones(4).path = [6 8 4 3];
drones(5).path = [9 1 10 8];
drones(6).path = [2 5 7 3];

% Enhanced drone properties with better colors
colors = [
    0.8 0.2 0.2;    % Red
    0.2 0.8 0.2;    % Green
    0.2 0.2 0.8;    % Blue
    0.8 0.8 0.2;    % Yellow
    0.8 0.2 0.8;    % Magenta
    0.2 0.8 0.8     % Cyan
];
numDrones = length(drones);

% Initialize drone state
for i = 1:numDrones
    drones(i).current = 1;
    drones(i).active = false;
    drones(i).alt = GS_slots(mod(i-1,length(GS_slots))+1);
    drones(i).color = colors(i,:);
    drones(i).startTime = 2.0 * (i-1); % Staggered start times (more realistic spacing)
    drones(i).id = i;
    drones(i).phase = 0; % Track current phase
    drones(i).phaseStartTime = 0; % When current phase started
    drones(i).pos = GS(drones(i).path(1),:); % Initialize position
    drones(i).trail = []; % Store trail points
    
    % Initialize graphics handles for drones and trails
    drones(i).patchHandle = [];
    drones(i).trailHandle = []; % This handle will no longer be used for drawing
    drones(i).textHandle = [];
end

%% Realistic simulation settings
simDuration = 60; % seconds - longer for realistic speeds
frameRate = 5; % frames per second
totalFrames = simDuration * frameRate;
simulationSpeed = 5; % Speed up the simulation

% Realistic movement parameters (1 m/s speeds)
droneSpeed = 1.0; % m/s for both vertical and horizontal movement
pauseAtStation = 2.0; % seconds to pause at each station

%% Main simulation loop
fprintf('\n=== ENHANCED DRONE SIMULATION ===\n');
fprintf('Simulation Duration: %d seconds\n', simDuration);
fprintf('Frame Rate: %d fps\n', frameRate);
fprintf('Total Drones: %d\n', numDrones);
fprintf('Drone Speed: %.1f m/s\n', droneSpeed);
fprintf('Station Pause: %.1f seconds\n', pauseAtStation);
fprintf('\n--- DRONE PATHS ---\n');
for d = 1:numDrones
    fprintf('Drone %d: Altitude %.0fm, Path [%s]\n', ...
            d, drones(d).alt, num2str(drones(d).path, '%d '));
end
fprintf('\n--- SIMULATION RUNNING ---\n');

figure('Position', [100, 100, 1400, 900]);
set(gcf, 'Color', 'white');

% Pre-draw static elements outside the loop
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

%% Draw Ground Stations with enhanced visualization (static)
% Pre-allocate arrays for handles if needed
gsPatchHandles = gobjects(size(GS,1), 1);
gsPlotHandles = gobjects(size(GS,1), 1);
gsTextHandles = gobjects(size(GS,1), 1);
for g = 1:size(GS,1)
    if useGSModel
        V_shift = V_gs + GS(g,:);
        gsPatchHandles(g) = patch('Faces', F_gs, 'Vertices', V_shift, ...
              'FaceColor', [0.4 0.4 0.8], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.7);
    else
        % Enhanced GS markers
        gsPlotHandles(g) = plot3(GS(g,1), GS(g,2), GS(g,3), 'ks', 'MarkerSize', 18, ...
              'MarkerFaceColor', [0.3 0.3 0.9], 'LineWidth', 2);
              
        % Landing pad with gradient effect (pre-drawn circles)
        theta = linspace(0, 2*pi, 32);
        for r = 0.3:0.3:1.5
            padX = GS(g,1) + r * cos(theta);
            padY = GS(g,2) + r * sin(theta);
            padZ = zeros(size(padX)) + 0.05;
            alpha = 1 - r/1.5;
            plot3(padX, padY, padZ, 'b-', 'LineWidth', 2, ...
                  'Color', [0 0 1 alpha]);
        end
    end
    
    % Enhanced GS labels
    gsTextHandles(g) = text(GS(g,1), GS(g,2), GS(g,3)+2.5, sprintf('GS-%d', g), ...
         'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0 0.8], ...
         'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
         'EdgeColor', 'blue', 'Margin', 2);
end

%% Connect Ground Stations with network lines (static)
for g = 1:size(GS,1)-1
    for h = g+1:size(GS,1)
        distance = norm(GS(g,:) - GS(h,:));
        if distance < 15 % Only connect nearby stations
            plot3([GS(g,1), GS(h,1)], [GS(g,2), GS(h,2)], [GS(g,3), GS(h,3)], ...
                  '--', 'LineWidth', 1, 'Color', [0.5, 0.8, 0.5], 'MarkerSize', 4);
        end
    end
end

% Add legend (can be done once)
legend('GS Network', 'Location', 'northeast');

% Initialize title handle for faster updates
titleHandle = title('');

for frame = 1:totalFrames
    tic; % Start timing for real-time control
    
    currTime = (frame / frameRate) * simulationSpeed;
    
    %% Process each drone with FIXED movement logic
    activeDrones = 0;
    completedDrones = 0;
    
    for d = 1:numDrones
        % Check if drone should start
        if ~drones(d).active && currTime >= drones(d).startTime
            drones(d).active = true;
            drones(d).phaseStartTime = currTime;
            drones(d).phase = 1; % Start with takeoff phase
            fprintf('[%.2f] Drone %d ACTIVATED - Mission start from GS-%d\n', ...
                    currTime, drones(d).id, drones(d).path(1));
        end
        
        if drones(d).active
            activeDrones = activeDrones + 1;
            
            if drones(d).current <= length(drones(d).path)
                % Calculate time in current phase
                phaseTime = currTime - drones(d).phaseStartTime;
                
                if drones(d).current < length(drones(d).path)
                    from = GS(drones(d).path(drones(d).current),:);
                    to = GS(drones(d).path(drones(d).current+1),:);
                    alt = drones(d).alt;
                    
                    % Calculate realistic durations based on 1 m/s speed
                    takeoffDuration = alt / droneSpeed; % Time to reach altitude
                    horizontalDistance = norm(to(1:2) - from(1:2)); % 2D distance
                    flightDuration = horizontalDistance / droneSpeed; % Time for horizontal flight
                    landingDuration = alt / droneSpeed; % Time to descend
                    
                    % FIXED: Use if-elseif chain to prevent multiple phase execution
                    if drones(d).phase == 1 % Takeoff
                        if phaseTime <= takeoffDuration
                            % Linear vertical movement at 1 m/s
                            progress = phaseTime / takeoffDuration;
                            drones(d).pos = [from(1), from(2), alt * progress];
                        else
                            % FIXED: Properly set position at end of takeoff
                            drones(d).pos = [from(1), from(2), alt];
                            drones(d).phase = 2;
                            drones(d).phaseStartTime = currTime;
                            fprintf('[%.2f] Drone %d TAKEOFF COMPLETE (%.1fs) - Flying to GS-%d\n', ...
                                    currTime, drones(d).id, takeoffDuration, drones(d).path(drones(d).current+1));
                        end
                        
                    elseif drones(d).phase == 2 % Horizontal movement at altitude
                        if phaseTime <= flightDuration
                            % Linear horizontal movement at 1 m/s
                            progress = phaseTime / flightDuration;
                            drones(d).pos = (1-progress)*[from(1), from(2), alt] + progress*[to(1), to(2), alt];
                        else
                            % FIXED: Properly set position at cruise altitude above target
                            drones(d).pos = [to(1), to(2), alt];
                            drones(d).phase = 3;
                            drones(d).phaseStartTime = currTime;
                            fprintf('[%.2f] Drone %d ARRIVED at GS-%d (flight: %.1fs) - Landing\n', ...
                                    currTime, drones(d).id, drones(d).path(drones(d).current+1), flightDuration);
                        end
                        
                    elseif drones(d).phase == 3 % Landing
                        if phaseTime <= landingDuration
                            % Linear descent at 1 m/s
                            progress = phaseTime / landingDuration;
                            drones(d).pos = [to(1), to(2), alt * (1 - progress)];
                        else
                            % FIXED: Properly set final landing position
                            drones(d).pos = to; % Land at ground station
                            % Move to next waypoint after landing
                            drones(d).current = drones(d).current + 1;
                            drones(d).phase = 0; % Pause phase
                            drones(d).phaseStartTime = currTime;
                            fprintf('[%.2f] Drone %d LANDED at GS-%d (descent: %.1fs) - Pausing\n', ...
                                    currTime, drones(d).id, drones(d).path(drones(d).current), landingDuration);
                        end
                        
                    elseif drones(d).phase == 0 % Pause at station
                        if phaseTime <= pauseAtStation
                            % Stay at current position (ground station)
                            drones(d).pos = GS(drones(d).path(drones(d).current),:);
                        else
                            if drones(d).current < length(drones(d).path)
                                drones(d).phase = 1; % Start next leg
                                drones(d).phaseStartTime = currTime;
                                fprintf('[%.2f] Drone %d RESUMING from GS-%d to GS-%d\n', ...
                                        currTime, drones(d).id, drones(d).path(drones(d).current), ...
                                        drones(d).path(drones(d).current+1));
                            end
                        end
                    end
                else
                    % Mission complete - stay at final position
                    finalPos = GS(drones(d).path(end),:);
                    drones(d).pos = finalPos;
                    completedDrones = completedDrones + 1;
                end
                
                % REMOVE OR COMMENT OUT THESE LINES TO STOP DRAWING TRAILS
                % % Store trail points (limit trail length for performance)
                % if length(drones(d).trail) > 50
                %     drones(d).trail = drones(d).trail(2:end,:);
                % end
                % drones(d).trail = [drones(d).trail; drones(d).pos];
                % 
                % % Update drone trail
                % if ~isempty(drones(d).trailHandle) && isvalid(drones(d).trailHandle)
                %     set(drones(d).trailHandle, 'XData', drones(d).trail(:,1), ...
                %                                'YData', drones(d).trail(:,2), ...
                %                                'ZData', drones(d).trail(:,3));
                % else
                %     drones(d).trailHandle = plot3(drones(d).trail(:,1), drones(d).trail(:,2), drones(d).trail(:,3), ...
                %                                   '-', 'Color', [drones(d).color, 0.6], 'LineWidth', 2);
                % end
                
                % Update drone visualization
                if drones(d).current <= length(drones(d).path)
                    Vd = V_drone + drones(d).pos;
                    if ~isempty(drones(d).patchHandle) && isvalid(drones(d).patchHandle)
                        set(drones(d).patchHandle, 'Vertices', Vd);
                    else
                        drones(d).patchHandle = patch('Faces', F_drone, 'Vertices', Vd, ...
                                                      'FaceColor', drones(d).color, 'EdgeColor', 'k', ...
                                                      'FaceAlpha', 0.9, 'EdgeAlpha', 0.3, 'LineWidth', 0.5);
                    end
                    
                    % Drone status indicator with realistic phase names
                    statusText = '';
                    if drones(d).phase == 0, statusText = 'PAUSED';
                    elseif drones(d).phase == 1, statusText = 'TAKEOFF';
                    elseif drones(d).phase == 2, statusText = 'CRUISE';
                    elseif drones(d).phase == 3, statusText = 'LANDING';
                    end
                    
                    if ~isempty(drones(d).textHandle) && isvalid(drones(d).textHandle)
                        set(drones(d).textHandle, 'Position', drones(d).pos + [0 0 1.2], ...
                                                   'String', sprintf('D%d-%s', drones(d).id, statusText));
                    else
                        drones(d).textHandle = text(drones(d).pos(1), drones(d).pos(2), drones(d).pos(3)+1.2, ...
                                                     sprintf('D%d-%s', drones(d).id, statusText), ...
                                                     'FontSize', 8, 'Color', 'black', 'FontWeight', 'bold', ...
                                                     'HorizontalAlignment', 'center', 'BackgroundColor', 'white');
                    end
                end
            end
        end
    end
    
    %% Enhanced Status Display
    if mod(frame, 15) == 0 % Update every 0.5s
        fprintf('[%.2f] STATUS: %d active, %d completed\n', ...
                currTime, activeDrones, completedDrones);
    end
    
    % Update title for mission progress
    set(titleHandle, 'String', sprintf('Enhanced Drone Simulation | Time: %.1fs | Active: %d | Completed: %d/%d', ...
                      currTime, activeDrones, completedDrones, numDrones));
    
    drawnow limitrate; % Use drawnow limitrate for faster rendering
    
    % Real-time control - maintain frame rate without slowing down
    elapsed = toc;
    targetTime = 1/(frameRate * simulationSpeed);
    if elapsed < targetTime
        pause(targetTime - elapsed);
    end
end

%% Final Statistics Report
fprintf('\n=== SIMULATION COMPLETE ===\n');
fprintf('Total Duration: %.1f seconds\n', simDuration);
fprintf('Frames Processed: %d (%.1f fps)\n', totalFrames, frameRate);
fprintf('Total Drones: %d\n', numDrones);

% Final mission status
fprintf('\n--- FINAL MISSION STATUS ---\n');
for d = 1:numDrones
    if drones(d).active
        if drones(d).current >= length(drones(d).path)
            status = '✓ COMPLETE';
        else
            status = sprintf('◐ PARTIAL (%d/%d)', drones(d).current, length(drones(d).path));
        end
        pathStr = sprintf('%d→%d→%d→%d', drones(d).path);
        fprintf('Drone %d: %s | Path: %s | Alt: %.0fm\n', ...
                d, status, pathStr, drones(d).alt);
    else
        fprintf('Drone %d: ✗ NOT STARTED\n', d);
    end
end
fprintf('==================================\n');

%% Helper function to create simple geometric drone
function [V, F] = createSimpleDrone()
    % Create an enhanced drone shape
    bodyLength = 1.0; bodyWidth = 0.4; bodyHeight = 0.15;
    
    % Main body vertices
    V = [
        -bodyLength/2, -bodyWidth/2, -bodyHeight/2;  % 1
         bodyLength/2, -bodyWidth/2, -bodyHeight/2;  % 2
         bodyLength/2,  bodyWidth/2, -bodyHeight/2;  % 3
        -bodyLength/2,  bodyWidth/2, -bodyHeight/2;  % 4
        -bodyLength/2, -bodyWidth/2,  bodyHeight/2;  % 5
         bodyLength/2, -bodyWidth/2,  bodyHeight/2;  % 6
         bodyLength/2,  bodyWidth/2,  bodyHeight/2;  % 7
        -bodyLength/2,  bodyWidth/2,  bodyHeight/2   % 8
    ];
    
    % Add propeller arms and rotors
    armLength = 0.6;
    rotorRadius = 0.2;
    propPositions = [
        -armLength, -armLength, bodyHeight/2;   % 9
         armLength, -armLength, bodyHeight/2;   % 10
         armLength,  armLength, bodyHeight/2;   % 11
        -armLength,  armLength, bodyHeight/2    % 12
    ];
    
    V = [V; propPositions];
    
    % Create rotor discs
    theta = linspace(0, 2*pi, 8);
    for i = 1:4
        for j = 1:length(theta)
            rotorX = propPositions(i,1) + rotorRadius * cos(theta(j));
            rotorY = propPositions(i,2) + rotorRadius * sin(theta(j));
            rotorZ = propPositions(i,3) + 0.05;
            V = [V; rotorX, rotorY, rotorZ];
        end
    end
    
    % Define faces for the main body
    F = [
        1, 2, 3, 4;  % bottom
        5, 8, 7, 6;  % top
        1, 5, 6, 2;  % front
        3, 7, 8, 4;  % back
        1, 4, 8, 5;  % left
        2, 6, 7, 3   % right
    ];
    
    % Add triangular faces for a more complex shape
    F = [F;
          1, 9, 5, 1;   % Connect to propellers
         2, 10, 6, 2;
         3, 11, 7, 3;
         4, 12, 8, 4];
end