clc; clear;

%% Load STL drone model
stlData = stlread('new_dronev_whole.stl');
F = stlData.ConnectivityList;
V0 = stlData.Points;

% Scale and center the model
scaleFactor = 0.005;
V0 = V0 - mean(V0, 1);         % Center model
V0 = V0 * scaleFactor;         % Shrink

%% Simulation Parameters
numUAVs = 6;
steps = 200;
boxSize = 15;
commRange = 5;                 % Connection range in meters

positions = rand(numUAVs, 3) * boxSize;
velocities = randn(numUAVs, 3) * 0.2;

%% Create figure
figure;
axis([0 boxSize 0 boxSize 0 boxSize]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on; hold on;
camlight; lighting gouraud;

%% Animation Loop
for t = 1:steps
    cla;

    % Update UAV positions
    positions = positions + velocities;

    % Bounce on walls
    for i = 1:numUAVs
        for dim = 1:3
            if positions(i, dim) < 0 || positions(i, dim) > boxSize
                velocities(i, dim) = -velocities(i, dim);
                positions(i, dim) = min(max(positions(i, dim), 0), boxSize);
            end
        end
    end

    % Plot UAVs
    for i = 1:numUAVs
        V_shift = V0 + positions(i, :);
        patch('Faces', F, 'Vertices', V_shift, ...
              'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'none', ...
              'FaceAlpha', 0.9);
    end

    % Plot connection lines (within range)
    for i = 1:numUAVs
        for j = i+1:numUAVs
            dist = norm(positions(i,:) - positions(j,:));
            if dist <= commRange
                plot3([positions(i,1), positions(j,1)], ...
                      [positions(i,2), positions(j,2)], ...
                      [positions(i,3), positions(j,3)], ...
                      'g-', 'LineWidth', 1.5);
            end
        end
    end

    % Draw communication range (optional spheres)
    for i = 1:numUAVs
        [sx, sy, sz] = sphere(10);
        surf(sx*commRange + positions(i,1), ...
             sy*commRange + positions(i,2), ...
             sz*commRange + positions(i,3), ...
             'FaceAlpha', 0.05, 'EdgeColor', 'none', 'FaceColor', 'blue');
    end

    title(['UAV Simulation – Step ' num2str(t)]);
    axis([0 boxSize 0 boxSize 0 boxSize]);
    drawnow;
    pause(0.05);
end