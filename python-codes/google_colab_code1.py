import random
import networkx as nx
import math
import time


def generate_poisson_positions(num_points, min_dist, width, height, k=30):
    cell_size = min_dist / math.sqrt(2)
    grid_width = int(width / cell_size) + 1
    grid_height = int(height / cell_size) + 1
    grid = [[None for _ in range(grid_height)] for _ in range(grid_width)]

    def get_cell_coords(p):
        return int(p[0] / cell_size), int(p[1] / cell_size)

    def in_neighborhood(p):
        gx, gy = get_cell_coords(p)
        for i in range(max(0, gx - 2), min(grid_width, gx + 3)):
            for j in range(max(0, gy - 2), min(grid_height, gy + 3)):
                q = grid[i][j]
                if q is not None:
                    dist = math.hypot(p[0] - q[0], p[1] - q[1])
                    if dist < min_dist:
                        return True
        return False

    points = []
    active_list = []

    first_point = (random.uniform(0, width), random.uniform(0, height))
    points.append(first_point)
    active_list.append(first_point)
    gx, gy = get_cell_coords(first_point)
    grid[gx][gy] = first_point

    while active_list and len(points) < num_points:
        idx = random.randint(0, len(active_list) - 1)
        point = active_list[idx]
        found = False
        for _ in range(k):
            angle = random.uniform(0, 2 * math.pi)
            r = random.uniform(min_dist, 2 * min_dist)
            new_x = point[0] + r * math.cos(angle)
            new_y = point[1] + r * math.sin(angle)
            new_point = (new_x, new_y)
            if 0 <= new_x < width and 0 <= new_y < height and not in_neighborhood(new_point):
                points.append(new_point)
                active_list.append(new_point)
                gx, gy = get_cell_coords(new_point)
                grid[gx][gy] = new_point
                found = True
                break
        if not found:
            active_list.pop(idx)

    return points[:num_points] if len(points) >= num_points else None


def print_gs_distances(positions):
    nodes = list(positions.keys())
    print("\nChecking distances between Ground Stations:")
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            d = math.hypot(positions[nodes[i]][0] - positions[nodes[j]][0],
                           positions[nodes[i]][1] - positions[nodes[j]][1])
            print(f"Distance between GS {nodes[i]} and GS {nodes[j]}: {d:.2f}")


def main():
    num_gs = int(input("Enter number of Ground Stations (at least 3): "))
    if num_gs < 3:
        print("You need at least 3 ground stations.")
        return

    num_drones = int(input("Enter number of drones: "))
    if num_drones < 1:
        print("You need at least 1 drone.")
        return

    simulation_time = float(input("Enter simulation time in seconds: "))
    if simulation_time <= 0:
        print("Simulation time must be positive.")
        return

    drone_speed = 1.0  # units per second (adjust as needed)

    G = nx.Graph()
    positions = {}
    min_distance = 15
    connection_threshold = 25
    frame_size = 100
    positions_list = generate_poisson_positions(num_gs, min_distance, frame_size, frame_size)

    if not positions_list:
        print("Failed to generate enough well-spaced GSs. Try fewer GSs or larger area.")
        return

    for i, pos in enumerate(positions_list):
        G.add_node(i)
        positions[i] = pos

    print_gs_distances(positions)

    for i in range(num_gs):
        for j in range(i + 1, num_gs):
            dist = math.hypot(positions[i][0] - positions[j][0], positions[i][1] - positions[j][1])
            if dist <= connection_threshold:
                G.add_edge(i, j)

    all_nodes = list(G.nodes)
    drone_info = []
    used_pairs = set()
    while len(drone_info) < num_drones:
        src, dst = random.sample(all_nodes, 2)
        if (src, dst) not in used_pairs and (dst, src) not in used_pairs:
            used_pairs.add((src, dst))
            drone_info.append((src, dst))

    drone_labels = [f"Drone {i+1}" for i in range(num_drones)]

    # Prepare paths for drones
    paths = []
    active_drones = [True] * num_drones
    reached_destination = [False] * num_drones

    for i in range(num_drones):
        src, dst = drone_info[i]
        try:
            path = nx.shortest_path(G, source=src, target=dst)
            print(f"{drone_labels[i]} Path: {path}")
            paths.append(path)
        except nx.NetworkXNoPath:
            print(f"No path found for {drone_labels[i]} between GS {src} and GS {dst}")
            paths.append([])
            active_drones[i] = False

    # Drone state: current segment index and progress (distance along that segment)
    # Initialize drone positions at their source nodes
    drone_segment_index = [0] * num_drones
    drone_progress = [0.0] * num_drones  # distance traveled on current segment

    def distance(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def is_near_any_gs(x, y, gs_positions, tol=1.0):
        return any(math.hypot(x - gx, y - gy) < tol for gx, gy in gs_positions.values())

    collision_pairs = set()
    stopped_due_to_collision = set()

    dt = 1 # time step in seconds
    elapsed_time = 0.0

    print("\nStarting simulation...\n")

    while elapsed_time < simulation_time:
        any_active = False
        drone_positions = []

        for i in range(num_drones):
            if not active_drones[i] or len(paths[i]) < 2:
                # Drone inactive or no path
                if paths[i]:
                    last_node_pos = positions[paths[i][-1]]
                    drone_positions.append((last_node_pos[0], last_node_pos[1], 0.0))
                else:
                    drone_positions.append((None, None, None))
                continue

            any_active = True

            seg_idx = drone_segment_index[i]
            path = paths[i]

            # Current segment start and end points
            start_node = path[seg_idx]
            end_node = path[seg_idx + 1]
            start_pos = positions[start_node]
            end_pos = positions[end_node]

            seg_length = distance(start_pos, end_pos)

            # Move drone along segment by drone_speed * dt
            drone_progress[i] += drone_speed * dt

            if drone_progress[i] >= seg_length:
                # Move to next segment
                drone_progress[i] -= seg_length
                drone_segment_index[i] += 1
                seg_idx = drone_segment_index[i]

                if seg_idx >= len(path) - 1:
                    # Reached destination
                    active_drones[i] = False
                    reached_destination[i] = True
                    drone_positions.append((end_pos[0], end_pos[1], 0.0))
                    print(f"✅ {drone_labels[i]} reached destination at time {elapsed_time:.2f}s.")
                    continue

                # Update start and end for new segment
                start_node = path[seg_idx]
                end_node = path[seg_idx + 1]
                start_pos = positions[start_node]
                end_pos = positions[end_node]
                seg_length = distance(start_pos, end_pos)

            # Calculate current position on segment (linear interpolation)
            ratio = drone_progress[i] / seg_length if seg_length > 0 else 1
            x = start_pos[0] + (end_pos[0] - start_pos[0]) * ratio
            y = start_pos[1] + (end_pos[1] - start_pos[1]) * ratio
            z = 0.0  # 2D plane, so z=0

            drone_positions.append((x, y, z))

        # Collision detection
        for i in range(num_drones):
            if not active_drones[i]:
                continue
            xi, yi, _ = drone_positions[i]
            if xi is None:
                continue

            # Ignore collisions near any GS
            if is_near_any_gs(xi, yi, positions):
                continue

            for j in range(i + 1, num_drones):
                if not active_drones[j]:
                    continue
                xj, yj, _ = drone_positions[j]
                if xj is None:
                    continue

                if is_near_any_gs(xj, yj, positions):
                    continue

                # Check collision threshold (0.5 units)
                if abs(xi - xj) < 0.5 and abs(yi - yj) < 0.5:
                    pair = tuple(sorted((i, j)))
                    if pair not in collision_pairs:
                        print(f"⚠️ Collision Detected between {drone_labels[i]} and {drone_labels[j]} at x={(xi + xj) / 2:.2f}, y={(yi + yj) / 2:.2f} at time {elapsed_time:.2f}s")
                        collision_pairs.add(pair)
                        active_drones[i] = False
                        active_drones[j] = False
                        stopped_due_to_collision.add(i)
                        stopped_due_to_collision.add(j)

        # Print all drone positions
        pos_strings = []
        for i, (x, y, z) in enumerate(drone_positions):
            if x is not None:
                pos_strings.append(f"{drone_labels[i]}: ({x:.2f}, {y:.2f}, {z:.2f}) {'[Stopped]' if not active_drones[i] else ''}")
            else:
                pos_strings.append(f"{drone_labels[i]}: (No path)")
        print("Time {:.2f}s | ".format(elapsed_time) + " | ".join(pos_strings))

        if not any_active:
            print("All drones have stopped or reached destination.")
            break

        time.sleep(dt)
        elapsed_time += dt

    print("\nSimulation ended.\n")
    print(f"Total drones: {num_drones}")
    print(f"Reached destination: {sum(reached_destination)}")
    print(f"Stopped due to collision: {len(stopped_due_to_collision)}")
    print(f"Stopped otherwise: {num_drones - sum(reached_destination) - len(stopped_due_to_collision)}")


if __name__ == "__main__":
    main()