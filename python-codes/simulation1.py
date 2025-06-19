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
    print("\n📡 Ground Station Distances:")
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            d = math.hypot(positions[nodes[i]][0] - positions[nodes[j]][0],
                           positions[nodes[i]][1] - positions[nodes[j]][1])
            print(f"GS {nodes[i]} ↔ GS {nodes[j]}: {d:.2f} units")


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

    drone_speed = 1.0
    min_distance = 15
    connection_threshold = 25
    frame_size = 100
    batch_size = 3
    batch_interval = 2  # seconds

    G = nx.Graph()
    positions = {}
    positions_list = generate_poisson_positions(num_gs, min_distance, frame_size, frame_size)

    if not positions_list:
        print("❌ Could not place GSs with given constraints. Try again with fewer GSs or a bigger area.")
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
    drone_info = [None] * num_drones
    drone_labels = [f"Drone {i+1}" for i in range(num_drones)]
    paths = [None] * num_drones
    active_drones = [False] * num_drones
    reached_destination = [False] * num_drones
    drone_segment_index = [0] * num_drones
    drone_progress = [0.0] * num_drones
    drone_positions = [None] * num_drones
    collision_pairs = set()
    stopped_due_to_collision = set()
    drones_initialized = 0
    collision_times = {}
    arrival_times = {}

    def distance(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def is_near_any_gs(x, y, gs_positions, tol=1.0):
        return any(math.hypot(x - gx, y - gy) < tol for gx, gy in gs_positions.values())

    print("\n⏱ Simulation Starting...\n")
    dt = 1
    elapsed_time = 0.0

    while elapsed_time < simulation_time:
        print(f"🕒 Time: {elapsed_time:.2f}s")

        if elapsed_time >= 2 and drones_initialized < num_drones and int(elapsed_time) % batch_interval == 0:
            batch_end = min(drones_initialized + batch_size, num_drones)
            for i in range(drones_initialized, batch_end):
                while True:
                    src, dst = random.sample(all_nodes, 2)
                    if (src, dst) not in drone_info and (dst, src) not in drone_info:
                        drone_info[i] = (src, dst)
                        break
                try:
                    path = nx.shortest_path(G, source=src, target=dst)
                    print(f"🚁 {drone_labels[i]} started: GS {src} → GS {dst}, Path: {path}")
                    paths[i] = path
                    active_drones[i] = True
                except nx.NetworkXNoPath:
                    print(f"❌ {drone_labels[i]}: No path between GS {src} and GS {dst}")
                    paths[i] = []
            drones_initialized = batch_end

        for i in range(num_drones):
            if not active_drones[i] or paths[i] is None or len(paths[i]) < 2:
                if paths[i]:
                    last_node_pos = positions[paths[i][-1]]
                    drone_positions[i] = (last_node_pos[0], last_node_pos[1], 0.0)
                continue

            seg_idx = drone_segment_index[i]
            path = paths[i]
            start_node = path[seg_idx]
            end_node = path[seg_idx + 1]
            start_pos = positions[start_node]
            end_pos = positions[end_node]
            seg_length = distance(start_pos, end_pos)

            drone_progress[i] += drone_speed * dt

            if drone_progress[i] >= seg_length:
                drone_progress[i] -= seg_length
                drone_segment_index[i] += 1
                seg_idx = drone_segment_index[i]
                if seg_idx >= len(path) - 1:
                    active_drones[i] = False
                    reached_destination[i] = True
                    drone_positions[i] = (end_pos[0], end_pos[1], 0.0)
                    arrival_times[i] = elapsed_time
                    print(f"✅ {drone_labels[i]} reached destination at {elapsed_time:.2f}s.")
                    continue
                start_node = path[seg_idx]
                end_node = path[seg_idx + 1]
                start_pos = positions[start_node]
                end_pos = positions[end_node]
                seg_length = distance(start_pos, end_pos)

            ratio = drone_progress[i] / seg_length if seg_length > 0 else 1
            x = start_pos[0] + (end_pos[0] - start_pos[0]) * ratio
            y = start_pos[1] + (end_pos[1] - start_pos[1]) * ratio
            drone_positions[i] = (x, y, 0.0)

        for i in range(num_drones):
            if not active_drones[i] or drone_positions[i] is None:
                continue
            xi, yi, _ = drone_positions[i]
            if is_near_any_gs(xi, yi, positions):
                continue
            for j in range(i + 1, num_drones):
                if not active_drones[j] or drone_positions[j] is None:
                    continue
                xj, yj, _ = drone_positions[j]
                if is_near_any_gs(xj, yj, positions):
                    continue
                if abs(xi - xj) < 0.5 and abs(yi - yj) < 0.5:
                    pair = tuple(sorted((i, j)))
                    if pair not in collision_pairs:
                        collision_time = elapsed_time
                        collision_x = (xi + xj) / 2
                        collision_y = (yi + yj) / 2
                        print(
                            f"⚠ Collision at {collision_time:.2f}s: {drone_labels[i]} and {drone_labels[j]} at ({collision_x:.2f}, {collision_y:.2f})")
                        collision_pairs.add(pair)
                        active_drones[i] = False
                        active_drones[j] = False
                        stopped_due_to_collision.add(i)
                        stopped_due_to_collision.add(j)
                        collision_times[i] = collision_time
                        collision_times[j] = collision_time

        print("Drone Positions:")
        for i, pos in enumerate(drone_positions):
            if pos:
                x, y, z = pos
                status = "[Stopped]" if not active_drones[i] else ""
                print(f"{drone_labels[i]}: ({x:.2f}, {y:.2f}, {z:.2f}) {status}")
            else:
                print(f"{drone_labels[i]}: No path or not launched yet.")
        print()

        elapsed_time += dt
        time.sleep(0.1)

    print("Simulation ended.\n")
    for i in range(num_drones):
        if i in stopped_due_to_collision:
            print(f"{drone_labels[i]} stopped due to collision at {collision_times.get(i, 'unknown')}s.")
        elif reached_destination[i]:
            print(f"{drone_labels[i]} successfully reached the destination at {arrival_times.get(i, 'unknown')}s.")
        else:
            print(f"⏸ {drone_labels[i]} did not complete the path.")


if __name__ == "__main__":
    main()