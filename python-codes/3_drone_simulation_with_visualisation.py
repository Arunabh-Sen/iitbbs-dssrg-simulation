import random
import networkx as nx
import matplotlib.pyplot as plt
import math


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


def generate_edges_based_on_distance(positions, radius):
    edges = []
    keys = list(positions.keys())
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            xi, yi = positions[keys[i]]
            xj, yj = positions[keys[j]]
            dist = math.hypot(xi - xj, yi - yj)
            if dist <= radius:
                edges.append((keys[i], keys[j]))
    return edges


def print_gs_distances(positions):
    nodes = list(positions.keys())
    print("\nChecking distances between Ground Stations:")
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            d = math.hypot(positions[nodes[i]][0] - positions[nodes[j]][0],
                           positions[nodes[i]][1] - positions[nodes[j]][1])
            print(f"Distance between GS {nodes[i]} and GS {nodes[j]}: {d:.2f}")


def main():
    num_gs = int(input("Enter number of Ground Stations (GS, at least 3): "))
    if num_gs < 3:
        print("You need at least 3 ground stations.")
        return

    G = nx.Graph()
    positions = {}

    min_distance = 15
    frame_size = 60
    connection_radius = 20  # Distance threshold to connect nodes

    positions_list = generate_poisson_positions(num_gs, min_distance, frame_size, frame_size)
    if not positions_list:
        print("❌ Failed to generate enough well-spaced GSs. Try fewer GSs or larger area.")
        return

    for i, pos in enumerate(positions_list):
        G.add_node(i)
        positions[i] = pos

    print_gs_distances(positions)

    # Connect nodes based on distance threshold using adjacency matrix
    edges = generate_edges_based_on_distance(positions, connection_radius)
    G.add_edges_from(edges)

    # Ensure the graph is fully connected by connecting disconnected components
    if not nx.is_connected(G):
        components = list(nx.connected_components(G))
        print(f"⚠️ Graph is disconnected with {len(components)} components. Connecting components...")
        # Connect each component to the next one by the closest pair of nodes
        for i in range(len(components) - 1):
            comp1 = components[i]
            comp2 = components[i + 1]
            min_dist = float('inf')
            closest_pair = None
            for node1 in comp1:
                for node2 in comp2:
                    x1, y1 = positions[node1]
                    x2, y2 = positions[node2]
                    dist = math.hypot(x1 - x2, y1 - y2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_pair = (node1, node2)
            if closest_pair:
                print(f"Connecting node {closest_pair[0]} and node {closest_pair[1]} to connect components.")
                G.add_edge(*closest_pair)

    if not nx.is_connected(G):
        print("⚠️ Even after adding edges, graph is not fully connected.")
    else:
        print("✅ Graph is now fully connected.")

    all_nodes = list(G.nodes)
    drone_info = []
    used_pairs = set()
    while len(drone_info) < 3:
        src, dst = random.sample(all_nodes, 2)
        if (src, dst) not in used_pairs and (dst, src) not in used_pairs:
            used_pairs.add((src, dst))
            drone_info.append((src, dst))

    drone_colors = ['red', 'green', 'blue']
    drone_labels = ['Drone 1', 'Drone 2', 'Drone 3']

    plt.ion()
    fig, ax = plt.subplots()
    nx.draw(G, positions, with_labels=True, node_color='lightblue', node_size=600, ax=ax)
    plt.title("Ground Stations Network")
    plt.show(block=False)

    drone_dots = []
    paths = []
    active_drones = [True] * 3
    mission_success = [False] * 3
    reached_destination = [False] * 3

    for i in range(3):
        src, dst = drone_info[i]
        try:
            path = nx.shortest_path(G, source=src, target=dst)
            print(f"{drone_labels[i]} Path: {path}")
            paths.append(path)
            drone_dot, = ax.plot([positions[src][0]], [positions[src][1]], 'o',
                                 color=drone_colors[i], markersize=10, label=drone_labels[i])
            drone_dots.append(drone_dot)
        except nx.NetworkXNoPath:
            print(f"No path found for {drone_labels[i]} between GS {src} and GS {dst}")
            paths.append([])
            drone_dots.append(None)
            active_drones[i] = False

    plt.legend()

    steps = 30
    z = 10
    max_path_len = max(len(path) for path in paths)
    drone_positions = [(0.0, 0.0)] * 3

    collision_points = []
    reported_collisions = set()

    # Keep track of drones allowed to ignore collisions at collision points
    ignore_collision_flags = [False, False, False]

    def is_at_gs(x, y, gs_positions, tol=0.01):
        return any(math.isclose(x, gx, abs_tol=tol) and math.isclose(y, gy, abs_tol=tol) for gx, gy in gs_positions.values())

    for segment in range(max_path_len - 1):
        for step in range(1, steps + 1):
            for i, path in enumerate(paths):
                if not active_drones[i] or len(path) < 2 or segment >= len(path) - 1:
                    continue
                start = path[segment]
                end = path[segment + 1]
                x1, y1 = positions[start]
                x2, y2 = positions[end]
                x = x1 + (x2 - x1) * (step / steps)
                y = y1 + (y2 - y1) * (step / steps)

                # If this drone has ignore collision flag True, just move it normally
                if ignore_collision_flags[i]:
                    drone_dots[i].set_data([x], [y])
                    drone_positions[i] = (x, y)
                    continue

                # If this drone is 3rd (index 2) or later drone, check if it is close to known collision point and ignore collision
                # Here, we generalize for any drone that has ignore_collision flag

                # For others, move normally
                drone_dots[i].set_data([x], [y])
                drone_positions[i] = (x, y)

            if step % 10 == 0:
                for i in range(3):
                    x, y = drone_positions[i]
                    print(f"{drone_labels[i]} coordinates at time {(segment * steps + step) * 0.1:.1f}s -> x={x:.2f}, y={y:.2f}, z={z}")

            # Check if drones reached destination
            for i in range(3):
                if active_drones[i] and paths[i]:
                    last_node_pos = positions[paths[i][-1]]
                    drone_x, drone_y = drone_positions[i]
                    dist_to_dest = math.hypot(drone_x - last_node_pos[0], drone_y - last_node_pos[1])
                    if dist_to_dest < 0.5:
                        reached_destination[i] = True
                        active_drones[i] = False
                        print(f"✅ {drone_labels[i]} reached destination.")

            # Check collisions between active drones except those ignoring collisions
            for i in range(3):
                for j in range(i + 1, 3):
                    if not active_drones[i] and not active_drones[j]:
                        continue

                    # If either drone ignores collisions, skip collision detection between them
                    if ignore_collision_flags[i] or ignore_collision_flags[j]:
                        continue

                    xi, yi = drone_positions[i]
                    xj, yj = drone_positions[j]

                    if is_at_gs(xi, yi, positions) or is_at_gs(xj, yj, positions):
                        continue

                    if abs(xi - xj) < 0.5 and abs(yi - yj) < 0.5:
                        # Collision point rounded for consistency
                        coll_point = (round((xi + xj) / 2, 2), round((yi + yj) / 2, 2))
                        if coll_point not in collision_points:
                            collision_points.append(coll_point)
                            active_drones[i] = False
                            active_drones[j] = False
                            print(f"❌ Collision detected between {drone_labels[i]} and {drone_labels[j]} at {coll_point}. Both stopped.")
                        else:
                            # Known collision point
                            # For any drone arriving 3rd or later on this collision point, allow to continue
                            # Mark those drones to ignore collisions at this point
                            if i == 2 and not ignore_collision_flags[i]:
                                ignore_collision_flags[i] = True
                                print(f"⚠️ {drone_labels[i]} arrived at known collision point {coll_point} and will ignore collision.")
                            if j == 2 and not ignore_collision_flags[j]:
                                ignore_collision_flags[j] = True
                                print(f"⚠️ {drone_labels[j]} arrived at known collision point {coll_point} and will ignore collision.")

            # Draw current drone positions and update plot
            fig.canvas.draw()
            plt.pause(0.05)

            if all(not active for active in active_drones):
                print("All drones stopped or reached destination. Ending simulation early.")
                break

        if all(not active for active in active_drones):
            break

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
