import random
import networkx as nx
import math

random.seed(42)

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


def main():
    num_gs = int(input("Enter number of Ground Stations (at least 3): "))
    num_drones = int(input("Enter number of drones: "))
    simulation_time = float(input("Enter simulation time in seconds: "))

    drone_speed = 1.0
    min_distance = 10
    connection_threshold = 25
    frame_size = 400

    G = nx.Graph()
    positions = {}
    positions_list = generate_poisson_positions(num_gs, min_distance, frame_size, frame_size)
    if not positions_list:
        print("Could not place GSs with given constraints.")
        return
    for i, pos in enumerate(positions_list):
        G.add_node(i)
        positions[i] = pos

    for i in range(num_gs):
        for j in range(i + 1, num_gs):
            dist = math.hypot(positions[i][0] - positions[j][0], positions[i][1] - positions[j][1])
            if dist <= connection_threshold:
                G.add_edge(i, j)

    for node in G.nodes():
        if G.degree(node) == 0:
            min_dist_node = float('inf')
            closest_node = None
            for other in G.nodes():
                if other != node:
                    dist = math.hypot(positions[node][0] - positions[other][0],
                                      positions[node][1] - positions[other][1])
                    if dist < min_dist_node:
                        min_dist_node = dist
                        closest_node = other
            if closest_node is not None:
                G.add_edge(node, closest_node)

    def distance(p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    class GroundStation:
        def __init__(self, id):
            self.id = id
            self.slots = {f'z{i}': 0 for i in range(1, 6)}  # z1 to z5
            self.drone_slot_map = {}

        def allocate_slot(self, elapsed_time, drone_id, drone_global_slot_map):
            print(f"[{elapsed_time:.2f}s] Drone {drone_id + 1} requesting slot at GS {self.id}")
            if drone_id in drone_global_slot_map:
                assigned_slot = drone_global_slot_map[drone_id]
                if self.slots[assigned_slot] <= elapsed_time:
                    self.slots[assigned_slot] = elapsed_time + 5.0
                    self.drone_slot_map[drone_id] = assigned_slot
                    print(
                        f"[{elapsed_time:.2f}s] GS {self.id} granted existing slot {assigned_slot} to Drone {drone_id + 1} till {self.slots[assigned_slot]:.2f}s")
                    return assigned_slot, self.slots[assigned_slot]
                else:
                    print(
                        f"[{elapsed_time:.2f}s] GS {self.id} cannot grant slot {assigned_slot} to Drone {drone_id + 1}; busy till {self.slots[assigned_slot]:.2f}s")
                    return None, None

            for z in ['z1', 'z2', 'z3', 'z4', 'z5']:
                if self.slots[z] <= elapsed_time:
                    self.slots[z] = elapsed_time + 5.0
                    self.drone_slot_map[drone_id] = z
                    drone_global_slot_map[drone_id] = z
                    print(
                        f"[{elapsed_time:.2f}s] GS {self.id} assigned new slot {z} to Drone {drone_id + 1} till {self.slots[z]:.2f}s")
                    return z, self.slots[z]
            return None, None

        def release_slot(self, z, drone_id):
            if drone_id in self.drone_slot_map and self.drone_slot_map[drone_id] == z:
                self.slots[z] = 0
                del self.drone_slot_map[drone_id]
                print(f"Slot {z} released by Drone {drone_id + 1}")

    gs_objects = {i: GroundStation(i) for i in range(num_gs)}
    all_nodes = list(G.nodes)
    drone_paths = []
    drone_labels = [f"Drone {i + 1}" for i in range(num_drones)]
    drone_positions = [None] * num_drones
    path_found_flags = [True] * num_drones
    drone_global_slot_map = {}

    # Changed here: use normal distribution to stagger the requests
    # mean = 0.2 * i (so roughly same average spacing), std dev = 0.1 (example)
    drone_stagger_offset = [max(0, random.gauss(0.2 * i, 0.1)) for i in range(num_drones)]

    for i in range(num_drones):
        src, dst = random.sample(all_nodes, 2)
        try:
            path = nx.shortest_path(G, src, dst)
            drone_paths.append((path, 0.0, 0, None, 0.0))
        except nx.NetworkXNoPath:
            drone_paths.append(([], 0.0, 0, None, 0.0))
            print(f"Drone {i + 1} has no path from GS {src} to GS {dst}")
            path_found_flags[i] = False

    dt = 0.2
    elapsed_time = 0.0
    collision_log = []
    removed_drones = set()
    last_collision_point = set()

    while elapsed_time < simulation_time:
        print(f"\n[Time: {elapsed_time:.2f}s]")
        positions_in_frame = {}

        for i in range(num_drones):
            if not path_found_flags[i] or i in removed_drones:
                continue

            path, progress, seg_idx, z_slot, slot_end_time = drone_paths[i]
            if not path or seg_idx >= len(path) - 1:
                continue

            if elapsed_time < drone_stagger_offset[i]:
                continue

            start, end = path[seg_idx], path[seg_idx + 1]
            start_pos, end_pos = positions[start], positions[end]
            seg_length = distance(start_pos, end_pos)

            if z_slot is None:
                print(f"[{elapsed_time:.2f}s] Drone {i + 1} is requesting a slot at GS {start}")
                z_slot, slot_end_time = gs_objects[start].allocate_slot(elapsed_time, i, drone_global_slot_map)
                if z_slot is None:
                    print(f"[{elapsed_time:.2f}s] Drone {i + 1} waiting at GS {start} for slot")
                    drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)
                    continue

            if progress == 0.0 and elapsed_time >= slot_end_time:
                gs_objects[start].release_slot(z_slot, i)
                print(f"Drone {i + 1} could not depart, slot expired at GS {start}")
                z_slot, slot_end_time = None, 0.0
                drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)
                continue

            progress += drone_speed * dt
            if progress >= seg_length:
                drone_positions[i] = positions[end]
                gs_objects[start].release_slot(z_slot, i)
                print(f"Drone {i + 1} arrived at GS {end}")
                new_slot, new_slot_end = gs_objects[end].allocate_slot(elapsed_time, i, drone_global_slot_map)
                if new_slot is None:
                    print(f"Drone {i + 1} waiting at GS {end} for slot")
                    progress = seg_length
                    drone_paths[i] = (path, progress, seg_idx, None, 0.0)
                    continue
                z_slot, slot_end_time = new_slot, new_slot_end
                seg_idx += 1
                progress = 0.0
                drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)
            else:
                curr_pos = (
                    start_pos[0] + (end_pos[0] - start_pos[0]) * (progress / seg_length),
                    start_pos[1] + (end_pos[1] - start_pos[1]) * (progress / seg_length)
                )
                drone_positions[i] = curr_pos
                drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)

                rounded_pos = (round(curr_pos[0], 2), round(curr_pos[1], 2))
                if rounded_pos in positions_in_frame:
                    other = positions_in_frame[rounded_pos]
                    if (other not in removed_drones and i not in removed_drones
                            and rounded_pos not in last_collision_point):
                        print(
                            f"\n*** Collision at {rounded_pos} between Drone {i + 1} and Drone {other + 1} at {elapsed_time:.2f}s ***")
                        collision_log.append((elapsed_time, i + 1, other + 1, rounded_pos))
                        removed_drones.update({i, other})
                        last_collision_point.add(rounded_pos)
                else:
                    positions_in_frame[rounded_pos] = i

        elapsed_time += dt

    print("\n--- Collision Report ---")
    if collision_log:
        for t, d1, d2, coord in collision_log:
            print(f"At {t:.2f}s: Collision at {coord} between Drone {d1} and Drone {d2}")
    else:
        print("No collisions occurred.")
    print(f"\nTotal number of collisions: {len(collision_log)}")


if __name__ == "__main__":
    main()