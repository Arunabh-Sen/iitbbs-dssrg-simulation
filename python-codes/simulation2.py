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


def main():
    num_gs = int(input("Enter number of Ground Stations (at least 3): "))
    num_drones = int(input("Enter number of drones: "))
    simulation_time = float(input("Enter simulation time in seconds: "))

    drone_speed = 1.0
    min_distance = 15
    connection_threshold = 25
    frame_size = 100

    G = nx.Graph()
    positions = {}
    positions_list = generate_poisson_positions(num_gs, min_distance, frame_size, frame_size)
    if not positions_list:
        print("Could not place GSs with given constraints.")
        return
    for i, pos in enumerate(positions_list):
        G.add_node(i)
        positions[i] = pos

    # Add edges between nodes within connection_threshold distance
    for i in range(num_gs):
        for j in range(i + 1, num_gs):
            dist = math.hypot(positions[i][0] - positions[j][0], positions[i][1] - positions[j][1])
            if dist <= connection_threshold:
                G.add_edge(i, j)

    # Ensure no isolated nodes: connect isolated nodes to their nearest neighbor
    for node in G.nodes():
        if G.degree(node) == 0:
            min_dist_node = float('inf')
            closest_node = None
            for other in G.nodes():
                if other != node:
                    dist = math.hypot(positions[node][0] - positions[other][0], positions[node][1] - positions[other][1])
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
            # Track reservation end time for each slot
            self.slots = {'z1': 0, 'z2': 0, 'z3': 0}
            # Map drone_id -> slot assigned at this GS
            self.drone_slot_map = {}

        def allocate_slot(self, elapsed_time, drone_id, drone_global_slot_map):
            # If drone already has a global slot assigned for entire path, try to assign same slot here if free or expired
            if drone_id in drone_global_slot_map:
                assigned_slot = drone_global_slot_map[drone_id]
                if self.slots[assigned_slot] <= elapsed_time:
                    # Assign slot to drone here and update reservation time
                    self.slots[assigned_slot] = elapsed_time + 5.0
                    self.drone_slot_map[drone_id] = assigned_slot
                    print(f"GS {self.id} allocated existing global slot {assigned_slot} to drone {drone_id+1} till {self.slots[assigned_slot]:.2f}s")
                    return assigned_slot, self.slots[assigned_slot]
                else:
                    # Slot is still reserved
                    print(f"GS {self.id} cannot allocate global slot {assigned_slot} to drone {drone_id+1} now; slot reserved till {self.slots[assigned_slot]:.2f}s")
                    return None, None

            # Drone has no global slot yet, so try to assign a free slot here that is expired (<= elapsed_time)
            for z in ['z1', 'z2', 'z3']:
                if self.slots[z] <= elapsed_time:
                    self.slots[z] = elapsed_time + 5.0  # 5 second reservation
                    self.drone_slot_map[drone_id] = z
                    drone_global_slot_map[drone_id] = z  # Assign global slot for this drone
                    print(f"GS {self.id} allocated NEW global slot {z} to drone {drone_id+1} till {self.slots[z]:.2f}s")
                    return z, self.slots[z]

            # No slot available
            return None, None

        def release_slot(self, z, drone_id):
            # Release only if the slot matches the drone's assigned slot at this GS
            if drone_id in self.drone_slot_map and self.drone_slot_map[drone_id] == z:
                self.slots[z] = 0
                del self.drone_slot_map[drone_id]

    gs_objects = {i: GroundStation(i) for i in range(num_gs)}

    all_nodes = list(G.nodes)
    drone_paths = []
    drone_labels = [f"Drone {i + 1}" for i in range(num_drones)]
    drone_positions = [None] * num_drones
    path_found_flags = [True] * num_drones

    # Global mapping drone_id -> z_slot for entire path
    drone_global_slot_map = {}

    for i in range(num_drones):
        src, dst = random.sample(all_nodes, 2)
        try:
            path = nx.shortest_path(G, src, dst)
            # drone_paths: (path, progress_on_segment, seg_idx, z_slot, slot_end_time)
            drone_paths.append((path, 0.0, 0, None, 0.0))
            print(f"{drone_labels[i]}: Path from GS {src} to GS {dst}: {path}")
        except nx.NetworkXNoPath:
            drone_paths.append(([], 0.0, 0, None, 0.0))
            print(f"{drone_labels[i]}: No path from GS {src} to GS {dst}")
            path_found_flags[i] = False

    dt = 1
    elapsed_time = 0.0
    while elapsed_time < simulation_time:
        print(f"\nTime: {elapsed_time:.2f}s")
        for i in range(num_drones):
            if not path_found_flags[i]:
                continue

            path, progress, seg_idx, z_slot, slot_end_time = drone_paths[i]
            if not path or seg_idx >= len(path) - 1:
                continue

            start, end = path[seg_idx], path[seg_idx + 1]
            start_pos, end_pos = positions[start], positions[end]
            seg_length = distance(start_pos, end_pos)

            # If drone currently has no slot assigned at this GS, allocate slot strictly following the logic
            if z_slot is None:
                z_slot, slot_end_time = gs_objects[start].allocate_slot(elapsed_time, i, drone_global_slot_map)
                if z_slot is None:
                    print(f"{drone_labels[i]} waiting at GS {start} (no Z-axis slot available)")
                    # Stay put, no progress increase
                    drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)
                    continue

            # Check if current slot expired (max 5 seconds)
            # Only check slot expiry if drone is at GS node and waiting for slot re-allocation
            if progress == 0.0 and elapsed_time >= slot_end_time:
                gs_objects[start].release_slot(z_slot, i)
                print(f"{drone_labels[i]} released {z_slot} at GS {start} (slot time expired)")
                z_slot = None
                slot_end_time = 0.0
                drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)
                # Continue to next iteration to allocate slot again or wait
                continue

            # Move drone along the segment
            progress += drone_speed * dt

            if progress >= seg_length:
                # Drone reached next GS
                drone_positions[i] = positions[end]

                # Release slot at previous GS
                gs_objects[start].release_slot(z_slot, i)
                print(f"{drone_labels[i]} released {z_slot} at GS {start} (arrived at GS {end})")

                # Allocate slot at new GS following same logic
                new_slot, new_slot_end = gs_objects[end].allocate_slot(elapsed_time, i, drone_global_slot_map)
                if new_slot is None:
                    print(f"{drone_labels[i]} waiting at GS {end} (no Z-axis slot available)")
                    # Keep drone at last position until slot available
                    progress = seg_length
                    drone_paths[i] = (path, progress, seg_idx, None, 0.0)
                    continue

                z_slot = new_slot
                slot_end_time = new_slot_end

                seg_idx += 1
                progress = 0.0
                drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)

                print(f"{drone_labels[i]} assigned {z_slot} at GS {end} till {slot_end_time:.2f}s")

            else:
                # Drone is in between GSs, just update progress and keep slot reservation active
                drone_positions[i] = (
                    start_pos[0] + (end_pos[0] - start_pos[0]) * (progress / seg_length),
                    start_pos[1] + (end_pos[1] - start_pos[1]) * (progress / seg_length),
                )
                drone_paths[i] = (path, progress, seg_idx, z_slot, slot_end_time)

        elapsed_time += dt
        time.sleep(0.1)  # For smoother output, remove if not needed

    # Final Summary
    print("\nFinal Summary:")
    for i in range(num_drones):
        if not path_found_flags[i]:
            print(f"{drone_labels[i]}: No valid path assigned.")
            continue

        path, progress, seg_idx, z_slot, slot_end_time = drone_paths[i]
        slot_used = drone_global_slot_map.get(i, "None")
        current_pos = drone_positions[i] if drone_positions[i] is not None else "At start"
        print(
            f"{drone_labels[i]} - Current GS segment: {path[seg_idx] if seg_idx < len(path) else 'Finished'}, "
            f"Global Z-axis slot: {slot_used}, Position: {current_pos}"
        )


if __name__ == "__main__":
    main()