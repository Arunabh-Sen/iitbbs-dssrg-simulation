# Algorithm 2
import time
import threading
from collections import defaultdict
import heapq
import random
from queue import Queue, Empty
import logging
drone_movements = []


class ThreadSafeLogger:
    """Thread-safe logger for precise timestamps"""

    def __init__(self):
        self.lock = threading.Lock()
        self.start_time = time.time()

    def log(self, message):
        current_time = time.time() - self.start_time
        with self.lock:
            print(f"[T={current_time:>10.6f}s] {message}")


class GS(threading.Thread):
    """Ground Station as a thread"""

    def __init__(self, gs_id, logger):
        super().__init__(daemon=True)
        self.gs_id = gs_id
        self.logger = logger
        self.slots = {'z1': None, 'z2': None, 'z3': None}  # (dr_id, end_time, last_request_time)
        self.incoming_table = []
        self.outgoing_table = []
        self.connections = {}
        self.request_queue = Queue()
        self.response_queues = {}  # {request_id: queue}
        self.running = True
        self.lock = threading.Lock()

    def add_connection(self, gs_id, distance):
        """Add connection to another GS with distance"""
        self.connections[gs_id] = float(distance)

    def run(self):
        """Main thread loop for GS"""
        self.logger.log(f"[THREAD] GS{self.gs_id} thread started")

        while self.running:
            try:
                # Check for incoming requests
                request = self.request_queue.get(timeout=0.1)
                self.handle_request(request)
            except Empty:
                # Check and free expired slots
                self.check_expired_slots()
                continue

    def check_expired_slots(self):
        """Check and free expired slots"""
        current_time = time.time()
        with self.lock:
            for slot in ['z1', 'z2', 'z3']:
                if self.slots[slot] is not None:
                    dr_id, end_time, last_request_time = self.slots[slot]
                    if current_time >= end_time:
                        self.logger.log(f"[EVENT] {slot} at GS{self.gs_id} freed (DR{dr_id} reservation expired)")
                        self.slots[slot] = None

    def handle_request(self, request):
        """Handle slot reservation request"""
        request_type = request['type']
        request_id = request['id']

        if request_type == 'reserve_slot':
            dr_id = request['dr_id']
            duration = request['duration']
            response = self.reserve_slot_internal(dr_id, duration)

            # Send response back
            if request_id in self.response_queues:
                self.response_queues[request_id].put(response)

        elif request_type == 'check_status':
            response = self.get_status()
            if request_id in self.response_queues:
                self.response_queues[request_id].put(response)

    def reserve_slot_internal(self, dr_id, duration):
        """Internal slot reservation logic"""
        current_time = time.time()

        with self.lock:
            # Find available slot
            available_slot = None
            for slot in ['z1', 'z2', 'z3']:
                if self.is_slot_available(slot, current_time):
                    available_slot = slot
                    break

            if available_slot:
                end_time = current_time + duration

                # Check if slot was previously occupied
                if self.slots[available_slot] is not None:
                    prev_dr, _, _ = self.slots[available_slot]
                    self.logger.log(
                        f"[EVENT] {available_slot} at GS{self.gs_id} reallocated from DR{prev_dr} to DR{dr_id}")

                self.slots[available_slot] = (dr_id, end_time, current_time)
                self.logger.log(
                    f"[EVENT] GS{self.gs_id} reserved {available_slot} for DR{dr_id} (duration: {duration:.6f}s)")

                return {'success': True, 'slot': available_slot, 'end_time': end_time}
            else:
                self.logger.log(f"[EVENT] GS{self.gs_id} has no available slots for DR{dr_id}")
                return {'success': False, 'slot': None, 'end_time': None}

    def is_slot_available(self, slot, current_time):
        """Check if slot is available"""
        if self.slots[slot] is None:
            return True

        dr_id, end_time, last_request_time = self.slots[slot]

        # If reservation expired
        if current_time >= end_time:
            return True

        # If 2 seconds passed since last request
        if current_time >= last_request_time + 2.0:
            self.logger.log(f"[EVENT] {slot} at GS{self.gs_id} available for reallocation (2-sec interval passed)")
            return True

        return False

    def request_slot_from(self, target_gs, dr_id, duration):
        """Request slot from another GS"""
        request_id = f"{self.gs_id}_{target_gs.gs_id}_{dr_id}_{time.time()}"
        response_queue = Queue()

        # Register response queue
        target_gs.response_queues[request_id] = response_queue

        # Send request
        request = {
            'type': 'reserve_slot',
            'id': request_id,
            'dr_id': dr_id,
            'duration': duration,
            'source_gs': self.gs_id
        }
        target_gs.request_queue.put(request)
        try:
            # Wait for response with timeout
            response = response_queue.get(timeout=5.0)

            if response['success']:
                # Update tables
                distance = self.connections[target_gs.gs_id]
                self.outgoing_table.append([target_gs.gs_id, response['slot'], duration])
                target_gs.incoming_table.append([self.gs_id, response['slot'], duration])

                self.logger.log(
                    f"[EVENT] GS{self.gs_id} successfully reserved {response['slot']} at GS{target_gs.gs_id} for DR{dr_id}")
                return response['slot'], duration
            else:
                return None, None

        except Empty:
            self.logger.log(f"[ERROR] Timeout waiting for response from GS{target_gs.gs_id}")
            return None, None
        finally:
            # Cleanup response queue
            if request_id in target_gs.response_queues:
                del target_gs.response_queues[request_id]

    def get_status(self):
        """Get current GS status"""
        current_time = time.time()
        with self.lock:
            status = {
                'gs_id': self.gs_id,
                'time': current_time,
                'slots': {},
                'incoming_table': self.incoming_table.copy(),
                'outgoing_table': self.outgoing_table.copy()
            }

            for slot, data in self.slots.items():
                if data is None:
                    status['slots'][slot] = "FREE"
                else:
                    dr_id, end_time, req_time = data
                    remaining = max(0, end_time - current_time)
                    status['slots'][slot] = f"DR{dr_id}(ends:{end_time:.6f}, rem:{remaining:.6f})"

            return status

    def print_status(self):
        """Print current status"""
        status = self.get_status()
        self.logger.log(f"--- GS{self.gs_id} Status ---")
        self.logger.log(f"Slots: {status['slots']}")
        self.logger.log(f"Incoming table: {status['incoming_table']}")
        self.logger.log(f"Outgoing table: {status['outgoing_table']}")

    def stop(self):
        """Stop the GS thread"""
        self.running = False


class DR(threading.Thread):
    """Drone as a thread"""

    def __init__(self, dr_id, start_gs, end_gs, start_delay, gs_network, logger):
        super().__init__(daemon=True)
        self.dr_id = dr_id
        self.start_gs = start_gs
        self.end_gs = end_gs
        self.start_delay = start_delay
        self.gs_network = gs_network
        self.logger = logger
        self.success = False
        self.completion_time = None
        self.mission_start_time = None
        self.mission_duration = None

    def run(self):
        """Main drone thread"""
        # Wait for start delay
        time.sleep(self.start_delay)

        self.mission_start_time = time.time()
        self.logger.log(
            f"[THREAD] DR{self.dr_id} thread started - Mission: GS{self.start_gs} -> GS{self.end_gs}")

        # Calculate shortest path
        path, total_distance = self.dijkstra_shortest_path(self.start_gs, self.end_gs)
        self.logger.log(f"[EVENT] DR{self.dr_id} shortest path: {' -> '.join([f'GS{gs}' for gs in path])}")
        self.logger.log(f"[EVENT] DR{self.dr_id} total distance: {total_distance:.6f}m")

        # Execute mission
        success = self.execute_mission(path)

        end_time = time.time()
        self.completion_time = end_time
        self.success = success
        self.mission_duration = end_time - self.mission_start_time

        if success:
            self.logger.log(f"[SUCCESS] DR{self.dr_id} completed mission in {self.mission_duration:.6f}s")
        else:
            self.logger.log(f"[FAILED] DR{self.dr_id} mission failed after {self.mission_duration:.6f}s")

    def execute_mission(self, path):
        """Execute the drone mission along the path"""
        current_gs_id = self.start_gs

        for i in range(len(path) - 1):
            current_gs_id = path[i]
            next_gs_id = path[i + 1]

            current_gs = self.gs_network[current_gs_id]
            target_gs = self.gs_network[next_gs_id]
            self.logger.log(f"[EVENT] DR{self.dr_id} at GS{current_gs_id}, requesting slot at GS{next_gs_id}")

            # Get distance/travel time
            if next_gs_id not in current_gs.connections:
                self.logger.log(f"[ERROR] No connection from GS{current_gs_id} to GS{next_gs_id}")
                return False

            travel_time = current_gs.connections[next_gs_id]

            # Request slot reservation
            allocated_slot, reservation_time = current_gs.request_slot_from(target_gs, self.dr_id, travel_time)

            if allocated_slot:
                self.logger.log(f"[EVENT] DR{self.dr_id} allocated {allocated_slot} at GS{next_gs_id}")
                drone_movements.append({
                    "dr_id": self.dr_id,
                    "from_gs": current_gs_id,
                    "to_gs": next_gs_id,
                    "slot": allocated_slot,
                    "start_time": time.time(),
                    "travel_time": travel_time
                })
                self.logger.log(
                    f"[EVENT] DR{self.dr_id} traveling from GS{current_gs_id} to GS{next_gs_id} (duration: {travel_time:.6f}s)")

                # Simulate travel time
                time.sleep(travel_time)

                self.logger.log(f"[EVENT] DR{self.dr_id} arrived at GS{next_gs_id}")

                # Print status of both GSs
                current_gs.print_status()
                target_gs.print_status()
            else:
                self.logger.log(f"[ERROR] DR{self.dr_id} could not get slot at GS{next_gs_id}")
                return False

        return True

    def dijkstra_shortest_path(self, start, end):
        """Calculate shortest path using Dijkstra's algorithm"""
        distances = {gs_id: float('inf') for gs_id in self.gs_network}
        distances[start] = 0.0
        previous = {}
        pq = [(0.0, start)]

        while pq:
            current_dist, current = heapq.heappop(pq)

            if current == end:
                break

            if current_dist > distances[current]:
                continue

            current_gs = self.gs_network[current]
            for neighbor, weight in current_gs.connections.items():
                distance = current_dist + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current
                    heapq.heappush(pq, (distance, neighbor))

        # Reconstruct path
        path = []
        current = end
        while current in previous:
            path.append(current)
            current = previous[current]
        path.append(start)
        path.reverse()

        return path, distances[end]


class DroneRoutingSimulation:
    """Main simulation controller"""

    def __init__(self):
        self.logger = ThreadSafeLogger()
        self.gs_network = {}
        self.gs_threads = {}
        self.dr_threads = {}

    def add_gs(self, gs_id):
        """Add a GS to the network"""
        gs = GS(gs_id, self.logger)
        self.gs_network[gs_id] = gs
        self.gs_threads[gs_id] = gs

    def add_connection(self, gs1_id, gs2_id, distance):
        """Add bidirectional connection between two GSs"""
        if gs1_id not in self.gs_network:
            self.add_gs(gs1_id)
        if gs2_id not in self.gs_network:
            self.add_gs(gs2_id)

        self.gs_network[gs1_id].add_connection(gs2_id, distance)
        self.gs_network[gs2_id].add_connection(gs1_id, distance)

    def start_gs_threads(self):
        """Start all GS threads"""
        for gs in self.gs_threads.values():
            gs.start()

    def create_network_topology(self):
        """Create the network topology based on the new graph"""
        self.logger.log("Creating network topology...")

        # Define the edges from your new graph
        edges = [
            (0, 1), (0, 2), (0, 3), (0, 4),
            (1, 6),
            (2, 7), (2, 8),
            (3, 9),
            (3, 5),
            (5, 7), (5, 2),
            (6, 8),
            (7, 9)
        ]

        # Assign distances to each edge (you can modify these values as needed)
        # Using random distances between 3-10 for variety, but you can set specific values
        edge_distances = {
            (0, 1): 5.0, (0, 2): 4.0, (0, 3): 6.0, (0, 4): 7.0,
            (1, 6): 8.0,
            (2, 7): 5.0, (2, 8): 6.0,
            (3, 9): 9.0,
            (3, 5): 4.0,
            (5, 7): 3.0, (5, 2): 5.0,
            (6, 8): 7.0,
            (7, 9): 6.0
        }

        # Create connections based on the edges
        for gs1, gs2 in edges:
            # Get distance for this edge (use the tuple or its reverse)
            distance = edge_distances.get((gs1, gs2)) or edge_distances.get((gs2, gs1))
            if distance is None:
                distance = 5.0  # Default distance if not specified

            self.add_connection(gs1, gs2, distance)
            self.logger.log(
                f"[SETUP] Added connection: GS{gs1} <-> GS{gs2} (distance: {distance:.6f}m)")

    def simulate_drones(self):
        """Simulate all drones with routes adapted to the new graph"""
        # Updated drone missions to work with the new graph topology (nodes 0-9)
        dr_missions = [
            (1, 0, 9), (2, 0, 8), (3, 1, 7), (4, 2, 6),
            (5, 3, 8), (6, 4, 9), (7, 5, 6), (8, 0, 5)
        ]

        self.logger.log("Starting drone missions...")

        # Create and start drone threads with 2-second intervals
        for i, (dr_id, start_gs, end_gs) in enumerate(dr_missions):
            start_delay = i * 2.0  # 2-second intervals
            dr = DR(dr_id, start_gs, end_gs, start_delay, self.gs_network, self.logger)
            self.dr_threads[dr_id] = dr
            dr.start()

    def run_simulation(self):
        """Run the complete simulation"""
        print("=" * 80)
        print("MULTI-THREADED DRONE ROUTING SIMULATION - NEW GRAPH TOPOLOGY")
        print("=" * 80)

        # Create network
        self.create_network_topology()

        # Start GS threads
        self.start_gs_threads()
        time.sleep(0.1)  # Let GS threads initialize

        # Start drone simulations
        self.simulate_drones()

        # Wait for all drones to complete
        for dr in self.dr_threads.values():
            dr.join()

        # Print final summary
        self.print_final_summary()

        # Stop GS threads
        for gs in self.gs_threads.values():
            gs.stop()

    def print_final_summary(self):
        """Print final simulation summary"""
        time.sleep(1.0)  # Let all threads finish logging

        print(f"\n{'=' * 100}")
        print("SIMULATION SUMMARY")
        print(f"{'=' * 100}")
        print(
            f"{'DR':<4} {'Route':<15} {'Status':<10} {'Mission Time (s)':<18} {'Start Delay (s)':<18}")
        print("-" * 100)

        # Calculate statistics
        successful_missions = []
        failed_missions = []
        total_missions = len(self.dr_threads)

        for dr_id in sorted(self.dr_threads.keys()):
            dr = self.dr_threads[dr_id]
            route = f"GS{dr.start_gs} -> GS{dr.end_gs}"
            status = "SUCCESS" if dr.success else "FAILED"
            mission_time = f"{dr.mission_duration:.6f}" if dr.mission_duration else "N/A"
            start_delay = f"{dr.start_delay:.6f}"

            print(f"{dr_id:<4} {route:<15} {status:<10} {mission_time:<18} {start_delay:<18}")

            if dr.success and dr.mission_duration:
                successful_missions.append(dr.mission_duration)
            else:
                failed_missions.append(dr_id)

        # Print statistics
        print(f"\n{'=' * 100}")
        print("MISSION STATISTICS")
        print(f"{'=' * 100}")
        print(f"Total Missions: {total_missions}")
        print(f"Successful: {len(successful_missions)}")
        print(f"Failed: {len(failed_missions)}")

        if successful_missions:
            avg_time = sum(successful_missions) / len(successful_missions)
            min_time = min(successful_missions)
            max_time = max(successful_missions)
            print(f"\nMission Time Statistics for Successful Missions:")
            print(f"  Average: {avg_time:.6f}s")
            print(f"  Minimum: {min_time:.6f}s")
            print(f"  Maximum: {max_time:.6f}s")

        if failed_missions:
            print(f"\nFailed Missions: DR{', DR'.join(map(str, failed_missions))}")

        print(f"{'=' * 100}")

        import json

        with open("drone_output.json", "w") as f:
            json.dump(drone_movements, f, indent=4)

def main():
    sim = DroneRoutingSimulation()
    sim.run_simulation()


if __name__ == "__main__":
    main()
