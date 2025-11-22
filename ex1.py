import search
import utils

id = ["207810490"]

# global variables
WALLS = set()
ROWS = 0
COLS = 0

class State:
    __slots__ = ('robots', 'taps', 'plants', '_hash')

    def __init__(self, robots, taps, plants):
        self.robots = robots 
        self.taps = taps
        self.plants = plants
        self._hash = hash((self.robots, self.taps, self.plants))

    def __eq__(self, other):
        return (self.robots == other.robots and 
                self.taps == other.taps and 
                self.plants == other.plants)

    def __hash__(self):
        return self._hash
    
    def __lt__(self, other):
        return self._hash < other._hash


class WateringProblem(search.Problem):
    def __init__(self, initial):
        global ROWS, COLS, WALLS
        ROWS, COLS = initial['Size']
        WALLS = initial['Walls']
        
        robots_list = []
        max_cap = 1 
        
        self.action_strings = {}
        
        for r_id, r_data in initial['Robots'].items():
            robots_list.append((r_id, r_data[0], r_data[1], r_data[2], r_data[3]))
            if r_data[3] > max_cap:
                max_cap = r_data[3]
            
            self.action_strings[(r_id, "UP")] = f"UP{{{r_id}}}"
            self.action_strings[(r_id, "DOWN")] = f"DOWN{{{r_id}}}"
            self.action_strings[(r_id, "LEFT")] = f"LEFT{{{r_id}}}"
            self.action_strings[(r_id, "RIGHT")] = f"RIGHT{{{r_id}}}"
            self.action_strings[(r_id, "LOAD")] = f"LOAD{{{r_id}}}"
            self.action_strings[(r_id, "POUR")] = f"POUR{{{r_id}}}"

        robots_list.sort()
        self.max_capacity = max_cap
        
        taps_list = []
        for loc, amount in initial['Taps'].items():
            taps_list.append((loc[0], loc[1], amount))
        taps_list.sort()
            
        plants_list = []
        for loc, needed in initial['Plants'].items():
            plants_list.append((loc[0], loc[1], needed))
        plants_list.sort()
        
        initial_state = State(
            robots=tuple(robots_list),
            taps=tuple(taps_list),
            plants=tuple(plants_list)
        )
        
        search.Problem.__init__(self, initial_state)

        self.taps_map = {}
        for idx, tap in enumerate(initial_state.taps):
            self.taps_map[(tap[0], tap[1])] = idx
            
        self.plants_map = {}
        for idx, plant in enumerate(initial_state.plants):
            self.plants_map[(plant[0], plant[1])] = idx

        self.dist_to_nearest_tap = self._bfs_map([(t[0], t[1]) for t in taps_list])

        self.plant_paths = {}
        for i, p in enumerate(plants_list):
            self.plant_paths[i] = self._bfs_map([(p[0], p[1])])

        self.static_plant_to_tap_dist = {}
        for i, p in enumerate(plants_list):
            p_pos = (p[0], p[1])
            self.static_plant_to_tap_dist[i] = self.dist_to_nearest_tap.get(p_pos, float('inf'))

    def _bfs_map(self, start_points):
        distances = {}
        queue = []
        visited = set()
        for sp in start_points:
            distances[sp] = 0
            queue.append((sp, 0))
            visited.add(sp)
        head = 0
        while head < len(queue):
            curr_pos, curr_dist = queue[head]
            head += 1
            cx, cy = curr_pos
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < ROWS and 0 <= ny < COLS:
                    if (nx, ny) not in WALLS and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        distances[(nx, ny)] = curr_dist + 1
                        queue.append(((nx, ny), curr_dist + 1))
        return distances

    def successor(self, state):
        successors = []
        current_robots = state.robots
        robots_locs = { (r[1], r[2]) for r in current_robots }
        
        for r_idx, robot in enumerate(current_robots):
            r_id, r_x, r_y, r_load, r_cap = robot
            
            # Moves
            moves = (("UP", -1, 0), ("DOWN", 1, 0), ("LEFT", 0, -1), ("RIGHT", 0, 1))
            for move_name, dx, dy in moves:
                new_x, new_y = r_x + dx, r_y + dy
                if not (0 <= new_x < ROWS and 0 <= new_y < COLS): continue
                if (new_x, new_y) in WALLS: continue
                if (new_x, new_y) in robots_locs: continue
                
                new_robot_data = (r_id, new_x, new_y, r_load, r_cap)
                new_robots = current_robots[:r_idx] + (new_robot_data,) + current_robots[r_idx+1:]
                new_state = State(new_robots, state.taps, state.plants)
                
                successors.append((self.action_strings[(r_id, move_name)], new_state))

            # Load
            if r_load < r_cap:
                if (r_x, r_y) in self.taps_map:
                    t_idx = self.taps_map[(r_x, r_y)]
                    t_tap = state.taps[t_idx]
                    if t_tap[2] > 0:
                        new_robot_data = (r_id, r_x, r_y, r_load + 1, r_cap)
                        new_robots = current_robots[:r_idx] + (new_robot_data,) + current_robots[r_idx+1:]
                        new_tap_data = (t_tap[0], t_tap[1], t_tap[2] - 1)
                        new_taps = state.taps[:t_idx] + (new_tap_data,) + state.taps[t_idx+1:]
                        new_state = State(new_robots, new_taps, state.plants)
                        successors.append((self.action_strings[(r_id, "LOAD")], new_state))

            # Pour
            if r_load > 0:
                if (r_x, r_y) in self.plants_map:
                    p_idx = self.plants_map[(r_x, r_y)]
                    p_plant = state.plants[p_idx]
                    if p_plant[2] > 0:
                        new_robot_data = (r_id, r_x, r_y, r_load - 1, r_cap)
                        new_robots = current_robots[:r_idx] + (new_robot_data,) + current_robots[r_idx+1:]
                        new_plant_data = (p_plant[0], p_plant[1], p_plant[2] - 1)
                        new_plants = state.plants[:p_idx] + (new_plant_data,) + state.plants[p_idx+1:]
                        new_state = State(new_robots, state.taps, new_plants)
                        successors.append((self.action_strings[(r_id, "POUR")], new_state))

        return successors

    def goal_test(self, state):
        for i in range(len(state.plants)):
            if state.plants[i][2] > 0:
                return False
        return True

    def h_astar(self, node):
        state = node.state

        # POUR lower bound
        total_needed = 0
        remaining_plants = []  # indices of plants still needing water
        for i, p in enumerate(state.plants):
            if p[2] > 0:
                total_needed += p[2]
                remaining_plants.append(i)

        if total_needed == 0:
            return 0

        # water currently on robots
        current_water = 0
        loaded_robots_indices = []
        for i, r in enumerate(state.robots):
            if r[3] > 0:
                current_water += r[3]
                loaded_robots_indices.append(i)

        # LOAD lower bound 
        deficit = total_needed - current_water
        if deficit < 0:
            deficit = 0

        # MOVE lower bounds
        lb_reachPlants = 0
        lb_transport = 0

        sum_nearest_dists = 0

        for i in remaining_plants:
            plant = state.plants[i]
            p_needed = plant[2]
            dist_map = self.plant_paths[i]

            # closest robot to this plant
            closest_robot_dist = float('inf')
            for r in state.robots:
                d = dist_map.get((r[1], r[2]), float('inf'))
                if d < closest_robot_dist:
                    closest_robot_dist = d

            # unreachable plant -> dead end
            if closest_robot_dist == float('inf'):
                return float('inf')  

            # update reachPlants bound
            if closest_robot_dist > lb_reachPlants:
                lb_reachPlants = closest_robot_dist

            # accumulate for LB_avg
            sum_nearest_dists += closest_robot_dist

            # best source distance: nearest tap OR already-loaded robot
            best_source_dist = self.static_plant_to_tap_dist[i]
            for r_idx in loaded_robots_indices:
                r = state.robots[r_idx]
                d = dist_map.get((r[1], r[2]), float('inf'))
                if d < best_source_dist:
                    best_source_dist = d

            if best_source_dist == float('inf'):
                return float('inf')

            trips = (p_needed + self.max_capacity - 1) // self.max_capacity  
            plant_transport = trips * best_source_dist
            if plant_transport > lb_transport:
                lb_transport = plant_transport

        lb_reachTap = 0
        if deficit > 0:
            min_dist_to_tap = float('inf')
            for r in state.robots:
                d = self.dist_to_nearest_tap.get((r[1], r[2]), float('inf'))
                if d < min_dist_to_tap:
                    min_dist_to_tap = d
            if min_dist_to_tap != float('inf'):
                lb_reachTap = min_dist_to_tap

        # LB_avg for multi-robots
        # even with perfect division, average robot must cover at least sum_nearest_dists / R moves
        R = len(state.robots)
        lb_avg = (sum_nearest_dists + R - 1) // R  # ceil

        # MST cover bound 
        lb_cover = 0
        if R == 1 and len(remaining_plants) >= 2:
            plant_positions = [(state.plants[i][0], state.plants[i][1]) for i in remaining_plants]

            # start -> nearest plant
            r0 = state.robots[0]
            start_cost = float('inf')
            for i in remaining_plants:
                d = self.plant_paths[i].get((r0[1], r0[2]), float('inf'))
                if d < start_cost:
                    start_cost = d

            # Prim MST on plants
            k = len(remaining_plants)
            in_mst = [False] * k
            min_edge = [float('inf')] * k
            min_edge[0] = 0
            mst_cost = 0

            for _ in range(k):
                v = -1
                best = float('inf')
                for j in range(k):
                    if not in_mst[j] and min_edge[j] < best:
                        best = min_edge[j]
                        v = j

                in_mst[v] = True
                mst_cost += best

                i_v = remaining_plants[v]
                dist_map_v = self.plant_paths[i_v]

                for u in range(k):
                    if not in_mst[u]:
                        pos_u = plant_positions[u]
                        dvu = dist_map_v.get(pos_u, float('inf'))
                        if dvu < min_edge[u]:
                            min_edge[u] = dvu

            if start_cost != float('inf'):
                lb_cover = start_cost + mst_cost

        lb_moves = max(lb_reachPlants, lb_reachTap, lb_transport, lb_cover, lb_avg)

        return total_needed + deficit + lb_moves


    def h_gbfs(self, node):
        """
        GBFS remains aggressive (SUM).
        """
        state = node.state
        total_score = 0
        loaded_robots_indices = []
        for i, r in enumerate(state.robots):
            if r[3] > 0: loaded_robots_indices.append(i)
        
        for i in range(len(state.plants)):
            plant = state.plants[i]
            if plant[2] == 0: continue
            
            dist_map = self.plant_paths[i]
            dist = self.static_plant_to_tap_dist[i]
            
            for r_idx in loaded_robots_indices:
                r = state.robots[r_idx]
                d = dist_map.get((r[1], r[2]), float('inf'))
                if d < dist: dist = d
            
            total_score += plant[2] * (dist + 1)
            
        return total_score

def create_watering_problem(game):
    return WateringProblem(game)