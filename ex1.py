import search
import utils

id = ["207810490"]

# global variables (Static map data)
WALLS = set()
ROWS = 0
COLS = 0

class State:
    """
    Ultra-Optimized State class.
    1. Uses __slots__ to save memory.
    2. Assumes data is ALREADY sorted and tuples (no re-sorting).
    3. Caches hash value.
    """
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
        
        # --- 1. Initial Parsing & Sorting (Done ONCE) ---
        robots_list = []
        for r_id, r_data in initial['Robots'].items():
            robots_list.append((r_id, r_data[0], r_data[1], r_data[2], r_data[3]))
        robots_list.sort()
        
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

        # --- 2. Fast Lookup Maps ---
        self.taps_map = {}
        for idx, tap in enumerate(initial_state.taps):
            self.taps_map[(tap[0], tap[1])] = idx
            
        self.plants_map = {}
        for idx, plant in enumerate(initial_state.plants):
            self.plants_map[(plant[0], plant[1])] = idx

        # --- 3. HEURISTIC PRE-CALCULATION ---
        # חישוב מרחק סטטי לברזים (חוסך זמן יקר ב-h_astar)
        self.static_plant_to_tap_dist = {}
        
        for p in plants_list:
            p_x, p_y = p[0], p[1]
            min_dist = float('inf')
            # בדיקת מרחק לכל הברזים
            for t in taps_list:
                d = abs(p_x - t[0]) + abs(p_y - t[1])
                if d < min_dist:
                    min_dist = d
            self.static_plant_to_tap_dist[(p_x, p_y)] = min_dist


    def successor(self, state):
        """ 
        Generates successor states using tuple slicing for speed.
        """
        successors = []
        current_robots = state.robots
        robots_locs = { (r[1], r[2]) for r in current_robots }
        
        for r_idx, robot in enumerate(current_robots):
            r_id, r_x, r_y, r_load, r_cap = robot
            
            # --- Moves ---
            moves = (("UP", -1, 0), ("DOWN", 1, 0), ("LEFT", 0, -1), ("RIGHT", 0, 1))
            for move_name, dx, dy in moves:
                new_x, new_y = r_x + dx, r_y + dy
                
                if not (0 <= new_x < ROWS and 0 <= new_y < COLS): continue
                if (new_x, new_y) in WALLS: continue
                if (new_x, new_y) in robots_locs: continue
                
                new_robot_data = (r_id, new_x, new_y, r_load, r_cap)
                new_robots = current_robots[:r_idx] + (new_robot_data,) + current_robots[r_idx+1:]
                
                new_state = State(new_robots, state.taps, state.plants)
                successors.append((f"{move_name}{{{r_id}}}", new_state))

            # --- Load ---
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
                        successors.append((f"LOAD{{{r_id}}}", new_state))

            # --- Pour ---
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
                        successors.append((f"POUR{{{r_id}}}", new_state))

        return successors

    def goal_test(self, state):
        for i in range(len(state.plants)):
            if state.plants[i][2] > 0:
                return False
        return True

    def h_astar(self, node):
        """
        Admissible Heuristic (Optimized):
        Sum of needed water + Max(Min Distance to Source).
        """
        state = node.state
        total_needed = 0
        max_min_dist = 0
        
        loaded_robots = [r for r in state.robots if r[3] > 0]
        
        for plant in state.plants:
            p_needed = plant[2]
            if p_needed == 0:
                continue
            
            total_needed += p_needed
            
            p_x, p_y = plant[0], plant[1]
            
            # 1. מרחק בסיס (חושב מראש)
            best_dist = self.static_plant_to_tap_dist.get((p_x, p_y), float('inf'))
            
            # 2. שיפור אם יש רובוט קרוב יותר
            for r in loaded_robots:
                dist = abs(r[1] - p_x) + abs(r[2] - p_y)
                if dist < best_dist:
                    best_dist = dist
            
            if best_dist != float('inf'):
                if best_dist > max_min_dist:
                    max_min_dist = best_dist
            else:
                return float('inf')

        return total_needed + max_min_dist

    def h_gbfs(self, node):
        """
        Greedy Heuristic (Optimized for speed)
        """
        state = node.state
        min_action_cost = float('inf')
        plants_needing_water = False
        
        loaded_robots = [r for r in state.robots if r[3] > 0]
        
        for plant in state.plants:
            if plant[2] == 0: continue
            plants_needing_water = True
            
            p_x, p_y = plant[0], plant[1]
            
            dist_from_tap = self.static_plant_to_tap_dist.get((p_x, p_y), float('inf'))
            current_cost = dist_from_tap + 2 
            
            for r in loaded_robots:
                dist = abs(r[1] - p_x) + abs(r[2] - p_y)
                if dist < current_cost:
                    current_cost = dist
            
            if current_cost < min_action_cost:
                min_action_cost = current_cost
        
        if not plants_needing_water:
            return 0
            
        return min_action_cost

# --- הנה הפונקציה שהייתה חסרה! ---
def create_watering_problem(game):
    print("<<create_watering_problem")
    return WateringProblem(game)