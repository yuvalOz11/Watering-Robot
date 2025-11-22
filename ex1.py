import search
import utils
import math

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
        
        # --- 1. Parsing ---
        robots_list = []
        max_cap = 1 
        self.action_strings = {}
        
        for r_id, r_data in initial['Robots'].items():
            robots_list.append((r_id, r_data[0], r_data[1], r_data[2], r_data[3]))
            if r_data[3] > max_cap:
                max_cap = r_data[3]
            
            for action in ["UP", "DOWN", "LEFT", "RIGHT", "LOAD", "POUR"]:
                self.action_strings[(r_id, action)] = f"{action}{{{r_id}}}"

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

        # --- Maps ---
        self.taps_map = {}
        for idx, tap in enumerate(initial_state.taps):
            self.taps_map[(tap[0], tap[1])] = idx
            
        self.plants_map = {}
        for idx, plant in enumerate(initial_state.plants):
            self.plants_map[(plant[0], plant[1])] = idx

        # --- 2. PRE-CALCULATION: BFS ---
        # מרחק מכל משבצת לברז הכי קרוב (כולל קירות)
        self.dist_to_nearest_tap = self._bfs_map([(t[0], t[1]) for t in taps_list])

        # מפות מרחקים לכל צמח
        self.plant_paths = {}
        for i, p in enumerate(plants_list):
            self.plant_paths[i] = self._bfs_map([(p[0], p[1])])

        # מטריצת מרחקים בין צמחים (ל-MST)
        self.plant_to_plant_dist = {}
        num_plants = len(plants_list)
        for i in range(num_plants):
            for j in range(i + 1, num_plants):
                p2 = plants_list[j]
                dist = self.plant_paths[i].get((p2[0], p2[1]), float('inf'))
                self.plant_to_plant_dist[(i, j)] = dist
                self.plant_to_plant_dist[(j, i)] = dist

        # מרחק סטטי מכל צמח לברז
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
        """
        Corrected Hybrid Heuristic (Admissible).
        """
        state = node.state
        total_needed = 0
        current_held_water = 0
        
        active_plant_indices = []
        for i, p in enumerate(state.plants):
            if p[2] > 0:
                total_needed += p[2]
                active_plant_indices.append(i)
        
        loaded_robots = [r for r in state.robots if r[3] > 0]
        for r in state.robots:
            current_held_water += r[3]

        if not active_plant_indices:
            return 0

        # --- 1. Action Costs ---
        h_pours = total_needed
        # עלות טעינה רק עבור המים שחסרים
        h_loads = max(0, total_needed - current_held_water)

        # --- 2. MST (Connectivity) ---
        min_dists = {} 
        has_loaded = len(loaded_robots) > 0
        
        for p_idx in active_plant_indices:
            dist = self.static_plant_to_tap_dist[p_idx]
            if has_loaded:
                plant_map = self.plant_paths[p_idx]
                for r in loaded_robots:
                    d = plant_map.get((r[1], r[2]), float('inf'))
                    if d < dist: dist = d
            min_dists[p_idx] = dist

        mst_val = 0
        visited = set()
        temp_dists = min_dists.copy()
        
        while len(visited) < len(active_plant_indices): 
            best_node = -1
            min_val = float('inf')
            for p_idx in active_plant_indices:
                if p_idx not in visited:
                    if temp_dists[p_idx] < min_val:
                        min_val = temp_dists[p_idx]
                        best_node = p_idx
            if best_node == -1: return float('inf')
            visited.add(best_node)
            mst_val += min_val
            for p_idx in active_plant_indices:
                if p_idx not in visited:
                    if best_node < p_idx:
                        dist_between = self.plant_to_plant_dist.get((best_node, p_idx), float('inf'))
                    else:
                        dist_between = self.plant_to_plant_dist.get((p_idx, best_node), float('inf'))
                    if dist_between < temp_dists[p_idx]:
                        temp_dists[p_idx] = dist_between

        h_mst = mst_val

        # --- 3. Resource Work (Fixed Admissibility) ---
        # חישוב עלות שינוע לכל הטיפות, ואז הפחתת ה"זיכוי" על המים שיש ביד.
        
        # רשימת כל ה"משלוחים" הנדרשים (כמות מים, מרחק שינוע)
        deliveries = []
        for p_idx in active_plant_indices:
            p_need = state.plants[p_idx][2]
            dist_tap = self.static_plant_to_tap_dist[p_idx]
            deliveries.append((dist_tap, p_need))
            
        # מיון לפי מרחק יורד (הכי רחוק = הכי יקר)
        # הנחה: המים שיש לנו ביד (חינם מבחינת שינוע מהברז) הולכים למשלוחים היקרים ביותר.
        deliveries.sort(key=lambda x: x[0], reverse=True)
        
        h_resource = 0
        water_credit = current_held_water
        
        for dist, amount in deliveries:
            if water_credit >= amount:
                # כל הכמות הזו מכוסה ע"י המים שביד -> עלות שינוע מהברז = 0
                water_credit -= amount
            else:
                # חלק מכוסה, השאר חייב להגיע מהברז
                amount_from_tap = amount - water_credit
                water_credit = 0
                
                # חישוב עבודה: כמות חלקי קיבולת כפול מרחק
                # שימוש ב-Ceil כדי לדייק במספר הנגלות לא עובד טוב עם הזיכוי החלקי,
                # אז נשתמש בחישוב רציף שהוא חסם תחתון בטוח.
                h_resource += (amount_from_tap / self.max_capacity) * dist

        # חזרה לברז (Return Trips) עבור צמחים ספציפיים שחייבים הרבה מים
        # (זה תוסף בטוח כי הוא נובע מנפח בלבד)
        for p_idx in active_plant_indices:
            needed = state.plants[p_idx][2]
            trips = math.ceil(needed / self.max_capacity)
            if trips > 1:
                dist_tap = self.static_plant_to_tap_dist[p_idx]
                h_resource += (trips - 1) * 2 * dist_tap

        # קנס התחלתי (אם הכל ריק)
        if h_loads > 0 and current_held_water == 0:
             min_dist_to_tap = float('inf')
             for r in state.robots:
                d = self.dist_to_nearest_tap.get((r[1], r[2]), float('inf'))
                if d < min_dist_to_tap: min_dist_to_tap = d
             if min_dist_to_tap != float('inf'):
                 h_resource += min_dist_to_tap

        return h_pours + h_loads + max(h_mst, h_resource)

    def h_gbfs(self, node):
        state = node.state
        total_score = 0
        loaded_robots_indices = [i for i, r in enumerate(state.robots) if r[3] > 0]
        
        for i in range(len(state.plants)):
            plant = state.plants[i]
            if plant[2] == 0: continue
            dist = self.static_plant_to_tap_dist[i]
            dist_map = self.plant_paths[i]
            for r_idx in loaded_robots_indices:
                r = state.robots[r_idx]
                d = dist_map.get((r[1], r[2]), float('inf'))
                if d < dist: dist = d
            total_score += plant[2] * (dist + 1)
        return total_score

def create_watering_problem(game):
    return WateringProblem(game)