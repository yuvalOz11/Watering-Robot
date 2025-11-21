import search
import utils

id = [207810490]

# global
WALLS = set()
ROWS = 0
COLS = 0


class State:
    """
    helper class representing a specific state in the search space.
    it converts dynamic data (robots, taps, plants) into immutable tuples
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
        return hash(self) < hash(other)



class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        """
        Constructor:
        1. Extracts static map data (Walls, Size) to global variables for memory efficiency.
        2. Parses dynamic data (Robots, Taps, Plants) from mutable dictionaries into 
           immutable tuples within a custom 'State' object.
        3. Initializes the parent 'search.Problem' with this hashable 'initial_state', 
           preventing crashes in algorithms that use sets (like A*).
        """
        global ROWS, COLS, WALLS
        ROWS, COLS = initial['Size']
        WALLS = initial['Walls']
        
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

        self.taps_map = {}
        for idx, tap in enumerate(initial_state.taps):
            # tap = (x, y, amount)
            self.taps_map[(tap[0], tap[1])] = idx
            
        self.plants_map = {}
        for idx, plant in enumerate(initial_state.plants):
            # plant = (x, y, needed)
            self.plants_map[(plant[0], plant[1])] = idx

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        """ 
        Generates the successor states.
        Optimized version using O(1) lookups for taps and plants.
        """
        successors = []
        current_robots = state.robots
        robots_locs = { (r[1], r[2]) for r in current_robots }
        
        for r_idx, robot in enumerate(current_robots):
            r_id, r_x, r_y, r_load, r_cap = robot
            
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
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        for i in range(len(state.plants)):
            if state.plants[i][2] > 0:
                return False
        return True
    
    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)

    def h_astar(self, node):
        """
        This heuristic estimates the cost using the Manhattan Distance metric.
        It is admissible (does not overestimate) because it calculates a relaxed lower bound:
        
        1. For each plant needing water, we search for the closest available water source:
           - Either a robot that already carries water.
           - Or a non-empty tap (ignoring the robot's travel to that tap to ensure admissibility).
           
        2. We calculate the Manhattan Distance (|x1-x2| + |y1-y2|) from the plant to that closest source.
        
        3. The total heuristic is the sum of these minimum distances plus the mandatory POUR actions.
           Since the water *must* travel at least this distance to reach the plant, this
           guarantees the heuristic never exceeds the true optimal cost.
        """
        state = node.state
        total_cost = 0
        
        active_taps = [t for t in state.taps if t[2] > 0]
        loaded_robots = [r for r in state.robots if r[3] > 0]
        
        for plant in state.plants:
            p_x, p_y, p_needed = plant[0], plant[1], plant[2]
            
            if p_needed == 0:
                continue
            
            min_dist = float('inf')
                        
            for r in loaded_robots:
                dist = abs(r[1] - p_x) + abs(r[2] - p_y) 
                if dist < min_dist:
                    min_dist = dist
            
            for t in active_taps:
                dist = abs(t[0] - p_x) + abs(t[1] - p_y) 
                if dist < min_dist:
                    min_dist = dist
            
            if min_dist == float('inf'):
                return float('inf')
                
            total_cost += min_dist + p_needed
            
        return total_cost

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        return self.h_astar(node)

def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()