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
    def __init__(self, robots, taps, plants):
        self.robots = tuple(sorted(robots)) 
        self.taps = tuple(sorted(taps))      
        self.plants = tuple(sorted(plants))  

    def __eq__(self, other):
        return (self.robots == other.robots and 
                self.taps == other.taps and 
                self.plants == other.plants)

    def __hash__(self):
        return hash((self.robots, self.taps, self.plants))
    
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
        
        taps_list = []
        for loc, amount in initial['Taps'].items():
            taps_list.append((loc[0], loc[1], amount))
            
        plants_list = []
        for loc, needed in initial['Plants'].items():
            plants_list.append((loc[0], loc[1], needed))
        
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
        robots_locs = { (r[1], r[2]) for r in state.robots }
        
        for r_idx, robot in enumerate(state.robots):
            r_id, r_x, r_y, r_load, r_cap = robot
            moves = [("UP", -1, 0), ("DOWN", 1, 0), ("LEFT", 0, -1), ("RIGHT", 0, 1)]
            for move_name, dx, dy in moves:
                new_x, new_y = r_x + dx, r_y + dy
                
                if not (0 <= new_x < ROWS and 0 <= new_y < COLS): continue
                if (new_x, new_y) in WALLS: continue
                if (new_x, new_y) in robots_locs: continue
                
                new_robots = list(state.robots)
                new_robots[r_idx] = (r_id, new_x, new_y, r_load, r_cap)
                new_state = State(tuple(new_robots), state.taps, state.plants)
                successors.append((f"{move_name}{{{r_id}}}", new_state))

            if r_load < r_cap:
                if (r_x, r_y) in self.taps_map:
                    t_idx = self.taps_map[(r_x, r_y)]
                    t_x, t_y, t_amount = state.taps[t_idx]
                    
                    if t_amount > 0:
                        new_robots = list(state.robots)
                        new_robots[r_idx] = (r_id, r_x, r_y, r_load + 1, r_cap)
                        
                        new_taps = list(state.taps)
                        new_taps[t_idx] = (t_x, t_y, t_amount - 1)
                        
                        new_state = State(tuple(new_robots), tuple(new_taps), state.plants)
                        successors.append((f"LOAD{{{r_id}}}", new_state))

            if r_load > 0:
                if (r_x, r_y) in self.plants_map:
                    p_idx = self.plants_map[(r_x, r_y)]
                    p_x, p_y, p_needed = state.plants[p_idx]
                    
                    if p_needed > 0:
                        new_robots = list(state.robots)
                        new_robots[r_idx] = (r_id, r_x, r_y, r_load - 1, r_cap)
                        
                        new_plants = list(state.plants)
                        new_plants[p_idx] = (p_x, p_y, p_needed - 1)
                        
                        new_state = State(tuple(new_robots), state.taps, tuple(new_plants))
                        successors.append((f"POUR{{{r_id}}}", new_state))

        return successors

    def goal_test(self, state):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        for plant in state.plants:
            if plant[2] > 0: 
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
        for p_idx, plant in enumerate(state.plants):
            p_x, p_y, p_needed = plant
            if p_needed == 0:
                continue
            min_dist_robot = float('inf')
            for robot in loaded_robots:
                r_x, r_y = robot[1], robot[2]
                dist = self.manhattan_distance(r_x, r_y, p_x, p_y)
                if dist < min_dist_robot:
                    min_dist_robot = dist
            min_dist_tap = float('inf')
            for tap in active_taps:
                t_x, t_y = tap[0], tap[1]
                dist = self.manhattan_distance(t_x, t_y, p_x, p_y)
                if dist < min_dist_tap:
                    min_dist_tap = dist       
            dist_to_resource = min(min_dist_robot, min_dist_tap)
            
            if dist_to_resource == float('inf'):
                return float('inf')
            total_cost += (dist_to_resource * 1) + p_needed  
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