import time

import ex1
import search


def run_problem(func, targs=(), kwargs=None):
    if kwargs is None:
        kwargs = {}
    result = (-3, "default")
    try:
        result = func(*targs, **kwargs)
    except Exception as e:
        result = (-3, e)
    return result


# Problem definitions
Problem_pdf = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
}

problem1 = {
    "Size": (3, 3),
    "Walls": set(),
    "Taps": {(1, 1): 3},
    "Plants": {(0, 2): 2},
    "Robots": {10: (2, 0, 0, 2)},
}

problem2 = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(0, 2): 3, (2, 0): 2},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
}

problem3 = {
    "Size": (5, 3),
    "Walls": {(1, 1), (3, 1)},
    "Taps": {(0, 0): 5},
    "Plants": {(4, 2): 4},
    "Robots": {10: (2, 0, 0, 2)},
}

problem4 = {
    "Size": (5, 5),
    "Walls": {(0, 1), (1, 1), (2, 1), (0, 3), (1, 3), (2, 3)},
    "Taps": {(3, 2): 1, (4, 2): 1},
    "Plants": {(0, 2): 1, (1, 2): 1},
    "Robots": {10: (3, 1, 0, 1), 11: (3, 3, 0, 1)},
}

problem5_deadend = {
    "Size": (3, 4),
    "Walls": set(),
    "Taps": {(1, 1): 3},
    "Plants": {(0, 3): 2, (2, 3): 2},
    "Robots": {10: (1, 0, 0, 2)},
}

problem6 = {
    "Size": (8, 8),
    "Walls": {*((r, c) for r in range(8) for c in range(8) if not (r == 1 and c in (0, 1, 2)))},
    "Taps": {(1, 1): 3},
    "Plants": {(1, 2): 3},
    "Robots": {10: (1, 0, 0, 3)},
}

problem7 = {
    "Size": (4, 4),
    "Walls": set(),
    "Taps": {(2, 2): 18},
    "Plants": {(0, 3): 3, (3, 0): 3},
    "Robots": {10: (2, 1, 0, 3), 11: (2, 0, 0, 3)},
}

problem_hard1 = {
    "Size": (5, 6),
    "Walls": {(1, 2), (1, 3), (3, 2), (3, 3)},
    "Taps": {(2, 2): 12},
    "Plants": {(0, 1): 3, (4, 5): 6},
    "Robots": {10: (2, 1, 0, 6), 11: (2, 4, 0, 3)},
}

problem_hard2 = {
    "Size": (5, 5),
    "Walls": {(0, 0), (0, 4), (4, 0), (4, 4)},
    "Taps": {(2, 2): 9},
    "Plants": {(1, 2): 3, (2, 1): 3, (3, 2): 3},
    "Robots": {10: (2, 3, 0, 3)},
}

problem_hard3 = {
    "Size": (5, 5),
    "Walls": {(0, 0), (0, 4), (4, 0), (4, 4)},
    "Taps": {(2, 2): 9},
    "Plants": {(1, 2): 3, (2, 1): 3, (3, 2): 3},
    "Robots": {10: (2, 3, 0, 3)},
}

problem_hard4 = {
    "Size": (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 6, (3, 3): 6},
    "Plants": {(0, 0): 3, (4, 5): 3},
    "Robots": {10: (1, 1, 0, 3), 11: (3, 4, 0, 2)},
}

problem_hard5 = {
    "Size": (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 8, (3, 3): 8},
    "Plants": {(0, 0): 4, (4, 5): 4},
    "Robots": {10: (1, 1, 0, 4), 11: (3, 4, 0, 3)},
}

problem_hard6 = {
    "Size": (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
}

problem_load = {
    "Size": (10, 4),
    "Walls": {(0, 1), (1, 1), (2, 1), (3, 1), (4, 1), (6, 1), (7, 1), (8, 1), (9, 1), (4, 2), (4, 3), (6, 2), (6, 3)},
    "Taps": {(5, 3): 20},
    "Plants": {(0, 0): 10, (9, 0): 10},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 20)},
}

problem_10x10_single = {
    "Size": (10, 10),
    "Walls": set(),
    "Taps": {(5, 5): 24},
    "Plants": {(0, 0): 5, (0, 9): 5, (9, 0): 5, (9, 9): 5},
    "Robots": {10: (9, 5, 0, 5)},
}

problem_12x12_snake = {
    "Size": (12, 12),
    "Walls": {
        (1, 3), (2, 3), (3, 3), (4, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3),
        (1, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (10, 6),
        (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (8, 9), (9, 9), (10, 9),
    },
    "Taps": {(5, 1): 24},
    "Plants": {(2, 11): 5, (7, 11): 5, (10, 10): 5, (0, 8): 5},
    "Robots": {10: (11, 1, 0, 3)},
}

problem_12x12_snake_hard = {
    "Size": (12, 12),
    "Walls": {
        (1, 3), (2, 3), (3, 3), (4, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3),
        (1, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (10, 6),
        (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (8, 9), (9, 9), (10, 9),
    },
    "Taps": {(5, 1): 24},
    "Plants": {(2, 11): 5, (7, 11): 5, (10, 10): 5, (0, 8): 5},
    "Robots": {10: (11, 1, 0, 2)},
}

optimal_map = {
    "problem1": 8, "problem2": 20, "problem3": 28, "problem4": 13,
    "problem5_deadend": None, "problem6": 8, "problem7": 21,
    "problem_hard1": 31, "problem_hard2": 24, "problem_hard3": 24,
    "problem_hard4": 25, "problem_hard5": 29, "problem_hard6": 33,
    "problem_load": 65, "problem_10x10_single": 106,
    "problem_12x12_snake": 249, "problem_12x12_snake_hard": 343,
}


def solve_problems(problem, algorithm, problem_name=None, optimal=None):
    try:
        p = ex1.create_watering_problem(problem)
    except Exception as e:
        print("Error creating problem: ", e)
        return None

    alg_label = algorithm.upper()
    print(f"Algorithm: {alg_label}")

    # Time the algorithm execution
    alg_start = time.time()
    if algorithm == "gbfs":
        result = run_problem(lambda p: search.greedy_best_first_graph_search(p, p.h_gbfs), targs=[p])
    else:
        result = run_problem(lambda p: search.astar_search(p, p.h_astar), targs=[p])
    alg_end = time.time()

    # Print time taken for this algorithm
    print(f"Time: \033[93m{alg_end - alg_start:.4f}\033[0m seconds")

    if result and isinstance(result[0], search.Node):
        solve = result[0].path()[::-1]
        solution = [pi.action for pi in solve][1:]
        print(f"The length of the solution is - \033[92m{len(solution)}\033[0m")
        print("Solution:")
        print(solution)
        print()
    else:
        print("No solution\n")


def main():
    start = time.time()

    problems = [
        problem1, problem2, problem3, problem4, problem5_deadend, problem6, problem7,
        problem_hard1, problem_hard2, problem_hard3, problem_hard4,
        problem_hard5, problem_hard6, problem_load, problem_10x10_single,
        problem_12x12_snake, problem_12x12_snake_hard
    ]

    problem_names = [
        "problem1", "problem2", "problem3", "problem4", "problem5_deadend", "problem6", "problem7",
        "problem_hard1", "problem_hard2", "problem_hard3", "problem_hard4",
        "problem_hard5", "problem_hard6", "problem_load", "problem_10x10_single",
        "problem_12x12_snake", "problem_12x12_snake_hard"
    ]

    for p_name, p in zip(problem_names, problems):
        optimal = optimal_map.get(p_name)
        print(f"\033[91mCurrently working on: {p_name}    (optimal: {optimal if optimal is not None else 'Unknown'})\033[0m")
        
        problem_start = time.time()
        for a in ['astar', 'gbfs']:
            solve_problems(p, a, problem_name=p_name, optimal=optimal)
        problem_end = time.time()
        
        print(f"\033[95mTotal time for {p_name}: {problem_end - problem_start:.4f} seconds\033[0m")
        print("\033[96m" + "-" * 60 + "\033[0m")

    end = time.time()
    print('Submission took:', end - start, 'seconds.')


if __name__ == '__main__':
    main()