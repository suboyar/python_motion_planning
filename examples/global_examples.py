"""
@file: global_examples.py
@breif: global planner application examples
@author: Yang Haodong, Wu Maojia
@update: 2024.11.22
"""
import sys, os
import random
import csv
import os
import time
from datetime import datetime
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils.environment.env import Grid, Map, Grid3D, Mountain
from python_motion_planning.utils.planner.search_factory import SearchFactory

def graph_search():
    # build environment
    # TODO: 15-20 situasjon
    # TODO: justere på Mountain.elveation_weight i ulike grad (10%, 25%, 50% f.eks)
    # TODO: Statestikk
    start = (5, 5)
    goal = (40, 25)
    env = Mountain()

    # Works
    planner = search_factory("a_star", start=start, goal=goal, env=env)
    planner = search_factory("dijkstra", start=start, goal=goal, env=env)
    # planner = search_factory("d_star_lite", start=start, goal=goal, env=env)
    # planner = search_factory("lpa_star", start=start, goal=goal, env=env)
    # planner = search_factory("gbfs", start=start, goal=goal, env=env)
    # planner = search_factory("jps", start=start, goal=goal, env=env)
    # planner = search_factory("theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("rrt_star", start=start, goal=goal, env=env)
    # planner = search_factory("lazy_theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("s_theta_star", start=start, goal=goal, env=env)
    # planner = search_factory("informed_rrt", start=start, goal=goal, env=env)
    # planner = search_factory("rrt", start=start, goal=goal, env=env)
    # planner = search_factory("rrt_connect", start=start, goal=goal, env=env)

    # Doesn't work
    # planner = search_factory("d_star", start=start, goal=goal, env=env)
    # planner = search_factory("voronoi", start=start, goal=goal, env=env, n_knn=4, max_edge_len=10.0, inflation_r=1.0) # Needs obstacles

    planner.run()

    # Example call in graph_search() before planner creation:
    ok, reason = validate_point(env, start)
    if not ok:
        raise ValueError(f"Start {start} invalid for environment: {reason}")
    ok, reason = validate_point(env, goal)
    if not ok:
        raise ValueError(f"Goal {goal} invalid for environment: {reason}")

def generate_random_goals(env, n=20, start=None, seed=None):
    """Return up to n random (x,y) goals inside env bounds and not in obstacles."""
    if seed is not None:
        random.seed(seed)
    goals = []
    tries = 0
    max_tries = n * 10
    while len(goals) < n and tries < max_tries:
        tries += 1
        if hasattr(env, "cols") and hasattr(env, "rows"):
            x = random.randrange(0, env.cols)
            y = random.randrange(0, env.rows)
        else:
            x = random.randrange(0, env.x_range)
            y = random.randrange(0, env.y_range)
        pt = (x, y)
        if start and (int(start[0]) == x and int(start[1]) == y):
            continue
        if getattr(env, "obstacles", None) and pt in env.obstacles:
            continue
        if pt in goals:
            continue
        goals.append(pt)
    return goals

def graph_search_with_random_goals():
     # build environment

    start = (5, 5)
    goal = (40, 25)
    env = Mountain()
    start = (5, 5)
    env = Mountain()

    # create 15-20 random valid goals
    goals = generate_random_goals(env, n=20, start=start, seed=42)
    print(f"Generated {len(goals)} goals")
    for i, goal in enumerate(goals, 1):
        x, y = int(goal[0]), int(goal[1])
        # simple validation inline (avoid dependency on validate_point placement)
        if hasattr(env, "cols") and hasattr(env, "rows"):
            in_bounds = 0 <= x < env.cols and 0 <= y < env.rows
        else:
            in_bounds = 0 <= x < env.x_range and 0 <= y < env.y_range
        if not in_bounds or (getattr(env, "obstacles", None) and (x, y) in env.obstacles):
            print(f"Skipping invalid goal {goal}")
            continue
        print(f"Goal {i}: {goal}")

        # plan with A* for each goal (replace with desired planner)
        planner = search_factory("a_star", start=start, goal=goal, env=env)
        cost, path, _ = planner.plan()
        print(f"  A* cost={cost}, path_len={len(path)}")
        # show the animation / popup for this solved problem
        # use planner.run() if you want the planner to re-run planning + animation,
        # or call the plot helper directly to display the computed path:
        try:
            planner.plot.animation(path, f"A* goal {i}", cost)
        except Exception:
            # fallback to run (some planner implementations expect run())
            planner.run()

    # if you still want to run a single planner + animation, pick one goal:
    # planner = search_factory("a_star", start=start, goal=goals[0], env=env)
    # planner.run()
    # Works
    planner = search_factory("a_star", start=start, goal=goal, env=env)

def sample_search():
    # build environment
    start = (18, 8)
    goal = (37, 18)
    env = Map(51, 31)
    env = Map(51, 31)
    obs_rect = [
        [14, 12, 8, 2],
        [18, 22, 8, 3],
        [26, 7, 2, 12],
        [32, 14, 10, 2]
    ]
    obs_circ = [
        [7, 12, 3],
        [46, 20, 2],
        [15, 5, 2],
        [37, 7, 3],
        [37, 23, 3]
    ]
    env.update(obs_rect=obs_rect, obs_circ=obs_circ)

    # creat planner
    planner = search_factory("rrt", start=start, goal=goal, env=env)
    planner = search_factory("rrt_connect", start=start, goal=goal, env=env)
    planner = search_factory("rrt_star", start=start, goal=goal, env=env)
    planner = search_factory("informed_rrt", start=start, goal=goal, env=env)

    # animation
    planner.run()

def evolutionary_search():
    start = (5, 5)
    goal = (45, 25)
    env = Grid(51, 31)
    obstacles = env.obstacles
    for i in range(10, 21):
        obstacles.add((i, 15))
    for i in range(15):
        obstacles.add((20, i))
    for i in range(15, 30):
        obstacles.add((30, i))
    for i in range(16):
        obstacles.add((40, i))
        env.update(obstacles)

    planner = search_factory("aco", start=start, goal=goal, env=env)
    planner = search_factory("pso", start=start, goal=goal, env=env)
    planner.run()

def run_and_measure(search_factory, planner_name, start, goal, env):
    """Run one planner and return metrics dict."""
    t0 = time.perf_counter()
    planner = search_factory(planner_name, start=start, goal=goal, env=env)
    try:
        res = planner.plan()
    except Exception as e:
        return {
            "planner": planner_name,
            "goal": goal,
            "error": str(e),
            "cost": None,
            "path_len": 0,
            "path_distance_m": None,
            "planning_time_s": None,
            "expanded": None
        }
    t1 = time.perf_counter()

    # unpack results (support different planner return shapes)
    cost = None
    path = None
    expand = None
    if isinstance(res, tuple):
        if len(res) >= 1: cost = res[0]
        if len(res) >= 2: path = res[1]
        if len(res) >= 3: expand = res[2]
    else:
        cost = res

    path_len = len(path) if path else 0
    path_distance_m = env.path_distance_meters(path) if (path and hasattr(env, "path_distance_meters")) else None
    expanded = None
    if expand is not None:
        try:
            expanded = len(expand)
        except Exception:
            expanded = None
    else:
        # fallback: some planners may expose attribute (best-effort)
        expanded = getattr(planner, "expanded", None)

    return {
        "planner": planner_name,
        "goal": goal,
        "error": None,
        "cost": cost,
        "path_len": path_len,
        "path_distance_m": path_distance_m,
        "planning_time_s": (t1 - t0),
        "expanded": expanded
    }

def generate_start_goal_pairs(env, n_pairs=20, seed=None):
    """Return up to n_pairs of (start, goal) pairs inside env bounds and not in obstacles."""
    if seed is not None:
        random.seed(seed)
    pairs = []
    tries = 0
    max_tries = n_pairs * 50
    while len(pairs) < n_pairs and tries < max_tries:
        tries += 1
        if hasattr(env, "cols") and hasattr(env, "rows"):
            s = (random.randrange(0, env.cols), random.randrange(0, env.rows))
            g = (random.randrange(0, env.cols), random.randrange(0, env.rows))
        else:
            s = (random.randrange(0, env.x_range), random.randrange(0, env.y_range))
            g = (random.randrange(0, env.x_range), random.randrange(0, env.y_range))
        if s == g: 
            continue
        if getattr(env, "obstacles", None) and (s in env.obstacles or g in env.obstacles):
            continue
        if (s, g) in pairs:
            continue
        pairs.append((s, g))
    return pairs

def compare_planners_on_pairs(planner_names, env, pairs, out_csv=None):
    """
    Run each planner in planner_names for every (start,goal) pair and save CSV.
    Each row contains planner, start_x/start_y, goal_x/goal_y, metrics...
    """
    sf = SearchFactory()
    results = []
    for start, goal in pairs:
        for pname in planner_names:
            print(f"Running {pname} -> start {start} goal {goal}")
            row = run_and_measure(sf, pname, start, goal, env)
            row_flat = {
                "timestamp": datetime.utcnow().isoformat(),
                "planner": row["planner"],
                "start_x": int(start[0]),
                "start_y": int(start[1]),
                "goal_x": int(goal[0]),
                "goal_y": int(goal[1]),
                "error": row["error"],
                "cost": row["cost"],
                "path_len": row["path_len"],
                "path_distance_m": row["path_distance_m"],
                "planning_time_s": row["planning_time_s"],
                "expanded": row["expanded"]
            }
            results.append(row_flat)

    if out_csv is None:
        os.makedirs("results", exist_ok=True)
        out_csv = f"results/compare_pairs_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}.csv"

    # ensure parent directory exists
    dirpath = os.path.dirname(out_csv)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)

    keys = ["timestamp","planner","start_x","start_y","goal_x","goal_y","error","cost","path_len","path_distance_m","planning_time_s","expanded"]
    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in results:
            writer.writerow(r)

    print(f"Saved results to {out_csv}")
    return out_csv, results


if __name__ == '__main__':
    '''
    path searcher constructor
    '''
    search_factory = SearchFactory()

    graph_search_with_random_goals()
    # sample_search()
    # evolutionary_search():

def validate_point(env, pt):
    """Return (True, None) if pt is valid for env, otherwise (False, reason)."""
    x, y = int(pt[0]), int(pt[1])
    # Mountain uses cols/rows; other Env types use x_range/y_range
    if hasattr(env, "cols") and hasattr(env, "rows"):
        in_bounds = 0 <= x < env.cols and 0 <= y < env.rows
    else:
        in_bounds = 0 <= x < env.x_range and 0 <= y < env.y_range
    if not in_bounds:
        return False, "out_of_bounds"
    if getattr(env, "obstacles", None) and (x, y) in env.obstacles:
        return False, "blocked_by_obstacle"
    return True, None
