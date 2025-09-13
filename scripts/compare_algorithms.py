import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # project root

from examples.global_examples import (
    Mountain, generate_start_goal_pairs, compare_planners_on_pairs
)

# experiment parameters
num_pairs = 20
env = Mountain(elevation_weight=0.1, elevation_scale=2.0)   # tune as needed

pairs = generate_start_goal_pairs(env, n_pairs=num_pairs, seed=123)
planners = ["a_star", "dijkstra", "gbfs", "jps", "rrt_connect", "rrt", "gbfs"]

out_csv, results = compare_planners_on_pairs(planners, env, pairs, out_csv="results/compare_astar_dijkstra_pairs.csv")
print("Saved:", out_csv)