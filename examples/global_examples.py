"""
@file: global_examples.py
@breif: global planner application examples
@author: Yang Haodong, Wu Maojia
@update: 2024.11.22
"""
import sys, os

import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from matplotlib import cbook, cm
from matplotlib.colors import LightSource

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid, Map, SearchFactory, Grid3D, Mountain, Node

def plot_mountain(start, goals, env):
    fig = plt.figure("planning")
    ax = fig.subplots(subplot_kw=dict(projection='3d'))
    dem = cbook.get_sample_data('jacksboro_fault_dem.npz')
    z = dem['elevation']
    nrows, ncols = z.shape
    x = np.linspace(dem['xmin'], dem['xmax'], ncols)
    y = np.linspace(dem['ymin'], dem['ymax'], nrows)
    x, y = np.meshgrid(x, y)

    region = np.s_[5:50, 5:50]
    x, y, z = x[region], y[region], z[region]

    # Set up plot
    ls = LightSource(270, 45)
    # To use a custom hillshading mode, override the built-in shading and pass
    # in the rgb colors of the shaded surface calculated from "shade".
    rgb = ls.shade(z, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
    surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, facecolors=rgb,
                       linewidth=0, antialiased=False, shade=False)

    ax.set_xlabel("Longitude (°)")
    ax.set_ylabel("Latitude (°)")
    ax.set_zlabel("Elevation (m)")

    start_coords = env.index_to_coords(int(start.x), int(start.y))
    ax.plot3D(*start_coords, 'rs', markersize=8, markeredgecolor='black', markeredgewidth=2, label='Start', zorder=100)
    for i, goal in enumerate(goals):
        goal_coords = env.index_to_coords(int(goal.x), int(goal.y))
        # Only label the first goal to avoid duplicate legend entries
        label = 'Goal' if i == 0 else None
        ax.plot3D(*goal_coords, 'mo', markersize=5, markeredgecolor='black', markeredgewidth=2, label=label, zorder=100)

    ax.legend(loc='upper left')
    plt.show()

def plot_mountain_2d(start, goals, env):
    fig, ax = plt.subplots(figsize=(12, 8))

    # Create 2D contour plot
    dem = cbook.get_sample_data('jacksboro_fault_dem.npz')
    z = dem['elevation']
    nrows, ncols = z.shape
    x = np.linspace(dem['xmin'], dem['xmax'], ncols)
    y = np.linspace(dem['ymin'], dem['ymax'], nrows)
    x, y = np.meshgrid(x, y)

    region = np.s_[5:50, 5:50]
    x, y, z = x[region], y[region], z[region]

    # 2D contour plot
    contour = ax.contourf(x, y, z, levels=20, cmap='terrain')
    plt.colorbar(contour, label='Elevation (m)')

    # Add start and goals
    start_coords = env.index_to_coords(int(start.x), int(start.y))
    ax.plot(start_coords[0], start_coords[1], 'rs', markersize=8,
            markeredgecolor='black', markeredgewidth=2, label='Start')

    for i, goal in enumerate(goals):
        goal_coords = env.index_to_coords(int(goal.x), int(goal.y))
        # Only label the first goal to avoid duplicate legend entries
        label = 'Goal' if i == 0 else None
        ax.plot(goal_coords[0], goal_coords[1], 'mo', markersize=10, markeredgecolor='black', markeredgewidth=2, label=label)
        ax.text(goal_coords[0], goal_coords[1] + 0.001, f'G{i+1}', fontsize=12, fontweight='bold', ha='center', color='black')

    def on_click(event):
        if event.inaxes != ax:
            return
        i, j = env.coords_to_index(event.xdata, event.ydata)
        print(f'Node(({i}, {j}))')
    fig.canvas.mpl_connect('button_press_event', on_click)

    ax.legend(loc='upper left')
    ax.set_xlabel("Longitude (°)")
    ax.set_ylabel("Latitude (°)")
    plt.show()


def run_bench(start, goals):
    algs = ["a_star", "dijkstra", "d_star_lite", "lpa_star", "gbfs",
            "jps", "theta_star", "rrt_star", "lazy_theta_star",
            "s_theta_star", "informed_rrt", "rrt", "rrt_connect"]

    # algs = ["informed_rrt"]

    for elevation_weight in [1.1, 1.25, 1.50]:
        df = pd.DataFrame(columns=algs)
        env = Mountain(elevation_weight)

        # Process all goals for this elevation weight
        all_rows = []
        for goal_idx, goal in enumerate(goals):
            row_data = {}
            print(f"Processing goal {goal_idx + 1}/{len(goals)}: ({goal.x}, {goal.y})")
            for alg in algs:
                try:
                    planner = search_factory(alg, start=(start.x, start.y), goal=(goal.x, goal.y), env=env)
                    cost = planner.plan_path()
                    row_data[alg] = cost
                    print(f"  {alg}: {cost}")
                except Exception as e:
                    print(f"  {alg}: FAILED - {e}")
                    exit(1)
            all_rows.append(row_data)

        # Create DataFrame with all goals for this elevation weight
        df = pd.DataFrame(all_rows)

        # Generate filename based on actual elevation weight
        weight_str = str(elevation_weight).replace('.', '_')
        filename = f"results_elevation_weight_{weight_str}.csv"
        df.to_csv(filename, index=False)
        print(f"\nResults saved to {filename}")


def graph_search():
    # build environment
    start = Node((6, 33))
    goals = [Node((21, 29)),
             Node((30, 10)),
             Node((20, 10)),
             Node((34, 12)),
             Node((29, 15)),
             Node((36, 19)),
             Node((42, 16)),
             Node((38, 26)),
             Node((43, 1)),
             Node((34, 7)),
             Node((34, 4)),
             Node((41, 3)),
             Node((40, 7)),
             Node((21, 39)),
             Node((37, 39)),
             Node((14, 2)),
             Node((5, 15)),
             Node((24, 16)),
             Node((26, 5)),
             Node((30, 26)),]




    run_bench(start, goals)

    # env = Mountain()
    # plot_mountain(start, goals, env)
    # plot_mountain_2d(start, goals, env)


    # Works
    # planner = search_factory("dijkstra", start=start, goal=goal, env=env)
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

    # planner.run()

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

if __name__ == '__main__':
    '''
    path searcher constructor
    '''
    search_factory = SearchFactory()

    graph_search()
    # sample_search()
    # evolutionary_search():
