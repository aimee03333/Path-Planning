import sys
sys.path.append("/mnt/data")

import csv
import os
import matplotlib.pyplot as plt
from collections import defaultdict, deque, Counter
from grid_env import GridEnvironment
from config import GRID_SIZE, NUM_RUNS
from utils.metrics import measure_performance
from algorithms.astar import astar
from algorithms.dstar_lite import dstar_lite
from algorithms.theta_star import theta_star


ALGORITHMS = {
    'A*': astar,
    'D*-Lite': dstar_lite,
    'Theta*': theta_star
}


def has_accessible_neighbors(grid, x, y):
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid) and grid[ny][nx] == 0:
            return True
    return False


def is_reachable(grid, start, goal):
    if grid[start[1]][start[0]] == 1 or grid[goal[1]][goal[0]] == 1:
        return False

    visited = set()
    queue = deque([start])
    width, height = len(grid[0]), len(grid)

    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] == 0 and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append((nx, ny))

    return False


current_dir = os.path.dirname(__file__)
all_results = []


successful_runs = 0
attempt = 0
MAX_ATTEMPTS = 100
while successful_runs < NUM_RUNS and attempt < MAX_ATTEMPTS:
    attempt += 1
    results = []
    env = GridEnvironment(*GRID_SIZE, obstacle_ratio=0.2)
    start, goal = (0, 0), (GRID_SIZE[0]-1, GRID_SIZE[1]-1)

    env.grid[start[1]][start[0]] = 0
    env.grid[goal[1]][goal[0]] = 0
    if not has_accessible_neighbors(env.grid, *start):
        print(f"[Attempt {attempt}] Skipped: Start has no neighbors.")
        continue
    if not is_reachable(env.grid, start, goal):
        print(f"[Attempt {attempt}] Skipped: Start and goal not connected.")
        continue

    for step in range(6):
        if step > 0:
            env.add_dynamic_obstacles(5)
            if not is_reachable(env.grid, start, goal):
                continue

        for algo_name, algo_func in ALGORITHMS.items():
            measured_algo = measure_performance(algo_func)
            path, exec_time, mem_kb = measured_algo(env.grid, start, goal)
            path_len = len(path) if path else 0
            results.append({
                'run': successful_runs + 1,
                'step': step,
                'algorithm': algo_name,
                'time_sec': round(exec_time, 5),
                'memory_kb': round(mem_kb, 2),
                'path_length': path_len
            })
            all_results.append(results[-1])

    
    metrics = ['time_sec', 'memory_kb', 'path_length']
    metric_labels = {'time_sec': 'Time (s)', 'memory_kb': 'Memory (KB)', 'path_length': 'Path Length'}
    style = {
        'A*': {'linestyle': '--', 'linewidth': 2.2},
        'D*-Lite': {'linestyle': '-.', 'linewidth': 2.2},
        'Theta*': {'linestyle': '-', 'linewidth': 2.2}
    }

    for metric in metrics:
        plt.figure()
        for algo in ALGORITHMS.keys():
            avg_values = []
            for step in range(6):
                vals = [r[metric] for r in results if r['algorithm'] == algo and r['step'] == step and r['path_length'] > 0]
                avg = sum(vals) / len(vals) if vals else 0
                avg_values.append(avg)
            plt.plot(range(6), avg_values, marker='o', label=algo, **style.get(algo, {}))

        plt.xlabel("# of Dynamic Obstacle Additions")
        plt.ylabel(metric_labels[metric])
        plt.title(f"Run {successful_runs} - {metric_labels[metric]} vs. Environment Changes")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plot_path = os.path.join(current_dir, f"run{successful_runs}_{metric}.png")
        plt.savefig(plot_path)
        plt.close()

    successful_runs += 1


csv_path = os.path.join(current_dir, "path_planning_experiment.csv")
with open(csv_path, mode='w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=all_results[0].keys())
    writer.writeheader()
    writer.writerows(all_results)


metrics = ['time_sec', 'memory_kb', 'path_length']
metric_labels = {'time_sec': 'Time (s)', 'memory_kb': 'Memory (KB)', 'path_length': 'Path Length'}
style = {
    'A*': {'linestyle': '--', 'linewidth': 2.2},
    'D*-Lite': {'linestyle': '-.', 'linewidth': 2.2},
    'Theta*': {'linestyle': '-', 'linewidth': 2.2}
}

for metric in metrics:
    plt.figure()
    for algo in ALGORITHMS.keys():
        avg_values = []
        for step in range(6):
            vals = [r[metric] for r in all_results if r['algorithm'] == algo and r['step'] == step and r['path_length'] > 0]
            avg = sum(vals) / len(vals) if vals else 0
            avg_values.append(avg)
        plt.plot(range(6), avg_values, marker='o', label=algo, **style.get(algo, {}))

    plt.xlabel("# of Dynamic Obstacle Additions")
    plt.ylabel(metric_labels[metric])
    plt.title(f"Average {metric_labels[metric]} over {NUM_RUNS} Runs")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plot_path = os.path.join(current_dir, f"average_{metric}.png")
    plt.savefig(plot_path)
    plt.close()

print(f"Successful runs: {successful_runs} / {NUM_RUNS}, tried: {attempt} times")
