# experiment.py
import random
import matplotlib.pyplot as plt
from grid_env import GridEnvironment
from config import GRID_SIZE, NUM_RUNS
from utils.metrics import measure_performance
from algorithms.astar import astar
from algorithms.dstar_lite import dstar_lite
from algorithms.theta_star import theta_star


random.seed(42)

ALGORITHMS = {
    'A*': astar,
    'D*-Lite': dstar_lite,
    'Theta*': theta_star
}

def run_experiment():
    static_results = {}
    dynamic_results = {}

    for name, algo in ALGORITHMS.items():
        s_time, s_mem, s_len = 0, 0, 0
        d_time, d_mem, d_len = 0, 0, 0

        for _ in range(NUM_RUNS):
            env = GridEnvironment(*GRID_SIZE, obstacle_ratio=0.2)
            start, goal = (0, 0), (GRID_SIZE[0]-1, GRID_SIZE[1]-1)
            measured_algo = measure_performance(algo)

            # Static environment
            path, exec_time, mem_kb = measured_algo(env.grid, start, goal)
            if path:
                s_time += exec_time
                s_mem += mem_kb
                s_len += len(path)

            # Dynamic environment
            env.add_dynamic_obstacles(30)
            path2, exec_time2, mem_kb2 = measured_algo(env.grid, start, goal)
            if path2:
                d_time += exec_time2
                d_mem += mem_kb2
                d_len += len(path2)

        static_results[name] = {
            'avg_time': s_time / NUM_RUNS,
            'avg_memory': s_mem / NUM_RUNS,
            'avg_path_length': s_len / NUM_RUNS
        }

        dynamic_results[name] = {
            'avg_time': d_time / NUM_RUNS,
            'avg_memory': d_mem / NUM_RUNS,
            'avg_path_length': d_len / NUM_RUNS
        }

    # Print Results
    print("\n--- Static Environment ---")
    for algo, res in static_results.items():
        print(f"{algo}: Time={res['avg_time']:.4f}s, Memory={res['avg_memory']:.2f}KB, Path Length={res['avg_path_length']:.2f}")

    print("\n--- Dynamic Environment ---")
    for algo, res in dynamic_results.items():
        print(f"{algo}: Time={res['avg_time']:.4f}s, Memory={res['avg_memory']:.2f}KB, Path Length={res['avg_path_length']:.2f}")

    # Visualization
    labels = list(ALGORITHMS.keys())
    x = range(len(labels))

    def plot_metric(static, dynamic, metric, ylabel):
        s_vals = [static[k][metric] for k in labels]
        d_vals = [dynamic[k][metric] for k in labels]

        plt.figure()
        plt.bar([i - 0.2 for i in x], s_vals, width=0.4, label='Static')
        plt.bar([i + 0.2 for i in x], d_vals, width=0.4, label='Dynamic')
        plt.xticks(x, labels)
        plt.ylabel(ylabel)
        plt.title(f'{metric.replace("_", " ").title()} Comparison')
        plt.legend()
        plt.tight_layout()
        plt.show()

    plot_metric(static_results, dynamic_results, 'avg_time', 'Time (s)')
    plot_metric(static_results, dynamic_results, 'avg_memory', 'Memory (KB)')
    plot_metric(static_results, dynamic_results, 'avg_path_length', 'Path Length')

if __name__ == '__main__':
    run_experiment()
