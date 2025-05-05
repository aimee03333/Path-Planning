# utils/visualizer.py
import matplotlib.pyplot as plt

def draw_grid(grid, path=None, start=None, goal=None):
    grid_data = [[1 if cell else 0 for cell in row] for row in grid]
    fig, ax = plt.subplots()
    ax.imshow(grid_data, cmap='Greys')

    if path:
        for (x, y) in path:
            ax.plot(x, y, 'go', markersize=4)
    if start:
        ax.plot(start[0], start[1], 'bs')
    if goal:
        ax.plot(goal[0], goal[1], 'rs')
    plt.show()
