# grid_env.py
import random
class GridEnvironment:
    def __init__(self, width, height, obstacle_ratio=0.2):
        self.width = width
        self.height = height
        self.obstacle_ratio = obstacle_ratio
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self.generate_obstacles()

    def generate_obstacles(self):
        for y in range(self.height):
            for x in range(self.width):
                if random.random() < self.obstacle_ratio:
                    self.grid[y][x] = 1

    def is_free(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0

    def add_dynamic_obstacles(self, count):
        added = 0
        while added < count:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            if self.grid[y][x] == 0:
                self.grid[y][x] = 1
                added += 1
