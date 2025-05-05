# algorithms/dstar_lite.py
import heapq
from collections import defaultdict

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(pos, grid):
    x, y = pos
    neighbors = []
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid):
            if grid[ny][nx] == 0:
                neighbors.append((nx, ny))
    return neighbors

class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.g = defaultdict(lambda: float('inf'))
        self.rhs = defaultdict(lambda: float('inf'))
        self.rhs[goal] = 0
        self.U = []
        self.km = 0
        self.last = start
        self.insert(goal, self.calculate_key(goal))

    def calculate_key(self, s):
        return (min(self.g[s], self.rhs[s]) + heuristic(self.start, s) + self.km, min(self.g[s], self.rhs[s]))

    def insert(self, s, key):
        heapq.heappush(self.U, (key, s))

    def update_vertex(self, u):
        if u != self.goal:
            neighbors = get_neighbors(u, self.grid)
            if neighbors:
                self.rhs[u] = min(self.g[s] + 1 for s in neighbors)
            else:
                self.rhs[u] = float('inf')

        if u in [item[1] for item in self.U]:
            self.U = [(k, s) for (k, s) in self.U if s != u]
            heapq.heapify(self.U)
        if self.g[u] != self.rhs[u]:
            self.insert(u, self.calculate_key(u))

    def compute_shortest_path(self):
        while self.U and (self.U[0][0] < self.calculate_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)
            if k_old < k_new:
                self.insert(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in get_neighbors(u, self.grid):
                    self.update_vertex(s)
            else:
                g_old = self.g[u]
                self.g[u] = float('inf')
                for s in get_neighbors(u, self.grid) + [u]:
                    self.update_vertex(s)

    def extract_path(self):
        path = []
        current = self.start
        if self.g[current] == float('inf'):
            return []
        while current != self.goal:
            path.append(current)
            current = min(get_neighbors(current, self.grid), key=lambda s: self.g[s] + 1, default=current)
            if self.g[current] == float('inf'):
                return []
        path.append(self.goal)
        return path

def dstar_lite(grid, start, goal):
    planner = DStarLite(grid, start, goal)
    planner.compute_shortest_path()
    return planner.extract_path()