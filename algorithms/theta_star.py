# algorithms/theta_star.py
import heapq
from collections import defaultdict
import math

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def line_of_sight(grid, s0, s1):
    x0, y0 = s0
    x1, y1 = s1
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    err = dx - dy
    while (x0, y0) != (x1, y1):
        if grid[y0][x0]:
            return False
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return True

def theta_star(grid, start, goal):
    width, height = len(grid[0]), len(grid)
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), start))
    came_from = {start: start}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current != came_from[current]:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                if grid[neighbor[1]][neighbor[0]] == 1:
                    continue
                if line_of_sight(grid, came_from[current], neighbor):
                    tentative_g = g_score[came_from[current]] + math.dist(came_from[current], neighbor)
                    if tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        came_from[neighbor] = came_from[current]
                        f = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f, neighbor))
                else:
                    tentative_g = g_score[current] + 1
                    if tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        came_from[neighbor] = current
                        f = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f, neighbor))
    return []
