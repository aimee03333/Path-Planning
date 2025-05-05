# GUI-enhanced simulation with step-by-step traversal and autonomous animation
import tkinter as tk
from tkinter import ttk
from grid_env import GridEnvironment
from utils.metrics import measure_performance
from algorithms import astar, dstar_lite, theta_star
import time


CELL_SIZE = 20
GRID_SIZE = 30
ALGO_MAP = {
    'A*': astar.astar,
    'D*-Lite': dstar_lite.dstar_lite,
    'Theta*': theta_star.theta_star
}

class PathPlannerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Path Planning Simulator")
        self.env = GridEnvironment(GRID_SIZE, GRID_SIZE, obstacle_ratio=0.2)
        self.start = (0, 0)
        self.goal = (GRID_SIZE - 1, GRID_SIZE - 1)
        self.algo_choice = tk.StringVar(value='A*')
        self.path = []

        self.canvas = tk.Canvas(self.root, width=GRID_SIZE*CELL_SIZE, height=GRID_SIZE*CELL_SIZE)
        self.canvas.pack()

        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill='x')
        ttk.Label(control_frame, text="Algorithm:").pack(side='left')
        ttk.OptionMenu(control_frame, self.algo_choice, 'A*', *ALGO_MAP.keys()).pack(side='left')
        ttk.Button(control_frame, text="Run", command=self.run_algorithm).pack(side='left')
        ttk.Button(control_frame, text="Step", command=self.step_through_path).pack(side='left')
        ttk.Button(control_frame, text="Reset", command=self.reset).pack(side='left')
        ttk.Button(control_frame, text="Add Obstacles", command=self.add_obstacles).pack(side='left')

        self.agent = None
        self.step_index = 0
        self.draw_grid()

    def draw_grid(self):
        self.canvas.delete("all")
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                color = 'white'
                if self.env.grid[y][x] == 1:
                    color = 'black'
                self.canvas.create_rectangle(x*CELL_SIZE, y*CELL_SIZE, (x+1)*CELL_SIZE, (y+1)*CELL_SIZE, fill=color)

        self.canvas.create_rectangle(self.start[0]*CELL_SIZE, self.start[1]*CELL_SIZE,
                                     (self.start[0]+1)*CELL_SIZE, (self.start[1]+1)*CELL_SIZE, fill='blue')
        self.canvas.create_rectangle(self.goal[0]*CELL_SIZE, self.goal[1]*CELL_SIZE,
                                     (self.goal[0]+1)*CELL_SIZE, (self.goal[1]+1)*CELL_SIZE, fill='red')

    def run_algorithm(self):
        algo = ALGO_MAP[self.algo_choice.get()]
        measured_algo = measure_performance(algo)
        (path, time_taken, memory_kb) = measured_algo(self.env.grid, self.start, self.goal)
        
        self.path = path
        self.step_index = 0
        self.draw_grid()

        for (x, y) in self.path:
            self.canvas.create_oval(x*CELL_SIZE+6, y*CELL_SIZE+6, x*CELL_SIZE+14, y*CELL_SIZE+14, fill='green')
        
        print(f"[{self.algo_choice.get()}] Time: {time_taken:.4f}s, Memory: {memory_kb:.2f}KB, Path length: {len(path)}")
        self.root.after(500, self.animate_path)

    def animate_path(self):
        if not self.path or self.step_index >= len(self.path):
            return
        if self.step_index == 0:
            self.agent = self.canvas.create_oval(self.start[0]*CELL_SIZE+4, self.start[1]*CELL_SIZE+4,
                                                 self.start[0]*CELL_SIZE+16, self.start[1]*CELL_SIZE+16, fill='orange')
        x, y = self.path[self.step_index]
        self.canvas.coords(self.agent, x*CELL_SIZE+4, y*CELL_SIZE+4, x*CELL_SIZE+16, y*CELL_SIZE+16)
        self.step_index += 1
        self.root.after(100, self.animate_path)

    def step_through_path(self):
        if not self.path:
            return
        if self.step_index == 0:
            self.agent = self.canvas.create_oval(self.start[0]*CELL_SIZE+4, self.start[1]*CELL_SIZE+4,
                                                 self.start[0]*CELL_SIZE+16, self.start[1]*CELL_SIZE+16, fill='orange')
        if self.step_index < len(self.path):
            x, y = self.path[self.step_index]
            self.canvas.coords(self.agent, x*CELL_SIZE+4, y*CELL_SIZE+4, x*CELL_SIZE+16, y*CELL_SIZE+16)
            self.step_index += 1

    def reset(self):
        self.env = GridEnvironment(GRID_SIZE, GRID_SIZE, obstacle_ratio=0.2)
        self.path = []
        self.step_index = 0
        self.agent = None
        self.draw_grid()

    def add_obstacles(self):
        self.env.add_dynamic_obstacles(20)
        self.draw_grid()

if __name__ == '__main__':
    root = tk.Tk()
    app = PathPlannerGUI(root)
    root.mainloop()
