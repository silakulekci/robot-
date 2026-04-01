import tkinter as tk
import random
import math
from collections import deque

class RobotVacuumExplorer:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Vacuum Explorer - SLAM Simulation")
        
        # --- Settings ---
        self.grid_size = 20
        self.rows, self.cols = 25, 25
        self.lidar_range = 6  # Ray casting distance
        self.running = False
        
        # --- Map Data ---
        # 0: Unknown (Gray), 1: Wall (Black), 2: Free (White), 3: Visited (Green)
        self.real_map = [[2 for _ in range(self.cols)] for _ in range(self.rows)]
        self.robot_map = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        
        self.start_pos = [self.rows // 2, self.cols // 2]
        self.robot_pos = list(self.start_pos)
        self.path_to_home = []
        self.phase = "IDLE" # "IDLE", "EXPLORING" or "RETURNING"

        # --- UI Build (Once) ---
        self.setup_ui()
        
        # Initial State
        self.reset_data()

    def setup_ui(self):
        bg_color = "#f4f4f9"
        self.root.configure(bg=bg_color)
        
        # Header
        self.label = tk.Label(self.root, text="Robot Ready | Press Start", 
                              font=("Segoe UI", 14, "bold"), bg=bg_color, fg="#333")
        self.label.pack(pady=10)

        self.canvas_frame = tk.Frame(self.root, bg=bg_color)
        self.canvas_frame.pack(padx=20)

        # Left Panel: Real World
        f1 = tk.Frame(self.canvas_frame, bg=bg_color)
        f1.pack(side=tk.LEFT, padx=15)
        tk.Label(f1, text="REAL WORLD (GROUND TRUTH)", font=("Arial", 9, "bold"), bg=bg_color).pack()
        self.c_real = tk.Canvas(f1, width=self.cols*self.grid_size, height=self.rows*self.grid_size, 
                                bg="white", highlightthickness=1, highlightbackground="#ccc")
        self.c_real.pack()

        # Right Panel: Robot's Map
        f2 = tk.Frame(self.canvas_frame, bg=bg_color)
        f2.pack(side=tk.LEFT, padx=15)
        tk.Label(f2, text="ROBOT'S BUILT MAP (SLAM)", font=("Arial", 9, "bold"), bg=bg_color).pack()
        self.c_robot = tk.Canvas(f2, width=self.cols*self.grid_size, height=self.rows*self.grid_size, 
                                 bg="white", highlightthickness=1, highlightbackground="#ccc")
        self.c_robot.pack()

        # Controls
        btn_f = tk.Frame(self.root, bg=bg_color)
        btn_f.pack(pady=20)
        
        self.btn_start = tk.Button(btn_f, text="START / PAUSE", command=self.toggle, 
                                   width=18, height=2, bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.btn_start.pack(side=tk.LEFT, padx=10)
        
        self.btn_reset = tk.Button(btn_f, text="NEW MAP / RESET", command=self.reset_data, 
                                   width=18, height=2, bg="#555", fg="white", font=("Arial", 10, "bold"))
        self.btn_reset.pack(side=tk.LEFT, padx=10)

    def reset_data(self):
        """Resets the simulation data without recreating UI widgets"""
        self.running = False
        self.phase = "EXPLORING"
        self.steps = 0
        self.real_map = [[2 for _ in range(self.cols)] for _ in range(self.rows)]
        self.robot_map = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        self.path_to_home = []
        self.robot_pos = list(self.start_pos)
        
        # Random Walls
        for _ in range(75):
            r, c = random.randint(0, self.rows-1), random.randint(0, self.cols-1)
            if [r, c] != self.start_pos:
                self.real_map[r][c] = 1
        
        self.update_lidar()
        self.draw_all()
        self.label.config(text="New Map Ready | Phase: EXPLORING", fg="#333")

    def update_lidar(self):
        """Ray Casting: Simulation of LiDAR sensors"""
        r0, c0 = self.robot_pos
        self.robot_map[r0][c0] = 3 # Mark as Visited
        
        for angle in range(0, 360, 10):
            rad = math.radians(angle)
            for dist in range(1, self.lidar_range + 1):
                nr = int(r0 + dist * math.sin(rad))
                nc = int(c0 + dist * math.cos(rad))
                
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.real_map[nr][nc] == 1:
                        self.robot_map[nr][nc] = 1 # Detected Wall
                        break
                    else:
                        if self.robot_map[nr][nc] == 0:
                            self.robot_map[nr][nc] = 2 # Detected Free Space
                else:
                    break

    def get_frontier(self):
        """Frontier-Based Exploration: Find nearest boundary between Known and Unknown"""
        queue = deque([tuple(self.robot_pos)])
        visited_bfs = {tuple(self.robot_pos)}
        
        while queue:
            curr_r, curr_c = queue.popleft()
            for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
                nr, nc = curr_r + dr, curr_c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.robot_map[nr][nc] == 0: # Unknown found!
                        return (curr_r, curr_c)
                    if self.robot_map[nr][nc] in [2, 3] and (nr, nc) not in visited_bfs:
                        visited_bfs.add((nr, nc))
                        queue.append((nr, nc))
        return None

    def find_path_to(self, target):
        """BFS Shortest Path Algorithm"""
        queue = deque([(tuple(self.robot_pos), [])])
        visited_bfs = {tuple(self.robot_pos)}
        
        while queue:
            (curr_r, curr_c), path = queue.popleft()
            if (curr_r, curr_c) == target:
                return path
            
            for dr, dc in [(0,1),(0,-1),(1,0),(-1,0)]:
                nr, nc = curr_r + dr, curr_c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    # Robot only moves through cells it knows are FREE or VISITED
                    if self.robot_map[nr][nc] in [2, 3] and (nr, nc) not in visited_bfs:
                        visited_bfs.add((nr, nc))
                        queue.append(((nr, nc), path + [(nr, nc)]))
        return []

    def move(self):
        if not self.running: return

        if self.phase == "EXPLORING":
            target = self.get_frontier()
            if target:
                path = self.find_path_to(target)
                if path:
                    self.robot_pos = list(path[0])
                else: self.phase = "RETURNING"
            else:
                self.phase = "RETURNING"
                self.path_to_home = self.find_path_to(tuple(self.start_pos))

        elif self.phase == "RETURNING":
            if self.path_to_home:
                self.robot_pos = list(self.path_to_home.pop(0))
            else:
                self.running = False
                self.label.config(text="MISSION SUCCESS: Robot at Home!", fg="#2ecc71")

        self.update_lidar()
        self.draw_all()
        self.update_stats()
        self.root.after(80, self.move) # Speed control

    def update_stats(self):
        known = sum(row.count(2) + row.count(1) + row.count(3) for row in self.robot_map)
        perc = int((known / (self.rows * self.cols)) * 100)
        if self.running:
            self.label.config(text=f"Phase: {self.phase} | Coverage: {perc}%")

    def draw_all(self):
        self.draw_map(self.c_real, self.real_map, True)
        self.draw_map(self.c_robot, self.robot_map, False)

    def draw_map(self, canvas, data, is_real):
        canvas.delete("all")
        # Colors: 0: Unknown(Gray), 1: Wall(Dark), 2: Free(White), 3: Visited(Light Green)
        colors = {0: "#bdc3c7", 1: "#2c3e50", 2: "#ffffff", 3: "#d1f2eb"}
        
        for r in range(self.rows):
            for c in range(self.cols):
                x1, y1 = c*self.grid_size, r*self.grid_size
                color = colors.get(data[r][c], "#ffffff")
                canvas.create_rectangle(x1, y1, x1+self.grid_size, y1+self.grid_size, 
                                        fill=color, outline="#ecf0f1")
        
        # Draw Robot (Red Circle)
        rx, ry = self.robot_pos[1]*self.grid_size, self.robot_pos[0]*self.grid_size
        canvas.create_oval(rx+4, ry+4, rx+self.grid_size-4, ry+self.grid_size-4, 
                           fill="#e74c3c", outline="#c0392b", width=1)
        
        # Mark Home Point
        hx, hy = self.start_pos[1]*self.grid_size, self.start_pos[0]*self.grid_size
        canvas.create_text(hx+10, hy+10, text="H", fill="#2980b9", font=("Arial", 10, "bold"))

    def toggle(self):
        self.running = not self.running
        if self.running: self.move()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotVacuumExplorer(root)
    root.mainloop()
