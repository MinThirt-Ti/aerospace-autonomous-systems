import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import heapq
from collections import deque
import random
import time

GRID_SIZE = 500
OBSTACLE_VAL = np.inf
SWAMP_VAL = 10
FREE_VAL = 1

START_NODE = (10, 10)
GOAL_NODE = (490, 490)

fig, ax = plt.subplots(figsize=(10, 10))
plt.subplots_adjust(bottom=0.15)

def get_neighbors(node, current_grid):
    dirs = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    result = []
    for dx, dy in dirs:
        nx, ny = node[0] + dx, node[1] + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and current_grid[nx, ny] != OBSTACLE_VAL:
            cost = current_grid[nx, ny] * (1.414 if dx != 0 and dy != 0 else 1.0)
            result.append(((nx, ny), cost))
    return result

def reconstruct_path(came_from, current, start_node):
    path = []
    while current in came_from and current != start_node:
        path.append(current)
        current = came_from[current]
    path.append(start_node)
    path.reverse()
    return path

def run_search(algo_type, current_grid):
    frontier = []
    came_from = {START_NODE: None}
    cost_so_far = {START_NODE: 0}
    
    if algo_type == 'BFS':
        q = deque([START_NODE])
    elif algo_type == 'DFS':
        stack = [START_NODE]
    else:
        heapq.heappush(frontier, (0, START_NODE))

    while frontier or (algo_type in ['BFS', 'DFS'] and (q if algo_type == 'BFS' else stack)):
        if algo_type == 'BFS':
            current = q.popleft()
        elif algo_type == 'DFS':
            current = stack.pop()
        else:
            _, current = heapq.heappop(frontier)

        if current == GOAL_NODE:
            break

        for next_node, move_cost in get_neighbors(current, current_grid):
            if algo_type in ['BFS', 'DFS']:
                if next_node not in came_from:
                    came_from[next_node] = current
                    if algo_type == 'BFS':
                        q.append(next_node)
                    else:
                        stack.append(next_node)
            else:
                new_cost = cost_so_far[current] + move_cost
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost
                    if algo_type == 'A*':
                        priority += np.sqrt((next_node[0]-GOAL_NODE[0])**2 + (next_node[1]-GOAL_NODE[1])**2)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current
                    
    return reconstruct_path(came_from, GOAL_NODE, START_NODE) if GOAL_NODE in came_from else []

def run_apf_euclidean(current_obstacles):
    current = np.array(START_NODE, dtype=float)
    g = np.array(GOAL_NODE, dtype=float)
    path = [tuple(current)]
    
    for _ in range(15000):
        dist_to_goal = np.linalg.norm(current - g)
        if dist_to_goal < 5.0:
            break
            
        att = -1.0 * (current - g) / dist_to_goal
        
        rep = np.zeros(2)
        for ox, oy, orad in current_obstacles:
            obs_pos = np.array([ox, oy])
            dist = np.linalg.norm(current - obs_pos)
            
            dist = max(dist, 0.1)
            
            buffer = orad + 8.0
            if dist < buffer:
                rep += 200.0 * (1/dist - 1/buffer) * (1/dist**2) * (current - obs_pos)/dist
                
        force = att + rep
        norm = np.linalg.norm(force)
        
        if norm < 0.05:
            force = np.array([random.uniform(-1, 1), random.uniform(-1, 1)])
            force = force / np.linalg.norm(force) * 5.0
        elif norm > 0:
            force = (force / norm) * 2.0
            
        current += force
        
        current[0] = np.clip(current[0], 0.0, float(GRID_SIZE - 1))
        current[1] = np.clip(current[1], 0.0, float(GRID_SIZE - 1))
        
        path.append(tuple(current))
    return path

def randomize_and_draw(event=None):
    print("Calculating paths... UI will freeze.")
    start_time = time.time()
    ax.clear()
    
    grid = np.full((GRID_SIZE, GRID_SIZE), FREE_VAL, dtype=float)
    obstacles = []

    def place_circle(x, y, r, val):
        xmin = max(0, x - r)
        xmax = min(GRID_SIZE, x + r + 1)
        ymin = max(0, y - r)
        ymax = min(GRID_SIZE, y + r + 1)
        for i in range(xmin, xmax):
            for j in range(ymin, ymax):
                if (i - x)**2 + (j - y)**2 <= r**2:
                    grid[i, j] = val

    for _ in range(50):
        ox, oy = random.randint(20, 480), random.randint(20, 480)
        orad = random.randint(5, 20)
        place_circle(ox, oy, orad, OBSTACLE_VAL)
        obstacles.append((ox, oy, orad))

    for _ in range(15):
        sx, sy = random.randint(20, 480), random.randint(20, 480)
        srad = random.randint(10, 30)
        place_circle(sx, sy, srad, SWAMP_VAL)

    grid[START_NODE[0], START_NODE[1]] = FREE_VAL
    grid[GOAL_NODE[0], GOAL_NODE[1]] = FREE_VAL

    path_bfs = run_search('BFS', grid)
    path_dfs = run_search('DFS', grid)
    path_dij = run_search('Dijkstra', grid)
    path_astar = run_search('A*', grid)
    path_apf = run_apf_euclidean(obstacles)

    # Layer 1: Background, Swamps, Obstacles
    cmap = plt.cm.gray
    ax.imshow(grid.T, cmap=cmap, origin='lower', vmax=SWAMP_VAL+5, zorder=1)

    # Layer 2: Checkpoints
    ax.plot(START_NODE[0], START_NODE[1], 'go', markersize=10, zorder=2, label='Start')
    ax.plot(GOAL_NODE[0], GOAL_NODE[1], 'r*', markersize=15, zorder=2, label='Goal')

    # Layer 3: Paths
    def plot_path(p, color_fmt, label):
        if p:
            px, py = zip(*p)
            ax.plot(np.array(px), np.array(py), color_fmt, linewidth=1.5, alpha=0.9, zorder=3, label=label)

    plot_path(path_bfs, 'c-', 'BFS')
    plot_path(path_dfs, 'm-', 'DFS')
    plot_path(path_dij, 'b-', 'Dijkstra')
    plot_path(path_astar, 'y-', 'A*')
    plot_path(path_apf, 'C1--', 'APF')

    ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
    ax.set_title("Pathfinding Map (500x500)")
    fig.canvas.draw_idle()
    print(f"Calculations finished in {time.time() - start_time:.2f} seconds.")

ax_button = plt.axes([0.4, 0.05, 0.2, 0.075])
btn = Button(ax_button, 'Randomize Map')
btn.on_clicked(randomize_and_draw)

randomize_and_draw()
plt.show()