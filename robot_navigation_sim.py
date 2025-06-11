import matplotlib.pyplot as plt
import numpy as np
import heapq

# Grid dimensions
GRID_WIDTH = 20
GRID_HEIGHT = 20

# Define start and goal positions
start = (0, 0)
goal = (19, 19)

# Generate a random grid with obstacles
def generate_grid():
    grid = np.zeros((GRID_HEIGHT, GRID_WIDTH))
    np.random.seed(42)
    for _ in range(100):
        x, y = np.random.randint(0, GRID_WIDTH), np.random.randint(0, GRID_HEIGHT)
        if (x, y) != start and (x, y) != goal:
            grid[y][x] = 1  # Obstacle
    return grid

# A* Algorithm
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    open_list = []
    closed_set = set()
    start_node = Node(start)
    goal_node = Node(goal)
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        (x, y) = current_node.position
        neighbors = [(x+1,y), (x-1,y), (x,y+1), (x,y-1)]

        for next_pos in neighbors:
            (nx, ny) = next_pos
            if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT:
                if grid[ny][nx] == 1 or next_pos in closed_set:
                    continue
                neighbor = Node(next_pos, current_node)
                neighbor.g = current_node.g + 1
                neighbor.h = heuristic(next_pos, goal)
                neighbor.f = neighbor.g + neighbor.h

                if any(next_pos == node.position and neighbor.g >= node.g for node in open_list):
                    continue
                heapq.heappush(open_list, neighbor)
    return None

# Visualization
def draw_grid(grid, path):
    fig, ax = plt.subplots()
    ax.imshow(grid, cmap='Greys')
    if path:
        path_x, path_y = zip(*path)
        ax.plot(path_x, path_y, marker='o', color='blue')
    ax.plot(start[0], start[1], 'go')  # Start
    ax.plot(goal[0], goal[1], 'ro')   # Goal
    plt.title("A* Path Planning")
    plt.gca().invert_yaxis()
    plt.show()

if __name__ == "__main__":
    grid = generate_grid()
    path = astar(grid, start, goal)
    draw_grid(grid, path)
    if path:
        print(f"Path found with {len(path)} steps.")
    else:
        print("No path found.")
