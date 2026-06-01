import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation


# Example environment
# 0 = free cell
# 1 = obstacle
grid = [
 [0,0,0,1,0,0,0,0],
 [1,1,0,1,0,1,1,0],
 [0,0,0,0,0,0,1,0],
 [0,1,1,1,1,0,1,0],
 [0,0,0,0,1,0,0,0],
 [1,1,1,0,1,1,1,0],
 [0,0,0,0,0,0,0,0],
 [0,1,1,1,1,1,0,0]
]
start = (0,0)
goal = (7,7)

def get_neighbors(current):
    neigh_list = []
    row, col = current
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    for dr,dc in directions:
        new_row = row+dr
        new_col = col+dc
        if new_row<len(grid) and new_row >=0 and new_col <len(grid[0]) and new_col>=0 and grid[new_row][new_col]==0:
            neigh_list.append((new_row,new_col))
    return neigh_list

def heuristic(current, goal):
    row, col = current
    row_g, col_g = goal
    return abs(row-row_g)+abs(col-col_g)

def reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path

def a_star(start, goal):
    open_list = [start]
    closed_list = []
    came_from = {}
    g_score = {start: 0}
    while open_list:
        best_node = None
        best_f = float('inf')

        for node in open_list:
            f = g_score[node] + heuristic(node, goal)
            if f < best_f:
                best_f = f
                best_node = node

        if best_node == goal:
            return reconstruct_path(came_from, best_node)
        open_list.remove(best_node)
        closed_list.append(best_node)
        neighbors = get_neighbors(best_node)
        for neighbor in neighbors:
            if neighbor in closed_list:
                continue
            else:
                tentative_g = g_score[best_node] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = best_node
                    g_score[neighbor] = tentative_g

                    if neighbor not in open_list:
                        open_list.append(neighbor)

def print_grid_with_path(grid, path, start, goal):
    for row in range(len(grid)):
        for col in range(len(grid[0])):

            current = (row,col)
            if current == start:
                print('S', end=" ")
            elif current == goal:
                print('G', end=" ")
            elif current in path:
                print('*', end=" ")
            elif grid[row][col] == 1:
                print('#', end=" ")
            else:
                print('.', end=" ")
        print()


def animate_robot(grid, path, start, goal):
    grid_vis = np.zeros((len(grid), len(grid[0])))

    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] == 1:
                grid_vis[r][c] = 1

    sr, sc = start
    gr, gc = goal
    grid_vis[sr][sc] = 3
    grid_vis[gr][gc] = 4

    cmap = ListedColormap([
        'white',   # free
        'black',   # obstacle
        'blue',    # path
        'green',   # start
        'red',     # goal
        'orange'   # robot
    ])

    fig, ax = plt.subplots()
    ax.set_title("A* Path Planning - Robot Movement")
    ax.set_xticks(range(len(grid[0])))
    ax.set_yticks(range(len(grid)))
    ax.grid(True)

    legend = [
        mpatches.Patch(color='green', label='Start'),
        mpatches.Patch(color='red', label='Goal'),
        mpatches.Patch(color='blue', label='Path'),
        mpatches.Patch(color='black', label='Obstacle'),
        mpatches.Patch(color='white', label='Free space'),
        mpatches.Patch(color='orange', label='Robot')
    ]
    ax.legend(handles=legend, bbox_to_anchor=(1.05, 1), loc='upper left')

    image = ax.imshow(grid_vis, cmap=cmap, vmin=0, vmax=5)


    def update(frame):
        temp = grid_vis.copy()

        for i in range(frame):
            r, c = path[i]
            if (r, c) != start and (r, c) != goal:
                temp[r][c] = 2

        r, c = path[frame]
        temp[r][c] = 5

        image.set_data(temp)
        return [image]

    ani = FuncAnimation(
        fig,
        update,
        frames=len(path),
        interval=700,
        repeat=False
    )

    plt.show()


path = a_star(start, goal)

print("Path:")
print(path)

print("\nGrid with path:")
print_grid_with_path(grid, path, start, goal)

#visualize(grid, path, start, goal)
animate_robot(grid, path, start, goal)
