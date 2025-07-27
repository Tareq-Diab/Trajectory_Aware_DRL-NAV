
import numpy as np 
import heapq
def grid_to_world(grid_x, grid_y, grid_size=200, world_size=20):
    """
    Convert grid coordinates to world coordinates.
    
    :param grid_x: x-coordinate in grid space
    :param grid_y: y-coordinate in grid space
    :param grid_size: size of the grid (default 200)
    :param world_size: size of the world in meters (default 20, ranging from -10 to 10)
    :return: tuple of (x, y) world coordinates
    """
    # Calculate the size of each grid cell in world units
    cell_size = world_size / grid_size
    
    # Convert grid coordinates back to world coordinates
    x = grid_x * cell_size - world_size / 2 + cell_size / 2
    y = grid_y * cell_size - world_size / 2 + cell_size / 2
    return x, y

def world_to_grid(x, y, grid_size=200, world_size=20):
    """
    Convert world coordinates to grid coordinates.
    
    :param x: x-coordinate in world space
    :param y: y-coordinate in world space
    :param grid_size: size of the grid (default 200)
    :param world_size: size of the world in meters (default 20, ranging from -10 to 10)
    :return: tuple of (grid_x, grid_y)
    """
    grid_x = int((x + world_size/2) / (world_size/grid_size))
    grid_y = int((y + world_size/2) / (world_size/grid_size))
    return grid_x, grid_y


def create_obstacle_grid(obstacles, grid_size=200, world_size=20):
    """
    Create a grid representation of obstacles.
    
    :param obstacles: list of (x, y) tuples representing obstacle coordinates in world space
    :param grid_size: size of the grid (default 200)
    :param world_size: size of the world in meters (default 20, ranging from -10 to 10)
    :return: numpy array representing the grid
    """
    grid = np.zeros((grid_size, grid_size), dtype=int)
    
    for obs in obstacles:
        grid_x, grid_y = world_to_grid(obs[0], obs[1], grid_size, world_size)
        if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
            grid[grid_y-5:grid_y+5, grid_x-5:grid_x+5] = 1  # Mark as obstacle
    
    return grid


def heuristic(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# A* pathfinding algorithm
def a_star(grid, start, goal):
    grid_size = len(grid)
    
    # Priority queue for open cells (stores cost and position)
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    # Dictionaries to track cost and path
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    # Allow diagonal movements in addition to up, down, left, right
    neighbor_directions = [(-1, 0), (1, 0), (0, -1), (0, 1), 
                           (-1, -1), (1, 1), (-1, 1), (1, -1)]  # Add diagonals
    
    while open_set:
        # Get the node in open_set with the lowest f_score
        _, current = heapq.heappop(open_set)
        
        # Check if we've reached the goal
        if current == goal:
            # Reconstruct the path by backtracking
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path
        
        # Explore neighbors (including diagonals)
        for dx, dy in neighbor_directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < grid_size and 0 <= neighbor[1] < grid_size:
                # Skip obstacles
                if grid[neighbor[1]][neighbor[0]] == 1:
                    continue
                
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + np.sqrt(dx**2 + dy**2)  # Diagonal distance is sqrt(2)
                
                # If a shorter path to neighbor is found
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None  # Return None if no path is found