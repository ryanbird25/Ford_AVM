

import heapq
import numpy as np
from occupancy_grid import og_coordinate, OccupancyGrid


#need to replace with location from mounted apriltag
CAR_WIDTH = 14 #cm
CAR_LENGTH = 33.5 #cm

TAG_X_OFFSET_FROM_CENTER = 0 #cm
TAG_Y_OFFSET_FROM_CENTER = 0 #cm

def heuristic(start:og_coordinate, goal:og_coordinate, grid_resolution:float):
        #adding a heading bonus if nearby so that it approaches parking lot straight
        heading_penalty = 0
        manhattan_distance = abs(start.x - goal.x) + abs(start.y - goal.y)


        if manhattan_distance < 150*grid_resolution: # if within a meter #TODO MATH NOT RIGHT
            heading_penalty = abs(start.heading - goal.heading) * 10
            
        return manhattan_distance + heading_penalty

def is_valid_position(grid, car_grd_length, car_grd_width, car_position: og_coordinate):
    # Check if the next position is within bounds and not occupied
    #grid: 2d numpy array
    #car_grd_length: length of the car in grid cells
    #car_grd_width: width of the car in grid cells
    #car_position: og_coordinate object with x, y, heading
    
    rows, cols = grid.shape
    car_x, car_y, heading = car_position.x, car_position.y, car_position.heading
    # grid_x, grid_y = car_position.x, car_position.y
    # car_x_center = car_x + TAG_X_OFFSET_FROM_CENTER*np.cos(np.radians(heading))
    # car_y_center = car_y + TAG_Y_OFFSET_FROM_CENTER*np.sin(np.radians(heading)) 

    # Convert car dimensions to grid cells
    dx = np.arange(car_x - car_grd_width// 2, np.round(car_x+car_grd_width / 2) + 1)
    dy = np.arange(-car_y - car_grd_length // 2, np.round(car_y+car_grd_length / 2) + 1)
    dx, dy = np.meshgrid(dx, dy)

    # Flatten the relative positions
    dx = dx.flatten()
    dy = dy.flatten()

    # Apply rotation based on the car's heading
    rotated_dx = (dx * np.cos(np.radians(heading)) - dy * np.sin(np.radians(heading))).astype(int)
    rotated_dy = (dx * np.sin(np.radians(heading)) + dy * np.cos(np.radians(heading))).astype(int)

    # Calculate the absolute grid positions
    grid_x = np.round(rotated_dx).astype(int)
    grid_y = np.round(rotated_dy).astype(int)

    # Check if the car's occupied cells are within bounds and not occupied
    if np.any(grid_x < 0) or np.any(grid_x >= rows) or np.any(grid_y < 0) or np.any(grid_y >= cols):
        return False
    
    if np.any(grid[grid_x, grid_y] == 1):
        return False
    else: 
        return True
    
    
def plan(occupancy_grid: OccupancyGrid, start:og_coordinate, goal:og_coordinate):
    grid = occupancy_grid.get_grid()
    rows, cols = grid.shape
    open_set = []
    #fscore: cost of current spot + hueristic
    #gscore: cost from start to current spot, tie breaker to prefer simple paths

    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)   
            return remove_collinear(path[::-1])  # Reverse path

        neighbors = []
        for dx, dy in [(-1,-1), (0,-1),(1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]:
            nh = np.arctan2(dy, dx)
            nx, ny = current.x + dx, current.y + dy

            new_og_coordinate = og_coordinate(nh, nx, ny)
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] == 0:
                if is_valid_position(grid, CAR_LENGTH/occupancy_grid.get_resolution(), CAR_WIDTH/occupancy_grid.get_resolution(), new_og_coordinate):
                    neighbors.append((nx, ny))

        for neighbor in neighbors:
            tentative_g = current_cost + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current

    return None  # No path found


    
def remove_collinear(plan:np.ndarray[og_coordinate]):
        # Remove collinear points from the path, leave 3 at each corner
    if len(plan) < 3:
        return plan

    filtered_plan = [plan[0]]
    for i in range(1, len(plan) - 1):
        og_p_1 = plan[i - 1]#occupancy grid points
        og_p_2 = plan[i]
        og_p_3 = plan[i + 1]

        next_to_turn = False

        if (i + 2) < len(plan):
            og_p_4 = plan[i + 2]

            if og_p_4.heading == og_p_3.heading:
                next_to_turn = True

        if (i -2) > 0:
            og_p_0 = plan[i - 2]

            if og_p_0.heading == og_p_1.heading:
                next_to_turn = True

    
        # Check if p2 is collinear with p1 and p3 or special case to leave on either side of corner
        if (og_p_1.heading == og_p_2.heading and og_p_2.heading == og_p_3.heading) or next_to_turn:
            filtered_plan.append(og_p_2)

    filtered_plan.append(plan[-1])
    return filtered_plan