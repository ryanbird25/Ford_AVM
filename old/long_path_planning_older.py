

import heapq
import numpy as np
from occupancy_grid import og_coordinate, OccupancyGrid

import rospy

#need to replace with location from mounted apriltag
CAR_WIDTH = 14 #cm
CAR_LENGTH = 33.5 #cm

TAG_X_OFFSET_FROM_CENTER = 0 #cm
TAG_Y_OFFSET_FROM_CENTER = 0 #cm

def heuristic(start:og_coordinate, goal:og_coordinate, grid_resolution:float):
        #adding a heading bonus if nearby so that it approaches parking lot straight
        heading_penalty = 0
        manhattan_distance = abs(start.x - goal.x) + abs(start.y - goal.y)


        # if manhattan_distance < 1.50*grid_resolution: # if within a meter #TODO MATH NOT RIGHT
        #     heading_penalty = abs(start.heading - goal.heading) * 10
            
        return manhattan_distance + heading_penalty

def is_valid_position(occupancy_grid: OccupancyGrid, car_grd_length, car_grd_width, car_position: og_coordinate):
    # Check if the next position is within bounds and not occupied
    grid = occupancy_grid.get_grid()
    rows, cols = grid.shape
    
    grid_x, grid_y = occupancy_grid.get_car_footprint([car_position.x*occupancy_grid.resolution, car_position.y*occupancy_grid.resolution, car_position.heading])
    # print("grid_x:", grid_x)
    # print("grid_y:", grid_y)
    # print("rows:", rows)
    # print("cols:", cols)
    # print("")
    # Check if the car's occupied cells are within bounds and not occupied
    if np.any(grid_x < 0) or np.any(grid_x >= cols) or np.any(grid_y < 0) or np.any(grid_y >= rows):
        return False
    
    if np.any(grid[grid_y, grid_x] == 1):
        return False
    else: 
        return True
    
    
def plan(occupancy_grid: OccupancyGrid, start:og_coordinate, goal:og_coordinate):
    grid = occupancy_grid.get_grid()
    rows, cols = grid.shape
    open_set = []
    #fscore: cost of current spot + hueristic
    #gscore: cost from start to current spot, tie breaker to prefer simple paths
    print("start", start)
    print("goal", goal)
    heapq.heappush(open_set, (0 + heuristic(start, goal, occupancy_grid.resolution), 0, start))

    came_from = {}

    found_set = set()
    found_set.add(start)
    g_score = {start: 0}

    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current in found_set:
            continue
        else:
            found_set.add(current)
        #print(len( open_set))
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)   
            #return remove_collinear(path[::-1])  # Reverse path
            return path[::-1]  # Reverse path

        neighbors = []
        for dx, dy in [(-1,-1), (0,-1),(1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]:
            nh = np.arctan2(dy, dx)
            nx, ny = current.x + dx, current.y + dy

            new_og_coordinate = og_coordinate(nx, ny, nh)
        
            if new_og_coordinate not in found_set and 0 <= nx < cols and 0 <= ny < rows and grid[ny, nx] == 0 :
                if is_valid_position(occupancy_grid, CAR_LENGTH/occupancy_grid.get_resolution(), CAR_WIDTH/occupancy_grid.get_resolution(), new_og_coordinate):
                    found_set.add(new_og_coordinate)
                    neighbors.append(new_og_coordinate)
                    rospy.loginfo("found valid neighbor")
                    #print("new_og_coordinate", new_og_coordinate)

        for neighbor in neighbors:
            #print("checking valid neighbor")
            turn_penalty = 0
            if current.heading != neighbor.heading:
                turn_penalty = 1
            tentative_g = current_cost + 1 + turn_penalty
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal, occupancy_grid.resolution)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current

    return None  # No path found


    
def remove_collinear(plan:np.ndarray):
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

def remove_collinear(plan:np.ndarray):
        # Remove collinear points from the path, leave 3 at each corner
    if len(plan) < 3:
        return plan  # If the plan has fewer than 3 points, return it as is.

    filtered_plan = [plan[0]]  # Always keep the first point.

    for i in range(1, len(plan) - 1):
        og_p_1 = plan[i - 1]  # Previous point
        og_p_2 = plan[i]      # Current point
        og_p_3 = plan[i + 1]  # Next point

        # Check if p2 is collinear with p1 and p3 using the cross-product
        is_collinear = (og_p_2.x - og_p_1.x) * (og_p_3.y - og_p_2.y) == (og_p_2.y - og_p_1.y) * (og_p_3.x - og_p_2.x)

        # Check if p2 is part of a turn (heading changes)
        is_turn = og_p_1.heading != og_p_2.heading or og_p_2.heading != og_p_3.heading

        # Keep the point if it's not collinear or if it's part of a turn
        if not is_collinear or is_turn:
            filtered_plan.append(og_p_2)

        # Ensure the point before and after a turn is preserved
        # if is_turn:
        #     if og_p_1 not in filtered_plan:
        #         filtered_plan.append(og_p_1)  # Add the point before the turn
        #     if og_p_3 not in filtered_plan:
        #         filtered_plan.append(og_p_3)  # Add the point after the turn

    filtered_plan.append(plan[-1])  # Always keep the last point.
    return filtered_plan

def path_to_driveable(path: list, occupancy_grid: OccupancyGrid):
    if len(path) < 2:
        return []  # If the path has fewer than 2 points, return an empty list.

    driveable_path = []

    for i in range(1, len(path)):
        # Current and previous points
        prev_point = path[i - 1]
        curr_point = path[i]

        # Calculate the distance change (Euclidean distance)
        distance_change = ((curr_point.x - prev_point.x) ** 2 + (curr_point.y - prev_point.y) ** 2) ** 0.5

        # Calculate the heading change
        heading_change = curr_point.heading - prev_point.heading

        # Normalize the heading change to the range [-π, π]
        heading_change = (heading_change + np.pi) % (2 * np.pi) - np.pi

        # Append the tuple (heading_change, distance_change) to the driveable path
        driveable_path.append((heading_change, distance_change))

    return driveable_path