

import heapq
import numpy as np
from occupancy_grid import og_coordinate, OccupancyGrid
import rospy
import math

AXIS_OF_ROTATION_FROM_CENTER_Y = .09

def heuristic(start:og_coordinate, goal:og_coordinate):
    manh = abs(start.x - goal.x) + abs(start.y - goal.y)
    return manh

def is_valid_position(occupancy_grid: OccupancyGrid, car_position: og_coordinate):
    # Check if the next position is within bounds and not occupied
    #car position is handed as the position of the front wheels
    grid = occupancy_grid.get_grid()
    rows, cols = grid.shape
    
    world_x = car_position.x * occupancy_grid.resolution
    world_y = car_position.y * occupancy_grid.resolution
    heading = car_position.heading

    # 2) Shift that front-wheel location back to the car’s center
    #    using the same offset that remove_car_footprint uses.
    cx = world_x + AXIS_OF_ROTATION_FROM_CENTER_Y * np.sin(heading)
    cy = world_y - AXIS_OF_ROTATION_FROM_CENTER_Y * np.cos(heading)
    center_world = np.array([cx, cy, heading])
    
    grid_x, grid_y = occupancy_grid.get_car_footprint_from_center(center_world)

    if np.any(grid_x < 0) or np.any(grid_x >= cols) or np.any(grid_y < 0) or np.any(grid_y >= rows):
        return False
    
    if np.any(grid[grid_y, grid_x] == 1):
        return False
    else: 
        return True

def plan(occupancy_grid: OccupancyGrid, start: og_coordinate, goal: og_coordinate):
    #Start and goal cords are the front wheel positions, this is done because the vehicle can rotate around the front wheels
    grid = occupancy_grid.get_grid()
    rows, cols = grid.shape

    open_set = [] #heap of (f_score, g_score, og_coordinate)
    g_score = {start: 0}
    came_from = {}
    visited = set()

    # push start
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))

    while open_set:
   
        f_curr, g_curr, current = heapq.heappop(open_set)
        #rospy.loginfo(f"[A*] pop: {current} g={g_curr:.2f} f={f_curr:.2f}")

        # skip if already expanded
        if current in visited:
            #rospy.loginfo(f"[A*] skip visited {current}")
            continue
        visited.add(current)

        if current.x == goal.x and current.y == goal.y:
         # reconstruct path (always includes the start)
            path = []
            node = current
            while True:
                path.append(node)
                # stop once we've literally hit our start
                if node.x == start.x and node.y == start.y and node.heading == start.heading:
                    break
                node = came_from[node]
            path.reverse()
            rospy.loginfo("UNFILTERED PATH: " + str(path))
            return path


        # generate neighbors
        for dx, dy in [(-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)]:
            nx, ny = current.x + dx, current.y + dy
            # quantize heading
            raw_h = np.arctan2(dy, dx) - np.pi/2
            nh = round(raw_h, 1)
            neighbor = og_coordinate(nx, ny, nh)

            # bounds & grid occupancy
            if not (0 <= nx < cols and 0 <= ny < rows):
                ##rospy.loginfo(f"[A*]   out of bounds")
                continue

            if grid[ny, nx] == 1:
                continue

            # footprint collision
            valid = is_valid_position(
                occupancy_grid,
                neighbor
            )
            if not valid:
                continue

            #  compute tentative g
            turn_cost = 0 if current.heading == neighbor.heading else 2
            tentative_g = g_curr + 1 + turn_cost

            # only improve if better
            prev_g = g_score.get(neighbor, float('inf'))
            if tentative_g < prev_g:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))

    return None



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

        # Keep the point if it's not collinear or if it's part of a turn
        if not is_collinear:# or is_turn:
            filtered_plan.append(og_p_2)

    filtered_plan.append(plan[-1])  # Always keep the last point.
    rospy.loginfo("\n\nfiltered path" + str(filtered_plan))
    return filtered_plan


def path_to_driveable(path: list, occupancy_grid: OccupancyGrid):
    #This function turns the occupancy grid coordinates into a series of heading and distance changes
    if len(path) < 2:
        return []  # If the path has fewer than 2 points, return an empty list.

    driveable_path = []
    heading_arrived_at_goal = 0
    for i in range(1, len(path)):
        # Current and previous points
        prev_point = path[i - 1]
        curr_point = path[i]


        dx = (curr_point.x - prev_point.x) * occupancy_grid.resolution
        dy = (curr_point.y - prev_point.y) * occupancy_grid.resolution#

        seg_theta = math.atan2(dy, dx) - np.pi/2

        # Calculate the distance change (Euclidean distance)
        distance_change = (((dx) ** 2 + (dy) ** 2) ** 0.5)
        
        heading_change = seg_theta - prev_point.heading

        # Append the tuple (heading_change, distance_change) to the driveable path
        rospy.loginfo("\n" + str(i) + "\n")
        rospy.loginfo("\ndistance change: " + str(distance_change) + "heading difference, from current" + str(heading_change))
        driveable_path.append((heading_change, distance_change))

    return driveable_path