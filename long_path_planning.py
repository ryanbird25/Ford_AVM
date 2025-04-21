

import heapq
import numpy as np
from occupancy_grid import og_coordinate, OccupancyGrid
import rospy

AXIS_OF_ROTATION_FROM_CENTER_Y = -.09

def heuristic(start:og_coordinate, goal:og_coordinate, grid_resolution:float):
        #adding a heading bonus if nearby so that it approaches parking lot straight
        # heading_penalty = 0
        # manhattan_distance = abs(start.x - goal.x) + abs(start.y - goal.y)


    manh = abs(start.x - goal.x) + abs(start.y - goal.y)
    heading_penalty = 0
    # If within 1.5 meters, add heading alignment bonus
    # if manh * grid_resolution < 1.5:
    #     # wrap-aware heading difference
    #     diff = (start.heading - goal.heading + np.pi) % (2 * np.pi) - np.pi
    #     heading_penalty = abs(diff) * 10

    return manh + heading_penalty

def is_valid_position(occupancy_grid: OccupancyGrid, car_position: og_coordinate):
    # Check if the next position is within bounds and not occupied
    #car position is handed as the position of the front wheels
    grid = occupancy_grid.get_grid()
    rows, cols = grid.shape
    
    car_center_x, car_center_y = OccupancyGrid.world_to_grid(car_position.x - AXIS_OF_ROTATION_FROM_CENTER * np.sin(car_position.heading), car_position.y + AXIS_OF_ROTATION_FROM_CENTER * np.cos(car_position.heading))
    grid_x, grid_y = occupancy_grid.get_car_footprint([car_center_x*occupancy_grid.resolution, car_center_y*occupancy_grid.resolution, car_position.heading])

    if np.any(grid_x < 0) or np.any(grid_x >= cols) or np.any(grid_y < 0) or np.any(grid_y >= rows):
        return False
    
    if np.any(grid[grid_y, grid_x] == 1):
        return False
    else: 
        return True

def plan(occupancy_grid: OccupancyGrid, start: og_coordinate, goal: og_coordinate):
    grid = occupancy_grid.get_grid()
    rows, cols = grid.shape

    # 1) Validate start/goal up front
    #rospy.loginfo(f"[A*] start={start}, valid? {is_valid_position(occupancy_grid,  start)}")
    #rospy.loginfo(f"[A*]  goal={goal}, valid? {is_valid_position(occupancy_grid,  goal)}")

    open_set = [] #heap of (f_score, g_score, og_coordinate)
    g_score = {start: 0}
    came_from = {}
    visited = set()

    # push start
    heapq.heappush(open_set, (heuristic(start, goal, occupancy_grid.resolution), 0, start))

    while open_set:
        # 2) report frontier sizes
        #rospy.loginfo(f"[A*] frontier={len(open_set)} visited={len(visited)}")

        f_curr, g_curr, current = heapq.heappop(open_set)
        #rospy.loginfo(f"[A*] pop: {current} g={g_curr:.2f} f={f_curr:.2f}")

        # 3) skip if already expanded
        if current in visited:
            #rospy.loginfo(f"[A*] skip visited {current}")
            continue
        visited.add(current)

        # 4) tolerant goal check
        if current.x == goal.x and current.y == goal.y:#remove heading constraints for gogal
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path = list(reversed(path))
            ##rospy.loginfo(f"[A*] goal reached, path len={len(path)}")
            return remove_collinear(path)

        # 5) generate neighbors
        for dx, dy in [(-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)]:
            nx, ny = current.x + dx, current.y + dy
            # quantize heading
            raw_h = np.arctan2(dy, dx)
            nh = round(raw_h, 1)
            neighbor = og_coordinate(nx, ny, nh)

            ##rospy.loginfo(f"[A*]  consider {neighbor}")

            # bounds & grid occupancy
            if not (0 <= nx < cols and 0 <= ny < rows):
                ##rospy.loginfo(f"[A*]   out of bounds")
                continue

            if grid[ny, nx] == 1:
                ##rospy.loginfo(f"[A*]   cell occupied")
                continue

            # footprint collision
            valid = is_valid_position(
                occupancy_grid,
                neighbor
            )
            ##rospy.loginfo(f"[A*]   footprint valid? {valid}")
            if not valid:
                continue

            # 6) compute tentative g
            turn_cost = 0 if current.heading == neighbor.heading else 1
            tentative_g = g_curr + 1 + turn_cost

            # 7) only improve if better
            prev_g = g_score.get(neighbor, float('inf'))
            if tentative_g < prev_g:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal, occupancy_grid.resolution)
                came_from[neighbor] = current
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                #rospy.loginfo(f"[A*]   push {neighbor} g={tentative_g:.2f} f={f_score:.2f}")

    # no path
    #rospy.logwarn("[A*] no path found")
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

        # Check if p2 is part of a turn (heading changes)
        # is_turn = og_p_1.heading != og_p_2.heading or og_p_2.heading != og_p_3.heading

        # Keep the point if it's not collinear or if it's part of a turn
        if not is_collinear:# or is_turn:
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
    heading_arrived_at_goal = 0
    for i in range(1, len(path)):
        # Current and previous points
        prev_point = path[i - 1]
        curr_point = path[i]

        # Calculate the distance change (Euclidean distance)
        distance_change = ((curr_point.x - prev_point.x) ** 2 + (curr_point.y - prev_point.y) ** 2) ** 0.5

        # Calculate the heading change
        heading_change = curr_point.heading - prev_point.heading
        heading_arrived_at_goal = curr_point.heading
        # Normalize the heading change to the range [-π, π]
        heading_change = (heading_change + np.pi) % (2 * np.pi) - np.pi

        # Append the tuple (heading_change, distance_change) to the driveable path
        driveable_path.append((heading_change, distance_change))

    heading_change = -np.pi -  heading_arrived_at_goal
    final_heading_change = (heading_change + np.pi) % (2 * np.pi) - np.pi
  

        # Normalize the heading change to the range [-π, π]
       

    if abs(heading_change) <= .1
        driveable_path.append((heading_change, 0))
    return driveable_path