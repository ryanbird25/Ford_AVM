import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from occupancy_grid import OccupancyGrid, og_coordinate

AXIS_OF_ROTATION_FROM_CENTER_Y = .09
TAG_Y_OFFSET_FROM_CENTER = -.07 #m

import rospy
PARKING_STARTX =  .9#m
PARKING_STARTY = .3 #m
PARKING_ENDX = 1.43 #m
PARKING_ENDY =  .8#m


def create_sim(og: OccupancyGrid, goal:og_coordinate, path, out_file="sim.mp4", fps=10):
    #og: filled in occupancy grid
    #goal: occupancy_
    frames = []

    world_center_x = goal.x*og.resolution# + (AXIS_OF_ROTATION_FROM_CENTER_Y * np.sin(goal.heading))
    world_center_y = goal.y*og.resolution #- (AXIS_OF_ROTATION_FROM_CENTER_Y * np.cos(goal.heading))
    center_goal_pos = np.array([world_center_x, world_center_y, goal.heading])
        
    goal_x_mask, goal_y_mask = og.get_car_footprint_from_center(center_goal_pos, res_increase=.25)
    
    metadata = dict(title='Path Simulation', artist='matplotlib')
    writer = FFMpegWriter(fps=fps, metadata=metadata)

    fig, ax = plt.subplots(figsize=(6,6))
    path_index = 0
    total = len(path)
    with writer.saving(fig, out_file, dpi=100):
        for grid_position in path:
            rospy.loginfo("\nSIMULATING POSITON: " + str(path_index) + "/" + str(total))
            #coordinate of the front wheel
            ax.clear()
            ax.imshow(og.get_grid(), cmap='Greys', origin='lower')

            car_x, car_y, car_heading = grid_position.x, grid_position.y, grid_position.heading
            # Display the grid
            
            # Highlight specific coordinates
           
            for og_cord in path[path_index:]:
                x,y = og_cord.x, og_cord.y
                ax.scatter(x, y, color='blue', s=50)  # Note: (x, y) -> (row, col)
            # rospy.loginfo("past show path")


            #THIS IS NOT RIGHT BECAUSE POSITIONS ARE GIVEN AS FRONT WHEEL LOCATIONS
           
            center_pos_x = car_x*og.resolution + (AXIS_OF_ROTATION_FROM_CENTER_Y * np.sin(car_heading))
            center_pos_y = car_y*og.resolution - AXIS_OF_ROTATION_FROM_CENTER_Y * np.cos(car_heading)
            gridx, gridy = og.get_car_footprint_from_tag_pos([center_pos_x, center_pos_y, car_heading], res_increase=.25)

            #PLOT CURRENT
            for i in range(len(gridy)):
                x,y = gridx[i], gridy[i]
                ax.scatter(x, y, color='red', s=5)


                #current goal position describes front wheel location, need it to describe tag locations
            for i in range(len(gridy)):
                x,y = goal_x_mask[i], goal_y_mask[i]
                ax.scatter(x, y, color='green', s=5)

            
            # rospy.loginfo("past show goal")
            x1, xn = int(PARKING_STARTX/og.resolution), (PARKING_ENDX/og.resolution)
            y1, yn = int(PARKING_STARTY/og.resolution), (PARKING_ENDY/og.resolution)
        
            ax.scatter(x1, y1, color='orange', s=50)
            ax.scatter(x1, yn, color='orange', s=50)
            ax.scatter(xn, y1, color='orange', s=50)
            ax.scatter(xn, yn, color='orange', s=50)

            path_index+=1

            writer.grab_frame()
        
        