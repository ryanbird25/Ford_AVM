import numpy as np
import open3d as o3d
import rospy
from point_cloud_parser import PointCloudParser
#from orchestrator import CAR_LENGTH, CAR_WIDTH
from dataclasses import dataclass

TAG_X_OFFSET_FROM_CENTER = 0 #m
TAG_Y_OFFSET_FROM_CENTER = .07 #m

CAR_WIDTH = .14 #cm
CAR_LENGTH = .335 #cm

OCCUPANCY_GRID_X = 1.70 #m
OCCUPANCY_GRID_Y = 2.35 #m

@dataclass
class og_coordinate():
    x: int
    y: int
    heading: float#excecuted in this order

    def __post_init__(self):
    
    # 1) round to 1 decimal
        h = round(self.heading, 1)
        # 2) if you hit *any* flavor of π, snap to −π
        if abs(abs(h) - np.pi) < 0.05:
            h = -np.pi
        self.heading = h

    def __hash__(self):
        # Combine x, y, and heading into a unique hash
        return hash((self.x, self.y, self.heading))

    def __eq__(self, other):
        # Check equality based on x, y, and heading
        return (
            isinstance(other, og_coordinate)
            and (self.x, self.y, self.heading) == (other.x, other.y, other.heading)
        )
    
    def __lt__(self, other):
        # Define a less-than comparison for sorting
        return (self.x, self.y, self.heading) < (other.x, other.y, other.heading)

class OccupancyGrid():

    def __init__(self, point_cloud: PointCloudParser, resolution: float = 0.01):
        self.resolution = resolution#m/grid
        self.grid = np.zeros((int(OCCUPANCY_GRID_Y//self.resolution), int(OCCUPANCY_GRID_X//self.resolution)), dtype=np.int8)
        self.update_grid_from_pointcloud(point_cloud)

    def get_resolution(self):
        return self.resolution
    
    def get_grid(self):
        return self.grid
    
    def world_to_grid(self, x: float, y: float):
        # Convert world coordinates to grid indices
        grid_x = int(np.round(x / self.resolution))
        grid_y = int(np.round(y / self.resolution))
        return grid_x, grid_y
    
    def get_car_footprint_from_tag_pos(self, car_position, offset = 0, res_increase=1):
        #car position is 3 item array
        #input is global position of the tag
        car_x, car_y, car_heading = car_position[0], car_position[1], car_position[2]

        car_x_center = car_x - TAG_Y_OFFSET_FROM_CENTER * np.sin(car_heading)
        car_y_center = car_y + TAG_Y_OFFSET_FROM_CENTER * np.cos(car_heading)

        dx = np.arange(-CAR_WIDTH / 2 - offset, CAR_WIDTH / 2 + offset + self.resolution, self.resolution/res_increase)
        dy = np.arange(-CAR_LENGTH / 2 - offset, CAR_LENGTH / 2 + offset + self.resolution, self.resolution/res_increase)

        dx, dy = np.meshgrid(dx, dy)

        dx = dx.flatten()
        dy = dy.flatten()

        # Apply rotation based on the car's heading
        rotated_dx = (dx * np.cos(car_heading)) - (dy * np.sin(car_heading))
        rotated_dy = (dx * np.sin(car_heading)) + (dy * np.cos(car_heading))


        rotated_dx += car_x_center
        rotated_dy += car_y_center
        # Calculate the absolute grid positions
        grid_x = np.round(rotated_dx/self.resolution).astype(int)
        grid_y = np.round(rotated_dy/self.resolution).astype(int)
        # rospy.loginfo("exiting Get Footprint")
        return grid_x, grid_y

    def get_car_footprint_from_center(self, car_position, offset = 0, res_increase=1):
        #car position is 3 item array
        #input is global position of the tag
        car_x_center, car_y_center, car_heading = car_position[0], car_position[1], car_position[2]

        dx = np.arange(-CAR_WIDTH / 2 - offset, CAR_WIDTH / 2 + offset + self.resolution, self.resolution/res_increase)
        dy = np.arange(-CAR_LENGTH / 2 - offset, CAR_LENGTH / 2 + offset + self.resolution, self.resolution/res_increase)

        dx, dy = np.meshgrid(dx, dy)

        dx = dx.flatten()
        dy = dy.flatten()

        # Apply rotation based on the car's heading
        rotated_dx = (dx * np.cos(car_heading)) - (dy * np.sin(car_heading))
        rotated_dy = (dx * np.sin(car_heading)) + (dy * np.cos(car_heading))


        rotated_dx += car_x_center
        rotated_dy += car_y_center
        # Calculate the absolute grid positions
        grid_x = np.round(rotated_dx/self.resolution).astype(int)
        grid_y = np.round(rotated_dy/self.resolution).astype(int)
        # rospy.loginfo("exiting Get Footprint")
        return grid_x, grid_y
    
    def update_grid_from_fiducials(self, fiducials_poses: list):
        # Update the occupancy grid based on fiducials
        for pose in fiducials_poses:
            xs, ys = self.get_car_footprint_from_tag_pos(pose)

            n_rows, n_cols = self.grid.shape
        # build a mask of only those indices that lie within [0..shape-1]
            valid = (xs >= 0) & (xs < n_cols) & (ys >= 0) & (ys < n_rows)
            self.grid[ys[valid], xs[valid]] = 1

    def remove_car_footprint(self, car_fiducial_position):
        # Clear the car footprint from the occupancy grid, input is fiducial in world space
        rospy.loginfo("Car Removed from fiducial position: "+ str(car_fiducial_position))
        # rospy.loginfo(str(get_current_world_position()))
        # rospy.loginfo(str(get_current_grid_position()))
        xs, ys = self.get_car_footprint_from_tag_pos(car_fiducial_position, .1, 4)

        n_rows, n_cols = self.grid.shape
        # build a mask of only those indices that lie within [0..shape-1]
        valid = (xs >= 0) & (xs < n_cols) & (ys >= 0) & (ys < n_rows)
        self.grid[ys[valid], xs[valid]] = 0


    def update_grid_from_pointcloud(self, point_cloud):
        # Update the occupancy grid based on the point cloud
        #point cloud is in [[x,y,z],.]
        self.grid = np.zeros((int(OCCUPANCY_GRID_Y//self.resolution), int(OCCUPANCY_GRID_X//self.resolution)), dtype=np.int8)

        for point in point_cloud:
            x, y = int(point[0]/self.resolution), int(point[1]/self.resolution)
            if 0 <= y < self.grid.shape[0] and 0 <= x < self.grid.shape[1]:
                self.grid[y][x] = 1

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    def update_grid_from_onboard(self, sensor_data, car_center):
        #sensor_dict of type [(sensor_position(x,y,heading array), sensor_value)]

        # for sensor_name, sensor_values in sensor_dict.items():
        #     sensor_position = sensor_values[0]
        #     sensor_distance_reading = sensor_values[1]



        
        for name, item in sensor_data.items():

            x_offset = 0
            y_offset = 0
            heading_change = 0

            if name == 'left':
                x_offset = -.08
                y_offset = -.05
                heading_change = np.pi/2 +np.pi/2
            elif name == 'frontleft':
                x_offset = -.0575
                y_offset = .18
                heading_change = np.pi/6 +np.pi/2
            elif name == "frontright":
                x_offset = .0575
                y_offset = .18
                heading_change = -np.pi/6 +np.pi/2

            
            cx = car_center[0] + x_offset*np.cos(car_center[2]) - y_offset*np.sin(car_center[2])
            cy =car_center[1] + x_offset*np.sin(car_center[2]) + y_offset*np.cos(car_center[2])
            ch = car_center[2] + heading_change 

            contact_x = (cx + (item) * np.cos(ch))
            contact_y = (cy + (item) * np.sin(ch))
            

            sensor_grid_x = int(np.round(cx / self.resolution))
            sensor_grid_y = int(np.round(cy / self.resolution))

            contact_grid_x = int(np.round(contact_x / self.resolution))
            contact_grid_y = int(np.round(contact_y / self.resolution))

            half = 1
        # Use Bresenham's Line Algorithm to mark all points between the sensor and contact point as unoccupied
            line_points = self.bresenham(sensor_grid_x, sensor_grid_y, contact_grid_x, contact_grid_y)
               
            for x, y in line_points:
                for dx in range(-half, half+1):
                    for dy in range(-half, half+1):
                        xi, yi = x + dx, y + dy
                        if 0 <= yi < self.grid.shape[0] and 0 <= xi < self.grid.shape[1]:
                            self.grid[yi, xi] = 0
            
            # for x, y in line_points:
            #     if 0 <= y < self.grid.shape[0] and 0 <= x < self.grid.shape[1]:
            #         self.grid[y, x] = 0  # Mark as unoccupied

            # # Mark the contact point as occupied
            if 0 <= contact_grid_y < self.grid.shape[0] and 0 <= contact_grid_x < self.grid.shape[1]:
                # self.grid[contact_grid_y, contact_grid_x] = 1
                for dx in range(-half, half+1):
                    for dy in range(-half, half+1):
                        xi, yi = contact_grid_x + dx, contact_grid_y + dy
                        if 0 <= yi < self.grid.shape[0] and 0 <= xi < self.grid.shape[1]:
                            self.grid[yi, xi] = 1


    
    

        
