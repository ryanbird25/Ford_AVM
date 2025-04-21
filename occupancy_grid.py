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
    
    def get_car_footprint(self, car_position, offset = 0, res_increase=1):
            #car position is 3 item array
            #input is global position of the tag
            car_x, car_y, car_heading = car_position[0], car_position[1], car_position[2]
            # rospy.loginfo("Entered Get Footprint")
            # rospy.loginfo(car_x)
            # rospy.loginfo(car_y)
            #TODO I think the fact that y is actually forward is cause a problem here
            # car_x_center = car_x + TAG_X_OFFSET_FROM_CENTER*np.sin(np.radians(car_heading))
            # car_y_center = car_y + TAG_Y_OFFSET_FROM_CENTER*np.cos(np.radians(car_heading)) 
            # Apply tag offset: since Y is forward, adjust accordingly
            car_x_center = car_x - TAG_Y_OFFSET_FROM_CENTER * np.sin(car_heading)
            car_y_center = car_y + TAG_Y_OFFSET_FROM_CENTER * np.cos(car_heading)
            # rospy.loginfo("updated car center")
            # rospy.loginfo(car_x_center)
            # rospy.loginfo(car_y_center)
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
            grid_x, grid_y = self.get_car_footprint(pose)
            self.grid[grid_y, grid_x] = 1

    def remove_car_footprint(self, car_fiducial_position):
        # Clear the car footprint from the occupancy grid, input is fiducial in world space
        rospy.loginfo("Car Removed from fiducial position: "+ str(car_fiducial_position))
        # rospy.loginfo(str(get_current_world_position()))
        # rospy.loginfo(str(get_current_grid_position()))
        grid_x, grid_y = self.get_car_footprint(car_fiducial_position, .05, 4)
        self.grid[grid_y, grid_x] = 0


    def update_grid_from_pointcloud(self, point_cloud):
        # Update the occupancy grid based on the point cloud
        #point cloud is in [[x,y,z],.]
        self.grid = np.zeros((int(OCCUPANCY_GRID_Y//self.resolution), int(OCCUPANCY_GRID_X//self.resolution)), dtype=np.int8)

        for point in point_cloud:
            x, y = int(point[0]/self.resolution), int(point[1]/self.resolution)
            if 0 <= y < self.grid.shape[0] and 0 <= x < self.grid.shape[1]:
                self.grid[y][x] = 1


    def update_grid_from_onboard(self, sensor_dict):
        #sensor_dict of type {sensor_name: [sensor_position(x,y,heading array), sensor_value]}
        for sensor_name, sensor_values in sensor_dict.items():
            sensor_position = sensor_values[0]
            sensor_distance_reading = sensor_values[1]

        contact_x = sensor_position[0] + sensor_distance_reading * np.cos(sensor_position[2])
        contact_y = sensor_position[1] + sensor_distance_reading * np.sin(sensor_position[2])
        # Update the occupancy grid based on the onboard sensor position
        grid_x, grid_y = self.world_to_grid(sensor_position[0], sensor_position[1])
        if 0 <= grid_x < self.grid.shape[0] and 0 <= grid_y < self.grid.shape[1]:
            self.grid[grid_x, grid_y] = 1

    
    

        
