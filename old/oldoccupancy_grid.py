import numpy as np
import open3d as o3d
from point_cloud_parser import ParsedPointCloud

from dataclasses import dataclass

TAG_X_OFFSET_FROM_CENTER = 0 #cm
TAG_Y_OFFSET_FROM_CENTER = 0 #cm

CAR_WIDTH = 14 #cm
CAR_LENGTH = 33.5 #cm

@dataclass
class og_coordinate():
    heading: float#excecuted in this order
    x: int
    y: int

class OccupancyGrid():

    def __init__(self, point_cloud: ParsedPointCloud):
        self.width = 170#cm
        self.length = 235#cm
        self.resolution = .5#cm/grid
        self.grid = None
        self.update_grid(self, point_cloud)

    def get_resolution(self):
        return self.resolution
    
    def get_grid(self):
        return self.grid
    
    def world_to_grid(self, x: float, y: float):
        # Convert world coordinates to grid indices
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return grid_x, grid_y
    
    def get_car_footprint(self, car_position: np.ndarray[float]):
            car_x, car_y, car_heading = car_position[0], car_position[1], car_position.heading[2]

            car_x_center = car_x + TAG_X_OFFSET_FROM_CENTER*np.cos(np.radians(car_heading))
            car_y_center = car_y + TAG_Y_OFFSET_FROM_CENTER*np.sin(np.radians(car_heading)) 

            # Convert car dimensions to grid cells
            dx = np.arange(car_x_center - CAR_WIDTH// 2, np.round(car_x_center+CAR_WIDTH / 2) + 1)
            dy = np.arange(-car_y_center - CAR_LENGTH // 2, np.round(car_y_center+CAR_LENGTH / 2) + 1)
            dx, dy = np.meshgrid(dx, dy)

            # Flatten the relative positions
            dx = dx.flatten()
            dy = dy.flatten()

            # Apply rotation based on the car's heading
            rotated_dx = (dx * np.cos(np.radians(car_heading)) - dy * np.sin(np.radians(car_heading))).astype(int)
            rotated_dy = (dx * np.sin(np.radians(car_heading)) + dy * np.cos(np.radians(car_heading))).astype(int)

            # Calculate the absolute grid positions
            grid_x = np.round(rotated_dx).astype(int)
            grid_y = np.round(rotated_dy).astype(int)

            return grid_x, grid_y
    
    def update_grid_from_fiducials(self, fiducials_poses: list[np.ndarray[float]]):
        # Update the occupancy grid based on fiducials
        for pose in fiducials_poses:
            grid_x, grid_y = self.get_car_footprint(pose)
            self.grid[grid_x, grid_y] = 1

    def remove_car_footprint(self, car_fiducial_position: np.ndarray[float]):
        # Clear the car footprint from the occupancy grid
        grid_x, grid_y = self.get_car_footprint(car_fiducial_position)
        self.grid[grid_x, grid_y] = 0


    def update_grid_from_pointcloud(self, point_cloud: np.ndarray[float]):
        # Update the occupancy grid based on the point cloud
        self.grid = np.zeros((self.length/self.resolution, self.width/self.resolution), dtype=np.int8)
        for point in point_cloud:
            x, y = int(point[0] / self.resolution), int(point[1] / self.resolution)
            if 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]:
                self.grid[x, y] = 1

    
    

        
