import numpy as np
import open3d as o3d
import rospy

X_DIMENSION = 1.65 # This is the Length of the space
Y_DIMENSION = 2.65 # This is the Length of the space
FLOOR_OFFSET = 0.04

class PointCloudParser:
    def __init__(self, point_cloud):
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(point_cloud)

        self.remove_floor()
        
    def get_point_cloud(self):
        points = np.asarray(self.point_cloud.points)[:,:-1]
        return points
    
    
    def visualize_point_cloud(self):

        self.point_cloud.paint_uniform_color([0, 1, 0]) 
        o3d.visualization.draw_geometries([self.point_cloud])
        
    def remove_floor(self):
        # 1) Define the eight corners of your floor slab in camera frame:
        bounding_points = np.array([
            [0,           0,            -FLOOR_OFFSET],
            [X_DIMENSION, 0,            -FLOOR_OFFSET],
            [X_DIMENSION, Y_DIMENSION,  -FLOOR_OFFSET],
            [0,           Y_DIMENSION,  -FLOOR_OFFSET],
            [0,           0,             FLOOR_OFFSET],
            [X_DIMENSION, 0,             FLOOR_OFFSET],
            [X_DIMENSION, Y_DIMENSION,   FLOOR_OFFSET],
            [0,           Y_DIMENSION,   FLOOR_OFFSET],
        ])

        # 2) Build an OrientedBoundingBox:
        obb = o3d.geometry.OrientedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(bounding_points)
        )

        # 3) Ask the box which point-indices lie *inside* it:
        inside_indices = obb.get_point_indices_within_bounding_box(
            self.point_cloud.points
        )

        # 4) Remove those points from the cloud (invert=True):
        self.point_cloud = self.point_cloud.select_by_index(
            inside_indices, invert=True
        )
        