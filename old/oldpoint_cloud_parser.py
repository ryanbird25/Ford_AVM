#current pc has 60 degree downard angle from horizontal, 96cm height

import numpy as np
import open3d as o3d

X_DIMENSION = 1.65
Y_DIMENSION = 2.65
FLOOR_OFFSET = 0.02
# def transform_points(points, T):
#     homog = np.hstack([points, np.ones((points.shape[0], 1))])
#     transformed = (T @ homog.T).T
#     return transformed[:, :3]

class PointCloudParser:
    def __init__(self, point_cloud):
        self.point_cloud = point_cloud
        self.remove_floor()
        
    def get_point_cloud(self):
        return self.point_cloud.points[:,:-1]
    
    def visualize_point_cloud(self):

        self.point_cloud.paint_uniform_color([0, 1, 0]) 
        o3d.visualization.draw_geometries([self.point_cloud])
    def remove_floor(self):
        boudning_points_in_frame = np.array([
                                        [0,  0,   -FLOOR_OFFSET],
                                        [Y_DIMENSION, 0,   -FLOOR_OFFSET],
                                        [Y_DIMENSION, X_DIMENSION,  -FLOOR_OFFSET],
                                        [0,  X_DIMENSION,  -FLOOR_OFFSET],
                                        [0,  0,   FLOOR_OFFSET],
                                        [Y_DIMENSION, 0,   FLOOR_OFFSET],
                                        [Y_DIMENSION, X_DIMENSION,  FLOOR_OFFSET],
                                        [0,  X_DIMENSION,  FLOOR_OFFSET],
                                    ])
        obb = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(boudning_points_in_frame))
        self.point_cloud = self.point_cloud.crop(obb, invert = True)
        


# pc = o3d.io.read_point_cloud("output.ply")
# pc_cord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=.2, origin=[0, 0, 0])
# og_cord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=.2, origin=[0, 0, 0])

# og_cord_frame.rotate(pc.get_rotation_matrix_from_xyz((np.deg2rad(50)+np.pi/2, np.pi ,0)), center=(0, 0, 0))
# og_cord_frame.translate((0, 0, .85), relative=False)


# R = pc.get_rotation_matrix_from_xyz((np.deg2rad(50)+np.pi/2, np.pi, 0))
# t = np.array([0, 0, .85])

# pc_T_og = np.eye(4)
# pc_T_og[:3, :3] = R
# pc_T_og[:3, 3] = t
# og_T_pc = np.linalg.inv(pc_T_og)

# boudning_points_in_og = np.array([
#                                         [0,  0,   -0.02],
#                                         [2, 0,   -0.02],
#                                         [2, 2,  -0.02],
#                                         [0,  2,  -0.02],
#                                         [0,  0,   0.02],
#                                         [2, 0,   0.02],
#                                         [2, 2,  .02],
#                                         [0,  2,  0.02],
#                                     ])

# points_in_frame = transform_points(boudning_points_in_og, pc_T_og)
# obb = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points_in_frame))
# obb.color = (1, 0, 0)







# cropped = pc.crop(obb, invert = True)
# pc.paint_uniform_color([0, 1, 0]) 
# pc.paint_uniform_color([0, 0, 1]) 

# o3d.visualization.draw_geometries([cropped, obb, og_cord_frame, pc_cord_frame])