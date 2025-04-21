import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2

def transform_points(points, T):
    homog = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed = (T @ homog.T).T
    return transformed[:, :3]

# 1. Start RealSense pipeline with depth stream only
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

# 2. Get depth sensor and scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()  # e.g., 0.001

# 3. Get intrinsics for depth stream
depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
intrinsics = depth_stream.get_intrinsics()

o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
    intrinsics.width, intrinsics.height,
    intrinsics.fx, intrinsics.fy,
    intrinsics.ppx, intrinsics.ppy
)

# 4. Set up filters
decimation = rs.decimation_filter()
spatial = rs.spatial_filter()

# hole_filling = rs.hole_filling_filter()
disparity = rs.disparity_transform(True)
disparity_inv = rs.disparity_transform(False)

# 5. Wait for frames and apply filtering
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()

# Apply filters in proper order
depth_frame = decimation.process(depth_frame)
depth_frame = disparity.process(depth_frame)
depth_frame = spatial.process(depth_frame)
depth_frame = disparity_inv.process(depth_frame)
# depth_frame = hole_filling.process(depth_frame)

# 6. Convert to Open3D depth image
depth_image_np = np.asanyarray(depth_frame.get_data())
depth_o3d = o3d.geometry.Image(depth_image_np)

# 7. Create point cloud from depth only
pc = o3d.geometry.PointCloud.create_from_depth_image(
    depth_o3d,
    o3d_intrinsics,
    depth_scale=1.0 / depth_scale,  # Converts from depth units to meters
    depth_trunc=3.0,
    stride=1
)



pc_cord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=.2, origin=[0, 0, 0])
og_cord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])



R = pc.get_rotation_matrix_from_xyz((np.deg2rad(45)+np.pi/2, np.pi, 0))
t = np.array([0, 0, .68])

pc_T_og = np.eye(4)
pc_T_og[:3, :3] = R
pc_T_og[:3, 3] = t

og_cord_frame.rotate(R, center=(0, 0, 0))
og_cord_frame.translate(t, relative=False)

boudning_points_in_og = np.array([
    [0,  0,   0.1],
    [1.6, 0,   0.1],
    [1.6, 2.3,  .1],
    [0,  2.3,  0.1],
    [0,  0,   -0.04],
    [1.6, 0,   -0.04],
    [1.6, 2.3,  -.04],
    [0,  2.3,  -0.04],
])

T_inv = np.linalg.inv(pc_T_og)

points_in_frame = transform_points(boudning_points_in_og, pc_T_og)
obb = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points_in_frame))
obb.color = (1, 0, 0)





cropped = pc.crop(obb)
# pc.paint_uniform_color([0, 1, 0]) 
# pc.paint_uniform_color([0, 0, 1]) 

o3d.visualization.draw_geometries([cropped, obb, og_cord_frame])

# cropped.transform(T_inv)

# points = np.asarray(cropped.points)
# points = points[:,:2]
# canvas = np.zeros((300, 300), dtype=np.uint8)

# # Draw points as black pixels
# for x, y in points.astype(int):
#     if 0 <= x < 300 and 0 <= y < 300:
#         canvas[np.round(10*y), np.round(10*x)] = 0

# cv2.imshow("2D Points", canvas)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# # Drop the Z-axis (keep only X and Y)
# points_2d = points[:, :2]

# 10. Stop pipeline
pipeline.stop()