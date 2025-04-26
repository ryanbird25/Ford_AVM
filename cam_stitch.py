# #!/usr/bin/env python3

# import rospy
# import tf2_ros
# import tf_conversions
# import geometry_msgs.msg
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from apriltag_ros.msg import AprilTagDetectionArray
# from tf2_geometry_msgs import do_transform_pose
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# import threading

# class AprilTagTFPublisher:
#     def __init__(self):
#         rospy.init_node('apriltag_tf_publisher')

#         self.br = tf2_ros.TransformBroadcaster()
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

#         self.cloud_pub = rospy.Publisher('/stitched_pointcloud', PointCloud2, queue_size=1)

#         self.camera_frames = {
#             "first_camera": {
#                 "tag_topic": "/first_camera/tag_detections",
#                 "cloud_topic": "/first_camera/depth/color/points",
#                 "cloud_frame": "first_camera_color_optical_frame",
#                 "cloud": None,
#                 "calibrated": False,
#                 "transform": None
#             },
#             "second_camera": {
#                 "tag_topic": "/second_camera/tag_detections",
#                 "cloud_topic": "/second_camera/depth/color/points",
#                 "cloud_frame": "second_camera_color_optical_frame",
#                 "cloud": None,
#                 "calibrated": False,
#                 "transform": None
#             }
#         }

#         self.lock = threading.Lock()

#         for cam_name, config in self.camera_frames.items():
#             rospy.Subscriber(config["tag_topic"], AprilTagDetectionArray, self.make_callback(cam_name))
#             rospy.Subscriber(config["cloud_topic"], PointCloud2, self.make_cloud_callback(cam_name))

#         self.rate = rospy.Rate(10)
#         self.run()

#     def make_cloud_callback(self, camera_name):
#         def callback(msg):
#             with self.lock:
#                 self.camera_frames[camera_name]["cloud"] = msg
#         return callback

#     def make_callback(self, camera_name):
#         def callback(msg):
#             if not self.camera_frames[camera_name]["calibrated"]:
#                 for detection in msg.detections:
#                     if 0 in detection.id:
#                         tag_pose = detection.pose.pose.pose

#                         inv_transform = tf_conversions.transformations.inverse_matrix(
#                             tf_conversions.transformations.compose_matrix(
#                                 translate=[tag_pose.position.x,
#                                            tag_pose.position.y,
#                                            tag_pose.position.z],
#                                 angles=tf_conversions.transformations.euler_from_quaternion([
#                                     tag_pose.orientation.x,
#                                     tag_pose.orientation.y,
#                                     tag_pose.orientation.z,
#                                     tag_pose.orientation.w
#                                 ])
#                             )
#                         )
#                         trans, rot = tf_conversions.transformations.translation_from_matrix(inv_transform), \
#                                      tf_conversions.transformations.quaternion_from_matrix(inv_transform)

#                         T = geometry_msgs.msg.TransformStamped()
#                         T.header.stamp = rospy.Time.now()
#                         T.header.frame_id = "world"
#                         T.child_frame_id = camera_name
#                         T.transform.translation.x = trans[0]
#                         T.transform.translation.y = trans[1]
#                         T.transform.translation.z = trans[2]
#                         T.transform.rotation.x = rot[0]
#                         T.transform.rotation.y = rot[1]
#                         T.transform.rotation.z = rot[2]
#                         T.transform.rotation.w = rot[3]

#                         with self.lock:
#                             self.camera_frames[camera_name]["transform"] = T
#                             self.camera_frames[camera_name]["calibrated"] = True
#                         rospy.loginfo(f"{camera_name} calibrated to world via tag_0")
#                         break
#         return callback

#     def run(self):
#         while not rospy.is_shutdown():
#             with self.lock:
#                 stitched_clouds = []
#                 for cam_name, config in self.camera_frames.items():
#                     if config["calibrated"] and config["cloud"]:
#                         config["transform"].header.stamp = rospy.Time.now()
#                         self.br.sendTransform(config["transform"])

#                         try:
#                             tf_to_world = self.tf_buffer.lookup_transform(
#                                 "world",
#                                 config["cloud_frame"],
#                                 rospy.Time(0),
#                                 rospy.Duration(1.0)
#                             )
#                             cloud_transformed = do_transform_cloud(config["cloud"], tf_to_world)
#                             stitched_clouds.append(cloud_transformed)
#                         except Exception as e:
#                             rospy.logwarn(f"Point cloud transform error from {cam_name}: {e}")

#                 if stitched_clouds:
#                     try:
#                         merged_cloud = self.merge_clouds(stitched_clouds)
#                         merged_cloud.header.stamp = rospy.Time.now()
#                         merged_cloud.header.frame_id = "world"
#                         self.cloud_pub.publish(merged_cloud)
#                     except Exception as e:
#                         rospy.logerr(f"Error merging point clouds: {e}")
#             self.rate.sleep()

#     def merge_clouds(self, cloud_list):
#         points = []
#         for cloud in cloud_list:
#             points.extend(list(pc2.read_points(cloud, skip_nans=True)))
#         header = cloud_list[0].header
#         fields = cloud_list[0].fields
#         return pc2.create_cloud(header, fields, points)

# if __name__ == '__main__':
#     try:
#         AprilTagTFPublisher()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from apriltag_ros.msg import AprilTagDetectionArray
from tf2_geometry_msgs import do_transform_pose
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import threading
import math

class AprilTagTFPublisher:
    def __init__(self):
        rospy.init_node('apriltag_tf_publisher')

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cloud_pub = rospy.Publisher('/stitched_pointcloud', PointCloud2, queue_size=1)

        self.camera_frames = {
            "first_camera": {
                "tag_topic": "/first_camera/tag_detections",
                "cloud_topic": "/first_camera/depth/color/points",
                "cloud_frame": "first_camera_color_optical_frame",
                "cloud": None,
                "calibrated": False,
                "transform": None
            },
            "second_camera": {
                "tag_topic": "/second_camera/tag_detections",
                "cloud_topic": "/second_camera/depth/color/points",
                "cloud_frame": "second_camera_color_optical_frame",
                "cloud": None,
                "calibrated": False,
                "transform": None
            }
        }

        # Dictionary to store the most recent tag transform (from each camera)
        # The key will be the camera name and the value a TransformStamped for tag_0.
        self.tag_detections = {}

        self.lock = threading.Lock()

        for cam_name, config in self.camera_frames.items():
            rospy.Subscriber(config["tag_topic"], AprilTagDetectionArray, self.make_callback(cam_name))
            rospy.Subscriber(config["cloud_topic"], PointCloud2, self.make_cloud_callback(cam_name))

        self.rate = rospy.Rate(10)
        self.run()

    def make_cloud_callback(self, camera_name):
        def callback(msg):
            with self.lock:
                self.camera_frames[camera_name]["cloud"] = msg
        return callback

    def make_callback(self, camera_name):
        def callback(msg):
            for detection in msg.detections:
                # Check for tag id 0 (adjust if you need to handle other tag ids)
                if 0 in detection.id:
                    with self.lock:
                        calibrated = self.camera_frames[camera_name]["calibrated"]

                    # If the camera is not yet calibrated, use the first detection to calibrate.
                    if not calibrated:
                        tag_pose = detection.pose.pose.pose
                        # Compute inverse transform from detection (i.e. from tag in camera frame)
                        inv_transform = tf_conversions.transformations.inverse_matrix(
                            tf_conversions.transformations.compose_matrix(
                                translate=[tag_pose.position.x,
                                           tag_pose.position.y,
                                           tag_pose.position.z],
                                angles=tf_conversions.transformations.euler_from_quaternion([
                                    tag_pose.orientation.x,
                                    tag_pose.orientation.y,
                                    tag_pose.orientation.z,
                                    tag_pose.orientation.w
                                ])
                            )
                        )
                        trans = tf_conversions.transformations.translation_from_matrix(inv_transform)
                        rot = tf_conversions.transformations.quaternion_from_matrix(inv_transform)

                        T = geometry_msgs.msg.TransformStamped()
                        T.header.stamp = rospy.Time.now()
                        T.header.frame_id = "world"
                        # For consistency when transforming clouds, it may be helpful to publish the sensor frame.
                        # If a static transform exists between the camera’s “base” and its optical frame,
                        # you could chain them. Here we publish the calibration to the sensor frame.
                        T.child_frame_id = self.camera_frames[camera_name]["cloud_frame"]
                        T.transform.translation.x = trans[0]
                        T.transform.translation.y = trans[1]
                        T.transform.translation.z = trans[2]
                        T.transform.rotation.x = rot[0]
                        T.transform.rotation.y = rot[1]
                        T.transform.rotation.z = rot[2]
                        T.transform.rotation.w = rot[3]

                        with self.lock:
                            self.camera_frames[camera_name]["transform"] = T
                            self.camera_frames[camera_name]["calibrated"] = True

                        rospy.loginfo(f"{camera_name} calibrated to world via tag_0")
                        # Continue on so we also attempt to update tag detection below.
                    
                    # If the camera is calibrated, compute and store the tag detection transform.
                    try:
                        # Look up the transform from the camera's sensor frame to world.
                        tf_to_world = self.tf_buffer.lookup_transform(
                            "world",
                            self.camera_frames[camera_name]["cloud_frame"],
                            rospy.Time(0),
                            rospy.Duration(1.0)
                        )
                        # Use the received tag pose (in the camera frame) and transform to world.
                        tag_pose_world = do_transform_pose(detection.pose.pose.pose, tf_to_world)
                        T_tag = geometry_msgs.msg.TransformStamped()
                        T_tag.header.stamp = rospy.Time.now()
                        T_tag.header.frame_id = "world"
                        # We publish a single common tag frame for tag id 0.
                        T_tag.child_frame_id = "tag_0"
                        T_tag.transform.translation.x = tag_pose_world.pose.position.x
                        T_tag.transform.translation.y = tag_pose_world.pose.position.y
                        T_tag.transform.translation.z = tag_pose_world.pose.position.z
                        T_tag.transform.rotation = tag_pose_world.pose.orientation

                        with self.lock:
                            # Store/update the tag detection for this camera.
                            self.tag_detections[camera_name] = T_tag
                    except Exception as e:
                        rospy.logwarn(f"Error transforming tag from {camera_name}: {e}")
                    # Process only one matching detection per message.
                    break
        return callback

    def fuse_transforms(self, t1, t2):
        """
        Fuse two TransformStamped objects (assumed to be for the same tag)
        by averaging the translation vectors and the quaternion rotations.
        """
        fused = geometry_msgs.msg.TransformStamped()
        fused.header.stamp = rospy.Time.now()
        fused.header.frame_id = "world"
        fused.child_frame_id = "tag_0"
        # Average translations:
        fused.transform.translation.x = (t1.transform.translation.x + t2.transform.translation.x) / 2.0
        fused.transform.translation.y = (t1.transform.translation.y + t2.transform.translation.y) / 2.0
        fused.transform.translation.z = (t1.transform.translation.z + t2.transform.translation.z) / 2.0

        # Average quaternions:
        q1 = [t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w]
        q2 = [t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w]
        # Ensure the quaternions are close in hemisphere, otherwise flip one.
        dot = sum(a * b for a, b in zip(q1, q2))
        if dot < 0:
            q2 = [-elem for elem in q2]
        # Simple average followed by normalization:
        fused_q = [(q1[i] + q2[i]) / 2.0 for i in range(4)]
        norm = math.sqrt(sum(x*x for x in fused_q))
        fused_q = [x / norm for x in fused_q]
        fused.transform.rotation.x = fused_q[0]
        fused.transform.rotation.y = fused_q[1]
        fused.transform.rotation.z = fused_q[2]
        fused.transform.rotation.w = fused_q[3]
        return fused

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                stitched_clouds = []
                # Publish each camera’s calibration transform.
                for cam_name, config in self.camera_frames.items():
                    if config["calibrated"] and config["cloud"]:
                        # Update the timestamp of the calibration transform.
                        config["transform"].header.stamp = rospy.Time.now()
                        self.br.sendTransform(config["transform"])

                        try:
                            tf_to_world = self.tf_buffer.lookup_transform(
                                "world",
                                config["cloud_frame"],
                                rospy.Time(0),
                                rospy.Duration(1.0)
                            )
                            cloud_transformed = do_transform_cloud(config["cloud"], tf_to_world)
                            stitched_clouds.append(cloud_transformed)
                        except Exception as e:
                            rospy.logwarn(f"Point cloud transform error from {cam_name}: {e}")

                if stitched_clouds:
                    try:
                        merged_cloud = self.merge_clouds(stitched_clouds)
                        merged_cloud.header.stamp = rospy.Time.now()
                        merged_cloud.header.frame_id = "world"
                        self.cloud_pub.publish(merged_cloud)
                    except Exception as e:
                        rospy.logerr(f"Error merging point clouds: {e}")

                # Get the list of tag transforms from all cameras.
                tag_tf_list = list(self.tag_detections.values())

            # Outside the lock, fuse and publish the tag transform if available.
            if tag_tf_list:
                if len(tag_tf_list) == 1:
                    fused_tag = tag_tf_list[0]
                else:
                    # Fuse detections from multiple cameras.
                    fused_tag = tag_tf_list[0]
                    for t in tag_tf_list[1:]:
                        fused_tag = self.fuse_transforms(fused_tag, t)
                self.br.sendTransform(fused_tag)

            self.rate.sleep()

    def merge_clouds(self, cloud_list):
        points = []
        # Read points from each point cloud and combine into one big list.
        for cloud in cloud_list:
            points.extend(list(pc2.read_points(cloud, skip_nans=True)))
        header = cloud_list[0].header
        fields = cloud_list[0].fields
        return pc2.create_cloud(header, fields, points)

if __name__ == '__main__':
    try:
        AprilTagTFPublisher()
    except rospy.ROSInterruptException:
        pass















#!/usr/bin/env python3

# import rospy
# import tf2_ros
# import tf_conversions
# import geometry_msgs.msg
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# from apriltag_ros.msg import AprilTagDetectionArray
# from tf2_geometry_msgs import do_transform_pose
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# import threading
# import math
# import numpy as np
# import open3d as o3d

# class AprilTagTFPublisher:
#     def __init__(self):
#         rospy.init_node('apriltag_tf_publisher')

#         self.br = tf2_ros.TransformBroadcaster()
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

#         self.cloud_pub = rospy.Publisher('/stitched_pointcloud', PointCloud2, queue_size=1)

#         self.tag_ids = [0, 14, 16]

#         self.camera_frames = {
#             "first_camera": {
#                 "tag_topic": "/first_camera/tag_detections",
#                 "cloud_topic": "/first_camera/depth/color/points",
#                 "cloud_frame": "first_camera_color_optical_frame",
#                 "cloud": None,
#                 "calibrated": False,
#                 "transform": None
#             },
#             "second_camera": {
#                 "tag_topic": "/second_camera/tag_detections",
#                 "cloud_topic": "/second_camera/depth/color/points",
#                 "cloud_frame": "second_camera_color_optical_frame",
#                 "cloud": None,
#                 "calibrated": False,
#                 "transform": None
#             }
#         }

#         self.tag_detections = {}
#         self.lock = threading.Lock()

#         for cam_name, config in self.camera_frames.items():
#             rospy.Subscriber(config["tag_topic"], AprilTagDetectionArray, self.make_callback(cam_name))
#             rospy.Subscriber(config["cloud_topic"], PointCloud2, self.make_cloud_callback(cam_name))

#         self.rate = rospy.Rate(10)
#         self.run()

#     def make_cloud_callback(self, camera_name):
#         def callback(msg):
#             with self.lock:
#                 self.camera_frames[camera_name]["cloud"] = msg
#         return callback

#     def make_callback(self, camera_name):
#         def callback(msg):
#             tag_poses = []
#             for detection in msg.detections:
#                 if any(tag_id in detection.id for tag_id in self.tag_ids):
#                     tag_pose = detection.pose.pose.pose
#                     tag_poses.append(tag_pose)

#             if tag_poses:
#                 transforms = []
#                 for tag_pose in tag_poses:
#                     matrix = tf_conversions.transformations.compose_matrix(
#                         translate=[tag_pose.position.x, tag_pose.position.y, tag_pose.position.z],
#                         angles=tf_conversions.transformations.euler_from_quaternion([
#                             tag_pose.orientation.x,
#                             tag_pose.orientation.y,
#                             tag_pose.orientation.z,
#                             tag_pose.orientation.w
#                         ])
#                     )
#                     inv = tf_conversions.transformations.inverse_matrix(matrix)
#                     trans = tf_conversions.transformations.translation_from_matrix(inv)
#                     rot = tf_conversions.transformations.quaternion_from_matrix(inv)
#                     transforms.append((trans, rot))

#                 if transforms:
#                     avg_trans = np.mean([t[0] for t in transforms], axis=0)
#                     avg_rot = np.mean([t[1] for t in transforms], axis=0)
#                     avg_rot /= np.linalg.norm(avg_rot)

#                     T = geometry_msgs.msg.TransformStamped()
#                     T.header.stamp = rospy.Time.now()
#                     T.header.frame_id = "world"
#                     T.child_frame_id = self.camera_frames[camera_name]["cloud_frame"]
#                     T.transform.translation.x = avg_trans[0]
#                     T.transform.translation.y = avg_trans[1]
#                     T.transform.translation.z = avg_trans[2]
#                     T.transform.rotation.x = avg_rot[0]
#                     T.transform.rotation.y = avg_rot[1]
#                     T.transform.rotation.z = avg_rot[2]
#                     T.transform.rotation.w = avg_rot[3]

#                     with self.lock:
#                         self.camera_frames[camera_name]["transform"] = T
#                         self.camera_frames[camera_name]["calibrated"] = True

#                     rospy.loginfo(f"{camera_name} calibrated to world using tags {self.tag_ids}")
#         return callback

#     def ros_to_o3d(self, cloud):
#         points = np.array(list(pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))))
#         o3d_cloud = o3d.geometry.PointCloud()
#         o3d_cloud.points = o3d.utility.Vector3dVector(points)
#         return o3d_cloud

#     def o3d_to_ros(self, o3d_cloud, header, _fields=None):
#         points = np.asarray(o3d_cloud.points)

#         # Create the proper fields for PointCloud2 (only x, y, z)
#         fields = [
#             pc2.PointField(name='x', offset=0,  datatype=pc2.PointField.FLOAT32, count=1),
#             pc2.PointField(name='y', offset=4,  datatype=pc2.PointField.FLOAT32, count=1),
#             pc2.PointField(name='z', offset=8,  datatype=pc2.PointField.FLOAT32, count=1)
#         ]

#         cloud = pc2.create_cloud(header, fields, points)
#         return cloud


#     def merge_clouds(self, cloud_list):
#         o3d_clouds = [self.ros_to_o3d(c) for c in cloud_list]

#         voxel_size = 0.01
#         o3d_clouds = [c.voxel_down_sample(voxel_size) for c in o3d_clouds]

#         ref = o3d_clouds[0]
#         for target in o3d_clouds[1:]:
#             icp_result = o3d.pipelines.registration.registration_icp(
#                 target, ref, 0.05, np.eye(4),
#                 o3d.pipelines.registration.TransformationEstimationPointToPoint()
#             )
#             target.transform(icp_result.transformation)
#             ref += target
#             ref = ref.voxel_down_sample(voxel_size)

#         return self.o3d_to_ros(ref, cloud_list[0].header, cloud_list[0].fields)

#     def run(self):
#         while not rospy.is_shutdown():
#             with self.lock:
#                 stitched_clouds = []
#                 for cam_name, config in self.camera_frames.items():
#                     if config["calibrated"] and config["cloud"]:
#                         config["transform"].header.stamp = rospy.Time.now()
#                         self.br.sendTransform(config["transform"])

#                         try:
#                             tf_to_world = self.tf_buffer.lookup_transform(
#                                 "world",
#                                 config["cloud_frame"],
#                                 rospy.Time(0),
#                                 rospy.Duration(1.0)
#                             )
#                             cloud_transformed = do_transform_cloud(config["cloud"], tf_to_world)
#                             stitched_clouds.append(cloud_transformed)
#                         except Exception as e:
#                             rospy.logwarn(f"Point cloud transform error from {cam_name}: {e}")

#                 if stitched_clouds:
#                     try:
#                         merged_cloud = self.merge_clouds(stitched_clouds)
#                         merged_cloud.header.stamp = rospy.Time.now()
#                         merged_cloud.header.frame_id = "world"
#                         self.cloud_pub.publish(merged_cloud)
#                     except Exception as e:
#                         rospy.logerr(f"Error merging point clouds: {e}")

#             self.rate.sleep()

# if __name__ == '__main__':
#     try:
#         AprilTagTFPublisher()
#     except rospy.ROSInterruptException:
#         pass
