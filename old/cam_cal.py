#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray
from tf2_geometry_msgs import do_transform_pose
import threading

class AprilTagTFPublisher:
    def __init__(self):
        rospy.init_node('apriltag_tf_publisher')

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.camera_frames = {
            "first_camera": {
                "topic": "/first_camera/tag_detections",
                "calibrated": False,
                "transform": None
            },
            "second_camera": {
                "topic": "/second_camera/tag_detections",
                "calibrated": False,
                "transform": None
            }
        }

        self.lock = threading.Lock()

        for cam_name, config in self.camera_frames.items():
            rospy.Subscriber(config["topic"], AprilTagDetectionArray,
                             callback=self.make_callback(cam_name))

        self.rate = rospy.Rate(10)
        self.run()

    def make_callback(self, camera_name):
        def callback(msg):
            if not self.camera_frames[camera_name]["calibrated"]:
                for detection in msg.detections:
                    if 0 in detection.id:
                        T = geometry_msgs.msg.TransformStamped()
                        T.header.stamp = rospy.Time.now()
                        T.header.frame_id = "world"
                        T.child_frame_id = camera_name

                        tag_pose = detection.pose.pose.pose
                        T.transform.translation = tag_pose.position
                        T.transform.rotation = tag_pose.orientation

                        # Invert tag pose (tag_0 wrt camera) to get camera wrt tag (world)
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
                        trans, rot = tf_conversions.transformations.translation_from_matrix(inv_transform), \
                                     tf_conversions.transformations.quaternion_from_matrix(inv_transform)

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
                        break

            else:
                # After calibration, publish all tag detections relative to world
                for detection in msg.detections:
                    tag_id = detection.id[0]
                    pose_in_cam = detection.pose.pose.pose

                    try:
                        cam_to_world = self.tf_buffer.lookup_transform("world", camera_name, rospy.Time(0),
                                                                       rospy.Duration(1.0))
                        tag_pose_world = do_transform_pose(detection.pose.pose, cam_to_world)

                        tag_tf = geometry_msgs.msg.TransformStamped()
                        tag_tf.header.stamp = rospy.Time.now()
                        tag_tf.header.frame_id = "world"
                        tag_tf.child_frame_id = f"tag_{tag_id}"
                        tag_tf.transform.translation = tag_pose_world.pose.position
                        tag_tf.transform.rotation = tag_pose_world.pose.orientation

                        self.br.sendTransform(tag_tf)
                    except Exception as e:
                        rospy.logwarn(f"Transform error: {e}")
        return callback

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                for cam_name, config in self.camera_frames.items():
                    if config["calibrated"] and config["transform"]:
                        config["transform"].header.stamp = rospy.Time.now()
                        self.br.sendTransform(config["transform"])
            self.rate.sleep()

if __name__ == '__main__':
    try:
        AprilTagTFPublisher()
    except rospy.ROSInterruptException:
        pass
