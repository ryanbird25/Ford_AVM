#!/usr/bin/env python3
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class CarVisualizerTF:
    def __init__(self):
        rospy.init_node('car_visualizer_tf')

        # which AprilTag to follow:
        self.tag_id      = rospy.get_param('~tag_id', 2)
        self.tag_frame   = f"tag_{self.tag_id}"

        # frames:
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.base_frame  = rospy.get_param('~base_frame',  'base_link')

        # TF broadcaster + listener
        self.br        = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # listen to every incoming /tf message
        rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=10)

        rospy.loginfo("car_visualizer_tf: waiting for %s in %s", 
                      self.tag_frame, self.world_frame)
        rospy.spin()

    def tf_callback(self, msg):
        # whenever apriltag_ros broadcasts tag_<ID> in /tf, grab it
        for t in msg.transforms:
            if t.child_frame_id != self.tag_frame:
                continue

            try:
                # look up the latest world -> tag_ID transform
                trans_stamped = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.tag_frame,
                    rospy.Time(0),
                    rospy.Duration(0.1)
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0,
                    "Couldn't find %s->%s: %s",
                    self.world_frame, self.tag_frame, e)
                return

            # re‑publish it as world -> base_link
            T = TransformStamped()
            T.header.stamp    = rospy.Time.now()
            T.header.frame_id = self.tag_frame
            T.child_frame_id  = self.base_frame
            T.transform = TransformStamped().transform
            T.transform       = trans_stamped.transform

            self.br.sendTransform(T)
            rospy.loginfo("Published %s in %s via %s",
                          self.base_frame, self.world_frame, self.tag_frame)
            break  # only handle first matching tag each callback

if __name__ == '__main__':
    try:
        CarVisualizerTF()
    except rospy.ROSInterruptException:
        pass
