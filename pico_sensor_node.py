#!/usr/bin/env python

import rospy
import requests
import time
from std_msgs.msg import String

PICO_IP = "192.168.137.53"
url = f"http://{PICO_IP}/data"

def main():
    # Initialize the ROS node
    rospy.init_node('pico_sensor_node', anonymous=True)

    # Create a publisher for the 'pico_data' topic
    pub = rospy.Publisher('pico_data', String, queue_size=10)

    rate = rospy.Rate(1)  # Set the loop rate to 1 Hz

    while not rospy.is_shutdown():
        try:
            res = requests.get(url)
            data = res.json()

            # Create a message to publish
            message = f"Front: {data['front']:.2f} cm, Back: {data['back']:.2f} cm, Left: {data['left']:.2f} cm"

            # Publish the data
            pub.publish(message)

            # Log the data to console
            rospy.loginfo(message)

        except Exception as e:
            rospy.logerr("Request failed: %s" % e)

        rate.sleep()  # Sleep to maintain the loop rate

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass