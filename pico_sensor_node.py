#!/usr/bin/env python

import rospy
import requests
from std_msgs.msg import String

PICO_IP = "192.168.137.34"
url = f"http://{PICO_IP}/data"

def main():
    # Initialize the ROS node
    rospy.init_node('pico_sensor_node', anonymous=True)

    # Create a publisher for the 'pico_data' topic
    pub = rospy.Publisher('pico_data', String, queue_size=10)
    rate = rospy.Rate(1)  # Hz

    # keep track of the last‐good reading
    last_data = {"front": 0.0, "back": 0.0, "left": 0.0}

    while not rospy.is_shutdown():
        try:
            res = requests.get(url, timeout=1)
            res.raise_for_status()            # HTTP errors → RequestException
            data = res.json()                 # ValueError on bad JSON
            last_data = data                  # update only on full success
        except ValueError:
            pass
        except requests.RequestException as e:
            pass

        # build & publish from last_data regardless of success/failure
        msg = (
            f"Front: {last_data['front']:.2f} cm, "
            f"Back:  {last_data['back']:.2f} cm, "
            f"Left:  {last_data['left']:.2f} cm"
        )
        pub.publish(msg)
        rospy.loginfo(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
