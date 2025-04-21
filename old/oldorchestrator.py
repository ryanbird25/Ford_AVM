#!/usr/bin/env python3

import rospy
from occupancy_grid import OccupancyGrid, og_coordinate 
from dataclasses import dataclass
from shapely.geometry import Polygon, Point
from shapely.affinity import rotate, translate
import matplotlib.pyplot as plt
import numpy as np
import path_planning
import point_cloud_parser
from sensor_msgs.msg import PointCloud2
from apriltag_ros.msg import AprilTagDetectionArray
import tf_conversions
import math

# Parking space and car parameters (in centimeters and radians)
PARKING_STARTX = 10   # cm
PARKING_STARTY = 10   # cm
PARKING_ENDX   = 100  # cm
PARKING_ENDY   = 40   # cm
CAR_WIDTH      = 14   # cm
CAR_LENGTH     = 33.5 # cm

# Offsets for the AprilTag relative to the car center (if needed)
TAG_X_OFFSET_FROM_CENTER = 0  # cm
TAG_Y_OFFSET_FROM_CENTER = 0  # cm

# Global variables to store the latest ROS messages.
global_point_cloud = None
global_fiducials_dict = {}  # Dictionary: {fiducial_num: np.array([x, y, heading])}

#write a get apriltag dictionary function 
#write a get point cloud function 

class parking_orchestrator:
    def __init__(self, point_cloud, car_fiducial_num: int, fiducials_dict):
        rospy.loginfo("Initializing parking orchestrator.")
        self.currently_updating = True
        # Parse the point cloud using the custom parser.
        PointCloudParser = point_cloud_parser.PointCloudParser(point_cloud)
        self.og = OccupancyGrid(PointCloudParser.get_point_cloud())
        
        self.fiducials_dict = fiducials_dict
        self.car_fiducial_num = car_fiducial_num

        # List of fiducials belonging to cars that are NOT the actively parking car.
        self.obstacle_fiducials_list = [value for key, value in fiducials_dict.items() if key != car_fiducial_num]
        
        # Update the occupancy grid with the footprint of obstacles.
        # (TODO: Save this as a mask and refresh it in the update cycle.)
        self.og.update_grid_from_fiducials(self.obstacle_fiducials_list)
        
        # Remove the current footprint of the goal car from the occupancy grid.
        self.og.remove_car_footprint(self.fiducials_dict[car_fiducial_num])
        
        # Find the parking space that is in the top-left area (as the car enters).
        self.goal_position = self.identify_parking_space()
        if self.goal_position:
            rospy.loginfo(f"Identified goal parking spot at: {self.goal_position}")
        else:
            rospy.logwarn("No valid parking spot found!")

        # Plan an initial path.
        self.path = path_planning.plan(self.og, self.get_current_position(), self.goal_position)
        rospy.loginfo(f"Initial path planned. Number of steps: {len(self.path)}")

        self.currently_updating = False
        self.currently_executing = False
        self.reached_goal = False

        # Visualize the occupancy grid and path.
        self.visualize_path_and_og()

    def refresh(self, point_cloud, fiducials_dict):
        rospy.loginfo("Refreshing occupancy grid and planning path.")
        self.currently_updating = True
        self.fiducials_dict = fiducials_dict
        PointCloudParser = point_cloud_parser.PointCloudParser(point_cloud)
        # Update the occupancy grid from the new point cloud.
        self.og.update_grid_from_pointcloud(PointCloudParser.get_point_cloud())
        # Update the grid with obstacle cars’ footprints.
        self.og.update_grid_from_fiducials(self.obstacle_fiducials_list)
        # Remove the footprint for the actively parking car.
        self.og.remove_car_footprint(self.fiducials_dict[self.car_fiducial_num])
        # Re-plan the path (assuming the goal remains the same).
        self.path = path_planning.plan(self.og, self.get_current_position(), self.goal_position)
        rospy.loginfo(f"Path updated. New path length: {len(self.path)}")
        self.currently_updating = False

    def identify_parking_space(self):
        rospy.loginfo("Identifying parking space...")
        # Extract the relevant region of the occupancy grid.
        parking_region = self.og.grid[PARKING_STARTX:PARKING_ENDX, PARKING_STARTY:PARKING_ENDY]
        best_x, best_y = None, None

        max_x = parking_region.shape[0]
        max_y = parking_region.shape[1]

        # Loop over the parking region to find a free space.
        for x in range(int(max_x - (CAR_WIDTH / self.og.resolution) // 2 + 1)):  # forward direction
            for y in range(int(max_y - (CAR_LENGTH / self.og.resolution) // 2 + 1)):  # left-to-right
                area = parking_region[x:int(x + CAR_WIDTH / self.og.resolution),
                                      y:int(y + CAR_LENGTH / self.og.resolution)]
                if np.all(area == 0):  # All cells free
                    abs_x = x + PARKING_ENDX / self.og.resolution
                    abs_y = y + PARKING_STARTY / self.og.resolution

                    # Choose the left and forwardmost free space
                    if best_x is None or abs_x > best_x or (abs_x == best_x and abs_y < best_y):
                        best_x, best_y = abs_x, abs_y

        if best_x is not None:
            center_x = best_x + (CAR_LENGTH / self.og.resolution) / 2
            center_y = best_y + (CAR_WIDTH / self.og.resolution) / 2
            rospy.loginfo(f"Parking spot selected at grid center ({center_x}, {center_y}).")
            return og_coordinate(center_x, center_y, np.pi)
        else:
            rospy.logwarn("No valid parking spot found.")
            return None

    def run(self):
        rospy.loginfo("Starting main orchestrator run loop.")
        while not self.reached_goal and not rospy.is_shutdown():
            if not self.currently_executing and not self.currently_updating:
                # Use the latest point cloud and fiducial data.
                rospy.loginfo("Refreshing data and sending next motion command.")
                # In a real setup, these messages would be pulled from ROS topics.
                self.refresh(global_point_cloud, global_fiducials_dict)
                self.send_next_motion_command()
            else:
                rospy.loginfo("Waiting: either executing motion or updating data...")
            rospy.sleep(0.5)  # Adjust the loop timing as necessary.

    def visualize_path_and_og(self):
        rospy.loginfo("Visualizing occupancy grid and planned path.")
        grid = self.og.get_grid()
        fig, ax = plt.subplots()
        # Display the occupancy grid (free: 0 in white; occupied: darker)
        ax.imshow(grid, cmap='Greys', origin='upper')
        # Plot the path points.
        for og_cord in self.path:
            x, y = og_cord.x, og_cord.y
            ax.scatter(y, x, color='blue', s=50)  # Note: axes are (col, row)
        # Add grid lines for reference.
        ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
        ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
        ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)
        ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
        plt.show()

    def send_next_motion_command(self):
        # Replace with the code necessary to send a motion command to your platform.
        rospy.loginfo("Sending next motion command (placeholder).")
        pass

    def get_current_position(self) -> og_coordinate:
        # Get the current grid coordinate from the car’s fiducial.
        # Expected fiducials_dict format: {fiducial_num: np.array([x, y, heading])}
        car_data = self.fiducials_dict[self.car_fiducial_num]
        x, y, heading = car_data
        grid_x, grid_y = self.og.world_to_grid(
            x + TAG_X_OFFSET_FROM_CENTER * np.cos(heading),
            y + TAG_Y_OFFSET_FROM_CENTER * np.sin(heading)
        )
        rospy.loginfo(f"Current car position in grid: ({grid_x}, {grid_y}) with heading: {heading}")
        return og_coordinate(grid_x, grid_y, heading)



def point_cloud_callback(msg):
    global global_point_cloud
    global_point_cloud = msg
    rospy.loginfo("Point cloud received.") #returns the point cloud

def fiducial_callback(msg):
    global global_fiducials_dict
    # Iterate over detections in the message.
    for detection in msg.detections:
        if detection.id:
            fiducial_num = detection.id[0]
            pos = detection.pose.pose.pose.position
            quat = detection.pose.pose.pose.orientation
            euler = tf_conversions.transformations.euler_from_quaternion( #quat to euler angles
                        [quat.x, quat.y, quat.z, quat.w]
                    )
            yaw = euler[2]
            # returning position as a NumPy array: [x, y, heading]
            global_point = np.array([pos.x, pos.y, yaw])
            global_fiducials_dict[fiducial_num] = global_point
            rospy.loginfo(f"Fiducial {fiducial_num} updated: {global_point}")

def main():
    rospy.init_node("parking_orchestrator_node")
    rospy.loginfo("Parking orchestrator node started.")
    
    # Subscribers for the point cloud and AprilTag detections.
    rospy.Subscriber("/stitched_pointcloud", PointCloud2, point_cloud_callback)
    rospy.Subscriber("/apriltag/detections", AprilTagDetectionArray, fiducial_callback)
    
    rate = rospy.Rate(10)
    rospy.loginfo("Waiting for initial data (point cloud and fiducial detections)...")

    while not rospy.is_shutdown():
        if global_point_cloud is not None and len(global_fiducials_dict) > 0:
            rospy.loginfo("Initial point cloud and fiducials received. Starting orchestration.")
            car_fiducial_num = rospy.get_param("~car_fiducial_num", 1)
            orchestrator = parking_orchestrator(global_point_cloud, car_fiducial_num, global_fiducials_dict)
            orchestrator.run()
            break
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Parking orchestrator node terminated.")
