from occupancy_grid import OccupancyGrid, og_coordinate
from long_path_planning import path_to_driveable, AXIS_OF_ROTATION_FROM_CENTER_Y
import time
FAR_PARK_Y_OFFSET = 0.2 #m
#from dataclasses import dataclass
#from shapely.geometry import Polygon, Point
#from shapely.affinity import rotate, translate
import matplotlib.pyplot as plt

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from apriltag_ros.msg import AprilTagDetectionArray
import long_path_planning
import point_cloud_parser
import open3d as o3d
import tf_conversions
import math
from tf2_msgs.msg import TFMessage
import tf2_ros
import tf_conversions


PARKING_STARTX =  .9#m
PARKING_STARTY = .3 #m
PARKING_ENDX = 1.43 #m
PARKING_ENDY =  .8#m

CAR_WIDTH = .14 #m
CAR_LENGTH = .335 #m

TAG_X_OFFSET_FROM_CENTER = 0 #m how far the center of the car is located from the tag center
TAG_Y_OFFSET_FROM_CENTER = .07 #actually 6.87 but not divisible by grid resolution

X_origin_offset = .45
Y_origin_offset = .69

global_point_cloud = None
global_fiducials_dict = {}   # maps tag_id -> np.array([x, y, yaw])
global_tag_list = [] 


#use fiducials to recreate space cars are occupying
#grow the moving particle rather than the obstacles

#on collinear points separate the distance by the turn radius that we are capable of
def og_distance(og_coordinate1:og_coordinate, og_coordinate2:og_coordinate):
        #distance between two og_coordinate objects
        return np.sqrt((og_coordinate1.x - og_coordinate2.x)**2 + (og_coordinate1.y - og_coordinate2.y)**2)

def point_cloud_callback(msg):
    global global_point_cloud
    points = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
    # points = -points
    # rospy.loginfo(points)
    global_point_cloud = np.array(list(points))
    global_point_cloud[:, 0] += X_origin_offset
    global_point_cloud[:,1] += Y_origin_offset
    #rospy.loginfo(global_point_cloud[0])
    # rospy.loginfo("Point cloud received.") #returns the point cloud
   

# def fiducial_callback(msg):
#     global global_fiducials_dict
#     # rospy.loginfo("Fiducial info recieved") #returns the point cloud
#     # Iterate over detections in the message.
#     for detection in msg.detections:
#         if detection.id and detection.id != 0:
#             fiducial_num = detection.id[0]
#             pos = detection.pose.pose.pose.position
#             quat = detection.pose.pose.pose.orientation
#             euler = tf_conversions.transformations.euler_from_quaternion( #quat to euler angles
#                         [quat.x, quat.y, quat.z, quat.w]
#                     )
#             yaw = euler[2]
#             # returning position as a NumPy array: [x, y, heading]
#             global_point = np.array([pos.x, pos.y, yaw])
#             global_fiducials_dict[fiducial_num] = global_point
#             # rospy.loginfo(f"Fiducial {fiducial_num} updated: {global_point}")

tf_buffer   = None
tf_listener = None  


# def tf_callback(msg: TFMessage):
#     global global_fiducials_dict, global_tag_list

#     # Process each incoming TransformStamped
#     for t in msg.transforms:
#         child = t.child_frame_id
#         if not child.startswith("tag_"):
#             continue

#         try:
#             tag_id = int(child.split("_", 1)[1])
#         except ValueError:
#             # frame named "tag_XYZ" but XYZ not int
#             continue

#         trans = t.transform.translation
#         rot   = t.transform.rotation

#         # quaternion → euler (roll, pitch, yaw)
#         roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(
#             [rot.x, rot.y, rot.z, rot.w]
#         )

#         # store [x, y, yaw]
#         global_fiducials_dict[tag_id] = np.array([-trans.x, -trans.y, -yaw])

#     # update the list of tags seen, sorted for consistency
#     global_tag_list = sorted(global_fiducials_dict.keys())

#     # (optional) log debug
#     rospy.logdebug(f"Tags: {global_tag_list}")
#     for tid in global_tag_list:
#         pos = global_fiducials_dict[tid]
#         rospy.logdebug(f"  tag_{tid} -> x={pos[0]:.3f}, y={pos[1]:.3f}, yaw={pos[2]:.3f}")
def tf_callback(msg: TFMessage):
    global tf_buffer, global_fiducials_dict, global_tag_list

    # make sure tf_buffer is ready
    if tf_buffer is None:
        rospy.logwarn_throttle(5.0, "tf_buffer not yet initialized")
        return

    for t in msg.transforms:
        child = t.child_frame_id
        if not child.startswith("tag_"):
            continue

        try:
            tag_id = int(child.split("_", 1)[1])
        except ValueError:
            continue

        # lookup in world frame
        try:
            trans_stamped = tf_buffer.lookup_transform(
                "world",
                child,
                rospy.Time(0),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logdebug(f"can't lookup world->{child}: {e}")
            continue

        t = trans_stamped.transform.translation
        r = trans_stamped.transform.rotation

        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(
            [r.x, r.y, r.z, r.w]
        )
        global_fiducials_dict[tag_id] = np.array([t.x+X_origin_offset, t.y+Y_origin_offset, yaw])

    global_tag_list = sorted(global_fiducials_dict.keys())
    

class parking_orchestrator:

    
    def __init__(self, point_cloud, car_fiducial_num, fiducials_dict):
        
        #point_cloud: combined 03d point cloud of the environment
        #fiducials_dict: dictionary of fiducials with their positions as 3D numpy array, should include the goal car
        t_og_0 = time.perf_counter()
        rospy.loginfo("\nEntered parking orchestrator init\n" )
        self.currently_updating = True
        PointCloudParser = point_cloud_parser.PointCloudParser(point_cloud)
        rospy.loginfo("\n Pointcloud Parsed \n" )
        self.goal_position = None
        self.sensor_dict = None
        
        self.og = OccupancyGrid(PointCloudParser.get_point_cloud())

        self.fiducials_dict = fiducials_dict
        self.car_fiducial_num = car_fiducial_num
        rospy.loginfo("\n FIDUCIALS ADDED \n" )

        #self.visualize_path_and_og(False, False)
        rospy.loginfo("\n Pointcloud Assigned to Occupancy Grid \n" )
        #self.visualize_path_and_og(False, True)
        
        self.visualize_path_and_og("1:initial_og", False, False)
        
        #CARS THAT ARE NOT THE ACTIVELY PARKING CAR
        non_relevant_tags = set([0,14,16])
        self.obstacle_fiducials_list = [value for key, value in fiducials_dict.items() if key != car_fiducial_num and key not in non_relevant_tags]
        
        # self.visualize_path_and_og("2:other_car_fiducials_overlayed", False, False)
        #FILLS IN FOOTPRINT OF CARS FROM THEIR FIDUCIALS 
        rospy.loginfo("\n GRID UPDATED WITH FIDUCIALS\n" )
        self.og.update_grid_from_fiducials(self.obstacle_fiducials_list)
        #TODO THIS ISNT WORKING WELL
        self.visualize_path_and_og("2:other_car_fiducials_overlayed", False, False)
        # self.visualize_path_and_og(False, False)
        # self.visualize_path_and_og(False, True)
        
        # return
        #REMOVES THE FOOTPRINT PRESENT IN THE POINTCLOUD FROM OUR GOAL CAR
        rospy.loginfo("current grid position: " + str(self.get_current_grid_position()))
        rospy.loginfo("current world position: " + str(self.get_current_world_position()))

        self.og.remove_car_footprint(self.fiducials_dict[car_fiducial_num])
        self.visualize_path_and_og("3:moving_car_removed", False, False)
        # self.visualize_path_and_og(False, False)
        #FIND THE PARKING SPACE THAT IS TOP LEFT AS THE CAR ENTERS
        #THIS IS AN og_coordinate object
        self.goal_position = self.identify_parking_space()
        self.goal_position.y -= FAR_PARK_Y_OFFSET/self.og.resolution#OFFSET FROM WHERE WE WANT TO APPROACH THE CLOSE PARK

        

        
        rospy.loginfo("\n GOAL POSITION CALCULATED: " + str(self.goal_position)+"\n" )
        self.visualize_path_and_og("4:current_and_goal", False, True)

        # np.savetxt("occupancy_grid.csv", self.og.grid, fmt="%d", delimiter=",")

        rospy.loginfo("current is: "+ str(self.get_current_grid_position()))
        rospy.loginfo("goal is: "+ str(self.goal_position))
        
        t_plan_0 = time.perf_counter()
        self.path = long_path_planning.plan(self.og, self.get_front_wheel_position(), self.goal_position)
        t_plan_1 = time.perf_counter()
        if self.path == None:
            rospy.loginfo("path_plan_failed")
            return

        self.next_instructions = path_to_driveable(self.path,self.og)
        rospy.loginfo("\n NEXT MOTION COMMAND: heading_change" + str(self.next_instructions[0]) + "distance change: " + str(self.next_instructions[1])+"\n")
        
        rospy.loginfo("\n PATH PLANNED:" +str(self.path)+"\n" )
        rospy.loginfo(f"Occupancy Grid Building took {(t_plan_0 - t_og_0):.3f} seconds")
        rospy.loginfo(f"Planning took {(t_plan_1 - t_plan_0):.3f} seconds")
        self.currently_updating = False
        self.currently_executing = False
        self.reached_goal = False

        if self.path is not None:
            self.visualize_path_and_og("5:grid_with_self_and_path", True, True)

    def refresh(self, point_cloud, fiducials_dict, sensor_dict=None):
        #point_cloud: combined 03d point cloud of the environment
        #fiducials_dict: dictionary of fiducials with their positions as 3D numpy array, should include the goal car
        
        #COULD UPDATE GOAL BUT IT SHOULDN'T CHANGE UNLESS THE CAR MOVES
        #self.goal_position = self.identify_parking_space()

        if self.get_current_grid_position() is not None and og_distance(self.get_current_grid_position(), self.goal_position) < 10*self.og.resolution:
            self.currently_updating = True
            self.fiducials_dict = fiducials_dict
            PointCloudParser = point_cloud_parser.PointCloudParser(point_cloud)
            self.og.update_grid_from_pointcloud(PointCloudParser.get_point_cloud())
            self.og.update_grid_from_fiducials(self.obstacle_fiducials_list)
            self.og.remove_car_footrospy.loginfo(self.fiducials_dict[self.car_fiducial_num])
            self.path = long_path_planning.plan(self.og, self.get_current_grid_position(), self.goal_position)
        else:
            self.currently_updating = True
            self.fiducials_dict = fiducials_dict

            PointCloudParser = point_cloud_parser.PointCloudParser(point_cloud)
            self.og = OccupancyGrid(PointCloudParser.get_point_cloud(), .1)
            self.og.update_grid_from_pointcloud(PointCloudParser.get_point_cloud())
            self.og.update_grid_from_fiducials(self.obstacle_fiducials_list)
            self.og.remove_car_footrospy.loginfo(self.fiducials_dict[self.car_fiducial_num])
            self.og.update_grid_from_onboard(sensor_dict)
            self.currently_updating = False


    def identify_parking_space(self):
        rows = math.ceil(CAR_LENGTH / self.og.resolution)
        cols = math.ceil(CAR_WIDTH  / self.og.resolution)

        min_x = int(PARKING_STARTX / self.og.resolution)
        max_x = int(PARKING_ENDX   / self.og.resolution)
        min_y = int(PARKING_STARTY / self.og.resolution)
        max_y = int(PARKING_ENDY   / self.og.resolution)

        for y in range(min_y, max_y - rows + 1):
            for x in reversed(range(min_x, max_x - cols + 1)):
                area = self.og.grid[y : y + rows, x : x + cols]
                if np.all(area == 0):
                    cx = x + cols//2
                    cy = y + rows//2
                    h = np.round(-np.pi,1)
                    return_cord = og_coordinate(cx, cy, h)
                    return return_cord

        return None

    def run(self):
        while(self.reached_goal == False):

            #PULL CURRENTLY EXCECUTING INFORMATION FROM ROS
            if self.currently_executing == False and self.currently_updating == False:


                #PULL THIS INFORMATION FROM ROS
                #self.refresh(point_cloud, fiducials_dict)


                
                
                self.send_next_motion_command()

            else:
                #check if the motion command has been completed
                continue

        
    def visualize_path_and_og(self, file_name, show_path = False, show_current=False, ):
        
        #visualize the occupancy grid and the path
        # rospy.loginfo("Entered Visualizer")
        fig, ax = plt.subplots()
        rospy.loginfo(global_fiducials_dict)
        # Display the grid
        ax.imshow(self.og.get_grid(), cmap='Greys', origin='lower')
        #flipped_og = np.flip(self.og.get_grid(), axis=0)
        # Highlight specific coordinates
        if show_path:
            rospy.loginfo("FINAL PATH IS:" +  str(self.path))
            for og_cord in self.path:
                x,y = og_cord.x, og_cord.y
                ax.scatter(x, y, color='blue', s=50)  # Note: (x, y) -> (row, col)
        # rospy.loginfo("past show path")

        if show_current:
            gridx,gridy = self.og.get_car_footprint(self.fiducials_dict[self.car_fiducial_num], res_increase=.25)
            # rospy.loginfo("got footprint")
            # rospy.loginfo(len(gridy))
            for i in range(len(gridy)):
                # rospy.loginfo(i)
                x,y = gridx[i], gridy[i]
                ax.scatter(x, y, color='red', s=5)

        rospy.loginfo("past adding current")
        # rospy.loginfo("past show current")
        if(self.goal_position != None):
            gridx,gridy = self.og.get_car_footprint([self.goal_position.x*self.og.resolution, self.goal_position.y*self.og.resolution, self.goal_position.heading], res_increase=.25)
            
            for i in range(len(gridy)):
                x,y = gridx[i], gridy[i]
                ax.scatter(x, y, color='green', s=5)

        
        rospy.loginfo("past show goal")
        x1, xn = int(PARKING_STARTX/self.og.resolution), (PARKING_ENDX/self.og.resolution)
        y1, yn = int(PARKING_STARTY/self.og.resolution), (PARKING_ENDY/self.og.resolution)
    
        ax.scatter(x1, y1, color='yellow', s=50)
        ax.scatter(x1, yn, color='yellow', s=50)
        ax.scatter(xn, y1, color='yellow', s=50)
        ax.scatter(xn, yn, color='yellow', s=50)
        
        plt.savefig(file_name + ".png")
        
    def send_next_motion_command(self):
        pass

    def get_current_grid_position(self) -> og_coordinate:
        #Get the grid coordinate pertaining to car's april tag
        
        #update with fiducial wrt car body

        car_x, car_y, car_heading = self.fiducials_dict[self.car_fiducial_num]
        grid_x, grid_y = self.og.world_to_grid(car_x - TAG_Y_OFFSET_FROM_CENTER * np.sin(car_heading), car_y + TAG_Y_OFFSET_FROM_CENTER * np.cos(car_heading))


        return og_coordinate(grid_x, grid_y, self.fiducials_dict[self.car_fiducial_num][2])
    
    def get_front_wheel_position(self) -> og_coordinate:
        #Get the grid coordinate pertaining to car's april tag
        #update with fiducial wrt car body

        #PREVIOUS
        #car_x, car_y, car_heading = self.fiducials_dict[self.car_fiducial_num]
        #grid_x, grid_y = self.og.world_to_grid(car_x - TAG_Y_OFFSET_FROM_CENTER * np.sin(car_heading), car_y + TAG_Y_OFFSET_FROM_CENTER * np.cos(car_heading))
        #CHANGED
        car_x, car_y, car_heading = self.fiducials_dict[self.car_fiducial_num]
        grid_x, grid_y = self.og.world_to_grid(car_x - AXIS_OF_ROTATION_FROM_CENTER * np.sin(car_heading), car_y + AXIS_OF_ROTATION_FROM_CENTER * np.cos(car_heading))
        

        return og_coordinate(grid_x, grid_y, self.fiducials_dict[self.car_fiducial_num][2])

    def get_current_world_position(self):
        #Get the world coordinate of the car's center
        
        #update with fiducial wrt car body

        car_x, car_y, car_heading = self.fiducials_dict[self.car_fiducial_num]
        car_x_center = car_x - TAG_Y_OFFSET_FROM_CENTER * np.sin(car_heading)
        car_y_center = car_y + TAG_Y_OFFSET_FROM_CENTER * np.cos(car_heading)
        return np.array([car_x_center, car_y_center, car_heading])

def main():
    global tf_buffer, tf_listener, global_point_cloud, global_tag_list, global_fiducials_dict

    # 1) Initialize ROS node
    rospy.init_node("parking_orchestrator_node")
    rospy.loginfo("Parking orchestrator node started.")

    # 2) Create TF2 buffer & listener *after* init_node
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 3) Subscribe to topics
    rospy.Subscriber("/stitched_pointcloud", PointCloud2, point_cloud_callback)
    rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=10)

    # 4) Read car tag ID (default = 2)
    car_fiducial_num = rospy.get_param("~car_fiducial_num", 3)

    rate = rospy.Rate(10)
    rospy.loginfo("Waiting for initial data (point cloud and fiducial detections)...")

    # 5) Block until we have both a point cloud and at least one tag
    while not rospy.is_shutdown():
        if global_point_cloud is not None and global_tag_list:
            rospy.loginfo("Initial data received. Starting orchestration.")
            # Pass the dict, not the list, into your orchestrator
            orchestrator = parking_orchestrator(
                global_point_cloud,
                car_fiducial_num,
                global_fiducials_dict
            )
            # If your orchestrator has a run() method, call it:
            # orchestrator.run()
            break
        rate.sleep()

if __name__ == "__main__":
    main()
