from occupancy_grid import OccupancyGrid, og_coordinate
from long_path_planning import path_to_driveable, AXIS_OF_ROTATION_FROM_CENTER_Y
from simulate_path import create_sim
import time
from std_msgs.msg import String, PointCloud2

import matplotlib.pyplot as plt

import numpy as np
import rospy
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


#Coordinates of the area previously designated to be the goal area for parking
#which is a subset of the entire space the system covers
#This is hardcoded and is the area that will be used to find a valid parking spot
PARKING_STARTX =  .9#m
PARKING_STARTY = .3 #m
PARKING_ENDX = 1.43 #m
PARKING_ENDY =  .8#m


#Dimensions of the car used for overlaying and removing footprint of the car form the apriltag
CAR_WIDTH = .14 #m
CAR_LENGTH = .335 #m
TAG_X_OFFSET_FROM_CENTER = 0 #m how far the center of the car is located from the tag center
TAG_Y_OFFSET_FROM_CENTER = .07

#This is the distance from the origin of the space to the origin of the apriltag that is used to
#stitch the parking process
X_origin_offset = .45
Y_origin_offset = .69

#These are the values that will be fetched from ros
global_point_cloud = None
global_fiducials_dict = {}   # maps tag_id -> np.array([x, y, yaw])
global_tag_list = [] 
global_sensor_data = {}



#on collinear points separate the distance by the turn radius that we are capable of
def og_distance(og_coordinate1:og_coordinate, og_coordinate2:og_coordinate):
        #distance between two og_coordinate objects
        return np.sqrt((og_coordinate1.x - og_coordinate2.x)**2 + (og_coordinate1.y - og_coordinate2.y)**2)

def point_cloud_callback(msg):
    global global_point_cloud
    points = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
    global_point_cloud = np.array(list(points))
    global_point_cloud[:, 0] += X_origin_offset
    global_point_cloud[:,1] += Y_origin_offset
    
tf_buffer   = None
tf_listener = None  

def pico_callback(data):
    global global_sensor_data  # Use the global variable here
    # Log received data
    rospy.loginfo(f"Received data: {data.data}")

    # Parse the data string to extract front, back, and left values
    try:
        # Data format is "Front: X cm, Back: Y cm, Left: Z cm"
        parts = data.data.split(", ")
        front = float(parts[0].split(": ")[1].replace(" cm", ""))
        back = float(parts[1].split(": ")[1].replace(" cm", ""))
        left = float(parts[2].split(": ")[1].replace(" cm", ""))
        rospy.loginfo(f"left: {left} back: {back} front: {front}")
        # Update the global sensor_data dictionary
        global_sensor_data = {
            'frontleft': front/100,
            'frontright': back/100,
            'left': left/100  # Adjust this according to the correct context for 'heading'
        }


    except Exception as e:
        rospy.logerr(f"Failed to parse data: {e}")
   


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

    
    def __init__(self, point_cloud, car_fiducial_num, fiducials_dict, sensor_data):
        global global_sensor_data, global_point_cloud, global_fiducials_dict

        self.car_fiducial_num = car_fiducial_num
        self.fiducials_dict = global_fiducials_dict
        self.point_cloud = global_point_cloud
        self.sensor_data = global_sensor_data
        self.goal_position = None
        self.non_relevant_tags = set([0,14,16])
        self.planned_short = False
        self.og = None
        self.PointCloudParser = None
        self.path = None
        rospy.loginfo("\nsensor data: " + str(self.sensor_data))

    def run_live(self, use_pc=True, use_onboard = False, use_pc_only_once = False):
        #This runs the system with the following options
        #use_pc: use the point cloud data
        #use_onboard: use the onboard data
        #use_pc_only_once: use the point cloud data only once and then overwrite it with the onboard data
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        
        refresh_pointcloud_on_iteration = True
        self.refresh(use_pc = use_pc, use_onboard = use_onboard)

        if use_pc_only_once:
            refresh_pointcloud_on_iteration = False
        
        self.grid_img = self.ax.imshow(
            self.og.get_grid(), cmap='Greys', origin='lower'
        )
        self.footprint_scatter = self.ax.scatter([], [], color='red',   s=5)
        self.goal_scatter      = self.ax.scatter([], [], color='green', s=5)
        self.path_scatter      = self.ax.scatter([], [], color='blue',  s=50)
        self.sensor_lines = {}
        self.sensor_lines = {}
        for name in ("frontleft", "frontright", "left"):
            line, = self.ax.plot([], [], 'o-',
                                markerfacecolor='orange',
                                markeredgecolor='orange',
                                markersize=5,
                                linewidth=1,
                                color='orange'  )
            self.sensor_lines[name] = line

        x1, xn = int(PARKING_STARTX/self.og.resolution), (PARKING_ENDX/self.og.resolution)
        y1, yn = int(PARKING_STARTY/self.og.resolution), (PARKING_ENDY/self.og.resolution)
        self.parking_scatter  = self.ax.scatter(
            [x1,x1,xn,xn], [y1,yn,y1,yn], color='orange', s=50
        )


        while True:
            self.refresh(use_pc = use_pc, use_onboard = use_onboard, refresh_iteration_wtih_pc = refresh_pointcloud_on_iteration)
            if self.goal_position != None:
                rospy.loginfo("Planning Path")
                self.path = long_path_planning.plan(self.og, self.get_front_wheel_position(), self.goal_position)
                
            else:
                rospy.loginfo("COULD NOT FIND A GOAL POSITION")
                self.path = None
            self.update_constant_visual(use_pc=use_pc, show_path=True, show_sensors=use_onboard)

    def run_simulation(self):
        #Gets path from the current position and creates a simulation video of the vehicle following the course
        self.refresh()
        
        self.visualize_path_and_og('grid', False, True)
        rospy.loginfo("starting path plan for goal: " + str(self.goal_position))
        self.path = long_path_planning.plan(self.og, self.get_front_wheel_position(), self.goal_position)
        self.visualize_path_and_og('full_grid_and_path', True, True)

        if self.path == None:
            rospy.loginfo("path_plan_failed")
            return
        else:
            rospy.loginfo("running simulation")
            create_sim(self.og, self.goal_position, self.path)

    def refresh(self, use_onboard = False, refresh_iteration_wtih_pc = True):
        #This function refreshes the occupancy grid and the point cloud
        #use_onboard: use the onboard data
        #refresh_iteration_wtih_pc: use the current occupancy grid and overwrite it with the onboard data
        refresh_start= time.perf_counter()#start timer
    
        self.fiducials_dict = global_fiducials_dict
        
        if refresh_iteration_wtih_pc:
            self.point_cloud = global_point_cloud
        else:
            self.point_cloud= []

        if refresh_iteration_wtih_pc:
            self.PointCloudParser = point_cloud_parser.PointCloudParser(self.point_cloud)
            self.og = OccupancyGrid(self.PointCloudParser.get_point_cloud(), resolution=.01)
            self.obstacle_fiducials_list = [value for key, value in self.fiducials_dict.items() if key != self.car_fiducial_num and key not in self.non_relevant_tags]
            self.og.update_grid_from_fiducials(self.obstacle_fiducials_list)

            
            self.og.remove_car_footprint(self.fiducials_dict[self.car_fiducial_num])

        #TODO
        if use_onboard:
            self.sensor_data = global_sensor_data
            rospy.loginfo("SENSORD DATA: " + str(self.sensor_data))
            if self.goal_position == None:
                if "left" in self.sensor_data.keys():
                    del self.sensor_data["left"]
                    del self.sensor_data["frontright"]
                    del self.sensor_data["frontleft"]
            elif np.sqrt((self.goal_position.x - self.get_current_grid_position().x)**2 + (self.goal_position.y - self.get_current_grid_position().y)**2) > .5/self.og.resolution:
                if "left" in self.sensor_data.keys():
                    del self.sensor_data["left"]
                    del self.sensor_data["frontright"]
                    del self.sensor_data["frontleft"]

            self.og.update_grid_from_onboard(self.sensor_data, self.get_current_world_position())

        self.goal_position = self.identify_parking_space()
        refresh_end = time.perf_counter()#start timer
        
        rospy.loginfo(f"\n\n\nRefreshing Occupancy took {(refresh_end - refresh_start):.3f} seconds\n\n")

    def identify_parking_space(self):
        #Identitfies open space within the predefined parking area
        rows = math.ceil(CAR_LENGTH / self.og.resolution)
        cols = math.ceil(CAR_WIDTH  / self.og.resolution)

        min_x = int(PARKING_STARTX / self.og.resolution)
        max_x = int(PARKING_ENDX   / self.og.resolution)
        min_y = int(PARKING_STARTY / self.og.resolution)
        max_y = int(PARKING_ENDY   / self.og.resolution)
        goal_heading = round(-np.pi,1)
        for y in range(min_y, max_y - rows + 1):
            for x in reversed(range(min_x, max_x - cols + 1)):
                area = self.og.grid[y : y + rows, x : x + cols]
                if np.all(area == 0):
                    cx = x + cols//2
                    cy = y + rows//2
                    h = np.round(-np.pi,1)

                    front_wheel_x, front_wheel_y = self.og.world_to_grid(cx*self.og.resolution - AXIS_OF_ROTATION_FROM_CENTER_Y * np.sin(goal_heading), cy*self.og.resolution+ AXIS_OF_ROTATION_FROM_CENTER_Y * np.cos(goal_heading))
                    return_cord = og_coordinate(front_wheel_x, front_wheel_y, h)
                    return return_cord

        return None

    def update_constant_visual(self, show_path=False, use_pc = True, show_sensors=False):
        #Updates the live visual according to the parameters
        if use_pc:
            self.grid_img.set_data(self.og.get_grid())
        else:
            self.og.grid = np.zeros(self.og.get_grid().shape)
            self.obstacle_fiducials_list = [value for key, value in self.fiducials_dict.items() if key != self.car_fiducial_num and key not in self.non_relevant_tags]
            self.og.update_grid_from_fiducials(self.obstacle_fiducials_list)
            self.grid_img.set_data(self.og.get_grid())

        if show_path and self.path != None:
            rospy.loginfo("FINAL PATH IS:" +  str(self.path))
            xs=[]
            ys=[]
            for og_cord in self.path:
                x,y = og_cord.x, og_cord.y
                xs.append(og_cord.x)
                ys.append(og_cord.y)
            path_pts = np.column_stack([xs, ys])
            self.path_scatter.set_offsets(path_pts)

        gridx,gridy = self.og.get_car_footprint_from_tag_pos(self.fiducials_dict[self.car_fiducial_num], res_increase=.25)
        footprint_pts = np.column_stack([gridx, gridy])
        self.footprint_scatter.set_offsets(footprint_pts)


        if(self.goal_position != None):
            #current goal position describes front wheel location, need it to describe tag locations
            world_center_x = self.goal_position.x*self.og.resolution + (AXIS_OF_ROTATION_FROM_CENTER_Y * np.sin(self.goal_position.heading))
            world_center_y = self.goal_position.y*self.og.resolution - (AXIS_OF_ROTATION_FROM_CENTER_Y * np.cos(self.goal_position.heading))
            center_world_pos = np.array([world_center_x, world_center_y, self.goal_position.heading])
        
            gridx,gridy = self.og.get_car_footprint_from_center(center_world_pos, res_increase=.25)
            goal_pts = np.column_stack([gridx, gridy])
            self.goal_scatter.set_offsets(goal_pts)

        if show_sensors:
            for name, line in self.sensor_lines.items():
                if name not in self.sensor_data:
                    line.set_data([], [])  

            for name, item in self.sensor_data.items():
                x_offset = 0
                y_offset = 0
                heading_change = 0

                if name == 'left':
                    x_offset = -.08
                    y_offset = .03
                    heading_change = np.pi/2 +np.pi/2
                elif name == 'frontleft':
                    x_offset = -.0575
                    y_offset = .18
                    heading_change = np.pi/6 +np.pi/2
                elif name == "frontright":
                    x_offset = .0575
                    y_offset = .18
                    heading_change = -np.pi/6 +np.pi/2

                car_center = self.get_current_world_position()
                cx = car_center[0] + x_offset*np.cos(car_center[2]) - y_offset*np.sin(car_center[2])
                cy =car_center[1] + x_offset*np.sin(car_center[2]) + y_offset*np.cos(car_center[2])
                ch = car_center[2] + heading_change 

                contact_x = (cx + (item) * np.cos(ch))
                contact_y = (cy + (item) * np.sin(ch))
                

                sensor_grid_x = int(np.round(cx / self.og.resolution))
                sensor_grid_y = int(np.round(cy / self.og.resolution))

                contact_grid_x = int(np.round(contact_x / self.og.resolution))
                contact_grid_y = int(np.round(contact_y / self.og.resolution))

                line = self.sensor_lines.get(name)
                if line:
                    if item <0:
                        line.set_data([], [])  
                    else:
                        line.set_data(
                            [sensor_grid_x, contact_grid_x],
                            [sensor_grid_y, contact_grid_y]
                        )

        self.fig.canvas.draw_idle()
        plt.pause(0.01)

    def visualize_path_and_og(self, file_name, show_path = False, show_current=False, show_sensor=False ):
        #Save a snapshot of the current occupancy grid and path to a png
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
            gridx,gridy = self.og.get_car_footprint_from_tag_pos(self.fiducials_dict[self.car_fiducial_num], res_increase=.25)
            for i in range(len(gridy)):
                # rospy.loginfo(i)
                x,y = gridx[i], gridy[i]
                ax.scatter(x, y, color='red', s=5)

        rospy.loginfo("past adding current")
        # rospy.loginfo("past show current")
        if(self.goal_position != None):
            #current goal position describes front wheel location, need it to describe tag locations
            world_center_x = self.goal_position.x*self.og.resolution + (AXIS_OF_ROTATION_FROM_CENTER_Y * np.sin(self.goal_position.heading))
            world_center_y = self.goal_position.y*self.og.resolution - (AXIS_OF_ROTATION_FROM_CENTER_Y * np.cos(self.goal_position.heading))
            center_world_pos = np.array([world_center_x, world_center_y, self.goal_position.heading])
        
            gridx,gridy = self.og.get_car_footprint_from_center(center_world_pos, res_increase=.25)
            
            for i in range(len(gridy)):
                x,y = gridx[i], gridy[i]
                ax.scatter(x, y, color='green', s=5)

        
        rospy.loginfo("past show goal")
        x1, xn = int(PARKING_STARTX/self.og.resolution), (PARKING_ENDX/self.og.resolution)
        y1, yn = int(PARKING_STARTY/self.og.resolution), (PARKING_ENDY/self.og.resolution)
    
        ax.scatter(x1, y1, color='orange', s=50)
        ax.scatter(x1, yn, color='orange', s=50)
        ax.scatter(xn, y1, color='orange', s=50)
        ax.scatter(xn, yn, color='orange', s=50)
        
        plt.savefig(file_name + ".png")
        

    def get_current_grid_position(self) -> og_coordinate:
        #Get the grid position of the car's center from the fiducial position and known tag offset
        car_x, car_y, car_heading = self.fiducials_dict[self.car_fiducial_num]
        grid_x, grid_y = self.og.world_to_grid(car_x - TAG_Y_OFFSET_FROM_CENTER * np.sin(car_heading), car_y + TAG_Y_OFFSET_FROM_CENTER * np.cos(car_heading))


        return og_coordinate(grid_x, grid_y, self.fiducials_dict[self.car_fiducial_num][2])
    
    def get_front_wheel_position(self) -> og_coordinate:
        #Get the grid position of the car's front wheel(its axis of rotation) from the fiducial position and known tag offset
        car_x, car_y, car_heading = self.fiducials_dict[self.car_fiducial_num]
        offset = TAG_Y_OFFSET_FROM_CENTER + AXIS_OF_ROTATION_FROM_CENTER_Y
        grid_x, grid_y = self.og.world_to_grid(car_x - offset * np.sin(car_heading), car_y + offset * np.cos(car_heading))
        
        return og_coordinate(grid_x, grid_y, self.fiducials_dict[self.car_fiducial_num][2])

    def get_current_world_position(self): 
        #Get the grid position of the car's center from the fiducial position and known tag offset
        car_x, car_y, car_heading = self.fiducials_dict[self.car_fiducial_num]
        car_x_center = car_x - TAG_Y_OFFSET_FROM_CENTER * np.sin(car_heading)
        car_y_center = car_y + TAG_Y_OFFSET_FROM_CENTER * np.cos(car_heading)
        return np.array([car_x_center, car_y_center, car_heading])

def main():
    global tf_buffer, tf_listener, global_point_cloud, global_tag_list, global_fiducials_dict, global_sensor_data

    # 1) Initialize ROS node
    rospy.init_node("parking_orchestrator_node")
    rospy.loginfo("Parking orchestrator node started.")

    # 2) Create TF2 buffer & listener *after* init_node
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 3) Subscribe to topics
    rospy.Subscriber("/stitched_pointcloud", PointCloud2, point_cloud_callback)
    rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=10)
    rospy.Subscriber("/pico_data", String, pico_callback)

    # 4) Read car tag ID (default = 2)
    car_fiducial_num = rospy.get_param("~car_fiducial_num", 1)

    rate = rospy.Rate(10)
    rospy.loginfo("Waiting for initial data (point cloud and fiducial detections)...")

    # 5) Block until we have both a point cloud and at least one tag
    orchestrator_made = False
    while not rospy.is_shutdown():
        if global_point_cloud is not None and global_tag_list and orchestrator_made is False:
            rospy.loginfo("Initial data received. Starting orchestration.")
            # Pass the dict, not the list, into your orchestrator
            orchestrator = parking_orchestrator(
                global_point_cloud,
                car_fiducial_num,
                global_fiducials_dict, 
                global_sensor_data
            )

            orchestrator.run_live(use_pc=True, use_onboard = True, use_pc_only_once=True)
            orchestrator_made = True
            break
        rate.sleep()
    

if __name__ == "__main__":
    main()
