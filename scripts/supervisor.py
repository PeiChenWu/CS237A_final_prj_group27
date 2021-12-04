#!/usr/bin/env python3

from enum import Enum
#import warnings
import os

import rospy
from asl_turtlebot.msg import DetectedObject
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np

# Suppress warning
#warnings.filterwarnings("ignore", message="TF_REPEATED_DATA ignoring data with redundant timestamp ")


class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


class SupervisorParams:

    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = rospy.get_param("sim")

        # How is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = rospy.get_param("rviz")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = rospy.get_param("map")

        # Threshold at which we consider the robot at a location
        self.pos_eps = rospy.get_param("~pos_eps", 0.1)
        self.theta_eps = rospy.get_param("~theta_eps", 0.3)

        # Time to stop at a stop sign
        self.stop_time = rospy.get_param("~stop_time", 5.)

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 0.5)

        # Time taken to cross an intersection
        self.crossing_time = rospy.get_param("~crossing_time", 3.)


        if verbose:
            print("SupervisorParams:")
            print("    use_gazebo = {}".format(self.use_gazebo))
            print("    rviz = {}".format(self.rviz))
            print("    mapping = {}".format(self.mapping))
            print("    pos_eps, theta_eps = {}, {}".format(self.pos_eps, self.theta_eps))
            print("    stop_time, stop_min_dist, crossing_time = {}, {}, {}".format(self.stop_time, self.stop_min_dist, self.crossing_time))
            


class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.params = SupervisorParams(verbose=True)

        # Current state
        self.x = 0
        self.y = 0
        self.theta = 0

        # Goal state
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # Current mode
        self.mode = Mode.IDLE
        self.prev_mode = None  # For printing purposes
        
        # Add a dictionary for object database - MHO
        self.object_db = {}
	
        # User index to object id database
        self.usr_idx_to_obj = {
            51: "bowl",
            11: "fire_hydrant",
            13: "stop_sign",
            74: "mouse",
            10: "traffic_light"
        }
        
        self.publisher_dict = {
	        51: rospy.Publisher('/obj_detected/bowl', Pose2D, queue_size=10),
	        11: rospy.Publisher('/obj_detected/fire_hydrant', Pose2D, queue_size=10),
	        13: rospy.Publisher('/obj_detected/stop_sign', Pose2D, queue_size=10),
	        74: rospy.Publisher('/obj_detected/mouse', Pose2D, queue_size=10),
	        10: rospy.Publisher('/obj_detected/traffic_light', Pose2D, queue_size=10)
	    }
        self.objs_to_be_rescued = []

	# Waypoints
        basedir = '/home/group27/catkin_ws/src/asl_turtlebot/scripts/'
        self.fname=os.path.join(basedir, "waypts.txt")
        print(self.fname)
        self.waypts = []

        ########## PUBLISHERS ##########

        # Command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)

        # Command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # MARKER PUBLISHER
       

        ########## SUBSCRIBERS ##########
        
        # OBJECT LIST SUBSCRIBER (WILL NEED THIS FOR RESCUE PHASE)
        
	# rescue command receiver # we can publish a string to /rescue_cmd, then inside the callback, we can create the object list and set the state to rescue state
        rospy.Subscriber('/rescue_cmd', String, self.rescue_cmd_callback)
        
        # bowl detector
        rospy.Subscriber('/detector/bowl', DetectedObject, self.obj_detected_callback)

        # tv detector
        rospy.Subscriber('/detector/fire_hydrant', DetectedObject, self.obj_detected_callback)

        # mouse detector
        rospy.Subscriber('/detector/mouse', DetectedObject, self.obj_detected_callback)
        
        # Stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        # traffic light 
        rospy.Subscriber('/detector/traffic_light', DetectedObject, self.obj_detected_callback)

        # High-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        
        #Listen for TF frames
        self.trans_listener = tf.TransformListener()

        self.origin_frame = "/map" if self.params.mapping else "/odom"
        self.trans_listener.waitForTransform(self.origin_frame, '/base_camera', rospy.Time(), rospy.Duration(5))
        
        # If using gazebo, we have access to perfect state
        if self.params.use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()

        # If using rviz, we can subscribe to nav goal click
        if self.params.rviz:
            #  STILL WORKING ON THIS - MINNIE & PEICHEN
            #self.use_waypt_flg=True
            #print("Waypt_flg", self.use_waypt_flg)
            #if (self.use_waypt_flg):
            #    waypts = np.genfromtxt(r'savewaypts.txt', delimiter=' ')
            #    print("Waypt_flg is true", waypts[0])
            #    self.x_g, self.y_g, self.theta_g = waypts[0]   
            #    self.mode = Mode.NAV

            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        else:
            self.x_g, self.y_g, self.theta_g = 1.5, -4., 0.
            self.mode = Mode.NAV
        
        
    def add_object_to_dict(self, dist, cl, obj_loc):
	
        if cl in self.object_db.keys():
            if abs(dist-self.object_db[cl][0])<0.1 and (dist<self.object_db[cl][0]):
                self.object_db[cl]  = [dist, (self.x, self.y, self.theta), (obj_loc)]
                print('update existing item')
                print(self.object_db)
                pose_obj_msg = Pose2D()
                pose_obj_msg.x = obj_loc[0]
                pose_obj_msg.y = obj_loc[1]
                self.publisher_dict[cl].publish(pose_obj_msg)	
        else:
            self.object_db[cl]  = [dist, (self.x, self.y, self.theta), (obj_loc)]
            print('add new item')
            print(self.object_db)
            pose_obj_msg = Pose2D()
            pose_obj_msg.x = obj_loc[0]
            pose_obj_msg.y = obj_loc[1]
            self.publisher_dict[cl].publish(pose_obj_msg)	

    ########## SUBSCRIBER CALLBACKS ##########
    
    def rescue_cmd_callback(self, msg):
        rospy.loginfo(msg)
        obj_idx = [s.strip() for s in msg.data.split(',')]
        
        for idx in obj_idx:
            if idx in self.usr_idx_to_obj :
                obj_id, obj_name = self.usr_idx_to_obj[idx]
                rospy.loginfo('Adding ' + obj_name + " to rescue list")
                self.objs_to_be_rescued.append(obj_id)
            else:
                rospy.logwarn('Cannot find object mapping for index: ' + str(idx))
	
    def obj_detected_callback(self,  msg):
        cl  = msg.id
        
        d = msg.distance
        thetaleft = msg.thetaleft
        thetaright = msg.thetaright
        
        #position of the object in the camera frame
        theta = thetaright + (thetaleft - thetaright) / 2
        x_c = d * np.cos(theta) 
        y_c = d * np.sin(theta)
        z_c = 0
        
        #print(f'object in camera frame @ x:{x_c}, y:{y_c}, z: {z_c}')
        #print(f'robot @ x:{self.x}, y:{self.y}, theta:{self.theta}')
        
        #camera frame -> map frame
        try:
            #(trans, rot) = self.trans_listener.lookupTransform(self.origin_frame, '/base_camera', rospy.Time(0))
           
            #quaternion = rot
            #rpy = tf.transformations.euler_from_quaternion(quaternion)
            
            #theta_z = rpy[2]
            
            trans = [self.x, self.y, 0]
            theta_z = self.theta + theta
            
            T_MC = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0, trans[0]], 
            [np.sin(theta_z),  np.cos(theta_z), 0, trans[1]], 
            [0,                              0, 1, trans[2]], 
            [0,                              0, 0,        1],
            ])
             
            x_c_t = np.array([x_c, y_c, z_c, 1])
            
            x_m = T_MC @ x_c_t # object in map co-ordinates
            
            x_m = x_m[0:3]
            
            self.add_object_to_dict(d, cl, x_m)
            #print(f'thetaleft:{thetaleft}, thetaright:{thetaright}')
            #print(f'trans: {trans}')
            #print(f'rot: {rpy}' )
            #print(f'test theta: {self.theta + theta}')
            #print(f'object id: {msg.id}')
            #print(f'object in map frame @ {x_m}')
            # if close enough and in nav mode, stop
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
        #print("Gazebo initial point")
        #print(self.x, self.y, self.theta)

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if self.params.mapping else "/odom"
        print("Rviz command received!")

        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (nav_pose_origin.pose.orientation.x,
                          nav_pose_origin.pose.orientation.y,
                          nav_pose_origin.pose.orientation.z,
                          nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
            
            print("We should append to the waypts file")
            print(self.x_g, self.y_g, self.theta_g)
            self.waypts.append((self.x_g, self.y_g, self.theta_g))

            np.savetxt(self.fname, np.array(self.waypts), delimiter=' ')

            #with open("waypts.txt", "a") as wayptfile:
            #    wayptfile.write('Waypoint\n')
            #    wayptfile.write(' '.join('{} {} {}'.format(self.x_g, self.y_g, self.theta_g)))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance


        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.params.stop_min_dist and self.mode == Mode.NAV:
            obj_detected_callback(self,  msg)
            #self.add_object_to_dict(dist, msg.id)
            self.init_stop_sign()

    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """

        return abs(x - self.x) < self.params.pos_eps and \
               abs(y - self.y) < self.params.pos_eps and \
               abs(theta - self.theta) < self.params.theta_eps

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return self.mode == Mode.STOP and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.params.stop_time)

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return self.mode == Mode.CROSS and \
               rospy.get_rostime() - self.cross_start > rospy.Duration.from_sec(self.params.crossing_time)

    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" if self.params.mapping else "/odom"
                translation, rotation = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x, self.y = translation[0], translation[1]
                self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current mode: %s", self.mode)
            self.prev_mode = self.mode

        ########## Code starts here ##########
        # TODO: Currently the state machine will just go to the pose without stopping
        #       at the stop sign.
        
        # EXPLORATION PHASE (INTEGRATE WITH CURRENT STATE MACHINE SHOULD BE SUFFICIENT)
        if self.mode == Mode.IDLE:
            # Send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # Moving towards a desired pose
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # At a stop sign
            if self.has_stopped():
                self.mode = Mode.CROSS
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # Crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose() # just go forward

        elif self.mode == Mode.NAV: 
            
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.POSE
            else:
                self.nav_to_pose()

        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))
            
       # RESCUE (WILL BE SIMILAR TO EXPLORATION HARD-CODED WAY POINTS, BUT COMMAND Robot position based on object list provided by TA and utilize the previous saved dictionary)
            
            

        ############ Code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
