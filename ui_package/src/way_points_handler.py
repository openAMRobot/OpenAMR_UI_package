#!/usr/bin/env python3

import rospy
import actionlib
import rospkg
import csv 
import time
import tf
import math
import yaml

from std_srvs.srv import Empty as Emp

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance, Quaternion

from std_msgs.msg import Empty, String, Bool, Float32
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.msg import Odometry, Path

PI = math.pi

class WayPointMover:
    def __init__(self):
        self.waypoints = []
        self.current_wp = 0
        self.on_the_route = False
        self.break_mission = False
        self.current_files = f"{rospkg.RosPack().get_path('ui_package')}/param/current_map_route.yaml"

        self.charge_connected = False
        self.current_pos = PoseWithCovariance()
        self.home_pose = PoseWithCovariance()
        # Initialization of node
        rospy.init_node('Way_points_handler')

        # Topics subscribers
        rospy.Subscriber( "ui_operation", String, self.ui_operation_callback)
        rospy.Subscriber(rospy.get_param("odom_topic", "odom"), Odometry, self.odom_callback)
        self.needCheckCharger = rospy.get_param("needCheckCharger", False)
        if self.needCheckCharger:
            rospy.Subscriber(rospy.get_param("charge_station_connected_topic", "charge_station_connected"), Bool, self.charge_station_calback)

        # Topics publishers
        self.ui_message_pub = rospy.Publisher("/ui_message" , String, queue_size=1)
        self.poseArray_publisher = rospy.Publisher("/WPs_topic", PoseArray, queue_size=1)
        
        # Actions 
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('Connecting to move_base...')
        self.move_base_client.wait_for_server()
        rospy.loginfo('CONNECTED to move_base')
        

        rospy.loginfo('Connected to move_base.')
        
        rospy.loginfo("------------ Way points handler started ------------")       
        

    def charge_station_calback(self, data):
        self.charge_connected = data.data
        
        self.ui_message_pub.publish(f"Robot connected to charge station: {self.charge_connected}")
        rospy.loginfo(f"Robot connected to charge station: {self.charge_connected}")  

        if self.charge_connected:        
            self.home_pose.pose.position = self.current_pos.pose.position
            self.home_pose.pose.orientation = self.current_pos.pose.orientation

       
    def get_cur_files(self):
        with open(self.current_files, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)  
        return data

    def read_wp(self):
        route_file = self.get_cur_files()["route_file"]

        del self.waypoints[:]
        with open(route_file, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            for line in reader:
                current_pose = PoseWithCovarianceStamped()
                current_pose.pose.pose.position.x = float(line[0])
                current_pose.pose.pose.position.y = float(line[1])
                current_pose.pose.pose.position.z = float(line[2])  # instead z coord use type of point
                current_pose.pose.pose.orientation.x = float(line[3])
                current_pose.pose.pose.orientation.y = float(line[4])
                current_pose.pose.pose.orientation.z = float(line[5])
                current_pose.pose.pose.orientation.w = float(line[6])
                current_pose.pose.covariance[0] = float(line[7])
                current_pose.pose.covariance[1] = float(line[8])
                current_pose.pose.covariance[2] = float(line[9])
                point_type = float(line[7])
                purpouse = float(line[10])

                self.waypoints.append((current_pose, point_type, purpouse)) 

        if self.waypoints == []:
            self.ui_message_pub.publish("The waypoint queue is empty.")


    def odom_callback(self, data):
        self.current_pos.pose = data.pose.pose
 
    def follow_func(self):
        if self.needCheckCharger and not self.charge_connected:
            self.ui_message_pub.publish(f"Robot can't start. Please connect to charge station")
            rospy.loginfo(f"Robot can't start. Please connect to charge station")  
            return
        
        if self.on_the_route:
            self.ui_message_pub.publish(f"The robot on the route already")
            rospy.loginfo(f"The robot on the route already") 
            return
        
        self.break_mission = False
        self.on_the_route = True
        self.current_wp = 0

        self.read_wp()
        points_amount = len(self.waypoints)

        for index, (waypoint, point_type, purpouse) in enumerate(self.waypoints):
            
            if self.break_mission:
                self.ui_message_pub.publish(f"break_mission")
                break
            self.ui_message_pub.publish("Following to "+str(index % points_amount+1)+" waypoint...")

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id =  "map" 
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation

            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()

            self.current_wp = index+1

            if not self.break_mission:
                self.ui_message_pub.publish(f"Doing some action #{purpouse} on {index+1} point")
            time.sleep(0.1)  
               
        time.sleep(1)        
        self.ui_message_pub.publish("Current route was successfully completed")
        self.on_the_route = False

    def next_wp_func(self): 
        rospy.loginfo("Following to next waypoint...") 
        self.ui_message_pub.publish("Following to next waypoint...") 

        self.read_wp()
            
        self.current_wp +=1

        if(len(self.waypoints) == 0): 
            self.ui_message_pub.publish("The waypoint queue is empty.")
            return
        
        if(self.current_wp > len(self.waypoints)):  
            self.current_wp = 1
        
        self.ui_message_pub.publish("Following to "+str(self.current_wp)+" waypoint...")
        point, pointType, purpouse = self.waypoints[self.current_wp-1]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" 
        goal.target_pose.pose.position = point.pose.pose.position
        goal.target_pose.pose.orientation = point.pose.pose.orientation

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()  
        time.sleep(0.1)    
        
        self.ui_message_pub.publish(f"Point #{self.current_wp} successfully reached")
        self.ui_message_pub.publish(f"Doing some action #{purpouse} on {self.current_wp} point")

    def previous_wp_func(self):
        rospy.loginfo("Following to previous waypoint...")
        self.ui_message_pub.publish("Following to previous waypoint...") 

        self.read_wp()
            
        self.current_wp -=1

        if(len(self.waypoints) == 0): 
            self.ui_message_pub.publish("The waypoint queue is empty.")
            return
        
        if(self.current_wp > len(self.waypoints)):  
            self.current_wp = 1
        
        self.ui_message_pub.publish("Following to "+str(self.current_wp)+" waypoint...")
        point, pointType, purpouse = self.waypoints[self.current_wp-1]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" 
        goal.target_pose.pose.position = point.pose.pose.position
        goal.target_pose.pose.orientation = point.pose.pose.orientation

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()  
        time.sleep(0.1)    
        
        self.ui_message_pub.publish(f"Point #{self.current_wp} successfully reached")
        self.ui_message_pub.publish(f"Doing some action #{purpouse} on {self.current_wp} point")

    def home_func(self):
        rospy.loginfo("Following to home position")
        self.ui_message_pub.publish("Following to home position")
                
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id =  "map" 
        goal.target_pose.pose.position = self.home_pose.pose.position
        new_orientation = self.chang_point_dir(self.home_pose.pose.orientation)
        goal.target_pose.pose.orientation.x =  new_orientation[0]
        goal.target_pose.pose.orientation.y =  new_orientation[1]
        goal.target_pose.pose.orientation.z =  new_orientation[2]
        goal.target_pose.pose.orientation.w =  new_orientation[3]

        self.move_base_client.send_goal(goal) 
        self.move_base_client.wait_for_result() 
        
    def stop_func(self):
        rospy.loginfo("Canceling current route")
        self.ui_message_pub.publish(f"Canceling current route")

        self.break_mission = True
        try:
            if self.move_base_client.get_state() == actionlib.GoalStatus.ACTIVE:
                self.move_base_client.cancel_goal()

            if self.move_base_client.get_state() == actionlib.GoalStatus.PREEMPTED:
                print("Goal was canceled.")
        except Exception as e:  
            rospy.loginfo(f"ERROR in stop_func {e}")  

    def function(self, typeFunc):
        self.ui_message_pub.publish(f"Function {typeFunc} will be done")
        # Add realizations of your functions
        
    def ui_operation_callback(self, data):
        print(f"ui_operation_callback : {data.data}")
        
        if(data.data == "follow_route") or (data.data == "start"):
            self.follow_func()                 
                                    
        elif data.data == "next_point":  
            self.next_wp_func()              
                                    
        elif data.data == "previous_point":  
            self.previous_wp_func()              
                                    
        elif data.data == "home":  
            self.home_func()           
                                    
        elif data.data == "stop": 
            self.stop_func()     
               
        else :
            command = data.data.split("_")
            if command[0] == "function":
                self.function(command[1])    

if(__name__ == "__main__"):
    try:
        controller = WayPointMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass