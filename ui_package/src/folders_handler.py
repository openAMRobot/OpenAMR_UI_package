#!/usr/bin/env python3

import rospy
import rospkg
import os
import shutil
import json
import time
import subprocess
import yaml
import csv 

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from ui_package.msg import ArrayPoseStampedWithCovariance
from nav_msgs.srv import LoadMap


class UIFoldersHandler:
    def __init__(self):
        os.system("rosnode kill /map_server")

        self.WPs = []
        self.waypoints = []
        self.position = 0

        # Initialization of node
        rospy.loginfo("------------ UI folders handler started ------------")
        rospy.init_node('ui_folders', anonymous=False)
        
        rospy.Subscriber(rospy.get_param("odom_topic", "/odom"), Odometry, self.odom_callback)
        
        rospy.Subscriber("ui_operation", String, self.ui_callback)
        rospy.Subscriber("/new_way_point", PoseWithCovarianceStamped, self.new_way_point_callback)
        rospy.Subscriber("/nav_data_req", Empty, self.nav_data_callback)
        rospy.Subscriber("WP_req", Empty, self.WP_req_callback)
        
        self.ui_pub = rospy.Publisher('ui_message', String, queue_size=1)
        self.poseArray_publisher = rospy.Publisher("WayPoints_topic", ArrayPoseStampedWithCovariance, queue_size=1)
        self.set_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.nav_data_pub = rospy.Publisher('nav_data_resp', String, queue_size=1)

        self.maps_folder = f"{rospkg.RosPack().get_path('ui_package')}/maps"
        self.routs_folder = f"{rospkg.RosPack().get_path('ui_package')}/paths"
        self.current_files = f"{rospkg.RosPack().get_path('ui_package')}/param/current_map_route.yaml"
        
        self.mappingCmd = rospy.get_param("mappingLaunch", "")
        self.navigationCmd = rospy.get_param("navigationLaunch", "")
        
        self.dict_cmd = None
        
        # TODO
        run_map_server_command = f"roslaunch ui_package map_server.launch map_file:={self.get_cur_files()['map_file']}"  
        print("--------------------------------------------")      
        print(run_map_server_command)      
        print("--------------------------------------------")      
        subprocess.Popen(run_map_server_command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        # --------

        rospy.loginfo('Connecting to map_server...')
        rospy.wait_for_service('/change_map')
        rospy.loginfo('CONNECTED to map_server')
        self.change_map_service = rospy.ServiceProxy('/change_map', LoadMap)
        
        
    def odom_callback(self, data):
        self.position = data.pose.pose

    def nav_data_callback(self, data):
        time.sleep(0.5)
        self.nav_data_pub.publish(self.get_paths())

    def new_way_point_callback(self, data):
      
        line =  f"{data.pose.pose.position.x},"
        line += f"{data.pose.pose.position.y},"
        line += f"{data.pose.pose.position.z},"
        line += f"{data.pose.pose.orientation.x},"
        line += f"{data.pose.pose.orientation.y},"
        line += f"{data.pose.pose.orientation.z},"
        line += f"{data.pose.pose.orientation.w},"
        line += f"{data.pose.covariance[0]},"
        line += f"{data.pose.covariance[1]},"
        line += f"{data.pose.covariance[2]}" 

        if(line not in self.WPs):
            self.WPs.append(line)

        if (len(self.WPs) == 1):
            self.ui_pub.publish("1 waypoint added to the route")
        else:
            self.ui_pub.publish(str(len(self.WPs)) + " waypoints added to the route")

    def convert_PoseArray(self, waypoints):
        poses = PoseArray()
        poses.header.frame_id = 'map'
        poses.poses = [pose.pose.pose for pose, purpose in waypoints]
        return poses

    def convert_PoseWithCovArray_to_PoseArrayCov(self, waypoints):
        poses = ArrayPoseStampedWithCovariance()
        for pose_arg, purpose in waypoints:       
            poses.poses.append(pose_arg)
        return poses

    def WP_req_callback(self, data):
        time.sleep(0.5)

        self.read_wp()  
        self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
 
    def read_wp(self):
        route_file = self.get_cur_files()["route_file"]
        
        del self.waypoints[:]
        if len(route_file) != 0:
            with open(route_file, 'r') as file:
                reader = csv.reader(file, delimiter=',')
                for line in reader:
                    current_pose = PoseWithCovarianceStamped()
                    current_pose.header.frame_id = 'map'
                    current_pose.pose.pose.position.x = float(line[0])
                    current_pose.pose.pose.position.y = float(line[1])
                    current_pose.pose.pose.position.z = float(line[2])
                    current_pose.pose.pose.orientation.x = float(line[3])
                    current_pose.pose.pose.orientation.y = float(line[4])
                    current_pose.pose.pose.orientation.z = float(line[5])
                    current_pose.pose.pose.orientation.w = float(line[6])
                    current_pose.pose.covariance[0] = float(line[7])
                    current_pose.pose.covariance[1] = float(line[8])
                    current_pose.pose.covariance[2] = float(line[9])

                    self.waypoints.append((current_pose, float(line[10]))) 

        if self.waypoints == []:
            self.ui_pub.publish("The waypoint queue is empty.")
    
    def get_paths(self):
        files = {}
        for group in os.listdir(self.routs_folder):
            files[group] = []
            for i in os.listdir(f"{self.routs_folder}/{group}"):
                files[group].append({i: os.listdir(f"{self.routs_folder}/{group}/{i}")})
        
        data = self.get_cur_files()  
        if len(data["route_file"]) != 0:
            data = data["route_file"].split("/")[-3:]
            group, map, route = data[0], data[1], data[2].split(".")[0]
        elif len(data["map_file"]) != 0:
            data = data["map_file"].split("/")[-2:]
            group, map, route = data[0], data[1].split(".")[0], "Null"
        else:
            group, map, route = "Null", "Null", "Null"
        
        response = {"structure":[], "active_files":{"group":group,"map":map,"route":route}}

        for i, j in files.items():
            response["structure"].append({i: j})
        response = str(response).replace('\'', '\"')
        return response

# ----- UI CALLBACKS -----
    def build_map_func(self):
        try:
            self.poseArray_publisher.publish(ArrayPoseStampedWithCovariance())
            self.ui_pub.publish("Mapping...")
            os.system("rosnode kill /amcl")
            os.system("rosnode kill /move_base")
            os.system("rosnode kill /map_server")
            time.sleep(1)
            
            subprocess.Popen(f"roslaunch {self.mappingCmd}", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)
            time.sleep(3)
            self.ui_pub.publish("Move the robot along the perimeter of the room and in the "
                                "center using the control buttons and return robot to start position")
        except Exception as e:
            rospy.loginfo(f"Error in build_map_func: {e}")

    def save_map_func(self):
        try:            
            self.ui_pub.publish("Saving map...")
            map_path_to_save = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_folder_path_to_save = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"

            map_save_command = "rosrun map_server map_saver -f "
            cmd = f"{map_save_command}{map_path_to_save}"

            # self.ui_pub.publish(cmd)  
            rospy.loginfo(cmd)                  
            os.system(cmd)
            
            os.mkdir(route_folder_path_to_save)
            self.set_cur_route("")
            self.WP_req_callback(Empty())
            self.ui_pub.publish("Map saved")                

            # 1. save new current map to file
            map_yaml_file = f"{map_path_to_save}.yaml"
            self.set_cur_map(map_path_to_save)
            rospy.loginfo(f"Current map was setted to {map_yaml_file}")  

            # 2. save curent position
            current = self.position

            # 3. reload
            os.system("rosnode kill /slam_gmapping")
            os.system("rosnode kill /map_server")

            self.ui_pub.publish("Localization...")
            time.sleep(1)

            subprocess.Popen(f"roslaunch {self.navigationCmd}", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)
            
            rospy.loginfo("running saved map")  
            # self.ui_pub.publish("running saved map")               
            command = f"roslaunch ui_package map_server.launch map_file:={map_yaml_file}"

            subprocess.Popen(command, stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)

            time.sleep(3)

            # 3. initial pose -> saved pos
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.frame_id = "map"
            current_pose.pose.pose = current  
            # self.ui_pub.publish("current_pose")  

            # self.set_robot_pose.publish(current) 
            self.set_pose.publish(current_pose)
            self.ui_pub.publish("You can set points or follow the route")


            self.nav_data_pub.publish(self.get_paths())

        except Exception as e:
            rospy.loginfo(f"Error in save_map_func: {e}")

    def change_map_func(self):
        try:                

            path_to_new_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            map_yaml_file = f"{path_to_new_map}.yaml"
            self.set_cur_map(path_to_new_map)

            routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}")
            
            if len(routes_on_map) != 0:
                path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}"
                self.set_cur_route(path_to_route)
            else:
                self.set_cur_route("")
                self.ui_pub.publish("No routes on the map")
            self.WP_req_callback(Empty())

            rospy.loginfo(path_to_new_map) 
            
            
            print(f"service change map {map_yaml_file}")
            print(self.change_map_service)
            print(self.change_map_service(map_yaml_file))
            
            # command = f"rosservice call /change_map 'map_url: '{map_yaml_file}'"
            # subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in change_map_func: {e}")

    def create_group_func(self):
        try:
            os.mkdir(f"{self.routs_folder}/{self.dict_cmd['group']}")                
            os.mkdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in create_group_func: {e}")

    def rename_map_func(self):
        try:
            old_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"
            new_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}" 

            old_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"  
            new_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}" 

            map_cur_path = None
            with open(f"{old_map_file}.yaml", 'r') as file:
                map_cur_path = yaml.load(file, Loader=yaml.FullLoader) 

            map_cur_path["image"] = f"{new_map_file}.pgm"

            with open(f"{old_map_file}.yaml", 'w') as file:
                yaml.dump(map_cur_path, file)

            os.rename(f"{old_map_file}.yaml", f"{new_map_file}.yaml")
            os.rename(f"{old_map_file}.pgm", f"{new_map_file}.pgm")

            os.rename(old_route_folder_file, new_route_folder_file)
            
            data = self.get_cur_files()
            if data["map_file"] == f"{old_map_file}.yaml":
                self.set_cur_map(new_map_file)
                self.set_cur_route(f"{new_route_folder_file}/{data['route_file'].split('/')[-1].split('.')[0]}")
            
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in rename_map_func: {e}")

    def delete_map_func(self):
        try:
            map_to_delete = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_map_folder_to_delete = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"   

            os.remove(f"{map_to_delete}.yaml")
            os.remove(f"{map_to_delete}.pgm")

            shutil.rmtree(route_map_folder_to_delete)
            
            os.system("rosnode kill /map_server")  
            time.sleep(0.5)  
            maps_in_group = os.listdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            if len(maps_in_group) != 0:
                path_to_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}"
                self.set_cur_map(path_to_map)                
                rospy.loginfo(f"Curr map: {path_to_map}")

                subprocess.Popen(f"roslaunch ui_package map_server.launch map_file:={path_to_map}.yaml", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)

                routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}")   
                rospy.loginfo(f"routes_on_map: {routes_on_map}")
                
                if len(routes_on_map) != 0:
                    path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}"
                    self.set_cur_route(path_to_route) 
                    rospy.loginfo(f"Curr route: {path_to_route}")
                else:
                    self.set_cur_route("")
                    self.ui_pub.publish("No routes on the map")

            else:
                self.set_cur_route("")
                self.set_cur_map("")
                self.ui_pub.publish("No maps in the group")
            self.WP_req_callback(Empty())         
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in delete_map_func: {e}")

    def delete_group_func(self):
        try:
            group_to_delete = f"{self.maps_folder}/{self.dict_cmd['group']}"
            route_group_folder_to_delete = f"{self.routs_folder}/{self.dict_cmd['group']}"
            
            os.system("rosnode kill /map_server")  
            time.sleep(0.5) 
            
            shutil.rmtree(group_to_delete)
            shutil.rmtree(route_group_folder_to_delete)

            groups_in_folder = os.listdir(f"{self.routs_folder}")   
            rospy.loginfo(f"groups_in_folder: {groups_in_folder}")

            if len(groups_in_folder) != 0:
                path_to_map = f"{self.maps_folder}/{groups_in_folder[0].split('.')[0]}"

                maps_in_group = os.listdir(path_to_map)
                if len(maps_in_group) != 0:
                    path_to_map = f"{path_to_map}/{maps_in_group[0].split('.')[0]}"
                    self.set_cur_map(path_to_map)                
                    rospy.loginfo(f"Curr map: {path_to_map}")

                    subprocess.Popen(f"roslaunch ui_package map_server.launch map_file:={path_to_map}.yaml", stdout=subprocess.PIPE,
                                shell=True, preexec_fn=os.setsid)

                    routes_on_map = os.listdir(f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}")   
                    rospy.loginfo(f"routes_on_map: {routes_on_map}")
                    
                    if len(routes_on_map) != 0:
                        path_to_route = f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}"
                        self.set_cur_route(path_to_route) 
                        rospy.loginfo(f"Curr route: {path_to_route}")
                    else:
                        self.set_cur_route("")
                        self.ui_pub.publish("No routes on the map")

                else:
                    self.set_cur_map("")
                    self.ui_pub.publish("No maps in the group")
            else:
                self.set_cur_map("")
                self.ui_pub.publish("No maps in the group")
            self.WP_req_callback(Empty()) 

            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in delete_group_func: {e}")

    def clear_route_func(self):
        try:
            del self.WPs[:]
            self.ui_pub.publish("Waypoints cleared, please set new poits on the map")
        except Exception as e:
            rospy.loginfo(f"Error in clear_route_func: {e}")

    def save_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}"

            # if(len(self.WPs) > 0):
            with open(f"{path_to_route}.csv", 'w') as file:
                for WP in self.WPs:
                    pos = WP + ", 1"
                    file.write(pos + '\n')
            self.ui_pub.publish(str(len(self.WPs)) + " waypoints saved")
            rospy.loginfo(f"{len(self.WPs)} waypoints saved")

            self.set_cur_route(path_to_route)

            # else:
            #     self.ui_pub.publish("Please, set at least 1 wayPoint")

            del self.WPs[:]
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in save_route_func: {e}")

    def edit_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route'].split('.')[0]}"
            self.set_cur_route(path_to_route)                

            self.read_wp()           
            for point, purpose in self.waypoints:
                if purpose == 2: continue
                line =  f"{point.pose.pose.position.x},{point.pose.pose.position.y},{point.pose.pose.position.z},{point.pose.pose.orientation.x},{point.pose.pose.orientation.y},{point.pose.pose.orientation.z},{point.pose.pose.orientation.w},{point.pose.covariance[0]},{point.pose.covariance[1]},{point.pose.covariance[2]}"

                if(line not in self.WPs):
                    self.WPs.append(line)
            
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in edit_route_func: {e}")

    def delete_route_func(self):
        try:
            file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}.csv"
            
            os.remove(file) 
            routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}")
            if len(routes_on_map) != 0:
                path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}"
                self.set_cur_route(path_to_route)
            else:
                self.set_cur_route("")
                self.ui_pub.publish("No routes on the map")

            self.WP_req_callback(Empty())
            self.nav_data_pub.publish(self.get_paths())  
               
        except Exception as e:
            rospy.loginfo(f"Error in delete_route_func: {e}")

    def change_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}" 
            self.set_cur_route(path_to_route) 
            self.read_wp()       
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints)) 
            self.nav_data_pub.publish(self.get_paths())        
        except Exception as e:
            rospy.loginfo(f"Error in change_route_func: {e}")

    def rename_route_func(self):
        try:
            old_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_old']}.csv" 
            new_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_new']}.csv" 

            os.rename(old_file, new_file)
            self.set_cur_route(new_file.split(".")[0]) 
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            rospy.loginfo(f"Error in rename_route_func: {e}")
   
    def ui_callback(self, data):
        rospy.loginfo("ui_callback")

        try:
            command = data.data.split("/") 
            rospy.loginfo(command)

            if len(command) > 1:  
                self.dict_cmd = json.loads(command[1])

            if command[0] == "build_map":
                rospy.loginfo("build_map")
                self.build_map_func()                
                                    
            elif command[0] == "save_map":  
                rospy.loginfo("build_map") 
                self.save_map_func()

            elif command[0] == "change_map":
                rospy.loginfo("change_map")
                self.change_map_func() 

            elif command[0] == "create_group":
                rospy.loginfo("create_group") 
                self.create_group_func()    

            elif command[0] == "rename_map":
                rospy.loginfo("rename_map") 
                self.rename_map_func() 

            elif command[0] == "delete_map":
                rospy.loginfo("delete_map")
                self.delete_map_func() 

            elif command[0] == "delete_group":
                rospy.loginfo("delete_group")
                self.delete_group_func()  

            elif command[0] == "clear_route": 
                rospy.loginfo("clear_route")   
                self.clear_route_func()  

            elif command[0] == "save_route":
                rospy.loginfo("save_route")
                self.save_route_func()

            elif command[0] == "edit_route":
                self.edit_route_func()
                rospy.loginfo("edit_route")

            elif command[0] == "delete_route":
                rospy.loginfo("delete_route")
                self.delete_route_func() 

            elif command[0] == "change_route":
                rospy.loginfo("change_route")
                self.change_route_func()

            elif command[0] == "rename_route":
                rospy.loginfo("rename_route")
                self.rename_route_func()

            elif command[0] == "rename_group":
                rospy.loginfo("rename_group")
                self.rename_group_func()

        except Exception as e:
            rospy.logerr(f"Error in ui_callback: {e}")
    
# -----------------------------------------------------------
    def set_cur_map(self, map_name):
        data = self.get_cur_files()  
        if len(map_name) == 0:
            data["map_file"] = ""
        else:
            data["map_file"] = f"{map_name}.yaml"
            
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)
     
    def set_cur_route(self, route_name):
        data = self.get_cur_files()
        if len(route_name)  == 0:
            data["route_file"] = ""
        else:
            data["route_file"] = f"{route_name}.csv"        
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)  
  
    def get_cur_files(self):
        with open(self.current_files, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)  
        return data
 

if (__name__ == "__main__"):

    try:
        controller = UIFoldersHandler()        
        rospy.spin()

    except rospy.ROSInterruptException:
        print("rospy.ROSInterruptException")
