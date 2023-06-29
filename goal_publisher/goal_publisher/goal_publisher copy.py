#!/usr/bin/python3

from logging import info
import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from example_interfaces.msg import Int32
from nav_msgs.msg import OccupancyGrid,Odometry
import numpy as np
import matplotlib.pyplot as plt
import time  # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2


class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")
        self.robot_name = "goal_publisher"
        # self.number = 1
        self.map_info = self.create_subscription(OccupancyGrid,"global_costmap/costmap",self.log_occupancy,1)
        self.odom_info = self.create_subscription(Odometry,"/odom",self.log_odom,1)
        # self.number_publisher = self.create_publisher(Int32,"number",10)
        # self.timer = self.create_timer(1.0/self.pub_freq,self.publish_number)
        self.navigator =  BasicNavigator
        self.get_logger().info("Started Goal Publisher")
        self.cost_map_resolution = 0.1
        self.map_height = 223
        self.map_width = 378
        self.mid_x = (self.map_width)//2
        self.mid_y = (self.map_height)//2
        self.x = 0
        self.y = 0
    def log_occupancy(self,msg):
        a =  self.convert_to_2d_array(msg.data,msg.info.width,msg.info.height)
        self.get_logger().info(f"Width: ,{msg.info.width,}Height: , {msg.info.height}")
        self.mid_x = msg.info.width//2
        self.mid_y = msg.info.height//2
        # plt.imshow(a, cmap='hot', interpolation='nearest')
        # plt.show()
        return
    def log_odom(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self._logger.info(f"x: {self.x}, y: {self.y}")
        a,b  = self.current_location_in_map(self.x,self.y)
        print(a-self.mid_x,"    ",b-self.mid_y)
        # self.get_logger().info(f"x = {msg.pose.pose.position.x} y = {msg.pose.pose.position.y}")
    def plot_map(self, x,y):
        plt.imshow(self.map, cmap='hot', interpolation='nearest')
        plt.plot(self.pltpoints,marker=".", markersize=2)
        plt.show()

    def occupancy_callback(self,msg):
        self.mid_x = msg.info.width//2
        self.mid_y = msg.info.height//2
        self.map_x, self.map_y = self.xy_to_map(self.x,self.y)
        self.cost_map_resolution = msg.info.resolution
        # self.get_logger().info(f"Updated Map")
        self.map = self.convert_to_2d_array(msg.data,msg.info.width,msg.info.height)
        self.map_received = True
        return
    
    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
                
    def convert_to_2d_array(self,coordinates, width, height):
        # Create a NumPy array from the coordinate list
        array_1d = np.array(coordinates)
        # Reshape the 1D array to a 2D array
        array_2d = np.reshape(array_1d, (height, width), order='C')
        return np.flipud(array_2d)
    
    def xy_to_map(self,x,y):
        x = int(self.mid_x+(x//self.cost_map_resolution))
        y = int(self.mid_y-(y//self.cost_map_resolution))
        return [x,y]
    

    def map_to_xy(self,mx,my):
        return[(mx-self.mid_x)*self.cost_map_resolution, (my - self.mid_y)*self.cost_map_resolution]



def main():
    rclpy.init(args = None)
    node  = GoalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
