#!/usr/bin/python3

from logging import info
import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from example_interfaces.msg import Bool
from nav_msgs.msg import OccupancyGrid,Odometry
import numpy as np
import matplotlib.pyplot as plt
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import yaml
from action_msgs.msg import GoalStatus

class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.waypoint_yaml = "/home/ros2/ros2_ws/src/goal_publisher/pose_output.yaml"
        self.goal_publisher = self.create_timer(3,self.publish_goals)
        self.obstacle_sub = self.create_subscription(Bool,"/yolo/procc",self.callback_obstacle,5)

        self.map_info = self.create_subscription(OccupancyGrid,"global_costmap/costmap",self.occupancy_callback,1)
        self.odom_info = self.create_subscription(Odometry,"/odom",self.odom_callback,1)
        self.navigator = BasicNavigator()
        self.get_logger().info("Started Goal Publisher")

        self.goal_handle = None
        self.cost_map_resolution = 0.05
        self.map_height = 223
        self.map_width = 378
        self.mid_x = (self.map_width)//2
        self.mid_y = (self.map_height)//2
        self.x = 0.0
        self.y = 0.0
        self.map_x, self.map_y = 0, 0
        self.map = np.ones((2,3))*200
        self.i = 0
        self.points = []
        self.goals = []
        self.goal_time = 0
        self.goal_created =False        
        self.map_received = False
        self.feedback = None

    def publish_goals(self):
        # print("Timer Called")
        if not self.goal_created:
                self.waypoint_maker()
                print(len(self.goals))
                self.goal_created =True
                self.get_logger().info("Creating Goals..")
                self.goToPose()
                return
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self._feedbackCallback(feedback)
            return
        self.get_logger().info(f"Sending Goal {self.i}")
        self.goToPose()

        
        
    def goToPose(self, pose = None):
        # Sends a `NavToPose` action request and waits for completion
        if pose==None:
            pose = self.goals[self.i]
        self.i += 1
        self.navigator.goToPose(pose)
        return True
    
    

    def _feedbackCallback(self, msg):
        self.feedback = msg
        fs = f"Total Progress: {self.i/len(self.goals)} , Time remaining: {self.feedback.estimated_time_remaining}, Goal distance remaining: {self.feedback.distance_remaining}"
        self.get_logger().info(fs)

        return
    

    def waypoint_maker(self):
        with open(self.waypoint_yaml, 'r') as file:
            waypoints = yaml.load(file, yaml.FullLoader)
            points = len(waypoints.keys())-1
            for i in range(1,points+1):
                wp = waypoints[i]
                orientation = wp['orientation']
                pos = wp['position']
                self.goals.append(self.goal_setter(pos['x'], pos['y'], pos['z'], orientation['x'], orientation['y'], orientation['z'], orientation['w']))
        return
                

    def goal_setter(self,gx,gy,gz,ox,oy,oz,ow):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.position.z = gz
        goal_pose.pose.orientation.x = ox
        goal_pose.pose.orientation.y = oy
        goal_pose.pose.orientation.z = oz
        goal_pose.pose.orientation.w = ow
        return goal_pose


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
        

    def xy_to_map(self,x,y):
        x = int(self.mid_x+(x//self.cost_map_resolution))
        y = int(self.mid_y-(y//self.cost_map_resolution))
        return [x,y]
    
    def convert_to_2d_array(self,coordinates, width, height):
        # Create a NumPy array from the coordinate list
        array_1d = np.array(coordinates)
        # Reshape the 1D array to a 2D array
        array_2d = np.reshape(array_1d, (height, width), order='C')
        return np.flipud(array_2d)

    def map_to_xy(self,mx,my):
        return[(mx-self.mid_x)*self.cost_map_resolution, (my - self.mid_y)*self.cost_map_resolution]

    def movement_direction(self,x,y):
        if (x - self.map_x) > 0 :
            return "down"
        elif (y - self.map_y) > 0:
            return "right"
        elif (x - self.map_x) < 0:
            return "up"
        else:
            return "left"
    def in_the_path(self,n_x,n_y):
        s_x = np.sign(self.x)
        s_y = np.sign(self.y)
        if s_x == 1 and s_y == 1:
            return (n_x - self.x > 0 and n_y - self.y > 0)
        elif s_x == -1 and s_y == 1:
            return (-n_x + self.x > 0 and n_y - self.y > 0)
        elif s_x == 1 and s_y == -1:
            return (n_x - self.x > 0 and -n_y + self.y > 0)
        elif s_x == -1 and s_y == -1:
            return (-n_x + self.x > 0 and -n_y + self.y > 0)

    def callback_obstacle(self,msg):
        if (self.i < len(self.goals)):
            next_goal = self.goals[self.i+1]
            n_x = next_goal.pose.position.x 
            n_y = next_goal.pose.position.y 
            x, y = 0,0
            if self.in_the_path(n_x, n_y) :
                mx ,my = self.map_x, self.map_y
                direction = self.movement_direction(mx,my)
                if direction in ["left", "right"]:
                    if (self.map[mx+1][my] < 75):
                        x, y = self.map_to_xy(mx-1, my)
                        next_goal.pose.position.x = x
                        next_goal.pose.position.y = y
                    elif (self.map[mx-1][my] < 75):
                        x, y = self.map_to_xy(mx-1, my)
                        next_goal.pose.position.x = x
                        next_goal.pose.position.y = y
                elif direction in ["up", "down"]:
                    if (self.map[mx][my-1] < 75):
                        x, y = self.map_to_xy(mx, my-1)
                        next_goal.pose.position.x = x
                        next_goal.pose.position.y = y
                    elif (self.map[mx][my+1] < 75):
                        x, y = self.map_to_xy(mx, my+1)
                        next_goal.pose.position.x = x
                        next_goal.pose.position.y = y
                self.goToPose(next_goal)
                self.get_logger().info(f"Obstacle detected, navigating to {x} {y} from {self.x},{self.y} with next goal at {n_x} {n_y}")
        plt.imshow(self.map, cmap='hot', interpolation='nearest')
        plt.plot(self.map_x,self.map_y,marker=".", markersize=20)
        plt.show()
        return

def main():
    rclpy.init(args = None)
    node  = GoalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
