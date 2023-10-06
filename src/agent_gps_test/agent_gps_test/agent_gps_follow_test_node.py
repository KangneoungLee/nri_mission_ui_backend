# Author:
# - Kangneoung Lee
  

import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float64, Int8
import cv2
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge

import os
import math
import time

import numpy as np

from dataclasses import dataclass, field

@dataclass
class agent_info:
  enable: bool
  cur_lat: float
  cur_long: float
  goal_lat: float
  goal_long: float
  lat_inc: float
  long_inc: float

class AgentGPSTest(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """

    super().__init__('agent_gps_test_node')
    
    self.agent_gps = [[30.622867, -96.334253],
    [30.623233, -96.333278],
    [30.622753, -96.332934],
    [30.622417, -96.333424],
    [30.622713, -96.333742]]
    
    self.agent_gps_init_sim_env = [[30.6218691, -96.3345895],
    [30.6227609, -96.3345895],
    [30.6227609, -96.3335605],
    [30.6218691, -96.3335605],
    [30.622315, -96.334075]]
    
    
    self.agent_gps1 = [[30.622496738224545, -96.3341396033455],
    [30.622833438224543, -96.3345281033455],
    [30.62283987644909, -96.33452080669103],
    [30.62250317644909, -96.33413230669103],
    [30.622509614673636, -96.33412501003654]]
    
    self.agent_gps2 = [[30.622846314673634, -96.33451351003654],
    [30.62285275289818, -96.33450621338206],
    [30.62251605289818, -96.33411771338206],
    [30.622522491122723, -96.33411041672757],
    [30.62285919112272, -96.33449891672757]]
    
    self.agent_gps3 = [[30.622865629347267, -96.3344916200731],
    [30.62252892934727, -96.3341031200731],
    [30.622535367571814, -96.33409582341861],
    [30.622872067571812, -96.33448432341861],
    [30.622878505796358, -96.33447702676412]]
    
    self.publist = []
    self.iopub_0 = self.create_publisher(NavSatFix, 'Robot0/global_position', 1)
    self.publist.append(self.iopub_0)
    self.iopub_1 = self.create_publisher(NavSatFix, 'Robot1/global_position', 1)
    self.publist.append(self.iopub_1)
    self.iopub_2 = self.create_publisher(NavSatFix, 'Robot2/global_position', 1)
    self.publist.append(self.iopub_2)
    self.iopub_3 = self.create_publisher(NavSatFix, 'Robot3/global_position', 1)
    self.publist.append(self.iopub_3)
    self.iopub_4 = self.create_publisher(NavSatFix, 'Robot4/global_position', 1)
    self.publist.append(self.iopub_4)
    self.iopub_5 = self.create_publisher(NavSatFix, 'Robot5/global_position', 1)
    self.publist.append(self.iopub_5)
    self.iopub_6 = self.create_publisher(NavSatFix, 'Robot6/global_position', 1)
    self.publist.append(self.iopub_6)
    self.iopub_7 = self.create_publisher(NavSatFix, 'Robot7/global_position', 1)
    self.publist.append(self.iopub_7)
    self.iopub_8 = self.create_publisher(NavSatFix, 'Robot8/global_position', 1)
    self.publist.append(self.iopub_8)
    self.iopub_9 = self.create_publisher(NavSatFix, 'Robot9/global_position', 1)
    self.publist.append(self.iopub_9)
    
    self.sublist = []
    self.iosub_0 = self.create_subscription(NavSatFix, 'Robot0/gps_goal', lambda msg: self.gps_callback(msg, 0), 3) 
    self.sublist.append(self.iosub_0)
    self.iosub_1 = self.create_subscription(NavSatFix, 'Robot1/gps_goal', lambda msg: self.gps_callback(msg, 1), 3)
    self.sublist.append(self.iosub_1)
    self.iosub_2 = self.create_subscription(NavSatFix, 'Robot2/gps_goal', lambda msg: self.gps_callback(msg, 2), 3)
    self.sublist.append(self.iosub_2)
    self.iosub_3 = self.create_subscription(NavSatFix, 'Robot3/gps_goal', lambda msg: self.gps_callback(msg, 3), 3)
    self.sublist.append(self.iosub_3)
    self.iosub_4 = self.create_subscription(NavSatFix, 'Robot4/gps_goal', lambda msg: self.gps_callback(msg, 4), 3)
    self.sublist.append(self.iosub_4)
    self.iosub_5 = self.create_subscription(NavSatFix, 'Robot5/gps_goal', lambda msg: self.gps_callback(msg, 5), 3)
    self.sublist.append(self.iosub_5)
    self.iosub_6 = self.create_subscription(NavSatFix, 'Robot6/gps_goal', lambda msg: self.gps_callback(msg, 6), 3)
    self.sublist.append(self.iosub_6)
    self.iosub_7 = self.create_subscription(NavSatFix, 'Robot7/gps_goal', lambda msg: self.gps_callback(msg, 7), 3)
    self.sublist.append(self.iosub_7)
    self.iosub_8 = self.create_subscription(NavSatFix, 'Robot8/gps_goal', lambda msg: self.gps_callback(msg, 8), 3)
    self.sublist.append(self.iosub_8)
    self.iosub_9 = self.create_subscription(NavSatFix, 'Robot9/gps_goal', lambda msg: self.gps_callback(msg, 9), 3)
    self.sublist.append(self.iosub_9)
    
    self.agent_infos = []
    for i in range(0, 5):
      single_agent_info = agent_info(False, self.agent_gps_init_sim_env[i][0], self.agent_gps_init_sim_env[i][1], -1, -1, 0, 0)
      self.agent_infos.append(single_agent_info)
              
    timer_period = 0.2 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback_sim_env)

  def gps_callback(self, msg, robot_index):
    self.agent_infos[robot_index].goal_lat = msg.latitude
    self.agent_infos[robot_index].goal_long = msg.longitude
    self.agent_infos[robot_index].lat_inc = (self.agent_infos[robot_index].goal_lat - self.agent_infos[robot_index].cur_lat)/40
    self.agent_infos[robot_index].long_inc = (self.agent_infos[robot_index].goal_long - self.agent_infos[robot_index].cur_long)/40
    self.get_logger().info(f"{robot_index}th agent goal_lat : {self.agent_infos[robot_index].goal_lat}")
    self.get_logger().info(f"{robot_index}th agent goal_long : {self.agent_infos[robot_index].goal_long}")
    self.get_logger().info(f"{robot_index}th agent lat_inc : {self.agent_infos[robot_index].lat_inc}")
    self.get_logger().info(f"{robot_index}th agent long_inc : {self.agent_infos[robot_index].long_inc}")    

  def timer_callback_sim_env(self):

    self.cur_gps_update()
    msg = NavSatFix()
    msg.latitude = self.agent_infos[0].cur_lat
    msg.longitude = self.agent_infos[0].cur_long
    self.iopub_0.publish(msg)
    
    msg.latitude = self.agent_infos[1].cur_lat
    msg.longitude = self.agent_infos[1].cur_long
    self.iopub_1.publish(msg)
    
    msg.latitude = self.agent_infos[2].cur_lat
    msg.longitude = self.agent_infos[2].cur_long
    self.iopub_2.publish(msg)
    
    msg.latitude = self.agent_infos[3].cur_lat
    msg.longitude = self.agent_infos[3].cur_long
    self.iopub_3.publish(msg)
    
    msg.latitude = self.agent_infos[4].cur_lat
    msg.longitude = self.agent_infos[4].cur_long
    self.iopub_4.publish(msg)

  def cur_gps_update(self):
    for robot_index in range(0, 5):
      if self.agent_infos[robot_index].goal_lat  <= 0:
        continue
      
      if abs(self.agent_infos[robot_index].goal_lat - self.agent_infos[robot_index].cur_lat) > 0.0000001:
        self.agent_infos[robot_index].cur_lat = self.agent_infos[robot_index].cur_lat + self.agent_infos[robot_index].lat_inc
        self.get_logger().info(f"{robot_index}th agent cur_lat : {self.agent_infos[robot_index].cur_lat}")
      elif abs(self.agent_infos[robot_index].goal_long - self.agent_infos[robot_index].cur_long) > 0.0000001:
        self.agent_infos[robot_index].cur_long = self.agent_infos[robot_index].cur_long + self.agent_infos[robot_index].long_inc
        self.get_logger().info(f"{robot_index}th agent cur_long : {self.agent_infos[robot_index].cur_long}")

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  agent_gps_test_ros = AgentGPSTest()
  
  # Spin the node so the callback function is called.
  rclpy.spin(agent_gps_test_ros)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  agent_gps_test_ros.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
