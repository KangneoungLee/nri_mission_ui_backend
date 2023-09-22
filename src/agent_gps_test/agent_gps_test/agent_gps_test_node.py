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
            
    timer_period = 0.2 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    

  def timer_callback(self):

    
    msg = NavSatFix()
    msg.latitude = self.agent_gps[0][0]
    msg.longitude = self.agent_gps[0][1]
    self.iopub_0.publish(msg)
    
    msg.latitude = self.agent_gps[1][0]
    msg.longitude = self.agent_gps[1][1]
    self.iopub_1.publish(msg)
    
    msg.latitude = self.agent_gps[2][0]
    msg.longitude = self.agent_gps[2][1]
    self.iopub_2.publish(msg)
    
    msg.latitude = self.agent_gps[3][0]
    msg.longitude = self.agent_gps[3][1]
    self.iopub_3.publish(msg)
    
    msg.latitude = self.agent_gps[4][0]
    msg.longitude = self.agent_gps[4][1]
    self.iopub_4.publish(msg)

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
