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
    
    self.agent_gps_sim_env = [[30.6218691, -96.3345895],
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
            
    timer_period = 0.2 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback_sim_env)
    

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

  def timer_callback1(self):

    msg = NavSatFix()
    msg.latitude = self.agent_gps1[0][0]
    msg.longitude = self.agent_gps1[0][1]
    self.iopub_0.publish(msg)
    
    msg.latitude = self.agent_gps1[1][0]
    msg.longitude = self.agent_gps1[1][1]
    self.iopub_1.publish(msg)
    
    msg.latitude = self.agent_gps1[2][0]
    msg.longitude = self.agent_gps1[2][1]
    self.iopub_2.publish(msg)
    
    msg.latitude = self.agent_gps1[3][0]
    msg.longitude = self.agent_gps1[3][1]
    self.iopub_3.publish(msg)
    
    msg.latitude = self.agent_gps1[4][0]
    msg.longitude = self.agent_gps1[4][1]
    self.iopub_4.publish(msg)

  def timer_callback2(self):

    msg = NavSatFix()
    msg.latitude = self.agent_gps2[0][0]
    msg.longitude = self.agent_gps2[0][1]
    self.iopub_0.publish(msg)
    
    msg.latitude = self.agent_gps2[1][0]
    msg.longitude = self.agent_gps2[1][1]
    self.iopub_1.publish(msg)
    
    msg.latitude = self.agent_gps2[2][0]
    msg.longitude = self.agent_gps2[2][1]
    self.iopub_2.publish(msg)
    
    msg.latitude = self.agent_gps2[3][0]
    msg.longitude = self.agent_gps2[3][1]
    self.iopub_3.publish(msg)
    
    msg.latitude = self.agent_gps2[4][0]
    msg.longitude = self.agent_gps2[4][1]
    self.iopub_4.publish(msg)

  def timer_callback3(self):

    msg = NavSatFix()
    msg.latitude = self.agent_gps3[0][0]
    msg.longitude = self.agent_gps3[0][1]
    self.iopub_0.publish(msg)
    
    msg.latitude = self.agent_gps3[1][0]
    msg.longitude = self.agent_gps3[1][1]
    self.iopub_1.publish(msg)
    
    msg.latitude = self.agent_gps3[2][0]
    msg.longitude = self.agent_gps3[2][1]
    self.iopub_2.publish(msg)
    
    msg.latitude = self.agent_gps3[3][0]
    msg.longitude = self.agent_gps3[3][1]
    self.iopub_3.publish(msg)
    
    msg.latitude = self.agent_gps3[4][0]
    msg.longitude = self.agent_gps3[4][1]
    self.iopub_4.publish(msg)

  def timer_callback_sim_env(self):

    msg = NavSatFix()
    msg.latitude = self.agent_gps_sim_env[0][0]
    msg.longitude = self.agent_gps_sim_env[0][1]
    self.iopub_0.publish(msg)
    
    msg.latitude = self.agent_gps_sim_env[1][0]
    msg.longitude = self.agent_gps_sim_env[1][1]
    self.iopub_1.publish(msg)
    
    msg.latitude = self.agent_gps_sim_env[2][0]
    msg.longitude = self.agent_gps_sim_env[2][1]
    self.iopub_2.publish(msg)
    
    msg.latitude = self.agent_gps_sim_env[3][0]
    msg.longitude = self.agent_gps_sim_env[3][1]
    self.iopub_3.publish(msg)
    
    msg.latitude = self.agent_gps_sim_env[4][0]
    msg.longitude = self.agent_gps_sim_env[4][1]
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
