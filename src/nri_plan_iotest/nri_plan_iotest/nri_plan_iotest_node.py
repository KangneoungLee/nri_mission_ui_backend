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



class NriPlanIoTest(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """

    super().__init__('nri_plan_iotest_node')
    
    self.iopub_1 = self.create_publisher(Image, 'coverage_control/color/density', 1)
    self.iopub_2 = self.create_publisher(Float64, 'coverage_control/map_resolution', 1)
    self.iopub_3 = self.create_publisher(NavSatFix, 'coverage_control/pivot_gps_lu', 1)
    self.iopub_4 = self.create_publisher(NavSatFix, 'coverage_control/pivot_gps_lb', 1)
    self.iopub_5 = self.create_publisher(NavSatFix, 'coverage_control/pivot_gps_ru', 1)
    self.iopub_6 = self.create_publisher(NavSatFix, 'coverage_control/pivot_gps_rb', 1)
    self.iopub_7 = self.create_publisher(Int8, 'coverage_control/cancel', 1)
    self.iopub_8 = self.create_publisher(Float64, 'coverage_control/datum_x_norm', 1)
    self.iopub_9 = self.create_publisher(Float64, 'coverage_control/datum_y_norm', 1)
        
    timer_period = 3.0 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
    timer_period2 = 10.0 # seconds
    self.timer = self.create_timer(timer_period2, self.timer_callback2)
    
    self.br = CvBridge()

  def timer_callback(self):
    img = cv2.imread('/home/artlab/nri_plan_ws/src/nri_plan_iotest/fielddensitymap.jpg')
    dim = (300, 300)
    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    self.iopub_1.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))
    
    msg = Float64()
    msg.data = 0.25
    self.iopub_2.publish(msg)
    
    msg = NavSatFix()
    #msg.latitude = 9.11
    #msg.longitude = 9.11
    msg.latitude = 30.62277
    msg.longitude = -96.3346
    self.iopub_3.publish(msg)

    msg = NavSatFix()
    #msg.latitude = 18.11
    #msg.longitude = 18.11
    msg.latitude = 30.62186
    msg.longitude = -96.33355
    self.iopub_4.publish(msg)

    msg = NavSatFix()
    #msg.latitude = 27.11
    #msg.longitude = 27.11
    msg.latitude = 30.62348
    msg.longitude = -96.33313
    self.iopub_5.publish(msg)    

    msg = NavSatFix()
    #msg.latitude = 36.11
    #msg.longitude = 36.11
    msg.latitude = 30.62276
    msg.longitude = -96.33253
    self.iopub_6.publish(msg) 
    
    msg = Float64()
    msg.data = 0.0
    self.iopub_8.publish(msg) 
    
    msg = Float64()
    msg.data = 0.5
    self.iopub_9.publish(msg) 

  def timer_callback2(self):
    msg = Int8()
    msg.data = 1
    self.iopub_7.publish(msg)
    self.iopub_7.publish(msg)
    self.iopub_7.publish(msg)

    
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  nri_plan_iotest_ros = NriPlanIoTest()
  
  # Spin the node so the callback function is called.
  rclpy.spin(nri_plan_iotest_ros)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  nri_plan_iotest_ros.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
