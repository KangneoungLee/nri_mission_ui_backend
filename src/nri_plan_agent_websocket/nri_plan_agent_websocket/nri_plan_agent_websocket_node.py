# Author:
# - Kangneoung Lee
  

import rclpy 
import roslibpy
from rclpy.node import Node 
from std_msgs.msg import Float64, Int8, Float64MultiArray

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from nri_plan_msgs.msg import Gps3WayPoints

import os
import math
import time

import numpy as np



class NriPlanAgWebSocket(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """

    super().__init__('nri_plan_agent_websocket_node')
    
    self.declare_parameter('max_agent_num', 10)
    self.declare_parameter('robot_gps_goal_topic', 'robot_gps_goal_topic_default')
    self.declare_parameter('robot_gps_topic', 'global_position')
    self.declare_parameter('robot_gps_goal_to_agent_topic', 'goal_default')
    self.declare_parameter('robot_gps_from_agent_topic', 'filtered_gps_default')
    
    self.max_agent_num = self.get_parameter('max_agent_num').value
    self.robot_gps_goal_topic = self.get_parameter('robot_gps_goal_topic').get_parameter_value().string_value
    self.robot_gps_topic = self.get_parameter('robot_gps_topic').get_parameter_value().string_value
    self.robot_gps_goal_to_agent_topic = self.get_parameter('robot_gps_goal_to_agent_topic').get_parameter_value().string_value
    self.robot_gps_from_agent_topic = self.get_parameter('robot_gps_from_agent_topic').get_parameter_value().string_value
    
    self.robot_automode_to_agent_topic = 'auto_mode'
    
    self.robot_ip_list = []
    self.client_list = []
    self.gps_goal_pub_to_agent_list = []
    self.automode_pub_to_agent_list = []
    self.gps_goal_sub_from_machine_list = []
    self.gps3p_goal_sub_from_machine_list = []
    self.gps_pub_to_machine_list = []
    self.gps_sub_from_agent_list = []
    
    for i in range(0, self.max_agent_num):
      agent_ip_param = 'agent_ip' + str(i)
      self.declare_parameter(agent_ip_param, agent_ip_param)
      ip = self.get_parameter(agent_ip_param).get_parameter_value().string_value
      self.robot_ip_list.append(ip)
      
      client = roslibpy.Ros(host=ip, port=9090)
      self.client_list.append(client)
      
      gps_goal_pub_to_agent = roslibpy.Topic(client, self.robot_gps_goal_to_agent_topic, 'sensor_msgs/NavSatFix')
      self.gps_goal_pub_to_agent_list.append(gps_goal_pub_to_agent)
      
      gps_automode_pub_to_agent = roslibpy.Topic(client, self.robot_automode_to_agent_topic, 'std_msgs/Float64MultiArray')
      self.automode_pub_to_agent_list.append(gps_automode_pub_to_agent)
      
      gps_goal_sub_from_machine_topic = 'Robot' + str(i) + '/' + self.robot_gps_goal_topic
      gps3p_goal_sub_from_machine_topic = 'Robot' + str(i) + '/' + self.robot_gps_goal_topic + '_3p'

      if i == 0:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 0), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 0), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 1:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 1), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 1), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 2:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 2), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 2), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 3:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 3), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 3), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 4:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 4), 3)      
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 4), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 5:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 5), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 5), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 6:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 6), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 6), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 7:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 7), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 7), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 8:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 8), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 8), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      elif i == 9:
        gps_goal_sub_from_machine = self.create_subscription(NavSatFix, gps_goal_sub_from_machine_topic, lambda msg: self.gps_goal_sub_fm_callback(msg, 9), 3)
        self.gps_goal_sub_from_machine_list.append(gps_goal_sub_from_machine)
        gps3p_goal_sub_from_machine = self.create_subscription(Gps3WayPoints, gps3p_goal_sub_from_machine_topic, lambda msg: self.gps3p_goal_sub_fm_callback(msg, 9), 3)
        self.gps3p_goal_sub_from_machine_list.append(gps3p_goal_sub_from_machine)
      
      gps_pub_to_machine_topic = 'Robot' + str(i) + '/' + self.robot_gps_topic
      gps_pub_to_machine = self.create_publisher(NavSatFix, gps_pub_to_machine_topic, 1)
      self.gps_pub_to_machine_list.append(gps_pub_to_machine)
      
      gps_sub_from_agent = roslibpy.Topic(client, self.robot_gps_from_agent_topic, 'sensor_msgs/NavSatFix')
      gps_sub_from_agent.subscribe(lambda message: self.gps_sub_fa_callback(message, i))
      self.gps_sub_from_agent_list.append(gps_sub_from_agent)
    
    self.try_connection()
      
    timer_period = 1.0 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)  
      
  def try_connection(self):
    for i in range(0, self.max_agent_num):
      if self.client_list[i].is_connected is False:
        try:
          self.client_list[i].run(timeout = 2)
          self.get_logger().info(f'{i}th agent connected to websocket')
        except:
          self.get_logger().info(f'{i}th agent can not be connected to websocket')
          
  
  def timer_callback(self):
    self.try_connection()
  
  def gps3p_goal_sub_fm_callback(self, msg, robot_index):
    automode_pub_to_agent = self.automode_pub_to_agent_list[robot_index]
    if self.client_list[robot_index].is_connected is True:
      automode_pub_to_agent.publish(roslibpy.Message({'data': [1.0, msg.latitudes[0], msg.longitudes[0], msg.latitudes[1], msg.longitudes[1], msg.latitudes[2], msg.longitudes[2], 0.5]
                                                      }))

  def gps_goal_sub_fm_callback(self, msg, robot_index):
    gps_goal_lat = msg.latitude
    gps_goal_long = msg.longitude
    gps_goal_pub_to_agent = self.gps_goal_pub_to_agent_list[robot_index]
    if self.client_list[robot_index].is_connected is True:
      gps_goal_pub_to_agent.publish(roslibpy.Message({'header': {
                                                      'stamp': {
                                                           'secs': int(0),
                                                           'nsecs': int(0)
                                                      },
                                                      'frame_id': 'map'
                                                      },
                                                      'latitude': gps_goal_lat,
                                                      'longitude': gps_goal_long,
                                                      'altitude': 0
                                                      }))
  
  def gps_sub_fa_callback(self, msg, robot_index):
    gps_lat = msg['latitude']
    gps_long = msg['longitude']
    gps_msg = NavSatFix()
    gps_msg.latitude = gps_lat
    gps_msg.longitude = gps_long
    gps_pub_to_machine = self.gps_pub_to_machine_list[robot_index]
    gps_pub_to_machine.publish(gps_msg)
                                                      
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  nri_plan_agent_websocket_ros = NriPlanAgWebSocket()
  
  # Spin the node so the callback function is called.
  rclpy.spin(nri_plan_agent_websocket_ros)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  nri_plan_agent_websocket_ros.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
