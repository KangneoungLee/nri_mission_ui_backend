# Author:
# - Kangneoung Lee
  

import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float64, Int8
import cv2
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge
from nri_plan_msgs.msg import Gps3WayPoints

import os
import math
import time

from haversine import haversine, Unit
import numpy as np
from dataclasses import dataclass, field

@dataclass
class agent_info:
  enable: bool
  wp_num: int
  cur_wp: int
  goal_send_cnt: int  
  cur_lat: float
  cur_long: float
  goal_lat: float
  goal_long: float
  part_img: np.ndarray
  waypoint: list[dict]
  dir_idx: int
  dir_cnt: int

class NriPlanWayPoint(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """

    super().__init__('nri_plan_waypoint')
    
    self.declare_parameter('max_agent_num', 10)
    self.declare_parameter('crop_row_width', 1.0)
    self.declare_parameter('goal_send_count_th', 3)
    self.declare_parameter('goal_reach_dist_th', 0.1)
    self.declare_parameter('trunk_dir', 'trunk_dir_default')
    self.declare_parameter('pivot_gps_lu_txt_dir', 'pivot_gps_lu_txt_dir_default')
    self.declare_parameter('pivot_gps_lb_txt_dir', 'pivot_gps_lb_txt_dir_default')
    self.declare_parameter('pivot_gps_ru_txt_dir', 'pivot_gps_ru_txt_dir_default')
    self.declare_parameter('pivot_gps_rb_txt_dir', 'pivot_gps_rb_txt_dir_default')
    self.declare_parameter('datum_txt_dir', 'datum_txt_dir_default')
    self.declare_parameter('robot_gps_goal_topic', 'robot_gps_goal_topic_default')
    self.declare_parameter('robot_gps_topic', 'global_position')
    
    self.max_agent_num = self.get_parameter('max_agent_num').value
    self.crop_row_width = self.get_parameter('crop_row_width').value
    self.goal_send_count_th = self.get_parameter('goal_send_count_th').value
    self.goal_reach_dist_th = self.get_parameter('goal_reach_dist_th').value
    self.trunk_dir = self.get_parameter('trunk_dir').get_parameter_value().string_value
    self.pivot_gps_lu_txt_dir = self.get_parameter('pivot_gps_lu_txt_dir').get_parameter_value().string_value
    self.pivot_gps_lb_txt_dir = self.get_parameter('pivot_gps_lb_txt_dir').get_parameter_value().string_value
    self.pivot_gps_ru_txt_dir = self.get_parameter('pivot_gps_ru_txt_dir').get_parameter_value().string_value
    self.pivot_gps_rb_txt_dir = self.get_parameter('pivot_gps_rb_txt_dir').get_parameter_value().string_value
    self.datum_txt_dir = self.get_parameter('datum_txt_dir').get_parameter_value().string_value
    self.robot_gps_goal_topic = self.get_parameter('robot_gps_goal_topic').get_parameter_value().string_value
    self.robot_gps_topic = self.get_parameter('robot_gps_topic').get_parameter_value().string_value
    
    self.waypoint_gen_option = 1 # value table : 0, 1
    self.npp_sub = self.create_subscription(Int8, "nri_plan_partition_ok", self.npp_callback, 3)
   
    self.npw_ok_topic_pub = self.create_publisher(Int8, 'nri_plan_waypoint_ok', 1)
    
    self.agent_gps_goal_pub = []
    self.agent_gps3p_goal_pub = []
    self.agent_gps_sub = []
    self.agent_infos = []

    for i in range(0, self.max_agent_num):
      gps_goal_pub_topic = 'Robot' + str(i) + '/' + self.robot_gps_goal_topic
      gps_goal_pub = self.create_publisher(NavSatFix, gps_goal_pub_topic, 1)
      self.agent_gps_goal_pub.append(gps_goal_pub)

      gps3p_goal_pub_topic = 'Robot' + str(i) + '/' + self.robot_gps_goal_topic + '_3p'
      gps3p_goal_pub = self.create_publisher(Gps3WayPoints, gps3p_goal_pub_topic, 1)
      self.agent_gps3p_goal_pub.append(gps3p_goal_pub)
      
      gps_sub_topic = 'Robot' + str(i) + '/' + self.robot_gps_topic
      
      # lamda function callback doesn't work with variable i.., therefore, hard coding is required to set up the number
      if i == 0:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 0), 2)
      elif i == 1:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 1), 2)
      elif i == 2:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 2), 2)
      elif i == 3:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 3), 2)
      elif i == 4:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 4), 2)
      elif i == 5:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 5), 2)
      elif i == 6:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 6), 2)
      elif i == 7:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 7), 2)
      elif i == 8:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 8), 2)
      elif i == 9:
        gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, 9), 2)                                                        
      self.agent_gps_sub.append(gps_sub)

      single_agent_info = agent_info(False, -1, -1, 0, -1, -1, -1, -1, np.zeros((1,1)), [], -1, -1)
      
      self.agent_infos.append(single_agent_info)
    
    timer_period = 1.0 # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)    
    self.time_stamp = self.get_clock().now().to_msg()
    
    self.pivot_lat = 0.0
    self.pivot_long = 0.0
    
    self.x_axis_del_lat = 0.0
    self.x_axis_del_long = 0.0
    self.y_axis_del_lat = 0.0
    self.y_axis_del_long = 0.0
    
    self.map_width = 0
    self.map_height = 0
    self.pixel_per_m = 0.0
    self.map_width_meter = 0.0
    
    self.datum_x_norm = 0.0
    self.datum_y_norm = 0.0
    
    self.wp_dir_x = [[0, 1, 0, 1], [0, -1, 0, -1], [0, 1, 0, 1] , [0, -1, 0, -1]]  #N - E - S - E,  N - W - S - W,  S - E - N - E,  S - W - N - W
    self.wp_dir_y = [[-1, 0, 1, 0], [-1, 0, 1, 0], [1, 0, -1, 0] , [1, 0, -1, 0]]  #N - E - S - E,  N - W - S - W,  S - E - N - E,  S - W - N - W

    self.completed = False
  def goal_reached_check(self, robot_index):
    cur_lat = self.agent_infos[robot_index].cur_lat
    cur_long = self.agent_infos[robot_index].cur_long
    goal_lat = self.agent_infos[robot_index].goal_lat
    goal_long = self.agent_infos[robot_index].goal_long
    self.get_logger().info(f"goal reach check... {robot_index} agent, cur_lat : {cur_lat}")
    self.get_logger().info(f"goal reach check... {robot_index} agent, goal_lat : {goal_lat}")
    self.get_logger().info(f"goal reach check... {robot_index} agent, cur_long : {cur_long}")
    self.get_logger().info(f"goal reach check... {robot_index} agent, goal_long : {goal_long}")
    dist = haversine((cur_lat, cur_long), (goal_lat, goal_long), unit=Unit.METERS)
    self.get_logger().info(f"goal reach check... {robot_index} agent, dist : {dist}")
    if dist < self.goal_reach_dist_th:
      return True
    else:
      return False #return False

  def send_gps_goal_3p(self):
    for i in range(0, self.max_agent_num):
      if self.agent_infos[i].enable == False:
        continue
      
      if self.agent_infos[i].wp_num != 3:
        self.get_logger().info(f" {i} th agent waypoint generation fail... num of wp = {self.agent_infos[i].wp_num} ")
        continue
      msg = Gps3WayPoints()
      st_msg = NavSatFix()
      
      for j in range(0,3):
        wp = self.agent_infos[i].waypoint[j]
        y_goal_pixel = wp['y']
        x_goal_pixel = wp['x']      
        gps_lat = self.pivot_lat + self.y_axis_del_lat * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_lat * x_goal_pixel/self.map_width 
        gps_long = self.pivot_long + self.y_axis_del_long * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_long * x_goal_pixel/self.map_width
        msg.latitudes.append(gps_lat)
        msg.longitudes.append(gps_long)
        
        if j == 0:
          st_msg.latitude = gps_lat
          st_msg.longitude = gps_long
      
      self.agent_gps3p_goal_pub[i].publish(msg)
      
      
      self.agent_gps_goal_pub[i].publish(st_msg)
      
  def send_gps_goal(self):
    for i in range(0, self.max_agent_num):
      if self.agent_infos[i].enable == False:
        continue
      
      if self.agent_infos[i].cur_wp >= self.agent_infos[i].wp_num:
        continue
      
      if self.agent_infos[i].cur_wp == 0:
        wp = self.agent_infos[i].waypoint[self.agent_infos[i].cur_wp]
        y_goal_pixel = wp['y']
        x_goal_pixel = wp['x']
        gps_lat = self.pivot_lat + self.y_axis_del_lat * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_lat * x_goal_pixel/self.map_width 
        gps_long = self.pivot_long + self.y_axis_del_long * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_long * x_goal_pixel/self.map_width
        self.agent_infos[i].goal_lat = gps_lat
        self.agent_infos[i].goal_long = gps_long
      
      reached = self.goal_reached_check(i)
      if reached is True:
        self.agent_infos[i].cur_wp = self.agent_infos[i].cur_wp + 1
        self.agent_infos[i].goal_send_cnt = 1
        
        wp = self.agent_infos[i].waypoint[self.agent_infos[i].cur_wp]
        y_goal_pixel = wp['y']
        x_goal_pixel = wp['x']
        
        gps_lat = self.pivot_lat + self.y_axis_del_lat * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_lat * x_goal_pixel/self.map_width 
        gps_long = self.pivot_long + self.y_axis_del_long * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_long * x_goal_pixel/self.map_width 
        self.agent_infos[i].goal_lat = gps_lat
        self.agent_infos[i].goal_long = gps_long
        
        msg = NavSatFix()
        msg.latitude = gps_lat
        msg.longitude = gps_long
        self.agent_gps_goal_pub[i].publish(msg)
      else:
        if self.agent_infos[i].goal_send_cnt > self.goal_send_count_th:
          continue
        self.agent_infos[i].goal_send_cnt = self.agent_infos[i].goal_send_cnt + 1
        msg = NavSatFix()
        msg.latitude = self.agent_infos[i].goal_lat
        msg.longitude = self.agent_infos[i].goal_long
        self.agent_gps_goal_pub[i].publish(msg)
         
  def gps_goal_debug(self):
    for i in range(0, self.max_agent_num):
      if self.agent_infos[i].enable == False:
        continue
      
      debug_txt_dir = self.trunk_dir + '/agent_wp_gps_debug_' + str(i) +'.txt'  
      f = open(debug_txt_dir, "w")
      f.writelines(f"waypoint number : {self.agent_infos[i].wp_num} \n")
      f.writelines(f"current waypoint idx : {self.agent_infos[i].cur_wp} \n")
      f.writelines(f"current dir idx : {self.agent_infos[i].dir_idx} \n")
      
      for j in range(0, self.agent_infos[i].wp_num):
        wp = self.agent_infos[i].waypoint[j]
        y_goal_pixel = wp['y']
        x_goal_pixel = wp['x']
        
        gps_lat = self.pivot_lat + self.y_axis_del_lat * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_lat * x_goal_pixel/self.map_width
        gps_long = self.pivot_long + self.y_axis_del_long * (self.map_height - y_goal_pixel)/self.map_height + self.x_axis_del_long * x_goal_pixel/self.map_width
        f.writelines(f"{j}th gps waypoint... gps_goal_lat : {gps_lat} gps_goal_long : {gps_long}\n")
      
      f.close()
      
  def timer_callback(self):    
    if self.completed is True:
      msg = Int8()
      msg.data = 1
      if self.waypoint_gen_option == 0:
        self.send_gps_goal()
      elif self.waypoint_gen_option == 1:
        self.send_gps_goal_3p()
        
      self.npw_ok_topic_pub.publish(msg)
      
    
    else:
      for i in range(0, self.max_agent_num):
        msg = NavSatFix()
        msg.latitude = 0.0
        msg.longitude = 0.0
        self.agent_gps_goal_pub[i].publish(msg)
          
      msg = Int8()
      msg.data = 0
      self.npw_ok_topic_pub.publish(msg)
    
  def npp_callback(self, msg):
    if msg.data == 0:
      self.completed = False
      for i in range(0, self.max_agent_num):
        self.agent_infos[i].enable = False
        self.agent_infos[i].wp_num = -1
        self.agent_infos[i].cur_wp = -1
        self.agent_infos[i].goal_send_cnt = 0
        #self.agent_infos[i].cur_lat = -1
        #self.agent_infos[i].cur_long = -1
        self.agent_infos[i].goal_lat = -1
        self.agent_infos[i].goal_long = -1
        self.agent_infos[i].part_img = np.zeros((1,1))     
        self.agent_infos[i].waypoint = []
        self.agent_infos[i].dir_idx = -1
        self.agent_infos[i].dir_cnt = -1    
    else:
      if self.completed is False:
        rd_ok = self.read_dataum()
        if rd_ok is False:
          self.get_logger().info(" read datum text file fail ")
          return
        
        self.read_gps_pivot()
        self.meter_of_map_width()
        
        for i in range(0, self.max_agent_num):
          rp_ok = self.read_partition(i)
          self.get_logger().info(f"{i}th agent availability status : {self.agent_infos[i].enable}")
          if rp_ok is False:
            self.get_logger().info(f"{i}th robot: read partition text file fail ")
            continue
        if self.waypoint_gen_option == 0:
          self.waypoint_gen()
          
        elif self.waypoint_gen_option == 1:
          self.waypoint_gen_3p() 
          
        self.waypoint_gen_debug()
        self.gps_goal_debug()
        self.completed = True
        
  def gps_callback(self, msg, robot_index):
    self.agent_infos[robot_index].cur_lat = msg.latitude
    self.agent_infos[robot_index].cur_long = msg.longitude
    #self.get_logger().info(f"gps_callback... {robot_index} agent, cur_lat : {self.agent_infos[robot_index].cur_lat}")
    #self.get_logger().info(f"gps_callback... {robot_index} agent, cur_long : {self.agent_infos[robot_index].cur_long}")
 
  def read_gps_pivot(self):
    f = open(self.pivot_gps_lb_txt_dir, "r")
    string = f.readline()
    lb_latitude = float(string.split()[0])
    lb_longitude = float(string.split()[1])
    f.close()

    f = open(self.pivot_gps_rb_txt_dir, "r")
    string = f.readline()
    rb_latitude = float(string.split()[0])
    rb_longitude = float(string.split()[1])
    f.close()
    
    f = open(self.pivot_gps_lu_txt_dir, "r")
    string = f.readline()
    lu_latitude = float(string.split()[0])
    lu_longitude = float(string.split()[1])
    f.close()
    
    self.pivot_lat = lb_latitude
    self.pivot_long = lb_longitude
    
    self.x_axis_del_lat = rb_latitude - lb_latitude
    self.x_axis_del_long = rb_longitude - lb_longitude
    
    self.y_axis_del_lat = lu_latitude - lb_latitude
    self.y_axis_del_long = lu_longitude - lb_longitude
  
  def meter_of_map_width(self):
    self.map_width_meter = haversine((self.pivot_lat, self.pivot_long), (self.pivot_lat + self.x_axis_del_lat, self.pivot_long + self.x_axis_del_long), unit=Unit.METERS)
    self.get_logger().info(f"self.map_width_meter : {self.map_width_meter}")
    
  def read_dataum(self):
    if not os.path.exists(self.datum_txt_dir):
      return False
    
    try:
      f = open(self.datum_txt_dir, "r")
      string = f.readline()
      self.datum_x_norm = float(string.split()[0])
      self.datum_y_norm = float(string.split()[1])
      f.close()
    except:
      return False
      
    return True
    
  def read_partition(self, robot_index):
    
    rp_ok = False
    
    txt_dir = self.trunk_dir + '/agent' + str(robot_index) +'.txt'
    img_dir = self.trunk_dir + '/agent' + str(robot_index) +'.png'
    
    if not os.path.exists(txt_dir) or not os.path.exists(img_dir):
      self.get_logger().info(f"one of path is fail")
      self.agent_infos[robot_index].enable = False
      rp_ok = False
      return rp_ok
      
    try:
      f = open(txt_dir, "r")
      string = f.readline()
      wp = {}
      wp['x'] = int(string.split()[0])
      wp['y'] = int(string.split()[1])
      f.close()
      
      img = cv2.imread(img_dir, cv2.IMREAD_GRAYSCALE)
      self.map_width = img.shape[1]
      self.map_height = img.shape[0]
      thres = 254
      max_val = 255
      ret, bin_img = cv2.threshold(img, thres, max_val, cv2.THRESH_BINARY_INV)
      debug_bin_img_dir = self.trunk_dir + '/bin_img_raw_' + str(robot_index) +'.png'
      cv2.imwrite(debug_bin_img_dir, bin_img)
      contours = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      contours = contours[0] if len(contours) == 2 else contours[1]   # if len is 2, first element is value and second one is hierarchy
      contour_props = []
      for contour in contours:
        x1, y1, w, h = cv2.boundingRect(contour)
        x2 = x1 + w
        y2 = y1 + h
        if x1 > 0:
          x1 = x1 -1
        if x2 < self.map_width - 1:
          x2 = x2 + 1
        if y1 > 0:
          y1 = y1 -1
        if y2 < self.map_height - 1:
          y2 = y2 + 1
        add = x1 , y1, x2, y2
        contour_props.append(add)
        #self.get_logger().info(f"{robot_index}th agent contour info... x1 : {x1}, x2 : {x2}, y1 : {y1}, y2 : {y2}")
      #bin_test_img = np.zeros((self.map_height, self.map_width), np.uint8)
      for data in contour_props:
        #self.get_logger().info(f"{robot_index}th agent contour info... data[0] : {data[0]}, data[2] : {data[2]}, data[1] : {data[1]}, data[3] : {data[3]}")
        #bin_test_img = cv2.cvtColor(bin_test_img, cv2.COLOR_GRAY2BGR);
        bin_img = cv2.rectangle(bin_img, (data[0], data[1]), (data[2], data[3]), (255, 255, 255), cv2.FILLED)
      #debug_bin_img_rec_dir = self.trunk_dir + '/bin_img_rec_' + str(robot_index) +'.png'
      #cv2.imwrite(debug_bin_img_rec_dir, bin_img)
      #kernel = np.ones((3, 3), np.uint8)  
      #bin_img = cv2.dilate(bin_img, kernel, iterations = 1)
      #debug_bin_img_dilate_dir = self.trunk_dir + '/bin_img_dilate_' + str(robot_index) +'.png'
      #cv2.imwrite(debug_bin_img_dilate_dir, bin_img)   
      self.agent_infos[robot_index].part_img = bin_img.copy() 
      
      x_init_correction = 0
      self.pixel_per_m =  self.map_width/self.map_width_meter
      self.crop_row_pixel = self.pixel_per_m * self.crop_row_width
      if (wp['x'] >= self.datum_x_norm*self.map_width) and (wp['y'] < self.datum_y_norm*self.map_height):
        self.agent_infos[robot_index].dir_idx = 0
        x_init_correction = self.crop_row_pixel
      elif (wp['x'] < self.datum_x_norm*self.map_width) and (wp['y'] < self.datum_y_norm*self.map_height):
        self.agent_infos[robot_index].dir_idx = 1
        x_init_correction = -self.crop_row_pixel
      elif (wp['x'] >= self.datum_x_norm*self.map_width) and (wp['y'] >= self.datum_y_norm*self.map_height):
        self.agent_infos[robot_index].dir_idx = 2
        x_init_correction = self.crop_row_pixel
      elif (wp['x'] < self.datum_x_norm*self.map_width) and (wp['y'] >= self.datum_y_norm*self.map_height):
        self.agent_infos[robot_index].dir_idx = 3
        x_init_correction = -self.crop_row_pixel
      
      wp['x'] = wp['x'] #+ x_init_correction
      self.agent_infos[robot_index].dir_cnt = self.agent_infos[robot_index].dir_cnt + 1
      self.agent_infos[robot_index].waypoint.append(wp)
      self.agent_infos[robot_index].wp_num = 1
      self.agent_infos[robot_index].cur_wp = 0

      rp_ok = True
      self.agent_infos[robot_index].enable = True
    except Exception as error:
      self.get_logger().info(f"image processing error : {error}")
      rp_ok = False
      self.agent_infos[robot_index].enable = False
      
    return rp_ok

  def vertical_search(self, robot_index, cur_x, cur_y, cur_dir_y, dir_idx):
    next_y_temp = cur_y + self.map_height*cur_dir_y
    
    if next_y_temp < 0:
      next_y_temp = 0
    else:
      next_y_temp = self.map_height - 1
    
    round_cur_x = round(cur_x)
    if dir_idx == 0 or dir_idx == 2:
      round_cur_x = round_cur_x + 1 # compensate truncation error
    else:
      round_cur_x = round_cur_x -1 # compensate truncation error
      
    if round_cur_x < 0:
      round_cur_x = 0
    elif round_cur_x >= self.map_width:
      round_cur_x = self.map_width - 1
    
    if self.agent_infos[robot_index].part_img[int(next_y_temp), int(round_cur_x)] == 255:
      return next_y_temp
    
    if next_y_temp == cur_y:
      return next_y_temp
      
    lo = 0
    ho = 0      
    if next_y_temp > cur_y :
      lo = cur_y
      ho = next_y_temp
      forward = True
    else:
      lo = next_y_temp
      ho = cur_y
      forward = False
    
    
    if forward is True:
      while(lo + 1 < ho):
        mid = int((lo + ho)/2)
        if self.agent_infos[robot_index].part_img[mid, int(round_cur_x)] == 255:
          lo = mid
        else:
          ho = mid
      return lo
    else:
      while(lo + 1 < ho):
        mid = int((lo + ho)/2)
        if self.agent_infos[robot_index].part_img[mid, int(round_cur_x)] == 255:
          ho = mid
        else:
          lo = mid
      return ho 

  def horizontal_search_3p(self, robot_index, cur_x, cur_y, cur_dir_x, dir_idx):
    round_cur_x = round(cur_x)
    if dir_idx == 0 or dir_idx == 2:
      round_cur_x = round_cur_x + 1 # compensate truncation error
    else:
      round_cur_x = round_cur_x -1 # compensate truncation error
    if round_cur_x < 0:
      round_cur_x = 0
      
    next_x_temp = round_cur_x + self.map_width*cur_dir_x
    
    if next_x_temp < 0:
      next_x_temp = 0
    else:
      next_x_temp = self.map_width - 1
    
    if self.agent_infos[robot_index].part_img[int(cur_y), int(next_x_temp)] == 255:
      return next_x_temp

    if next_x_temp == round_cur_x:
      return next_x_temp
    
    lo = 0
    ho = 0

    if next_x_temp > round_cur_x :
      lo = round_cur_x
      ho = next_x_temp
      forward = True
    else:
      lo = next_x_temp
      ho = round_cur_x
      forward = False

    if forward is True:
      while(lo + 1 < ho):
        mid = int((lo + ho)/2)
        if self.agent_infos[robot_index].part_img[int(cur_y), mid] == 255:
          lo = mid
        else:
          ho = mid
      return lo
    else:
      while(lo + 1 < ho):
        mid = int((lo + ho)/2)
        if self.agent_infos[robot_index].part_img[int(cur_y), mid] == 255:
          ho = mid
        else:
          lo = mid
      return ho 

      
  def horizontal_search(self, robot_index, cur_x, cur_y, cur_dir_x):
    next_x_temp = cur_x + self.crop_row_pixel*cur_dir_x
    if next_x_temp < 0:
      next_x_temp = 0
      return cur_x, True
    elif next_x_temp >= self.map_width:
      next_x_temp = self.map_width - 1
      return cur_x, True
    
    round_next_x_temp = round(next_x_temp)
    px_val = self.agent_infos[robot_index].part_img[int(cur_y), int(round_next_x_temp)]
    
    if px_val == 255:
      return next_x_temp, False
   
    while(True):
      next_x_temp = next_x_temp + self.crop_row_pixel*cur_dir_x
      round_next_x_temp = round(next_x_temp)
      if next_x_temp < 1 or next_x_temp >= self.map_width - 1:
        return cur_x, True
      
      if self.agent_infos[robot_index].part_img[int(cur_y), int(round_next_x_temp)] == 255:
        return next_x_temp, False
        
  def waypoint_gen(self):
    for i in range(0, self.max_agent_num):
      if self.agent_infos[i].enable is False:
        continue
      end_flag = False
      while(True):
      
        dir_x = self.wp_dir_x[self.agent_infos[i].dir_idx]
        dir_y = self.wp_dir_y[self.agent_infos[i].dir_idx]
        cur_dir_x = dir_x[self.agent_infos[i].dir_cnt]
        cur_dir_y = dir_y[self.agent_infos[i].dir_cnt]
      
        cur_x = self.agent_infos[i].waypoint[self.agent_infos[i].cur_wp]['x']
        cur_y = self.agent_infos[i].waypoint[self.agent_infos[i].cur_wp]['y']
      
        next_x = cur_x
        next_y = cur_y

        if cur_dir_x == 0:
          next_y = self.vertical_search(i, cur_x, cur_y, cur_dir_y, self.agent_infos[i].dir_idx)
        elif cur_dir_y == 0:
          next_x, end_flag = self.horizontal_search(i, cur_x, cur_y, cur_dir_x)
        
        if end_flag is True:
          break
          
        wp = {}
        wp['x'] = next_x
        wp['y'] = next_y

        self.agent_infos[i].waypoint.append(wp)

        self.agent_infos[i].wp_num = self.agent_infos[i].wp_num + 1
        self.agent_infos[i].cur_wp = self.agent_infos[i].cur_wp + 1
      
        self.agent_infos[i].dir_cnt = self.agent_infos[i].dir_cnt + 1
        if self.agent_infos[i].dir_cnt >= 4:
          self.agent_infos[i].dir_cnt = 0
           
      self.agent_infos[i].cur_wp = 0
      self.agent_infos[i].dir_cnt = 0

  def waypoint_gen_3p(self):
    for i in range(0, self.max_agent_num):
      if self.agent_infos[i].enable is False:
        continue
      end_flag = False
      for j in range(0,2):
      
        dir_x = self.wp_dir_x[self.agent_infos[i].dir_idx]
        dir_y = self.wp_dir_y[self.agent_infos[i].dir_idx]
        cur_dir_x = dir_x[self.agent_infos[i].dir_cnt]
        cur_dir_y = dir_y[self.agent_infos[i].dir_cnt]
      
        cur_x = self.agent_infos[i].waypoint[self.agent_infos[i].cur_wp]['x']
        cur_y = self.agent_infos[i].waypoint[self.agent_infos[i].cur_wp]['y']
      
        next_x = cur_x
        next_y = cur_y

        if cur_dir_x == 0:
          next_y = self.vertical_search(i, cur_x, cur_y, cur_dir_y, self.agent_infos[i].dir_idx)
        elif cur_dir_y == 0:
          next_x= self.horizontal_search_3p(i, cur_x, cur_y, cur_dir_x, self.agent_infos[i].dir_idx)
                    
        wp = {}
        wp['x'] = next_x
        wp['y'] = next_y

        self.agent_infos[i].waypoint.append(wp)

        self.agent_infos[i].wp_num = self.agent_infos[i].wp_num + 1
        self.agent_infos[i].cur_wp = self.agent_infos[i].cur_wp + 1
      
        self.agent_infos[i].dir_cnt = self.agent_infos[i].dir_cnt + 1
        if self.agent_infos[i].dir_cnt >= 4:
          self.agent_infos[i].dir_cnt = 0
           
      self.agent_infos[i].cur_wp = 0
      self.agent_infos[i].dir_cnt = 0          

  def waypoint_gen_debug(self):
    for i in range(0, self.max_agent_num):
      if self.agent_infos[i].enable is False:
        continue
      debug_txt_dir = self.trunk_dir + '/agent_wp_debug_' + str(i) +'.txt'
      debug_img_dir = self.trunk_dir + '/agent_wp_debug_' + str(i) +'.png'
      f = open(debug_txt_dir, "w")
      f.writelines(f"waypoint number : {self.agent_infos[i].wp_num} \n")
      f.writelines(f"current waypoint idx : {self.agent_infos[i].cur_wp} \n")
      f.writelines(f"current dir idx : {self.agent_infos[i].dir_idx} \n")
      
      img = self.agent_infos[i].part_img.copy()
      img_rgb = np.zeros((img.shape[0], img.shape[1], 3))
      img_rgb[:,:,0] = img
      img_rgb[:,:,1] = img
      img_rgb[:,:,2] = img
      for j, wp in enumerate(self.agent_infos[i].waypoint):
        f.writelines(f"{j}th waypoint x : {wp['x']},   waypoint y : {wp['y']} \n")
        img_rgb[int(wp['y']), int(wp['x']), 0] = 0
        img_rgb[int(wp['y']), int(wp['x']), 1] = 255
        img_rgb[int(wp['y']), int(wp['x']), 2] = 0

      cv2.imwrite(debug_img_dir, img_rgb)
      f.close()     
   
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  nri_plan_waypoint_ros = NriPlanWayPoint()
  
  # Spin the node so the callback function is called.
  rclpy.spin(nri_plan_waypoint_ros)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  nri_plan_waypoint_ros.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
