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

from haversine import haversine, Unit
import numpy as np
from dataclasses import dataclass, field

@dataclass
class agent_info:
  enable: bool
  wp_num: int
  cur_wp: int
  cur_status: int  # -1: init 0: ready, 1: prepared, 2: sent
  cur_lat: float
  cur_long: float
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
    self.declare_parameter('crop_row_width', 1)
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
    self.trunk_dir = self.get_parameter('trunk_dir').get_parameter_value().string_value
    self.pivot_gps_lu_txt_dir = self.get_parameter('pivot_gps_lu_txt_dir').get_parameter_value().string_value
    self.pivot_gps_lb_txt_dir = self.get_parameter('pivot_gps_lb_txt_dir').get_parameter_value().string_value
    self.pivot_gps_ru_txt_dir = self.get_parameter('pivot_gps_ru_txt_dir').get_parameter_value().string_value
    self.pivot_gps_rb_txt_dir = self.get_parameter('pivot_gps_rb_txt_dir').get_parameter_value().string_value
    self.datum_txt_dir = self.get_parameter('datum_txt_dir').get_parameter_value().string_value
    self.robot_gps_goal_topic = self.get_parameter('robot_gps_goal_topic').get_parameter_value().string_value
    self.robot_gps_topic = self.get_parameter('robot_gps_topic').get_parameter_value().string_value
   
    self.npp_sub = self.create_subscription(Int8, "nri_plan_partition_ok", self.npp_callback, 3)
   
    self.npw_ok_topic_pub = self.create_publisher(Int8, 'nri_plan_waypoint_ok', 1)
    
    self.agent_gps_goal_pub = []
    self.agent_gps_sub = []
    self.agent_infos = []
    for i in range(0, self.max_agent_num):
      gps_gol_pub_topic = 'Robot' + str(i) + '/' + self.robot_gps_goal_topic
      gps_goal_pub = self.create_publisher(NavSatFix, gps_gol_pub_topic, 1)
      self.agent_gps_goal_pub.append(gps_goal_pub)
      
      gps_sub_topic = 'Robot' + str(i) + '/' + self.robot_gps_topic
      gps_sub = self.create_subscription(NavSatFix, gps_sub_topic, lambda msg: self.gps_callback(msg, i), 3)
      self.agent_gps_sub.append(gps_sub)

      single_agent_info = agent_info(False, -1, -1, -1, -1, -1, np.zeros((1,1)), [], -1, -1)
      
      self.agent_infos.append(single_agent_info)
    
    timer_period = 2.0 # seconds
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
    
    self.datum_x_norm = 0.0
    self.datum_y_norm = 0.0
    
    self.wp_dir_x = [[0, 1, 0, 1], [0, -1, 0, -1], [0, 1, 0, 1] , [0, -1, 0, -1]]  #N - E - S - E,  N - W - S - W,  S - E - N - E,  S - W - N - W
    self.wp_dir_y = [[-1, 0, 1, 0], [-1, 0, 1, 0], [1, 0, -1, 0] , [1, 0, -1, 0]]  #N - E - S - E,  N - W - S - W,  S - E - N - E,  S - W - N - W

    self.completed = False
    

  def timer_callback(self):    
    if self.completed is True:
      msg = Int8()
      msg.data = 1
      self.npw_ok_topic_pub.publish(msg)
    
    else:
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
        self.agent_infos[i].cur_status = -1
        self.agent_infos[i].cur_lat = -1
        self.agent_infos[i].cur_long = -1
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
        for i in range(0, self.max_agent_num):
          rp_ok = self.read_partition(i)
          if rp_ok is False:
            self.get_logger().info(f"{i}th robot: read partition text file fail ")
            continue
        
        self.read_gps_pivot()
        self.crop_row_width_conv()
        self.waypoint_gen()
        self.waypoint_gen_debug()
        self.completed = True
        
  def gps_callback(self, msg, robot_index):
    self.agent_infos[robot_index].cur_lat == msg.latitude
    self.agent_infos[robot_index].cur_long == msg.longitude
 
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
  
  def crop_row_width_conv(self):
    map_width_meter = haversine((self.pivot_lat, self.pivot_long), (self.pivot_lat + self.x_axis_del_lat, self.pivot_long + self.x_axis_del_long), unit=Unit.METERS)
    self.pixel_per_m =  self.map_width/map_width_meter
    self.crop_row_pixel = self.pixel_per_m * self.crop_row_width

    
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
      #cv2.imshow(f"{robot_index} robot bin_img ", bin_img)
      #cv2.waitKey(0)
      contours = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      contours = contours[0] if len(contours) == 2 else contours[1]   # if len is 2, first element is value and second one is hierarchy
      contour_props = []
      for contour in contours:
        x1, y1, w, h = cv2.boundingRect(contour)
        x2 = x1 + w
        y2 = y1 + h
        add = x1, y1, x2, y2
        contour_props.append(add)
      
      for data in contour_props:
        cv2.rectangle(bin_img, (data[0], data[1]), (data[2], data[3]), cv2.FILLED)
      #cv2.imshow(f"{robot_index} robot bin_img after drawing contour rect", bin_img)
      #cv2.waitKey(0)      
      self.agent_infos[robot_index].part_img = bin_img.copy() 
      
      x_init_correction = 0
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
      
      wp['x'] = wp['x'] + x_init_correction  
      self.agent_infos[robot_index].dir_cnt = self.agent_infos[robot_index].dir_cnt + 1
      self.agent_infos[robot_index].waypoint.append(wp)
      self.agent_infos[robot_index].wp_num = 0
      self.agent_infos[robot_index].cur_wp = 0

      rp_ok = True
      self.agent_infos[robot_index].enable = True
    except:
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
    
    
    while(lo + 1 < ho):
      mid = int((lo + ho)/2)
      if self.agent_infos[robot_index].part_img[mid, int(round_cur_x)] == 255:
        lo = mid
      else:
        ho = mid
    
    if forward is True:
      return lo
    else:
      return ho 
      
      
  def horizontal_search(self, robot_index, cur_x, cur_y, cur_dir_x):
    next_x_temp = cur_x + self.crop_row_pixel*cur_dir_x
    
    if next_x_temp < 0:
      next_x_temp = 0
    elif next_x_temp >= self.map_width:
      next_x_temp = self.map_width - 1
    
    round_next_x_temp = round(next_x_temp)
    
    if self.agent_infos[robot_index].part_img[int(cur_y), int(round_next_x_temp)] == 255:
      return next_x_temp, False
    
    next_x_temp = next_x_temp - self.crop_row_pixel*cur_dir_x/2
    round_next_x_temp = round(next_x_temp)
    if self.agent_infos[robot_index].part_img[int(cur_y), int(round_next_x_temp)] == 0:
      return cur_x, True
    
    #next_x_temp = next_x_temp + self.crop_row_pixel*cur_dir_x/2
    
    while(True):
      next_x_temp = next_x_temp + cur_dir_x
      round_next_x_temp = round(next_x_temp)
      if next_x_temp < 0 or next_x_temp >= self.map_width:
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

  def waypoint_gen_debug(self):
    for i in range(0, self.max_agent_num):
      if self.agent_infos[i].enable is False:
        continue
      debug_txt_dir = self.trunk_dir + '/agent_wp_debug_' + str(i) +'.txt'
      debug_img_dir = self.trunk_dir + '/agent_wp_debug_' + str(i) +'.png'
      f = open(debug_txt_dir, "w")
      f.writelines(f"waypoint number : {self.agent_infos[i].wp_num} ")
      f.writelines(f"current waypoint idx : {self.agent_infos[i].cur_wp} ")
      f.writelines(f"current dir idx : {self.agent_infos[i].dir_idx} ")
      for j, wp in enumerate(self.agent_infos[i].waypoint):
        f.writelines(f"{j}th waypoint x : {wp['x']},   waypoint y : {wp['y']}")
        img = self.agent_infos[robot_index].part_img.copy()
        img_rgb = np.zeros((img.shape[0], img.shape[1], 3))
        img_rgb[:,:,0] = img
        img_rgb[:,:,1] = img
        img_rgb[:,:,2] = img
        color = (255, 0, 0)
        img_rgb = cv2.circle(img_rgb, (int(wp['x']), int(wp['y'])), 1, color, 1)
        org_x = int(wp['x'])
        org_y = int(wp['y'])
        if org_x >= self.map_width:
          org_x = org_x - 2
        else:
          org_x = org_x + 2

        if org_y >= self.map_height:
          org_y = org_y - 2
        else:
          org_y = org_y + 2

        fontScale = 1
        thickness = 1
        img_rgb = cv2.putText(img_rgb, str(j), (org_x, org_y), cv2.FONT_HERSHEY_SIMPLEX, 
                   fontScale, (0, 255, 0), thickness, cv2.LINE_AA)

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
