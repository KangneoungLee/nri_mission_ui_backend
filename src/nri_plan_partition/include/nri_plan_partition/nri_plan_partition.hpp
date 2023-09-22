#ifndef NRI_PLAN_PARTITION_HPP_
#define NRI_PLAN_PARTITION_HPP_

#include <mutex>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/int8.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "nri_plan_partition/dynamic_voronoi.h"

#define PI 3.14159265


namespace nri_plan_partition
{ 


class NriPlanPartition : public rclcpp::Node
{
    private:
       std::string robot_gps_topic_;
       std::string density_save_dir_;
       std::string map_resol_txt_dir_;
       std::string datum_txt_dir_;
       std::string pivot_gps_lu_txt_dir_;
       std::string pivot_gps_lb_txt_dir_;
       std::string pivot_gps_ru_txt_dir_;
       std::string pivot_gps_rb_txt_dir_;
       std::string trunk_dir_;
       rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr NppOkPub_; // monitoring publisher 
       rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr NpiOkSub_;
       std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> gps_sub_vec_;
       
       long int LoopRate_Milisec_ = 2000;
       
       int map_height_ = 200;
       int map_width_ = 200;
       
       int max_pixel_length_ = 400;
       int max_agent_num_ = 10;
       int weight_width_ = 1;
       int weight_height_ = 0;
       int lamda_ = 1;
       
       bool is_hete_cov_radius_ = false;
       bool is_dropout_use_ = true;
       bool is_img_save_ = false;
       bool is_propa_connected_area_ = false;
       
       float datum_x_norm_;
       float datum_y_norm_;
       
       float x_axis_del_lat_;
       float x_axis_del_long_;
       float y_axis_del_lat_;
       float y_axis_del_long_;
       
       float home_lat_;
       float home_long_;
       bool gps_pivot_ok_;
       
       std::mutex lock_;
       
       float travel_time_duration_ = 20.0;
       
       bool running_ = false;
       bool npp_ok_ = 0;
       int cur_agent_num_ = 0;
       bool* agent_gps_receive_ = NULL;
       int* agent_idx_voro_ = NULL;
       struct gps_info* gps_info_array_ = NULL;
       float map_resol_ = 1.0;
       unsigned char* densityPtr_ = NULL;
       std::vector<unsigned char*> agent_partition_maps_;
       
       bool RealTime_InhbtDropout_ = false;
       bool RealTime_DropoutAct_ = false;
       float RealTime_CcMetric_ = 10;
       float RealTime_CcMetric_prev_ = 10;
       float RealTime_CcMetricDif_prev_ = 0;
       float RealTime_CcMetricDif_final_ = 0;
       float CcRate_ = 0.3;
       
       rclcpp::TimerBase::SharedPtr timer_;
       
       DynamicVoronoi* DynVoroHandle_;
       
       void InitialVariable();
       void WritePartition();
       void ReadDensity();
       void ReadDatum();
       void ReadMapResol();
       void GpsToMap(float latitude, float longitude, int & x_out, int & y_out);
       void NpiOkCallback(const std_msgs::msg::Int8 & msg);
       void GpsSubCallback(const sensor_msgs::msg::NavSatFix & msg, int index);
       void ReadAgentOffLine();
       void ReadGpsPivot();
     
    public:
       NriPlanPartition(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()); /*constructor*/
       ~NriPlanPartition(); /*destructor*/
       void Loop();    
};
}// namespace nri_plan_partition

#endif  // NRI_PLAN_PARTITION_HPP_
