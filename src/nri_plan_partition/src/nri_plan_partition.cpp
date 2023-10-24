#include "nri_plan_partition/nri_plan_partition.hpp"
#include "rclcpp/rclcpp.hpp"
//#include "boost/bind.hpp>

using namespace std::placeholders;
//using std::placeholders::_1;
using namespace std::chrono_literals;

namespace nri_plan_partition
{

struct gps_info
{
   double latitude;
   double longitude;
   rclcpp::Time receive_time;
   bool init_ok;
};

NriPlanPartition::NriPlanPartition(const rclcpp::NodeOptions & options):Node("nri_plan_partition_node")
{
    
    auto option = this->get_node_options();
    
    this->declare_parameter("robot_gps_topic", rclcpp::ParameterValue("global_position"));
    this->get_parameter("robot_gps_topic", robot_gps_topic_);
    
    this->declare_parameter("density_save_dir", rclcpp::ParameterValue("/trunk/density.png"));
    this->declare_parameter("map_resol_txt_dir", rclcpp::ParameterValue("/trunk/resol.txt"));
    this->declare_parameter("datum_txt_dir", rclcpp::ParameterValue("/trunk/datum.txt"));
    this->declare_parameter("pivot_gps_lu_txt_dir", rclcpp::ParameterValue("/trunk/gps_lu.txt"));
    this->declare_parameter("pivot_gps_lb_txt_dir", rclcpp::ParameterValue("/trunk/gps_lb.txt"));
    this->declare_parameter("pivot_gps_ru_txt_dir", rclcpp::ParameterValue("/trunk/gps_ru.txt"));
    this->declare_parameter("pivot_gps_rb_txt_dir", rclcpp::ParameterValue("/trunk/gps_rb.txt"));
    this->declare_parameter("trunk_dir", rclcpp::ParameterValue("/trunk/"));
   
    this->get_parameter("density_save_dir", density_save_dir_);
    this->get_parameter("map_resol_txt_dir", map_resol_txt_dir_);
    this->get_parameter("datum_txt_dir", datum_txt_dir_);
    this->get_parameter("pivot_gps_lu_txt_dir", pivot_gps_lu_txt_dir_);
    this->get_parameter("pivot_gps_lb_txt_dir", pivot_gps_lb_txt_dir_);
    this->get_parameter("pivot_gps_ru_txt_dir", pivot_gps_ru_txt_dir_);
    this->get_parameter("pivot_gps_rb_txt_dir", pivot_gps_rb_txt_dir_);
    this->get_parameter("trunk_dir", trunk_dir_);
    
    this->declare_parameter("max_pixel_length", rclcpp::ParameterValue(400));
    this->get_parameter("max_pixel_length", max_pixel_length_);

    this->declare_parameter("max_agent_num", rclcpp::ParameterValue(10));
    this->get_parameter("max_agent_num", max_agent_num_);
    
    this->declare_parameter("weight_width", rclcpp::ParameterValue(1));
    this->get_parameter("weight_width", weight_width_);

    this->declare_parameter("weight_height", rclcpp::ParameterValue(0));
    this->get_parameter("weight_height", weight_height_);

    this->declare_parameter("lamda", rclcpp::ParameterValue(1));
    this->get_parameter("lamda", lamda_);    

    this->declare_parameter("is_hete_cov_radius", rclcpp::ParameterValue(false));
    this->get_parameter("is_hete_cov_radius", is_hete_cov_radius_); 
    
    this->declare_parameter("is_dropout_use", rclcpp::ParameterValue(true));
    this->get_parameter("is_dropout_use", is_dropout_use_);
    
    this->declare_parameter("is_img_save", rclcpp::ParameterValue(false));
    this->get_parameter("is_img_save", is_img_save_);

    this->declare_parameter("is_propa_connected_area", rclcpp::ParameterValue(false));
    this->get_parameter("is_propa_connected_area", is_propa_connected_area_);

    this->declare_parameter("travel_time_duration", rclcpp::ParameterValue(20.0));
    this->get_parameter("travel_time_duration", travel_time_duration_);
    

    gps_info_array_ = new struct gps_info[max_agent_num_];
    gps_pivot_ok_ = false;
    
    for (int i = 0; i < max_agent_num_; i++)
    {
        gps_info_array_[i].latitude = -1.0;
        gps_info_array_[i].longitude = -1.0;
        gps_info_array_[i].init_ok = false;
        unsigned char* agent_part_map = NULL;
        agent_partition_maps_.push_back(agent_part_map);
    }
    
    datum_x_norm_ = 0.0;
    datum_y_norm_ = 0.0;
    
    RealTime_InhbtDropout_ = false;
    
    float DropOutWeight = ((float)max_agent_num_)/((float)max_agent_num_-1);	
    
    DynVoroHandle_ = new DynamicVoronoi(map_height_, map_width_, DropOutWeight, weight_width_, weight_height_, lamda_, false, is_dropout_use_, false, is_hete_cov_radius_, is_propa_connected_area_, is_img_save_, densityPtr_);
    //this->ReadAgentOffLine(); // just for test purpose
    
    //(const std::string &topic_name, size_t qos_history_depth, CallbackT &&callback)   
    NpiOkSub_ = this->create_subscription<std_msgs::msg::Int8>("nri_plan_interface_ok", 10, std::bind(&NriPlanPartition::NpiOkCallback, this, _1));
    NppOkPub_ = this->create_publisher<std_msgs::msg::Int8>("nri_plan_partition_ok", 10);
    
    this->declare_parameter("loop_rate_milisec", rclcpp::ParameterValue(2000));
    this->get_parameter("loop_rate_milisec", LoopRate_Milisec_);
    
    timer_ = create_wall_timer(std::chrono::milliseconds(LoopRate_Milisec_), std::bind(&NriPlanPartition::Loop, this));
    
    for(int index=0; index< max_agent_num_; index++)
    {
      std::string robot_str = "Robot"; 
      std::string sub_topic = robot_str + std::to_string(index) + "/" + robot_gps_topic_;
      std::function<void(sensor_msgs::msg::NavSatFix)> fnc = std::bind(&NriPlanPartition::GpsSubCallback, this, _1, index);
      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(sub_topic, 2, fnc);
      gps_sub_vec_.push_back(gps_sub);
    
    }

}

void NriPlanPartition::InitialVariable()
{
    RealTime_InhbtDropout_ = false;
    RealTime_DropoutAct_ = false;
    RealTime_CcMetric_ = 10;
    RealTime_CcMetric_prev_ = 10;
    RealTime_CcMetricDif_prev_ = 0;
    RealTime_CcMetricDif_final_ = 0;
    gps_pivot_ok_ = false;
}

void NriPlanPartition::GpsSubCallback(const sensor_msgs::msg::NavSatFix & msg, int index)
{
    gps_info_array_[index].latitude = msg.latitude;
    gps_info_array_[index].longitude = msg.longitude;
    gps_info_array_[index].init_ok = true;
    gps_info_array_[index].receive_time = this->get_clock()->now();
}

void NriPlanPartition::GpsToMap(float lat_in, float long_in, int & x_out, int & y_out)
{
    
    double x_axis_del_lat_scale = x_axis_del_lat_ * 10000000;
    double x_axis_del_long_scale = x_axis_del_long_ * 10000000;
    
    double y_axis_del_lat_scale = y_axis_del_lat_ * 10000000;
    double y_axis_del_long_scale = y_axis_del_long_ * 10000000;
    
    double del_lat_in = lat_in - home_lat_;
    double del_long_in = long_in - home_long_;
    
    double del_lat_in_scale = del_lat_in * 10000000;
    double del_long_in_scale = del_long_in * 10000000;
    
    if((x_axis_del_lat_scale < 1)&&(x_axis_del_lat_scale >= 0))
    {
      x_axis_del_lat_scale = 1;
    }
    else if ((x_axis_del_lat_scale > -1)&&(x_axis_del_lat_scale < 0))
    {
      x_axis_del_lat_scale = -1;
    }
    
    if ((x_axis_del_long_scale < 1)&&(x_axis_del_long_scale >= 0))
    {
      x_axis_del_long_scale = 1;
    }
    else if ((x_axis_del_long_scale > -1)&&(x_axis_del_long_scale < 0))
    {
      x_axis_del_long_scale = -1;
    }
    
    if ((y_axis_del_lat_scale < 1)&&(y_axis_del_lat_scale >= 0))
    {
      y_axis_del_lat_scale = 1;
    }
    else if ((y_axis_del_lat_scale > -1)&&(y_axis_del_lat_scale < 0))
    {
      y_axis_del_lat_scale = -1;
    }
    
    if ((y_axis_del_long_scale < 1)&&(y_axis_del_long_scale >= 0))
    {
      y_axis_del_long_scale = 1;
    }
    else if ((y_axis_del_long_scale > -1)&&(y_axis_del_long_scale < 0))
    {
      y_axis_del_long_scale = -1;
    }
       
    double b = (del_lat_in_scale/x_axis_del_lat_scale - del_long_in_scale/x_axis_del_long_scale)/(y_axis_del_lat_scale/x_axis_del_lat_scale - y_axis_del_long_scale/x_axis_del_long_scale);
    double a = (del_lat_in_scale/y_axis_del_lat_scale - del_long_in_scale/y_axis_del_long_scale)/(x_axis_del_lat_scale/y_axis_del_lat_scale - x_axis_del_long_scale/y_axis_del_long_scale);
    
    x_out = (int)(a*(double)map_width_);
    y_out = map_height_ - (int)(b*(double)map_height_);
    
    if(x_out <= 1) x_out = 2;
    else if(x_out >= map_width_ - 1)  x_out = map_width_ - 2;
    
    if(y_out <= 1) y_out = 2;
    else if(y_out >= map_height_ - 1)  y_out = map_height_ - 2;

}

void NriPlanPartition::ReadGpsPivot()
{

    std::ifstream pivot_gps_lb_text;
    pivot_gps_lb_text.open(pivot_gps_lb_txt_dir_);
    
    std::string text_s;
    std::getline(pivot_gps_lb_text, text_s);
    std::istringstream ss(text_s);
    std::string word;
    pivot_gps_lb_text.close();
    
    ss >> word;
    float lb_lat = std::stof(word);
    home_lat_ = lb_lat;
    
    ss >> word;
    float lb_long = std::stof(word);
    home_long_ = lb_long;
    
    
    std::ifstream pivot_gps_lu_text;
    pivot_gps_lu_text.open(pivot_gps_lu_txt_dir_);
    
    std::string text_s2;
    std::getline(pivot_gps_lu_text, text_s2);
    std::istringstream ss2(text_s2);
    pivot_gps_lu_text.close();
    
    ss2 >> word;
    float lu_lat = std::stof(word);
    
    ss2 >> word;
    float lu_long = std::stof(word); 
    
    
    std::ifstream pivot_gps_rb_text;
    pivot_gps_rb_text.open(pivot_gps_rb_txt_dir_);
    
    std::string text_s3;
    std::getline(pivot_gps_rb_text, text_s3);
    std::istringstream ss3(text_s3);
    pivot_gps_rb_text.close();
    
    ss3 >> word;
    float rb_lat = std::stof(word);
    
    ss3 >> word;
    float rb_long = std::stof(word); 
    
    y_axis_del_lat_ = lu_lat - lb_lat;
    y_axis_del_long_ = lu_long - lb_long;
    x_axis_del_lat_ = rb_lat - lb_lat;
    x_axis_del_long_ = rb_long - lb_long;
   
    gps_pivot_ok_ = true;
    std::cout << "____Read GPS Pivot Information completed____ " << std::endl;
}

void NriPlanPartition::NpiOkCallback(const std_msgs::msg::Int8 & msg)
{
    if (msg.data == 0) 
    {
       npp_ok_ = 0;
       this->InitialVariable();
    }
    else if((npp_ok_ == 0)&&(running_ == false))
    {
        // insert read gps pivot
        this->ReadGpsPivot();
        DynVoroHandle_->InitializeAgent();
        this->ReadDensity();
        DynVoroHandle_->ResizeMap(map_height_, map_width_, densityPtr_);
        
        for (int i = 0; i < max_agent_num_; i++)
        {
          if (gps_info_array_[i].init_ok == false) continue;
          if ((this->get_clock()->now() - gps_info_array_[i].receive_time) > rclcpp::Duration(2, 0)) continue;
          int x_pos_pixel, y_pos_pixel;
          this->GpsToMap(gps_info_array_[i].latitude, gps_info_array_[i].longitude, x_pos_pixel, y_pos_pixel);
          int temp_cov_radius = 20;
          DynVoroHandle_->PushPoint(x_pos_pixel, y_pos_pixel, i, temp_cov_radius); 
        }
        
        cur_agent_num_ = DynVoroHandle_->CurAgentNum();
        
        this->GenTempPartition();
        
        running_ = true;

        this->ReadDatum();
        
        DynVoroHandle_->PushDatum(datum_x_norm_*((float)map_width_), datum_y_norm_*((float)map_height_));
        
        std::cout << "____Datum set completed____ " << " datum_x_ : "<< datum_x_norm_*((float)map_width_) << " datum_y_ : "<< datum_y_norm_*((float)map_height_) <<std::endl;
        
        float DropOutWeight = 1024.0;
        if (cur_agent_num_ <= 1) RealTime_InhbtDropout_ = true;
        else DropOutWeight = ((float)cur_agent_num_)/((float)cur_agent_num_-1);
        
        std::string img_dir =  "/home/artlab/nri_plan_ws/src/nri_plan_partition/init_pos_test/";
	std::string label_dir = "/home/artlab/nri_plan_ws/src/nri_plan_partition/label/label.txt";
	std::string img_save_dir = img_dir + "map_" + "_0"+"_init" +".png";
	DynVoroHandle_->Colorized(img_save_dir,label_dir);

        DynVoroHandle_->InitializeCell();
        DynVoroHandle_->DropOutWupdate(DropOutWeight);
        DynVoroHandle_->ExpandedVoronoi();
        DynVoroHandle_->CentroidCal();
        DynVoroHandle_->MoveAgents();
        
        rclcpp::Time init_time = this->get_clock()->now();
        while( (this->get_clock()->now() - init_time) < rclcpp::Duration(2, 0))
        {
           if((is_dropout_use_ == true)&&(RealTime_InhbtDropout_ == false))
           {
               init_time = this->get_clock()->now();
               
               RealTime_DropoutAct_ = DynVoroHandle_->AgentDropOut();
               
               if (RealTime_DropoutAct_ == true)
               {
                   DynVoroHandle_->InitializeCell();
                   DynVoroHandle_->ExpandedVoronoi();
                   DynVoroHandle_->CentroidCal();
                   DynVoroHandle_->MoveAgents();
                   DynVoroHandle_->ResetDropout();
                   RealTime_DropoutAct_ = false; 
               }
           }
           
           DynVoroHandle_->InitializeCell();
           DynVoroHandle_->ExpandedVoronoi();
           DynVoroHandle_->CentroidCal();
           DynVoroHandle_->MoveAgents();
           
           RealTime_CcMetric_prev_ = RealTime_CcMetric_;
           RealTime_CcMetric_ = DynVoroHandle_->CoverageMetric();
           float RealTime_CcMetricDif =  RealTime_CcMetric_prev_ - RealTime_CcMetric_;
           RealTime_CcMetricDif_final_ = CcRate_*RealTime_CcMetricDif + (1-CcRate_)*RealTime_CcMetricDif_prev_;
           RealTime_CcMetricDif_prev_ = RealTime_CcMetricDif_final_;
           
           if ((RealTime_InhbtDropout_ == false)&&(RealTime_CcMetricDif_final_ <= 0.05))
           {
                RealTime_InhbtDropout_ = true;
           }
        }
        
        std::cout << "____Internal Partitioning competed____ " << std::endl;
  
        this->WritePartition();
        
        npp_ok_ = 1;
        running_ = false;
    }
}


void NriPlanPartition::WritePartition()
{
    for (int x = 0; x<map_width_;x++)
    {
        for (int y = 0; y<map_height_;y++)
	{
	    //std::cout << "x : "  << x << " y: " << y << std::endl;
	    VoroCell* vorocell_temp = DynVoroHandle_->GetSingleCellByIndex(x, y);
	    int cell_index = DynVoroHandle_->GetIndex(x, y);
	    //std::cout << "x : "  << x << " y: " << y << " cell_index : " << cell_index<< std::endl;
	    int agent_index = vorocell_temp->agentclass_;
	    //std::cout << "x : "  << x << " y: " << y << " agent_index : " << agent_index<< std::endl;
	    float density = DynVoroHandle_->GetDensity(x,y);
            //std::cout << "x : "  << x << " y: " << y << " density : " << density<< std::endl;	  
	    if((agent_index >= cur_agent_num_)||(density <= 0.0)) continue;
	    
	    unsigned char* temp_map =  agent_partition_maps_[agent_index];
	    temp_map[cell_index] = 128;
			
	    
	}
    }
    
    std::cout << "____Internal Partition copy completed____ " << std::endl;
	    
    for(int i = 0; i < cur_agent_num_; i++)
    {
        //std::stringstream s_stream(trunk_dir_);
	//std::string substr;
	//std::getline(s_stream, substr, '.');
	int restored_index = DynVoroHandle_->GetActualIndex(i);
	std::string part_img_dir = trunk_dir_ + "/agent" + std::to_string(restored_index) + ".png";
	unsigned char* temp_map =  agent_partition_maps_[i]; // the index should be 'i' because the agent_partition_maps_ is referring the index in DynamicVoronoi
	cv::Mat part_mat(map_height_, map_width_, CV_8UC1, temp_map);
	bool check = cv::imwrite(part_img_dir, part_mat);
	//rclcpp::Time init_time = this->get_clock()->now();
	//while( (this->get_clock()->now() - init_time) < rclcpp::Duration(1, 0)) 
	//{bool dummy = true;}
	
	//std::cout << " agent num :  " << restored_index << " cv imwrite check : " << check << std::endl;  
	
	std::ofstream StartingInfoTxt;
	std::string info_dir =   trunk_dir_ + "/agent" + std::to_string(restored_index) + ".txt";
	StartingInfoTxt.open(info_dir);
	
	int start_x_pixel, start_y_pixel;
	DynVoroHandle_->GetStartingPt(i, start_x_pixel, start_y_pixel); // the index should be 'i' because the starting point is referring the index in DynamicVoronoi
	
	//std::cout << " start_x_pixel :  " << start_x_pixel << " start_y_pixel : " << start_y_pixel << std::endl;
	
	StartingInfoTxt << start_x_pixel <<" " << start_y_pixel << std::endl;
	StartingInfoTxt.close();
    }
    
    std::cout << "____Save partiton result completed____ " << std::endl;
	
}

void NriPlanPartition::ReadDensity()
{
    cv::Mat img = cv::imread(density_save_dir_, cv::IMREAD_GRAYSCALE); //cv::imread(density_save_dir_, cv::IMREAD_GRAYSCALE);
    int width_org = img.cols;
    int height_org = img.rows;
    
    if ((width_org >= height_org)&&(width_org > max_pixel_length_))
    {
    	float ratio = ((float)height_org)/((float)width_org);
    	map_width_ = (int)max_pixel_length_;
    	map_height_ = (int)(ratio*((float)map_width_));
    }
    else if((height_org >= width_org)&&(height_org > max_pixel_length_))
    {
    	float ratio = ((float)width_org)/((float)height_org);
    	map_height_ = (int)max_pixel_length_;
    	map_width_ = (int)(ratio*((float)map_height_));    
    }
    else
    {
        map_width_ = width_org;
        map_height_ = height_org;
    }
    
    if(!(densityPtr_ == NULL)) delete[] this->densityPtr_;
    densityPtr_ = new unsigned char[map_height_*map_width_];
    cv::resize(img, img, cv::Size(map_width_, map_height_));
    cv::Mat mat_255(cv::Size(map_width_, map_height_), CV_8UC1, cv::Scalar(255));
    
    cv::absdiff(mat_255, img, img);
    //cv::imwrite("/home/artlab/nri_plan_ws/src/trunk/density_resize.png", img);
    
    img.convertTo(img, CV_32FC1);  //refering unsigned char type is not working so convert image to float type

    for(int row=0; row<map_height_; row++)  
    {
        for(int col=0; col<map_width_; col++)
	{
            int index =col + row*map_width_;
	    densityPtr_[index] =  (unsigned char)img.at<float>(row, col);
	}
    }


    std::cout << "____Read Density Map completed____ " << std::endl;
}

void NriPlanPartition::GenTempPartition()
{
    
    for (int i =0; i < agent_partition_maps_.size(); i ++)
    {
        if(!(agent_partition_maps_[i] == NULL)) delete[] agent_partition_maps_[i];        
    }
    
    if(!agent_partition_maps_.empty()) agent_partition_maps_.clear();
    
    for (int i =0; i < cur_agent_num_; i ++)
    {
        unsigned char* part_map = new unsigned char[map_height_*map_width_];
        memset(part_map, 255, map_height_*map_width_*sizeof(unsigned char));
        agent_partition_maps_.push_back(part_map);
    }
    
    std::cout << "____Generate Temp partition completed____ " << std::endl;

}

void NriPlanPartition::ReadDatum()
{
    std::ifstream datum_text;
    datum_text.open(datum_txt_dir_);
    
    std::string text_s;
    std::getline(datum_text, text_s);
    std::istringstream ss(text_s);
    std::string word;
    
    ss >> word;
    
    datum_x_norm_ = std::stof(word);

    ss >> word;
    
    datum_y_norm_ = std::stof(word);
    //datum_y_norm_ = 1 - datum_y_norm_; //this is because the axis is different with UI


}


void NriPlanPartition::ReadMapResol()
{
    std::ifstream mapresol_text;
    mapresol_text.open(map_resol_txt_dir_);
    
    std::string text_s;
    std::getline(mapresol_text, text_s);
    std::istringstream ss(text_s);
    std::string word;
    
    ss >> word;
    
    map_resol_ = std::stof(word);


}

void NriPlanPartition::ReadAgentOffLine()
{
    cur_agent_num_ = 0;
    std::string agent_info_dir = "/home/artlab/nri_plan_ws/src/trunk/agent_info_test.txt";
    std::ifstream agent_info_text;
    agent_info_text.open(agent_info_dir);
    for(int i = 0; i < max_agent_num_; i++)
    {
       std::string text_s;
       std::getline(agent_info_text,text_s);
       std::istringstream ss(text_s);
       std::string word;
       
       ss>>word; // to remove index

       ss>>word;
       float x_pos_pixel = std::stof(word);

       ss>>word;
       float y_pos_pixel = std::stof(word);

       ss>>word;
       float max_speed = std::stof(word);
       float cov_radius = max_speed * travel_time_duration_;

       DynVoroHandle_->PushPoint(x_pos_pixel, y_pos_pixel, cov_radius); 
       cur_agent_num_ = cur_agent_num_ + 1;

    }
}

void NriPlanPartition::Loop()
{
    auto msg = std_msgs::msg::Int8();
    if ((npp_ok_ == 0)&&(running_ == true)) msg.data = 2;
    else msg.data = npp_ok_;
    NppOkPub_->publish(msg);
}


NriPlanPartition::~NriPlanPartition()
{
    delete DynVoroHandle_;
    delete[] densityPtr_;
    delete[] gps_info_array_;

    for (int i = 0; i < agent_partition_maps_.size(); i++)
    {
        if(!(agent_partition_maps_[i] == NULL)) delete[] agent_partition_maps_[i];
    }
}


}// namespace nri_plan_partition


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nri_plan_partition::NriPlanPartition>();
    //node->ParamClient();
    //node->Loop();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
