#include <charging_indentifi/charging_indentifi.h>

// STL Includes.
#include <math.h>
#include <algorithm>


ChargInden::ChargInden(rclcpp::Node::SharedPtr node) :
  node_(node),
  running_(false),
  tracking_frame_("base_laser"),
  found_dock_(false),
  init_dock(false)
{
  createLogFile();
  node_->declare_parameter<std::string>("publish_frame", "base_link");
  node_->declare_parameter<float>("max_alignment_error", 0.01);
  node_->declare_parameter<float>("vmark_angle", 2.25);
  node_->declare_parameter<float>("vmark_length", 0.239);
  node_->declare_parameter<bool>("debug_mode", true);
  node_->declare_parameter<float>("split_thresh", 0.03);
  node_->declare_parameter<int>("split_min_point", 15);
  node_->declare_parameter<float>("scan_to_baselink_transform.px", 0.195);
  node_->declare_parameter<float>("scan_to_baselink_transform.py", 0.0);
  node_->declare_parameter<float>("scan_to_baselink_transform.pz", 0.1227);
  node_->declare_parameter<float>("scan_to_baselink_transform.roll", 0.0);
  node_->declare_parameter<float>("scan_to_baselink_transform.pitch", 0.0);
  node_->declare_parameter<float>("scan_to_baselink_transform.yaw", 1.56);
  node_->declare_parameter<double>("vangle", 0.7);
  node_->declare_parameter<double>("angle_tolerance", 0.2);
  node_->declare_parameter<double>("rdp_epsilon", 0.007);
  node_->declare_parameter<double>("vlength", 0.12);
  node_->declare_parameter<double>("length_tolerance", 0.04);
  node_->declare_parameter<float>("scan_range_min", 0.05);
  node_->declare_parameter<float>("scan_range_max", 2.0);
  node_->declare_parameter<float>("scan_angle_min", 3.92);
  node_->declare_parameter<float>("scan_angle_max", 6.28);
  node_->declare_parameter<float>("mask_threshold", 0.02);
  node_->declare_parameter<float>("dist_vox", 0.01);



  node_->get_parameter("publish_frame", param_.publish_frame);
  node_->get_parameter("max_alignment_error", param_.max_alignment_error);
  node_->get_parameter("vmark_angle", param_.vmark_angle);
  node_->get_parameter("vmark_length", param_.vmark_length);
  node_->get_parameter("debug_mode", param_.debug_mode);
  node_->get_parameter("split_thresh", param_.split_thresh);
  int split_min_point;
  node_->get_parameter("split_min_point", split_min_point);
  param_.split_min_point = static_cast<uint32_t>(split_min_point);
  node_->get_parameter("vangle", param_.vangle);
  node_->get_parameter("angle_tolerance", param_.angle_tolerance);
  node_->get_parameter("rdp_epsilon", param_.rdp_epsilon);
  node_->get_parameter("vlength", param_.vlength);
  node_->get_parameter("length_tolerance", param_.length_tolerance);
  node_->get_parameter("scan_range_min", param_.scan_range_min);
  node_->get_parameter("scan_range_max", param_.scan_range_max);
  node_->get_parameter("scan_angle_min", param_.scan_angle_min);
  node_->get_parameter("scan_angle_max", param_.scan_angle_max);
  node_->get_parameter("mask_threshold", param_.mask_threshold);
  node_->get_parameter("dist_vox", param_.dist_vox);

  float px, py, pz, roll, pitch, yaw;
  node_->get_parameter("scan_to_baselink_transform.px", px);
  node_->get_parameter("scan_to_baselink_transform.py", py);
  node_->get_parameter("scan_to_baselink_transform.pz", pz);
  node_->get_parameter("scan_to_baselink_transform.roll", roll);
  node_->get_parameter("scan_to_baselink_transform.pitch", pitch);
  node_->get_parameter("scan_to_baselink_transform.yaw", yaw);
  tf2::Vector3 translation2;
  translation2.setValue(px, py, pz);
  tf2::Quaternion quaternion2;
  quaternion2.setRPY(roll, pitch, yaw);
  param_.scan_to_baselink_transform.setOrigin(translation2);
  param_.scan_to_baselink_transform.setRotation(quaternion2);


  float b_arr[] = {0.20657,  0.41314, 0.20657};
  float a_arr[] = {1.00000, -0.36953, 0.19582};
  std::vector<float> b(b_arr, b_arr + sizeof(b_arr)/sizeof(float));
  std::vector<float> a(a_arr, a_arr + sizeof(a_arr)/sizeof(float));
  dock_pose_filter_.reset(new LinearPoseFilter2D(b, a));

  // load point cloud
  // V mark
  float maxx = param_.vmark_length*0.5 / tan(param_.vmark_angle*0.5);
  if (1)  
  { 
    for (double x = 0; x < 0.123; x += 0.005)
    {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.z = 0;
      if (x <= maxx)
      {
        p.y = (maxx-x)*tan(param_.vmark_angle*0.5);
        ideal_cloud_.push_back(p);
        front_cloud_.push_back(p);
        p.y = -p.y;
        ideal_cloud_.push_back(p);
        front_cloud_.push_back(p);
      }
    }
  }
  else
  {
    // trapezoid mark
    for (double y = -0.091; y <= 0.091; y += 0.001)
    {
      geometry_msgs::msg::Point p;
      p.x = p.z = 0.0;
      p.y = y;
      ideal_cloud_.push_back(p);
      front_cloud_.push_back(p);
    }
    for (double x = 0.0; x < 0.048; x += 0.001)
    {
      geometry_msgs::msg::Point p;
      p.x = -x;
      p.y = 0.091 + x*1.15;
      p.z = 0.0;
      ideal_cloud_.push_back(p);
      p.y = -0.091 - x*1.15;
      ideal_cloud_.insert(ideal_cloud_.begin(), p);
    }
  }


  if (param_.debug_mode)
  {
    debug_points_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("dock_points", 10);
    debug_points2_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("dock_points2", 10);
    debug_lines_ = node_->create_publisher<visualization_msgs::msg::Marker>("dock_lines", 10);
  }
  srv_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&ChargInden::callback, this, std::placeholders::_1));
  StationPose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("station_pose", 1);
  service_ = node_->create_service<app_msgs::srv::ChargerLocation>("charge_location", 
            std::bind(&ChargInden::handle_service_request, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,srv_cb_group_);

}


ChargInden::~ChargInden()
{
}

void ChargInden::createLogFile() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y%m%d%H%M%S");
    //return oss.str();
    std::string log_directory = "log";
    std::string log_file_path = log_directory + "/charge_node" + oss.str() + ".log";

    // 检查日志目录是否存在，如果不存在则创建
    if (!std::filesystem::exists(log_directory)) {
        std::filesystem::create_directory(log_directory);
    }

    // 打开日志文件
    log_file_.open(log_file_path, std::ios::out | std::ios::app);
    /* if (!log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_file_path.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Log file created: %s", log_file_path.c_str());
        log_file_ << "Log file created" << std::endl;
    } */
}

void ChargInden::handle_service_request(const std::shared_ptr<app_msgs::srv::ChargerLocation::Request> request,
                                std::shared_ptr<app_msgs::srv::ChargerLocation::Response> response)
{
  geometry_msgs::msg::PoseStamped pose;
  bool start_ = start(pose);
  int i = 0;
  while (!init_dock)
  {
    i++;
    usleep(200000);
    if (i > 30)
    {
      break;
    }
    
  }
  
  stop();
  if (found_dock_)
  {
    response->state = 0;
    response->charger_pose = dock_;
    std::cout << "dock_.pose.position" << dock_.pose.position.x << ", " << dock_.pose.position.y << ", " << dock_.pose.position.z << std::endl;
    log_file_ << "dock_.pose.position" << dock_.pose.position.x << ", " << dock_.pose.position.y << ", " << dock_.pose.position.z << std::endl;
    //std::cout << "dock_.pose.position" << dock_.pose.position<< std::endl;
  }
  else{
    response->state = 1;
    std::cout << "NOT FOUND DOCK" << std::endl;
    log_file_ << "NOT FOUND DOCK" << std::endl;
    //response->charger_pose = dock_;
  }
  
}

double ChargInden::getPoseDistance(const geometry_msgs::msg::Pose a,
                       const geometry_msgs::msg::Pose b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  return sqrt(dx*dx + dy*dy);
}


bool ChargInden::start(const geometry_msgs::msg::PoseStamped& pose)
{
  found_dock_ = false;
  dock_ = pose;
  running_ = true;
  init_dock = false;
  return true;
}

bool ChargInden::stop()
{
  running_ = false;
  return true;
}


void ChargInden::callback(const sensor_msgs::msg::LaserScan scan)
{
  
  static int found_dock_num  = 0;
  static float init_score  = 1;
  static geometry_msgs::msg::Pose init_best_pose;
  if (!running_ || init_dock)
  // if (!running_)
  {
    found_dock_num = 0;
    init_score = 1;
    return;
  } 
  //std::cout << "!!!!!!!!!!!!found_dock_num  " << found_dock_num << std::endl;
  
  if (found_dock_num > 10)
  {
    init_dock = true;
  } 
  found_dock_num++;
  rclcpp::Time now1 = node_->now();
  
  //std::cout << "ChargInden::run" << std::endl;

  /* if (dock_.header.frame_id == "" ||
      (dock_.pose.orientation.z == 0.0 && dock_.pose.orientation.w == 0.0))
  {
    //std::cout << "null orientation" << std::endl;

    for (size_t i = scan.ranges.size()/2; i < scan.ranges.size(); i++)
    {
      if (std::isfinite(scan.ranges[i]))
      { 
        double angle = scan.angle_min + i * scan.angle_increment;*/
        dock_.header = scan.header;
        dock_.pose.position.x = 0.0;
        dock_.pose.position.y = 0.0;
        dock_.pose.orientation.x = 0.0;
        dock_.pose.orientation.y = 0.0;
        dock_.pose.orientation.z = 0.0;
        dock_.pose.orientation.w = 1.0;
        
/*         break;
      }
    }
  } */
  laser_processor::ScanMask mask;
  laser_processor::ScanProcessor processor(scan, mask, param_);
  // split 
  processor.splitConnected(param_.split_thresh);  //param
  processor.removeLessThan(param_.split_min_point); //param

///////////////////////////////////////////////////////////////
  if(0)
  {
    int i1 = 0;
    sensor_msgs::msg::PointCloud2 cloud1;
    cloud1.header.frame_id = "base_laser"; // 设置坐标系
    cloud1.width = 3000; // 假设有 100 个点
    cloud1.height = 0; // 非有序点云
    //cloud.is_bigendian = false;
    //cloud.is_dense = false; // 没有无效点
    sensor_msgs::PointCloud2Modifier modifier1(cloud1);
    modifier1.setPointCloud2FieldsByString(2,"xyz", "rgb");
    modifier1.resize(3000);
    sensor_msgs::PointCloud2Iterator<float> iter_xyz(cloud1, "x");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud1, "rgb");

      for (std::list<laser_processor::SampleSet*>::iterator j = processor.getClusters().begin();
          j != processor.getClusters().end();
          j++)
      {
        laser_processor::SampleSet* cluster = *j;
        for (laser_processor::SampleSet::iterator p = cluster->begin();
            p != cluster->end();
            p++)
        {
          iter_xyz[0] = (*p)->x;
          iter_xyz[1] = (*p)->y;
          iter_xyz[2] = 0.0;

          if (i1 == 0)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 255; // R
            iter_rgb[1] = 0;   // G
            iter_rgb[2] = 0;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 1)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 0; // R
            iter_rgb[1] = 255;   // G
            iter_rgb[2] = 0;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 2)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 0; // R
            iter_rgb[1] = 0;   // G
            iter_rgb[2] = 255;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 3)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 255; // R
            iter_rgb[1] = 0;   // G
            iter_rgb[2] = 255;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 4)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 0; // R
            iter_rgb[1] = 255;   // G
            iter_rgb[2] = 255;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else 
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 255; // R
            iter_rgb[1] = 255;   // G
            iter_rgb[2] = 0;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          //++i;
          ++iter_xyz;
          ++iter_rgb;
        }
        i1++;
      }
    debug_points2_->publish(cloud1);
    return;
  }
  //return;
//////////////////////////////////////

  std::priority_queue<DockCandidatePtr, std::vector<DockCandidatePtr>, CompareCandidates> candidates;
  std::vector<DockCandidatePtr> candidates1;
  std::cout << "found_dock_ " << found_dock_ << std::endl;
  log_file_ << "found_dock_ " << found_dock_ << std::endl;
  for (std::list<laser_processor::SampleSet*>::iterator i = processor.getClusters().begin();
       i != processor.getClusters().end();
       i++)
  {
    DockCandidatePtr c = extract(*i, candidates1);
    if (c->points.size() )
    {
      candidates.push(c);
    }
  }


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(param_.debug_mode)
  {
    int i1 = 0;
    sensor_msgs::msg::PointCloud2 cloud1;
    cloud1.header.frame_id = "base_laser"; // 设置坐标系
    cloud1.width = 3000; // 假设有 100 个点
    cloud1.height = 0; // 非有序点云
    //cloud.is_bigendian = false;
    //cloud.is_dense = false; // 没有无效点
    sensor_msgs::PointCloud2Modifier modifier1(cloud1);
    modifier1.setPointCloud2FieldsByString(2,"xyz", "rgb");
    modifier1.resize(3000);
    sensor_msgs::PointCloud2Iterator<float> iter_xyz(cloud1, "x");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud1, "rgb");
      std::cout << "candidates1.size();" << candidates1.size() << std::endl;
      log_file_ << "candidates1.size();" << candidates1.size() << std::endl;
      for (size_t i = 0; i < candidates1.size(); i++)
      {
        DockCandidatePtr cand = candidates1[i];
        
        for (size_t j = 0; j < cand->points.size(); j++)
        {
          iter_xyz[0] = cand->points[j].x;
          iter_xyz[1] = cand->points[j].y;
          iter_xyz[2] = 0.0;

          if (i1 == 0)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 255; // R
            iter_rgb[1] = 0;   // G
            iter_rgb[2] = 0;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 1)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 0; // R
            iter_rgb[1] = 255;   // G
            iter_rgb[2] = 0;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 2)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 0; // R
            iter_rgb[1] = 0;   // G
            iter_rgb[2] = 255;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 3)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 255; // R
            iter_rgb[1] = 0;   // G
            iter_rgb[2] = 255;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else if(i1 == 4)
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 0; // R
            iter_rgb[1] = 255;   // G
            iter_rgb[2] = 255;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          else 
          {
            // 设置颜色（RGBA）
            iter_rgb[0] = 255; // R
            iter_rgb[1] = 255;   // G
            iter_rgb[2] = 0;   // B
            //iter_rgb[3] = 255; // Alpha
          }
          //++i;
          ++iter_xyz;
          ++iter_rgb;
        }
        i1++;
      }
    debug_points2_->publish(cloud1);
  }
  //return;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





  std::cout << "candidates.size() " << candidates.size() << std::endl;
  log_file_ << "candidates.size() " << candidates.size() << std::endl;
  DockCandidatePtr best;
  geometry_msgs::msg::Pose best_pose;
  double min_score = 1;
  while (!candidates.empty())
  {
    geometry_msgs::msg::Pose pose = dock_.pose;
    double score = fit(candidates.top(), pose);
    //std::cout << "score " << score << std::endl;
    if (score <= min_score && score >=0)
    {
      best = candidates.top();
      best_pose = pose;
      min_score = score;
    }
    else  // Let's see what's wrong with this point cloud.
    {
      if (0)
      {
        DockCandidatePtr not_best = candidates.top();

        // Create point cloud
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = scan.header.stamp;
        cloud.header.frame_id = tracking_frame_;
        cloud.width = cloud.height = 0;

        // Allocate space for points
        sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
        cloud_mod.setPointCloud2FieldsByString(1, "xyz");
        cloud_mod.resize(not_best->points.size());

        // Fill in points
        sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
        for (size_t i = 0; i < not_best->points.size(); i++)
        {
          cloud_iter[0] = not_best->points[i].x;
          cloud_iter[1] = not_best->points[i].y;
          cloud_iter[2] = not_best->points[i].z;
          ++cloud_iter;
        }
        debug_points_->publish(cloud);
      }
    }
    candidates.pop();
  }
  log_file_ << "min_score " << min_score << std::endl;
  std::cout << "min_score " << min_score << std::endl;
  if (min_score > 0.02)
  {
    std::cout << "!!!!!!!!!not ind!!!!!!!!!!!!!!!" << std::endl;
    log_file_ << "!!!!!!!!!not ind!!!!!!!!!!!!!!!" << std::endl;
  }
  
  /* if (!best)
  {
    std::cout << "***************** NOT FOUND " << min_score << std::endl;
    //return;
  }   */

  if (!init_dock)
  {
    if (min_score < init_score)
    {
      init_score = min_score;
      init_best_pose = best_pose;
    }
    if (init_score < 0.02)
    {
      found_dock_ = true;

      tf2::Transform transform;
      tf2::Vector3 translation;
      translation.setValue(init_best_pose.position.x, init_best_pose.position.y, init_best_pose.position.z);
      tf2::Quaternion quaternion;
      quaternion.setValue(init_best_pose.orientation.x, init_best_pose.orientation.y, init_best_pose.orientation.z, init_best_pose.orientation.w);
      transform.setOrigin(translation);
      transform.setRotation(quaternion);

      tf2::Transform transform1;
      tf2::Vector3 translation1;
      translation1.setValue(0.0, 0, 0);
      //tf2::Quaternion quaternion1;
      //quaternion.setValue(0, 0, 0, 1);
      transform1.setOrigin(translation1);
      transform1.setRotation(tf2::Quaternion(0, 0, 0, 1));
      tf2::Transform tf_result = transform * transform1;

      // calculate scan to baselink
      tf2::Transform tf_result2 = param_.scan_to_baselink_transform * tf_result;
      geometry_msgs::msg::Pose init_best_pose1;
      init_best_pose1.position.x = tf_result2.getOrigin().x();
      init_best_pose1.position.y = tf_result2.getOrigin().y();
      init_best_pose1.position.z = tf_result2.getOrigin().z();
      init_best_pose1.orientation.x = tf_result2.getRotation().x();
      init_best_pose1.orientation.y = tf_result2.getRotation().y();
      init_best_pose1.orientation.z = tf_result2.getRotation().z();
      init_best_pose1.orientation.w = tf_result2.getRotation().w();



      dock_.pose = init_best_pose1;
    }

    return;
  }
  
  
  
  // Update stamp
  dock_.header.stamp = scan.header.stamp;
  dock_.header.frame_id = tracking_frame_;

  // If this is the first time we've found dock, take whole pose
  
  
  //if (!found_dock_)
  //{
    if (min_score < init_score)
    {
      init_score = min_score;
      init_best_pose = best_pose;
    }
    std::cout << "init_score " << init_score << std::endl;
    log_file_ << "init_score " << init_score << std::endl;
    if (init_score == 1)
    {
      return;
    }
    
    //dock_.pose = init_best_pose;
    // Reset the dock pose filter.
    dock_pose_filter_->reset();
    // Set the filter state to the current pose estimate.
    dock_pose_filter_->setFilterState(dock_.pose, dock_.pose);
  //}
  /* else
  {
    // Check that pose is not too far from current pose
    double d = getPoseDistance(dock_.pose, best_pose);
    if (d > 0.2)
    {
      init_dock = false;
      found_dock_ = false;
      //return;
    }
  } */

  // Update
  if (0)
  {
    // Create point cloud
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = scan.header.stamp;
    cloud.header.frame_id = tracking_frame_;
    cloud.width = cloud.height = 0;

    // Allocate space for points
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(best->points.size());

    // Fill in points
    sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
    for (size_t i = 0; i < best->points.size(); i++)
    {
      cloud_iter[0] = best->points[i].x;
      cloud_iter[1] = best->points[i].y;
      cloud_iter[2] = best->points[i].z;
      ++cloud_iter;
    }
    debug_points_->publish(cloud);
  }

    tf2::Transform transform;
    tf2::Vector3 translation;
    translation.setValue(init_best_pose.position.x, init_best_pose.position.y, init_best_pose.position.z);
    tf2::Quaternion quaternion;
    quaternion.setValue(init_best_pose.orientation.x, init_best_pose.orientation.y, init_best_pose.orientation.z, init_best_pose.orientation.w);
    transform.setOrigin(translation);
    transform.setRotation(quaternion);

    tf2::Transform transform1;
    tf2::Vector3 translation1;
    translation1.setValue(0.0, 0, 0);
    //tf2::Quaternion quaternion1;
    //quaternion.setValue(0, 0, 0, 1);
    transform1.setOrigin(translation1);
    transform1.setRotation(tf2::Quaternion(0, 0, 0, 1));
    tf2::Transform tf_result = transform * transform1;

    // calculate scan to baselink
    tf2::Transform tf_result2 = param_.scan_to_baselink_transform * tf_result;

    init_best_pose.position.x = tf_result2.getOrigin().x();
    init_best_pose.position.y = tf_result2.getOrigin().y();
    init_best_pose.position.z = tf_result2.getOrigin().z();
    init_best_pose.orientation.x = tf_result2.getRotation().x();
    init_best_pose.orientation.y = tf_result2.getRotation().y();
    init_best_pose.orientation.z = tf_result2.getRotation().z();
    init_best_pose.orientation.w = tf_result2.getRotation().w();



  dock_.header.frame_id = param_.publish_frame;
  dock_.pose = init_best_pose;
  dock_stamp_ = scan.header.stamp;
  found_dock_ = true;
  rclcpp::Time now2 = node_->now();
  rclcpp::Duration duration = now2-now1;
  std::cout << "!!!!!!!!!cost time: " << duration.nanoseconds()/1000000 << "ms" << std::endl;
  log_file_ << "!!!!!!!!!cost time: " << duration.nanoseconds()/1000000 << "ms" << std::endl;
    StationPose_publisher_->publish(dock_);
}

DockCandidatePtr ChargInden::extract(laser_processor::SampleSet* cluster)
{
  DockCandidatePtr candidate(new DockCandidate());
  ChargInden::precandidate input;
  for (laser_processor::SampleSet::iterator p = cluster->begin();
       p != cluster->end();
       p++)
  {
    geometry_msgs::msg::Point pt;
    pt.x = (*p)->x;
    pt.y = (*p)->y;
    pt.z = 0;
    input.points.push_back(pt);    
    candidate->points.push_back(pt);    
  }
  std::vector<ChargInden::precandidate> precand = processCandidates(input);
  /* std::cout << "precand.size()  " << precand.size() << std::endl;
  for (size_t i = 0; i < precand.size(); i++)
  {
    std::cout << i << "------ dist:" << precand[i].dist << "  k:" << precand[i].k << std::endl;
  } */
  

  geometry_msgs::msg::Point centroid = icp_2d::getCentroid(candidate->points);
  double dx = centroid.x - dock_.pose.position.x;
  double dy = centroid.y - dock_.pose.position.y;
  candidate->dist = (dx*dx + dy*dy);

  if (param_.debug_mode && precand.size())
  {
    visualization_msgs::msg::Marker marker_msg;
    ToMarkerMsg(precand, marker_msg);
    debug_lines_->publish(marker_msg);
  }

  return candidate;
}

DockCandidatePtr ChargInden::extract(laser_processor::SampleSet* cluster, 
                                  std::vector<DockCandidatePtr>& candidates1)
{
  DockCandidatePtr candidate(new DockCandidate());
  ChargInden::precandidate input;


  for (laser_processor::SampleSet::iterator p = cluster->begin();
       p != cluster->end();
       p++)
  {
    geometry_msgs::msg::Point pt;
    pt.x = (*p)->x;
    pt.y = (*p)->y;
    pt.z = 0;
    input.points.push_back(pt);    
    //candidate->points.push_back(pt);    
  }
  std::vector<ChargInden::precandidate> precand = processCandidates(input);
  std::cout << "precand.size()  " << precand.size() << std::endl;
  log_file_ << "precand.size()  " << precand.size() << std::endl;
  if (precand.size() < 2)
  {
    return candidate;
  }
  
  for (size_t i = 0; i < precand.size(); i++)
  {
    DockCandidatePtr candi(new DockCandidate());
    candi->points = precand[i].points;

    ///debug////////////////////////
    /* candidates1.push_back(candi);
    continue; */
    ////////////////////////////////
    std::cout << i << "------ dist:" << precand[i].dist << "  k:" << precand[i].k << std::endl;
    log_file_ << i << "------ dist:" << precand[i].dist << "  k:" << precand[i].k << std::endl;
    if (i == precand.size() - 1)
    {
      if ( fabs(fabs(precand[i].k - precand[0].k) - param_.vangle) < param_.angle_tolerance)
      {
        std::cout <<"#### ##### angle:" << fabs(precand[i].k - precand[0].k) << std::endl;
        log_file_ <<"#### ##### angle:" << fabs(precand[i].k - precand[0].k) << std::endl;
        for (size_t j0 = 0; j0 < precand[0].points.size(); j0++)
        {
          candidate->points.push_back(precand[0].points[j0]);
        }
        for (size_t j1 = 0; j1 < precand[i].points.size(); j1++)
        {
          candidate->points.push_back(precand[i].points[j1]);
        }
        DockCandidatePtr candi1(new DockCandidate());
        candi1->points = precand[0].points;
        candidates1.push_back(candi1);
        DockCandidatePtr candi2(new DockCandidate());
        candi2->points = precand[i].points;
        candidates1.push_back(candi2);
        std::cout << "candidate.point.size " << candidate->points.size() << std::endl;
        log_file_ << "candidate.point.size " << candidate->points.size() << std::endl;
        return candidate;
      }
      
      return candidate;
    }
    
    if (fabs(fabs(precand[i].k - precand[i+1].k) - param_.vangle) < param_.angle_tolerance)
    {
      std::cout <<"######## angle:" << fabs(precand[i].k - precand[i+1].k)  << std::endl;
      log_file_ <<"######## angle:" << fabs(precand[i].k - precand[i+1].k)  << std::endl;
        for (size_t j0 = 0; j0 < precand[i].points.size(); j0++)
        {
          candidate->points.push_back(precand[i].points[j0]);
        }
        for (size_t j1 = 0; j1 < precand[i+1].points.size(); j1++)
        {
          candidate->points.push_back(precand[i+1].points[j1]);
        }
        DockCandidatePtr candi1(new DockCandidate());
        candi1->points = precand[i].points;
        candidates1.push_back(candi1);
        DockCandidatePtr candi2(new DockCandidate());
        candi2->points = precand[i+1].points;
        candidates1.push_back(candi2);
        //std::cout << "candidate.point.size " << candidate->points.size() << std::endl;
        return candidate;
    }
    

  }
  return candidate;

  geometry_msgs::msg::Point centroid = icp_2d::getCentroid(candidate->points);
  double dx = centroid.x - dock_.pose.position.x;
  double dy = centroid.y - dock_.pose.position.y;
  candidate->dist = (dx*dx + dy*dy);

  if (param_.debug_mode && precand.size())
  {
    visualization_msgs::msg::Marker marker_msg;
    ToMarkerMsg(precand, marker_msg);
    debug_lines_->publish(marker_msg);
  }

  return candidate;
}

void ChargInden::ToMarkerMsg(std::vector<ChargInden::precandidate> precand, 
                                           visualization_msgs::msg::Marker &marker_msg)
{
  marker_msg.ns = "lines";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<precandidate>::const_iterator cit = precand.begin(); cit != precand.end(); ++cit)
  {
    marker_msg.points.push_back(cit->points.front());
    marker_msg.points.push_back(cit->points.back());
  }
  marker_msg.header.frame_id = "base_laser";
}

double ChargInden::calculateCurvature(const geometry_msgs::msg::Point& prev, 
                          const geometry_msgs::msg::Point& current, 
                          const geometry_msgs::msg::Point& next) {
    // 计算两个向量并求夹角作为曲率近似
    double angle1 = atan2(current.y - prev.y, current.x - prev.x);
    double angle2 = atan2(next.y - current.y, next.x - current.x);
    double curvature = std::abs(angle2 - angle1);
    return curvature;
}

double ChargInden::calculateDistanceToLine(const geometry_msgs::msg::Point& p0, 
                               const geometry_msgs::msg::Point& p1, 
                               const geometry_msgs::msg::Point& p2) {
    double numerator = std::abs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + p2.x * p1.y - p2.y * p1.x);
    double denominator = std::sqrt(std::pow(p2.y - p1.y, 2) + std::pow(p2.x - p1.x, 2));
    return numerator / denominator;
}

void ChargInden::rdpSegmentation(const std::vector<geometry_msgs::msg::Point>& points, 
                     double epsilon, 
                     std::vector<ChargInden::precandidate>& segments) {
    if (points.size() < 2) {
        // 点的数量不足以形成线段
        return;
    }

    // 找到距离直线最远的点
    double maxDistance = 0.0;
    size_t index = 0;
    for (size_t i = 1; i < points.size() - 1; ++i) {
        double distance = calculateDistanceToLine(points[i], points.front(), points.back());
        if (distance > maxDistance) {
            maxDistance = distance;
            index = i;
        }
    }

    // 如果最大距离小于阈值，则认为是一个线段
    if (maxDistance < epsilon) {
        ChargInden::precandidate segment;
        segment.points=points;
        segment.k = atan2(points.back().y - points.front().y, points.back().x - points.front().x);
        segment.dist = std::hypot(points.back().x - points.front().x, 
                                    points.back().y - points.front().y);
        std::cout << "@@@@@ dist:" << segment.dist << "  k:" << segment.k << std::endl;
        log_file_ << "@@@@@ dist:" << segment.dist << "  k:" << segment.k << std::endl;
        if (fabs(segment.dist - param_.vlength) < param_.length_tolerance)
        {
          segments.push_back(segment);
        }
        /* segment.points.push_back(points.front());
        segment.points.push_back(points.back());
        segments.push_back(segment); */
        
        
    } else {
        // 否则，递归分割
        std::vector<geometry_msgs::msg::Point> segment1(points.begin(), points.begin() + index + 1);
        std::vector<geometry_msgs::msg::Point> segment2(points.begin() + index + 1, points.end());
        //std::cout << points.size() << " = " << segment1.size() << " + " << segment2.size() << std::endl; 
        rdpSegmentation(segment1, epsilon, segments);
        rdpSegmentation(segment2, epsilon, segments);
    }
}


std::vector<ChargInden::precandidate> ChargInden::processCandidates(const ChargInden::precandidate& input) {
    std::vector<ChargInden::precandidate> segments;
    std::vector<ChargInden::precandidate> segments1;

    //////////////  use Ramer-Douglas-Peucker /////////////////////
    rdpSegmentation(input.points, param_.rdp_epsilon, segments);


    return segments;
    /////////////////////////////////////////////////////////////////

    ChargInden::precandidate current_segment;
    double total_dist = 0;
    double total_curvature = 0;

    for (size_t i = 1; i < input.points.size() - 1; ++i) {
        double angle1 = atan2(input.points[i].y - input.points[i - 1].y, input.points[i].x - input.points[i - 1].x);
        double angle2 = atan2(input.points[i + 1].y - input.points[i].y, input.points[i + 1].x - input.points[i].x);
        double curvature = std::abs(angle2 - angle1);
        double distance = calculateDistanceToLine(input.points[i], input.points[i - 1], input.points[i + 1]);
        

        //if (curvature < 0.3 && distance < 0.001) {
        if (distance < 0.001) {
        //if (curvature < 0.25) {
            current_segment.points.push_back(input.points[i]);

            if (i > 1) {
                double dist = std::hypot(input.points[i].x - input.points[i - 1].x, 
                                         input.points[i].y - input.points[i - 1].y);
                total_dist += dist;
            }

            total_curvature += angle1;
        } else {
            if (!current_segment.points.empty()) {
                current_segment.dist = total_dist;
                current_segment.k = total_curvature / current_segment.points.size();
                //if(current_segment.dist >= 0.05 && current_segment.dist <= 0.2){
                  segments.push_back(current_segment);
                //}
                
                current_segment.points.clear();
                total_dist = 0;
                total_curvature = 0;
            }
        }
    }

    if (!current_segment.points.empty()) {
        current_segment.dist = total_dist;
        current_segment.k = total_curvature / current_segment.points.size();
        //if(current_segment.dist >= 0.05 && current_segment.dist <= 0.2){
            segments.push_back(current_segment);
        //}
        //segments.push_back(current_segment);
    }

    /// merge
    std::vector<ChargInden::precandidate> mergedSegments;

    while (!segments.empty()) {
        auto currentSegment = segments.front();
        segments.erase(segments.begin());  // 移除当前处理的线段

        bool merged = false;
        for (auto it = segments.begin(); it != segments.end(); /* no increment */) {
            // 计算距离和斜率差
            double dist = std::hypot(currentSegment.points.back().x - it->points.front().x,
                                 currentSegment.points.back().y - it->points.front().y);
            double kDiff = std::abs(currentSegment.k - it->k);

            if (dist < 0.01 && kDiff < 0.2) {
                // 合并线段
                currentSegment.points.insert(currentSegment.points.end(), it->points.begin(), it->points.end());
                currentSegment.dist += dist;
                currentSegment.dist += it->dist;
                currentSegment.k = (currentSegment.k * currentSegment.points.size() 
                                + it->k * it->points.size()) / (currentSegment.points.size() + it->points.size());
                //currentSegment.dist = calculateSegmentLength(currentSegment.points);
                //currentSegment.k = calculateSegmentSlope(currentSegment.points.front(), currentSegment.points.back());
                it = segments.erase(it);  // 移除并更新迭代器
                merged = true;
            } else {
                ++it;  // 只有在不移除元素时才增加迭代器
            }
        }

        if (currentSegment.dist >= 0.05 && currentSegment.dist <= 0.2) {
            mergedSegments.push_back(currentSegment);
        }
    }

    return mergedSegments;
}

double ChargInden::fit(const DockCandidatePtr& candidate, geometry_msgs::msg::Pose& pose)
{
  // Setup initial pose
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.rotation = pose.orientation;

  // Initial yaw. Presumably the initial goal orientation estimate.
   tf2::Quaternion init_pose, cand_pose;

  // ICP the dock
  double fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform);
  //std::cout << "fitness " << fitness << std::endl;
  tf2::Quaternion quaternion(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w);

  if (fabs(angles::normalize_angle(tf2::getYaw(tf2::inverse(cand_pose)*init_pose))) > 3.1415*(2.0/3.0) )
  {
    tf2::Quaternion q10;
    q10.setRPY(0, 0, 3.1415 + tf2::getYaw(tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)));
    //transform.rotation = tf2::toMsg(q10);
    transform.rotation.x = q10.x();
    transform.rotation.y = q10.y();
    transform.rotation.z = q10.z();
    transform.rotation.w = q10.w();
    
  }

  if (fitness >= 0.0)
  {
    // Initialize the number of times we retry if the fitness is bad.
    double retry = 3;
    cand_pose = tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
    while (retry-- &&
            (
              fitness                                                       > param_.max_alignment_error ||
              fabs(angles::normalize_angle(tf2::getYaw(tf2::inverse(cand_pose)*init_pose))) > 3.1415/4.0
            )
          )
    {
      // Try one more time.
      // Perturb the pose to try to get it out of the local minima.
      transform.translation.x += retry*(0.75/100.0)*static_cast<double>((rand() % 200) - 100);
      transform.translation.y += retry*(0.75/100.0)*static_cast<double>((rand() % 200) - 100);
      tf2::Quaternion q11;
      q11.setRPY(0, 0, retry*(0.28/100.0)*double((rand() % 200) - 100) + tf2::getYaw(tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)));
      transform.rotation.x = q11.x();
      transform.rotation.y = q11.y();
      transform.rotation.z = q11.z();
      transform.rotation.w = q11.w();

      // Rerun ICP.
      fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform);
      cand_pose = tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
      if (fabs(angles::normalize_angle(tf2::getYaw(tf2::inverse(cand_pose)*init_pose))) > 3.1415*(2.0/3.0) )
      {
        tf2::Quaternion q12;
        q12.setRPY(0, 0, 3.1415 + tf2::getYaw(tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)));
        transform.rotation.x = q12.x();
        transform.rotation.y = q12.y();
        transform.rotation.z = q12.z();
        transform.rotation.w = q12.w();
      }
      std::cout << "retry: " << 6-retry << "  fitness: " << fitness << std::endl;
      log_file_ << "retry: " << 6-retry << "  fitness: " << fitness << std::endl;
    }
     
    // If the dock orientation is still really borked, fail.
    cand_pose = tf2::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
    //std::cout << "fitness2 " << fitness << std::endl;
    if (fabs(angles::normalize_angle(tf2::getYaw(tf2::inverse(cand_pose)*init_pose))) > 3.1415/2.0 )
    {
      fitness = -1.0;
    }
    std::cout << "fitness3 " << fitness << std::endl;
    log_file_ << "fitness3 " << fitness << std::endl;
    // Check that fitness is good enough
    //if (!found_dock_ && fabs(fitness) > param_.max_alignment_error)
    if (fabs(fitness) > 0.02)
    {
      // If not, signal no fit
      fitness = -1.0;
    }
    //std::cout << "fitness4 " << fitness << std::endl;
    // If width of candidate is smaller than the width of dock
    // then the whole dock is not visible...

    // Transform ideal cloud, and store for visualization
    candidate->points = icp_2d::transform(ideal_cloud_,
                                          transform.translation.x,
                                          transform.translation.y,
                                          icp_2d::thetaFromQuaternion(transform.rotation));

    // Get pose
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation = transform.rotation;
    return fitness;
  }

  // Signal no fit
  return -1.0;
}

bool ChargInden::isValid(const tf2::Quaternion& q)
{
  return 1e-3 >= fabs(1.0 - q.length());
}

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("zme_charging_indentifi_node"); 
    rclcpp::executors::MultiThreadedExecutor executor;           // 创建多线程执行器
    executor.add_node(node_);                                     // 将节点添加到执行器
    auto node = std::make_shared<ChargInden>(node_);
    executor.spin(); // 开始执行

    rclcpp::shutdown();
    return 0;
}
