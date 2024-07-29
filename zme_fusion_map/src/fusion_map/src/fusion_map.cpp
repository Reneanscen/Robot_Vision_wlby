#include <fusion_map/fusion_map.h>




FusionMapNode::FusionMapNode() : Node("zme_fusion_map_node"), tf_broadcaster_(this), running_(true), esdf_mode(false) {
    this->declare_parameter<bool>("esdf_mode", false);
    this->get_parameter("esdf_mode", esdf_mode);
    createLogFile();
    log_file_ << "Fusion Map start!  esdf_mode: " << esdf_mode << std::endl;
    if (esdf_mode)
        SetParameters();
    initializeSubscribers();
}

FusionMapNode::~FusionMapNode() {
    running_ = false;
    if (log_file_.is_open()) {
        log_file_.close();
    }
    if (processing_thread.joinable()) {
        processing_thread.join();
    }
    
}

void FusionMapNode::SetParameters() {
  //std::cout << "set parameters" << std::endl;
  this->declare_parameter<double>("resolution", 0.1);
  this->get_parameter("resolution", parameters_.resolution_);

  this->declare_parameter<int>("visualize_every_n_updates", 1);
  this->get_parameter("visualize_every_n_updates", parameters_.visualize_every_n_updates_);

  this->declare_parameter<double>("min_ray_length", 0.5);
  this->get_parameter("min_ray_length", parameters_.min_ray_length_);

  this->declare_parameter<double>("max_ray_length", 5.0);
  this->get_parameter("max_ray_length", parameters_.max_ray_length_);

  double slice_vis_level_tmp, vis_lower_bound_tmp, vis_upper_bound_tmp;

  this->declare_parameter<double>("slice_vis_max_dist", 2.0);
  this->get_parameter("slice_vis_max_dist", parameters_.slice_vis_max_dist_);

  this->declare_parameter<double>("slice_vis_level", 5.0);
  this->get_parameter("slice_vis_level", slice_vis_level_tmp);

  this->declare_parameter<double>("vis_lower_bound", -5.0);
  this->get_parameter("vis_lower_bound", vis_lower_bound_tmp);

  this->declare_parameter<double>("vis_upper_bound", 10.0);
  this->get_parameter("vis_upper_bound", vis_upper_bound_tmp);

  parameters_.slice_vis_level_ = static_cast<int>(slice_vis_level_tmp / parameters_.resolution_);
  parameters_.vis_lower_bound_ = static_cast<int>(vis_lower_bound_tmp / parameters_.resolution_);
  parameters_.vis_upper_bound_ = static_cast<int>(vis_upper_bound_tmp / parameters_.resolution_);

  this->declare_parameter<double>("center_x", 322.477357419);
  this->get_parameter("center_x", parameters_.center_x_);

  this->declare_parameter<double>("center_y", 237.076346481);
  this->get_parameter("center_y", parameters_.center_y_);

  this->declare_parameter<double>("focal_x", 384.458089392);
  this->get_parameter("focal_x", parameters_.focal_x_);

  this->declare_parameter<double>("focal_y", 383.982755697);
  this->get_parameter("focal_y", parameters_.focal_y_);

  this->declare_parameter<int>("ray_cast_num_thread", 0);
  this->get_parameter("ray_cast_num_thread", parameters_.ray_cast_num_thread_);

  double radius_x, radius_y, radius_z;

  this->declare_parameter<double>("radius_x", 3.0);
  this->get_parameter("radius_x", radius_x);

  this->declare_parameter<double>("radius_y", 3.0);
  this->get_parameter("radius_y", radius_y);

  this->declare_parameter<double>("radius_z", 1.5);
  this->get_parameter("radius_z", radius_z);

  parameters_.radius_ = Eigen::Vector3d(radius_x, radius_y, radius_z);

  this->declare_parameter<bool>("global_map", true);
  this->get_parameter("global_map", parameters_.global_map_);

  this->declare_parameter<bool>("global_update", true);
  this->get_parameter("global_update", parameters_.global_update_);

  this->declare_parameter<bool>("global_vis", true);
  this->get_parameter("global_vis", parameters_.global_vis_);

  if (!parameters_.global_map_) {
    parameters_.global_vis_ = parameters_.global_update_ = false;
  }

  this->declare_parameter<bool>("use_depth_filter", true);
  this->get_parameter("use_depth_filter", parameters_.use_depth_filter_);

  this->declare_parameter<double>("depth_filter_tolerance", 0.1);
  this->get_parameter("depth_filter_tolerance", parameters_.depth_filter_tolerance_);

  this->declare_parameter<double>("depth_filter_max_dist", 10.0);
  this->get_parameter("depth_filter_max_dist", parameters_.depth_filter_max_dist_);

  this->declare_parameter<double>("depth_filter_min_dist", 0.1);
  this->get_parameter("depth_filter_min_dist", parameters_.depth_filter_min_dist_);

  this->declare_parameter<int>("depth_filter_margin", 0);
  this->get_parameter("depth_filter_margin", parameters_.depth_filter_margin_);

#ifdef HASH_TABLE
  parameters_.l_cornor_ << -100.f, -100.f, -100.f;
  parameters_.r_cornor_ << 100.f, 100.f, 100.f;
  this->declare_parameter<int>("reserved_size", 1000000);
  this->get_parameter("reserved_size", parameters_.reserved_size_);
#else
  double lx, ly, lz;
  double rx, ry, rz;

  this->declare_parameter<double>("lx", -20.0);
  this->get_parameter("lx", lx);

  this->declare_parameter<double>("ly", -20.0);
  this->get_parameter("ly", ly);

  this->declare_parameter<double>("lz", -5.0);
  this->get_parameter("lz", lz);

  this->declare_parameter<double>("rx", 20.0);
  this->get_parameter("rx", rx);

  this->declare_parameter<double>("ry", 20.0);
  this->get_parameter("ry", ry);

  this->declare_parameter<double>("rz", 5.0);
  this->get_parameter("rz", rz);

  parameters_.l_cornor_ << lx, ly, lz;
  parameters_.r_cornor_ << rx, ry, rz;
  parameters_.map_size_ = parameters_.r_cornor_ - parameters_.l_cornor_;
#endif

  this->declare_parameter<double>("update_esdf_every_n_sec", 0.1);
  this->get_parameter("update_esdf_every_n_sec", parameters_.update_esdf_every_n_sec_);

  parameters_.T_B_C_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

  parameters_.T_D_B_ << 0.971048, -0.120915, 0.206023, 0.00114049,
            0.15701, 0.973037, -0.168959, 0.0450936,
            -0.180038, 0.196415, 0.96385, 0.0430765,
            0.0, 0.0, 0.0, 1.0;

#ifdef PROBABILISTIC
  this->declare_parameter<double>("p_hit", 0.70);
  this->get_parameter("p_hit", parameters_.p_hit_);

  this->declare_parameter<double>("p_miss", 0.35);
  this->get_parameter("p_miss", parameters_.p_miss_);

  this->declare_parameter<double>("p_min", 0.12);
  this->get_parameter("p_min", parameters_.p_min_);

  this->declare_parameter<double>("p_max", 0.97);
  this->get_parameter("p_max", parameters_.p_max_);

  this->declare_parameter<double>("p_occ", 0.80);
  this->get_parameter("p_occ", parameters_.p_occ_);
#endif
}

void FusionMapNode::createLogFile() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y%m%d%H%M%S");
    //return oss.str();
    std::string log_directory = "log";
    std::string log_file_path = log_directory + "/fusion_map_node" + oss.str() + ".log";

    // 检查日志目录是否存在，如果不存在则创建
    if (!std::filesystem::exists(log_directory)) {
        std::filesystem::create_directory(log_directory);
    }

    // 打开日志文件
    log_file_.open(log_file_path, std::ios::out | std::ios::app);
    if (!log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_file_path.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Log file created: %s", log_file_path.c_str());
        log_file_ << "Log file created" << std::endl;
    }
}

void FusionMapNode::initializeSubscribers() {
    odom_init = false;
    lidar_init = false;
    tof_init = false;
    realsense_init = false;
    ground_init = false;

    // ros parameter
    this->declare_parameter<float>("scan_to_baselink_transform.px", 0.195);
    this->declare_parameter<float>("scan_to_baselink_transform.py", 0.0);
    this->declare_parameter<float>("scan_to_baselink_transform.pz", 0.0227);
    this->declare_parameter<float>("scan_to_baselink_transform.roll", 0.0);
    this->declare_parameter<float>("scan_to_baselink_transform.pitch", 0.0);
    this->declare_parameter<float>("scan_to_baselink_transform.yaw", 1.56);

    this->declare_parameter<float>("tof_to_baselink_transform.px", 0.22);
    this->declare_parameter<float>("tof_to_baselink_transform.py", 0.0);
    this->declare_parameter<float>("tof_to_baselink_transform.pz", 0.0);
    this->declare_parameter<float>("tof_to_baselink_transform.roll", 1.56);
    this->declare_parameter<float>("tof_to_baselink_transform.pitch", 0.0);
    this->declare_parameter<float>("tof_to_baselink_transform.yaw", 1.56);

    this->declare_parameter<float>("realsense_to_baselink_transform.px", 0.035140);
    this->declare_parameter<float>("realsense_to_baselink_transform.py", 0.028017);
    this->declare_parameter<float>("realsense_to_baselink_transform.pz", 0.95);
    this->declare_parameter<float>("realsense_to_baselink_transform.roll", -2.142103);
    this->declare_parameter<float>("realsense_to_baselink_transform.pitch", 0.003516);
    this->declare_parameter<float>("realsense_to_baselink_transform.yaw", -1.60501);

    float px1, py1, pz1, roll1, pitch1, yaw1;
    this->get_parameter("scan_to_baselink_transform.px", px1);
    // std::cout << "px1: " << px1 << std::endl;
    this->get_parameter("scan_to_baselink_transform.py", py1);
    this->get_parameter("scan_to_baselink_transform.pz", pz1);
    this->get_parameter("scan_to_baselink_transform.roll", roll1);
    this->get_parameter("scan_to_baselink_transform.pitch", pitch1);
    this->get_parameter("scan_to_baselink_transform.yaw", yaw1);
    transform_lidar_to_baselink = createTransformMatrix(px1, py1, pz1, roll1, pitch1, yaw1);
    
    float px2, py2, pz2, roll2, pitch2, yaw2;
    this->get_parameter("tof_to_baselink_transform.px", px2);
    // std::cout << "px2: " << px2 << std::endl;
    this->get_parameter("tof_to_baselink_transform.py", py2);
    this->get_parameter("tof_to_baselink_transform.pz", pz2);
    this->get_parameter("tof_to_baselink_transform.roll", roll2);
    this->get_parameter("tof_to_baselink_transform.pitch", pitch2);
    this->get_parameter("tof_to_baselink_transform.yaw", yaw2);
    transform_tof_to_baselink = createTransformMatrix(px2, py2, pz2, roll2, pitch2, yaw2);

    float px3, py3, pz3, roll3, pitch3, yaw3;
    this->get_parameter("realsense_to_baselink_transform.px", px3);
    
    this->get_parameter("realsense_to_baselink_transform.py", py3);
    this->get_parameter("realsense_to_baselink_transform.pz", pz3);
    this->get_parameter("realsense_to_baselink_transform.roll", roll3);
    this->get_parameter("realsense_to_baselink_transform.pitch", pitch3);
    this->get_parameter("realsense_to_baselink_transform.yaw", yaw3);
    
    transform_realsense_to_baselink = createTransformMatrix(px3, py3, pz3, roll3, pitch3, yaw3);

    processing_thread = std::thread(&FusionMapNode::processData, this);

    const rmw_qos_profile_t rmw_qos_profile_custom = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        5,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
    };
    const rclcpp::QoSInitialization qos_initialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2);
    rclcpp::QoS qos(qos_initialization, rmw_qos_profile_custom);

    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fusion_map_pcl", 1);
    if (esdf_mode)
    {
        text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ESDF/text", 1);
        slice_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ESDF/slice", 1);
        occupancy_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ESDF/occupancy", 1);
    }
    
    
    // 初始化订阅者...
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 1, std::bind(&FusionMapNode::laserCallback, this, std::placeholders::_1)); 
    
    tof_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sunny_topic/tof_frame/pointcloud", 1, std::bind(&FusionMapNode::tofCallback, this, std::placeholders::_1));
    // 初始化RealSense D435i订阅者
    realsense_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 1, std::bind(&FusionMapNode::realsenseCallback, this, std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 1, std::bind(&FusionMapNode::odometryCallback, this, std::placeholders::_1));

    ground_sub_ = this->create_subscription<zme_msg_srv::msg::ObstacleavoidList>(
            "/Visual_obstacle_avoidance", 1, std::bind(&FusionMapNode::groundCallback, this, std::placeholders::_1));
    
    realsense_dep_sub_ = this->create_subscription<custom_image_msg::msg::Image4m>(
            "/head_cam_depth", qos, std::bind(&FusionMapNode::realsenseDepCallback, this, std::placeholders::_1));

    /* auto laser_timer = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&FusionMapNode::laserCallback, this));
    
    auto tof_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FusionMapNode::tofCallback, this));

    auto realsense_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FusionMapNode::realsenseCallback, this));

    auto odometry_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FusionMapNode::odometryCallback, this));

    auto ground_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FusionMapNode::groundCallback, this));

    auto realsense_dep_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FusionMapNode::realsenseDepCallback, this)); */
    
    //esdf ********************************************************************************
         //parameters_.SetParameters(node);
    if (esdf_mode)
    {
#ifdef HASH_TABLE
     esdf_map_ = new fiesta::ESDFMap(Eigen::Vector3d(0, 0, 0), parameters_.resolution_, parameters_.reserved_size_);
#ifdef SIGNED_NEEDED
       inv_esdf_map_ = new fiesta::ESDFMap(Eigen::Vector3d(0, 0, 0), parameters_.resolution_, parameters_.reserved_size_);
#endif
#else
     esdf_map_ = new fiesta::ESDFMap(parameters_.l_cornor_, parameters_.resolution_, parameters_.map_size_);
#ifdef SIGNED_NEEDED
     inv_esdf_map_ = new fiesta::ESDFMap(parameters_.l_cornor_, parameters_.resolution_, parameters_.map_size_);
#endif
#endif

#ifdef PROBABILISTIC
     esdf_map_->SetParameters(parameters_.p_hit_, parameters_.p_miss_,
                              parameters_.p_min_, parameters_.p_max_, parameters_.p_occ_);
#endif
#ifndef HASH_TABLE
     set_free_.resize(esdf_map_->grid_total_size_);
     set_occ_.resize(esdf_map_->grid_total_size_);
     std::fill(set_free_.begin(), set_free_.end(), 0);
     std::fill(set_occ_.begin(), set_occ_.end(), 0);
#endif
    }
/*      // For Jie Bao
//     transform_sub_ = node.subscribe("/vins_estimator/camera_pose", 10, &Fiesta::PoseCallback, this);
//     depth_sub_ = node.subscribe("/camera/depth/image_rect_raw", 10, &Fiesta::DepthCallback, this);
    transform_sub_ = node.subscribe("transform", 10, &Fiesta::PoseCallback, this);
    depth_sub_ = node.subscribe("depth", 10, &Fiesta::DepthCallback, this);

     // Cow_and_Lady
     // depth_sub_ = node.subscribe("/camera/depth_registered/points", 1000, PointcloudCallback);
     // transform_sub_ = node.subscribe("/kinect/vrpn_client/estimated_transform", 1000, PoseCallback);

     //EuRoC
//    depth_sub_ = node.subscribe("/dense_stereo/pointcloud", 1000, PointcloudCallback);
//    transform_sub_ = node.subscribe("/vicon/firefly_sbx/firefly_sbx", 1000, PoseCallback);

     slice_pub_ = node.advertise<visualization_msgs::Marker>("ESDFMap/slice", 1, true);
     occupancy_pub_ = node.advertise<sensor_msgs::PointCloud>("ESDFMap/occ_pc", 1, true);
     text_pub_ = node.advertise<visualization_msgs::Marker>("ESDFMap/text", 1, true);

     update_mesh_timer_ =
         node.createTimer(ros::Duration(parameters_.update_esdf_every_n_sec_),
                          &Fiesta::UpdateEsdfEvent, this); */
}

Eigen::Matrix4f FusionMapNode::createTransformMatrix(float x, float y, float z, float roll, float pitch, float yaw) {
    Eigen::Translation3f translation(x, y, z);
    Eigen::AngleAxisf rotation_x(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_y(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Transform<float, 3, Eigen::Affine> transform = translation * rotation_z * rotation_y * rotation_x;

    return transform.matrix();
}

void FusionMapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    
    static sensor_msgs::msg::LaserScan laser_msg;
    laser_msg = *msg;
    static int lidar_n = 0;
    if (!lidar_init)
    {
        lidar_n ++;
        if (lidar_n >= 10)
        {
            lidar_init = true;
        }
        
        return;
    }
    static double last_scan_time = 0;
    rclcpp::Time now_scan_time_ = rclcpp::Time(laser_msg.header.stamp);
    double now_scan_time = now_scan_time_.seconds();
    if (last_scan_time>0)
    {
        if (now_scan_time - last_scan_time < 0.2)
        {
            return;
        }
    }
    last_scan_time = now_scan_time;
    
    for (size_t i = 0; i < laser_msg.ranges.size(); i++)
    {
        if (i*0.0029183398000895977 > 0 && i*0.0029183398000895977 < 0.4 && laser_msg.ranges[i] < 0.22)
        {
            laser_msg.ranges[i] = NAN;
        }
        else if (i*0.0029183398000895977 < 1.57 && laser_msg.ranges[i] < 0.33)
        {
            laser_msg.ranges[i] = NAN;
        }
        
        
    }
    

    //std::cout << "start laser" << std::endl;
    log_file_ << "Get Laser " << std::endl;
    std::unique_lock<std::mutex> lock(sensor_mutex_);
    // 处理激光雷达数据...
    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 cloud;
    projector_.projectLaser(laser_msg, cloud);
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud, *pcl_cloud);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(pcl_cloud.makeShared());
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    
    lidar_data_ = *cloud_filtered;
    lidar_time_ = rclcpp::Time(laser_msg.header.stamp);

    // 融合
    //processData();
    //laser_msg = *msg;
    //std::cout << "stop laser" << std::endl;
}

void FusionMapNode::tofCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    static sensor_msgs::msg::PointCloud2 tof_msg;
    static int tof_n = 0;
    tof_msg = *msg;
    if (!tof_init)
    {
        tof_n ++;
        if (tof_n >= 10)
        {
            tof_init = true;
        }
        //tof_msg = *msg;
        return;
    }


    static double last_tof_time = 0;
    rclcpp::Time now_tof_time_ = rclcpp::Time(tof_msg.header.stamp);
    double now_tof_time = now_tof_time_.seconds();
    if (last_tof_time>0)
    {
        if (now_tof_time - last_tof_time < 0.3)
        {
            return;
        }
    }
    last_tof_time = now_tof_time;


    log_file_ << "Get TOF " << std::endl;
    std::unique_lock<std::mutex> lock(sensor_mutex_);
    // 应用体素滤波...
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(tof_msg, *pcl_cloud);

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(pcl_cloud.makeShared());
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    tof_data_ = *cloud_filtered;
    tof_time_ = rclcpp::Time(tof_msg.header.stamp);
    
    //std::cout << "stop tof" << std::endl;
}

void FusionMapNode::groundCallback(const zme_msg_srv::msg::ObstacleavoidList::SharedPtr msg) {
    static zme_msg_srv::msg::ObstacleavoidList ground_msg;
    ground_msg = *msg;
    /* static int ground_n = 0;
    if (!ground_init)
    {
        ground_n ++;
        if (ground_n >= 10)
        {
            ground_init = true;
        }
        //ground_msg = *msg;
        return;
    } */

    static double last_ground_time = 0;
    rclcpp::Time now_ground_time_ = rclcpp::Time(ground_msg.obstacle[0].point_cloud.header.stamp);
    double now_ground_time = now_ground_time_.seconds();
    if (last_ground_time>0)
    {
        if (now_ground_time - last_ground_time < 0.3)
        {
            return;
        }
    }
    last_ground_time = now_ground_time;

    //std::cout << "start ground" << std::endl;
    log_file_ << "Get Ground Semantic " << std::endl;
    std::unique_lock<std::mutex> lock(sensor_mutex_);

pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground0(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < ground_msg.obstacle.size(); i++)
    {
        pcl::PCLPointCloud2::Ptr pcl_cloud0(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(ground_msg.obstacle[i].point_cloud, *pcl_cloud0);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*pcl_cloud0, *cloud_ground);
        *cloud_ground0 += *cloud_ground;
    }
pcl::toPCLPointCloud2(*cloud_ground0, *pcl_cloud);


    //////////////////////////////////////////////////////////////////////
    // 应用体素滤波...
    
    //pcl_conversions::toPCL(ground_msg, *pcl_cloud);

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(pcl_cloud.makeShared());
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    ground_data_ = *cloud_filtered;
    ground_time_ = rclcpp::Time(ground_msg.obstacle[0].point_cloud.header.stamp);
    //ground_msg = *msg;
    //std::cout << "stop ground" << std::endl;
}

void FusionMapNode::realsenseCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    static sensor_msgs::msg::PointCloud2 realsense_msg;
    realsense_msg = *msg;
    static int realsense_n = 0;
    if (!realsense_init)
    {
        realsense_n ++;
        if (realsense_n >= 10)
        {
            realsense_init = true;
        }
        //realsense_msg = *msg;
        return;
    }

    static double last_realsense_time = 0;
    rclcpp::Time now_realsense_time_ = rclcpp::Time(realsense_msg.header.stamp);
    double now_realsense_time = now_realsense_time_.seconds();
    if (last_realsense_time>0)
    {
        if (now_realsense_time - last_realsense_time < 0.3)
        {
            return;
        }
    }
    last_realsense_time = now_realsense_time;
        
    log_file_ << "Get realsense pcl " << std::endl;
    std::unique_lock<std::mutex> lock(sensor_mutex_);
    // 应用体素滤波...
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(realsense_msg, *pcl_cloud);

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(pcl_cloud.makeShared());
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    realsense_data_ = *cloud_filtered;
    realsense_time_ = rclcpp::Time(realsense_msg.header.stamp);
    
    //std::cout << "stop realsense" << std::endl;
}

void FusionMapNode::realsenseDepCallback(const custom_image_msg::msg::Image4m::SharedPtr msg) {
    //std::unique_lock<std::mutex> lock(sensor_mutex_);
    static sensor_msgs::msg::PointCloud2 realsense_msg;
    static custom_image_msg::msg::Image4m realsense_dep_msg;
    realsense_dep_msg = *msg;
    static int realsense_n = 0;
    if (!realsense_init)
    {
        realsense_n ++;
        if (realsense_n >= 10)
        {
            realsense_init = true;
        }
        
        return;
    }    

    static double last_realsense_dep_time = 0;
    rclcpp::Time now_realsense_dep_time_ = rclcpp::Time(realsense_dep_msg.header.stamp);
    double now_realsense_dep_time = now_realsense_dep_time_.seconds();
    if (last_realsense_dep_time>0)
    {
        if (now_realsense_dep_time - last_realsense_dep_time < 0.3)
        {
            return;
        }
    }
    last_realsense_dep_time = now_realsense_dep_time;

    rclcpp::Time now1 = now();
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    // Depth image to point cloud  ////////////////////////////////////////////////////////////////////////////////
    // 相机内参（示例值，根据你的相机进行调整）
    float fx = 9.1271350097656250e+02; // Focal length in x
    float fy = 9.1257855224609375e+02; // Focal length in y
    float cx = realsense_dep_msg.width / 2.0f; // Optical center in x
    float cy = realsense_dep_msg.height / 2.0f; // Optical center in y

    pcl::PointCloud<pcl::PointXYZ> tempCloud;
    //tempCloud.width = realsense_dep_msg.width;
    //tempCloud.height = realsense_dep_msg.height;
    tempCloud.is_dense = false;
    //tempCloud.points.resize(tempCloud.width * tempCloud.height);
    unsigned char* data = const_cast<unsigned char*>(&realsense_dep_msg.data[0]);

    for (int v = 0; v < realsense_dep_msg.height; ++v)
    {
        for (int u = 0; u < realsense_dep_msg.width; ++u)
        {
            // 计算像素在data中的位置
            const unsigned char* pixel = data + v * realsense_dep_msg.step + u * 3; // 3为CV_8UC3的通道数
            unsigned short high = static_cast<unsigned short>(pixel[0]); // 蓝色通道，高8位
            unsigned short low = static_cast<unsigned short>(pixel[1]); // 绿色通道，低8位
            unsigned short depth = (high << 8) | low; // 组合成16位深度值
            float depthInMeters = depth / 1000.0f;

            // 计算3D坐标
            if (depthInMeters > 0 && depthInMeters < 5) {  // 有效深度值
                pcl::PointXYZ point;
                point.x = (u - cx) * depthInMeters / fx;
                point.y = (v - cy) * depthInMeters / fy;
                point.z = depthInMeters;
                tempCloud.push_back(point);  // 只有有效点才被添加到点云中
            }
        }
    }

    // 转换为PCLPointCloud2
    pcl::toPCLPointCloud2(tempCloud, *pcl_cloud);
    rclcpp::Time now2 = now();
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    log_file_ << "Get realsense depth " << std::endl;
    log_file_ << "use_t1: " << now2.seconds() - now1.seconds() << std::endl;
    std::unique_lock<std::mutex> lock(sensor_mutex_);
    // 应用体素滤波...
    
    // pcl_conversions::toPCL(realsense_msg, *pcl_cloud);

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(pcl_cloud.makeShared());
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    realsense_data_ = *cloud_filtered;
    rclcpp::Time new_reals_t = rclcpp::Time(realsense_dep_msg.header.stamp);
    double diff_t = new_reals_t.seconds() - realsense_time_.seconds();
    //std::cout << "diff_t: " << diff_t << std::endl;
    realsense_time_ = rclcpp::Time(realsense_dep_msg.header.stamp);
    rclcpp::Time now3 = now();
    log_file_ << "use_t2: " << now3.seconds() - now2.seconds() << std::endl;
    //realsense_dep_msg = *msg;
    //static 
    //std::cout << std::fixed;
    //std::cout << "[ " << target_time.seconds() << ", " << odom_buffer_.back().first.seconds() << " ]" << std::endl;
    //std::cout << "stop realsense" << std::endl;
}

void FusionMapNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // std::cout << "start odom" << std::endl;
    // 处理里程计数据...
    std::unique_lock<std::mutex> lock(sensor_mutex_);
    nav_msgs::msg::Odometry corr_msg = *msg;
    rclcpp::Time msg_time = rclcpp::Time(corr_msg.header.stamp);
    //rclcpp::Duration additional_duration = rclcpp::Duration::from_seconds(0.15);
    //rclcpp::Time msg_time = msg_time1 + additional_duration;

    static double last_odom_time = 0;
    rclcpp::Time now_odom_time_ = rclcpp::Time(msg_time);
    double now_odom_time = now_odom_time_.seconds();
    if (last_odom_time>0)
    {
        if (now_odom_time - last_odom_time < 0.1)
        {
            return;
        }
    }
    last_odom_time = now_odom_time;
    
    // 将最新的里程计数据和时间戳存储到缓冲区中
    odom_buffer_.push_back(std::make_pair(msg_time, corr_msg));
    // 移除超过1秒历史的数据
    while (!odom_buffer_.empty() && (msg_time - odom_buffer_.front().first).seconds() > 10.0) {
        odom_buffer_.pop_front();
        odom_init = true;
    }
    //nav_msgs::msg::Odometry odom1 = odom_buffer_.back().second();
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = now();
    transformStamped.header.frame_id = "odom"; // odom frame ID
    transformStamped.child_frame_id = "base_link"; // base_link frame ID

    // 设置变换的平移和旋转分量（假设这些值是随时间变化的）
    transformStamped.transform.translation.x = corr_msg.pose.pose.position.x; // 替换为 x 值
    transformStamped.transform.translation.y = corr_msg.pose.pose.position.y; // 替换为 y 值
    transformStamped.transform.translation.z = corr_msg.pose.pose.position.z; // 替换为 z 值

    transformStamped.transform.rotation.x = corr_msg.pose.pose.orientation.x; // 替换为旋转 x 分量
    transformStamped.transform.rotation.y = corr_msg.pose.pose.orientation.y; // 替换为旋转 y 分量
    transformStamped.transform.rotation.z = corr_msg.pose.pose.orientation.z; // 替换为旋转 z 分量
    transformStamped.transform.rotation.w = corr_msg.pose.pose.orientation.w; // 替换为旋转 w 分量

    //tf_broadcaster_.sendTransform(transformStamped);
    // std::cout << "stop odom" << std::endl;
}

void FusionMapNode::processData() {
    while (running_) {
        std::unique_lock<std::mutex> lock(sensor_mutex_);
        log_file_ << "process init " << lidar_init << tof_init << realsense_init << odom_init << std::endl;
        //if (lidar_init && tof_init && realsense_init && odom_init)
        if (lidar_init && tof_init && odom_init)
        {
            //std::cout << "!!!!!!not init" << std::endl;
            //return;
            // 获取雷达的时间戳
            //rclcpp::Time laser_time = ;

            // 获取TOF和realsense的时间戳
            //rclcpp::Time tof_time = ...;
            //rclcpp::Time realsense_time = ...;

            // 计算TOF和realsense时刻的位姿
            bool isget_lidar, isget_tof, isget_realsense, isget_ground;
            
            Eigen::Matrix4f lidar_pose = interpolatePose(lidar_time_, isget_lidar);
            log_file_ << "lidar is get  " << isget_lidar << std::endl;
            Eigen::Matrix4f tof_pose = interpolatePose(tof_time_, isget_tof);
            log_file_ << "tof is get  " << isget_tof << std::endl;
            Eigen::Matrix4f realsense_pose = interpolatePose(realsense_time_, isget_realsense);
            log_file_ << "realsense is get  " << isget_realsense << std::endl;
            Eigen::Matrix4f ground_pose = interpolatePose(ground_time_, isget_ground);
            log_file_ << "ground is get  " << isget_ground << std::endl;
            // std::cout << "1" << std::endl;
            // 计算位姿差异
            //Eigen::Matrix4f tof_pose_diff = calculatePoseDifference(tof_pose, laser_time);
            //Eigen::Matrix4f realsense_pose_diff = calculatePoseDifference(realsense_pose, laser_time);

            // 将TOF和realsense数据变换到lidar坐标系下
            pcl::PointCloud<pcl::PointXYZ>::Ptr data_lidar(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr data_tof(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr data_realsense(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr data_ground(new pcl::PointCloud<pcl::PointXYZ>);
            
            int tt = ++tot_;
            if(isget_lidar)
                data_lidar = applyTransform(lidar_data_,   lidar_pose * transform_lidar_to_baselink, false);
            if (data_lidar && esdf_mode)
            {
                Eigen::Matrix4f lidar_origin = lidar_pose * transform_lidar_to_baselink;
                raycast_origin_ = Eigen::Vector3d(lidar_origin(0, 3), lidar_origin(1, 3), lidar_origin(2, 3));
                RaycastProcess(0, data_lidar->points.size(), tt, *data_lidar);
            }
                
            /* pcl::PCLPointCloud2 cloud0;
            pcl::toPCLPointCloud2(*data_lidar, cloud0);
            sensor_msgs::msg::PointCloud2 cloud1;
            pcl_conversions::fromPCL(cloud0, cloud1);
            cloud1.header.frame_id = "base_link";
            pcl_pub_->publish(cloud1); */

            if(isget_tof)
                data_tof = applyTransform(tof_data_, tof_pose * transform_tof_to_baselink, false);
            if (data_tof && esdf_mode)
            {
                Eigen::Matrix4f tof_origin = tof_pose * transform_tof_to_baselink;
                raycast_origin_ = Eigen::Vector3d(tof_origin(0, 3), tof_origin(1, 3), tof_origin(2, 3));
                RaycastProcess(0, data_tof->points.size(), tt, *data_tof);
            }
            
            /* pcl::PCLPointCloud2 cloud0;
            pcl::toPCLPointCloud2(*data_tof, cloud0);
            sensor_msgs::msg::PointCloud2 cloud1;
            pcl_conversions::fromPCL(cloud0, cloud1);
            cloud1.header.frame_id = "base_link";
            pcl_pub_->publish(cloud1); */

            if(isget_realsense)
                data_realsense = applyTransform(realsense_data_, realsense_pose * transform_realsense_to_baselink, false);
            if (data_realsense && esdf_mode)
            {
                Eigen::Matrix4f realsense_origin = realsense_pose * transform_realsense_to_baselink;
                raycast_origin_ = Eigen::Vector3d(realsense_origin(0, 3), realsense_origin(1, 3), realsense_origin(2, 3));
                RaycastProcess(0, data_realsense->points.size(), tt, *data_realsense);
            }
                

            if(isget_ground)
                data_ground = applyTransform(ground_data_, ground_pose * transform_realsense_to_baselink, true);
            /* if (data_ground)
            {
                Eigen::Matrix4f lidar_origin = lidar_pose * transform_lidar_to_baselink;
                raycast_origin_ = Eigen::Vector3d(lidar_pose(0, 3), lidar_pose(1, 3), lidar_pose(2, 3));
                RaycastProcess(0, data_ground->points.size(), tt, *data_ground);
            } */
            if (esdf_mode)
            {
                cur_pos_ = Eigen::Vector3d(odom_buffer_.back().second.pose.pose.position.x, 
                                                odom_buffer_.back().second.pose.pose.position.y,
                                                odom_buffer_.back().second.pose.pose.position.z);
                UpdateEsdfEvent();
            }

            /* sensor_msgs::msg::PointCloud2 cloud1;
            pcl_conversions::fromPCL(ground_data_, cloud1);
            cloud1.header.frame_id = "base_link";
            pcl_pub_->publish(cloud1);
            return; */
            // std::cout << "3" << std::endl;
            // 合并点云
            if (!esdf_mode)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                mergePointClouds(merged_cloud, data_lidar, data_tof, data_realsense, data_ground);
                // std::cout << "4" << std::endl;
                // 去除重复或相邻点 & outliers
                filterPointCloud(merged_cloud);
                // std::cout << "5" << std::endl;
                // 压缩到二维
                flattenPointCloud(merged_cloud);
                // std::cout << "6" << std::endl;
                // 投影到当前时刻的里程计坐标下
                projectPointCloud(merged_cloud);
                // std::cout << "7" << std::endl;
                // 生成局部栅格地图
                generateGridMap(merged_cloud);
            }
            // TODO: 进一步处理或发布栅格地图

            // TODO: 开始融合sensor data
            // 1. 投射到同一个点云变量
            // 2. 压缩到2维下
            
            // TODO: 将2D点云转换到当前时刻now()下对应里程计位姿坐标系中
            // 1. 发布点云，则是当前最新的融合障碍物点云

            // TODO: 一直维护一个3 × 3的局部栅格地图
            // 1. 将最新的融合障碍物点云不断的更新到栅格地图中

            // TODO: 全局融合地图涉及到SLAM优化，因此要绑定雷达帧node_id
            // 1. 每一帧都要绑定上node_id，没node_id则删除
            // 2. SLAM完成建图后会有一个全局node_id位姿优化，根据优化重新计算点云的全局位姿
            // 3. 拼接点云到全局栅格地图中
        }
        else{
            log_file_ << "!!!!!!not init" << std::endl;
        }
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
}

Eigen::Matrix4f FusionMapNode::interpolatePose(const rclcpp::Time& target_time, bool& isget) {

    //isget = true;
    //return Eigen::Matrix4f::Identity();
    // TODO: 实现根据速度插值计算位姿
    if (odom_buffer_.empty() || !(target_time.nanoseconds() > 0) ) {
        isget = false;
        return Eigen::Matrix4f::Identity();
        //throw std::runtime_error("Odometry buffer is empty.");
    }
    

    // 寻找最接近的两个位姿
    auto it1 = odom_buffer_.begin();
    auto it2 = it1;
    ++it2;
    for (; it2 != odom_buffer_.end(); ++it1, ++it2) {
        if (it2->first >= target_time) {
            break;
        }
    }
    
    if (it2 == odom_buffer_.end()) {
        std::cout << std::fixed;
        log_file_ << "[ " << target_time.seconds() << ", " << odom_buffer_.back().first.seconds() << " ]" << std::endl;
        //throw std::runtime_error("Target time is out of range of the odometry buffer.");
        //isget = false;
        //return Eigen::Matrix4f::Identity();
        // 计算时间比例
        double t1 = it1->first.seconds();
        double t2 = it2->first.seconds();
        double alpha = (target_time.seconds() - t1) / (t2 - t1);

        Eigen::Vector3f pos1(it1->second.pose.pose.position.x, it1->second.pose.pose.position.y, it1->second.pose.pose.position.z);
        Eigen::Quaternionf rot1(it1->second.pose.pose.orientation.w, it1->second.pose.pose.orientation.x, it1->second.pose.pose.orientation.y, it1->second.pose.pose.orientation.z);

        // 构建变换矩阵
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation.block<3,3>(0,0) = rot1.toRotationMatrix();
        transformation.block<3,1>(0,3) = pos1;
        isget = true;
        return transformation;
    }
    else{
        // 计算时间比例
        double t1 = it1->first.seconds();
        double t2 = it2->first.seconds();
        double alpha = (target_time.seconds() - t1) / (t2 - t1);

        // 插值位置
        Eigen::Vector3f pos1(it1->second.pose.pose.position.x, it1->second.pose.pose.position.y, it1->second.pose.pose.position.z);
        Eigen::Vector3f pos2(it2->second.pose.pose.position.x, it2->second.pose.pose.position.y, it2->second.pose.pose.position.z);
        Eigen::Vector3f interpolated_pos = (1 - alpha) * pos1 + alpha * pos2;

        // 插值旋转
        Eigen::Quaternionf rot1(it1->second.pose.pose.orientation.w, it1->second.pose.pose.orientation.x, it1->second.pose.pose.orientation.y, it1->second.pose.pose.orientation.z);
        Eigen::Quaternionf rot2(it2->second.pose.pose.orientation.w, it2->second.pose.pose.orientation.x, it2->second.pose.pose.orientation.y, it2->second.pose.pose.orientation.z);
        Eigen::Quaternionf interpolated_rot = rot1.slerp(alpha, rot2);

        // 构建变换矩阵
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation.block<3,3>(0,0) = interpolated_rot.toRotationMatrix();
        transformation.block<3,1>(0,3) = interpolated_pos;

        isget = true;
        return transformation;
    }
    
}

//Eigen::Matrix4f FusionMapNode::calculatePoseDifference(const nav_msgs::msg::Odometry& pose, const rclcpp::Time& reference_time) {
    // TODO: 实现位姿差异的计算
//}

pcl::PointCloud<pcl::PointXYZ>::Ptr FusionMapNode::applyTransform(pcl::PCLPointCloud2& data, const Eigen::Matrix4f& transform, bool isground) {
    //std::cout << "Transform Matrix:" << std::endl << transform << std::endl;
    // TODO: 使用Eigen变换矩阵对点云数据进行变换
    // 将 pcl::PCLPointCloud2 转换为 pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    log_file_ << "apply" << std::endl;
    pcl::fromPCLPointCloud2(data, *cloud);

    // 应用变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    /* for (size_t i = 0; i < cloud->points.size(); ++i) {
        Eigen::Vector4f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1.0);
        Eigen::Vector4f transformed_point = transform * point;
        pcl::PointXYZ transformed_pcl_point(transformed_point[0], transformed_point[1], transformed_point[2]);
        transformed_cloud->push_back(transformed_pcl_point);
    } */
    if (isground)
    {
        log_file_ << "*cloud.size   " << cloud->size() << std::endl;
        log_file_ << "(*cloud)[1].z" << (*cloud)[1].z << std::endl;
    }
        
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    if (isground)
        log_file_ << "*transformed_cloud.size   " << transformed_cloud->size() << std::endl;
    if (isground)
    {
        log_file_ << "(*transformed_cloud)[1].z" << (*transformed_cloud)[1].z << std::endl;
        for (auto& point : *transformed_cloud) {
            log_file_ << "point.z" << point.z << std::endl;
            point.z += 0.5;
            log_file_ << "!!!!point.z" << point.z << std::endl;
        }
    }
    
    

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(transformed_cloud);
    pass.setFilterFieldName("z");  // 设置要过滤的维度为 Z 轴
    pass.setFilterLimits(0.0, 1.5); // 设置 z 值的过滤范围（在 0.05m 和 1.0m 之间）
    pass.filter(*filtered_cloud); // 应用过滤器
    if (isground)
        log_file_ << "*filtered_cloud.size   " << filtered_cloud->size() << std::endl;

    /* pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(filtered_cloud); // 使用上一步的输出作为输入
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-4.0, 4.0); // 设置x值的过滤范围
    pass_x.filter(*filtered_cloud);

    // 最后对Y轴进行过滤
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(filtered_cloud); // 使用上一步的输出作为输入
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-4.0, 4.0); // 设置y值的过滤范围
    pass_y.filter(*filtered_cloud); // 可直接写回到final_cloud */

    // 将变换后的 pcl::PointCloud<pcl::PointXYZ> 转换回 pcl::PCLPointCloud2
    //pcl::PCLPointCloud2 data1;
    //pcl::toPCLPointCloud2(*filtered_cloud, data1);
    return filtered_cloud;
}

void FusionMapNode::mergePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& merged_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr data_lidar,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr data_tof, pcl::PointCloud<pcl::PointXYZ>::Ptr data_realsense,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr data_ground) {
    // 假设 lidar_data_, tof_data_ 和 realsense_data_ 是三个PCLPointCloud2对象
    /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tof(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_realsense(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromPCLPointCloud2(data_lidar, *cloud_lidar);
    pcl::fromPCLPointCloud2(data_tof, *cloud_tof);
    pcl::fromPCLPointCloud2(data_realsense, *cloud_realsense);
    pcl::fromPCLPointCloud2(data_ground, *cloud_ground); */

    // 合并点云
    if (data_tof)
    {
        if (!data_tof->empty())
            *data_lidar += *data_tof;
    }
    else{
        log_file_ << "no data_tof" << std::endl;
    }

    if (data_realsense)
    {
        if (!data_realsense->empty())
            *data_lidar += *data_realsense;
    }
    else{
        log_file_ << "no data_realsense" << std::endl;
    }
    
    if (data_ground)
    {
        if (!data_ground->empty())
            *data_lidar += *data_ground;
    }
    else{
        log_file_ << "no data_ground" << std::endl;
    }
    
    
    
    
    *merged_cloud = *data_lidar;

    //pcl::toPCLPointCloud2(*cloud_lidar, merged_cloud);
}

void FusionMapNode::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    /* pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2(cloud));
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(temp_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);  // 可以调整体素大小
    sor.filter(cloud); */
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    //pcl::fromPCLPointCloud2(cloud, *cloud_xyz);

    // 应用体素网格滤波
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f); // 可以调整体素大小
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    sor.filter(*cloud);

    // 应用半径离群值移除滤波
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.1); // 设置搜索半径，可根据实际情况调整
    outrem.setMinNeighborsInRadius(2); // 设置半径内最小邻居数目，可根据实际情况调整
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    outrem.filter(*cloud);

    // 将滤波后的PointCloud<pcl::PointXYZ>转换回PCLPointCloud2
    //pcl::toPCLPointCloud2(*cloud_filtered, cloud);

}

void FusionMapNode::flattenPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    //pcl::fromPCLPointCloud2(cloud, *cloud_xyz);

    for (auto& point : cloud->points) {
        point.z = 0;  // 将Z坐标设为0，压缩到二维平面
    }

    //pcl::toPCLPointCloud2(*cloud_xyz, cloud);
}

void FusionMapNode::projectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // 获取当前时刻的里程计位姿
    bool isget;
    Eigen::Matrix4f current_odom_pose = interpolatePose(tof_time_, isget);

    // 将点云转换到当前里程计位姿下
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    //pcl::fromPCLPointCloud2(cloud, *cloud_xyz);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *cloud, current_odom_pose.inverse());

    //pcl::toPCLPointCloud2(*transformed_cloud, cloud);

    // TODO: 根据需要生成局部栅格地图
}

void FusionMapNode::generateGridMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // 定义栅格地图的大小和分辨率
    // TODO: 创建和更新栅格地图
    pcl::PCLPointCloud2 cloud0;
    pcl::toPCLPointCloud2(*cloud, cloud0);
    sensor_msgs::msg::PointCloud2 cloud1;
    pcl_conversions::fromPCL(cloud0, cloud1);
    cloud1.header.frame_id = "base_link";

    rclcpp::Time msg_time = rclcpp::Time(lidar_time_);
    cloud1.header.stamp = msg_time;
    
    // TODO: add lidar time to cloud1

    pcl_pub_->publish(cloud1);
}

#ifdef PROBABILISTIC
void FusionMapNode::RaycastProcess(int i, int part, int tt, pcl::PointCloud<pcl::PointXYZ> cloud_) {
     Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
     for (int idx = part*i; idx < part*(i + 1); idx++) {
          std::vector<Eigen::Vector3d> output;
          if (idx > cloud_.points.size())
               break;
          pcl::PointXYZ pt = cloud_.points[idx];
          int cnt = 0;
          if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
               continue;
          //Eigen::Vector4d tmp = transform_*Eigen::Vector4d(pt.x, pt.y, pt.z, 1);
          Eigen::Vector3d point = Eigen::Vector3d(pt.x, pt.y, pt.z);

          int tmp_idx;
          //std::cout << "point" << point << std::endl;
          //std::cout << "raycast_origin_" << raycast_origin_ << std::endl;
          double length = (point - raycast_origin_).norm();
          //std::cout << "length" << length << std::endl;
          if (length < parameters_.min_ray_length_)
               continue;
          else if (length > parameters_.max_ray_length_) {
               point = (point - raycast_origin_)/length*parameters_.max_ray_length_ + raycast_origin_;
               tmp_idx = esdf_map_->SetOccupancy((Eigen::Vector3d) point, 0);
          } else
               tmp_idx = esdf_map_->SetOccupancy((Eigen::Vector3d) point, 1);
#ifdef SIGNED_NEEDED
          tmp_idx = inv_esdf_map_->SetOccupancy((Eigen::Vector3d) point, 0);
#endif
//         //TODO: -10000 ?

          if (tmp_idx!=-10000) {
#ifdef HASH_TABLE
               if (set_occ_.find(tmp_idx) != set_occ_.end())
                   continue;
                 else set_occ_.insert(tmp_idx);
#else
               log_file_ << "tmp_idx" << tmp_idx << ";  set_occ_.size()" << set_occ_.size() << std::endl;
               if (set_occ_[tmp_idx]==tt)
                    continue;
               else
                    set_occ_[tmp_idx] = tt;
#endif
          }
          Raycast(raycast_origin_/parameters_.resolution_,
                  point/parameters_.resolution_,
                  parameters_.l_cornor_/parameters_.resolution_,
                  parameters_.r_cornor_/parameters_.resolution_,
                  &output);

          for (int i = output.size() - 2; i >= 0; i--) {
               Eigen::Vector3d tmp = (output[i] + half)*parameters_.resolution_;

               length = (tmp - raycast_origin_).norm();
               if (length < parameters_.min_ray_length_)
                    break;
               if (length > parameters_.max_ray_length_)
                    continue;
               int tmp_idx;
               tmp_idx = esdf_map_->SetOccupancy(tmp, 0);
#ifdef SIGNED_NEEDED
               tmp_idx = inv_esdf_map_->SetOccupancy(tmp, 1);
#endif
               //TODO: -10000 ?
               if (tmp_idx!=-10000) {
#ifdef HASH_TABLE
                    if (set_free_.find(tmp_idx) != set_free_.end()) {
                        if (++cnt >= 1) {
                          cnt = 0;
                          break;
                        }
                      } else {
                        set_free_.insert(tmp_idx);
                        cnt = 0;
                      }
#else
                    if (set_free_[tmp_idx]==tt) {
                         if (++cnt >= 1) {
                              cnt = 0;
                              break;
                         }
                    } else {
                         set_free_[tmp_idx] = tt;
                         cnt = 0;
                    }
#endif
               }
          }
     }
}

/* void FusionMapNode::RaycastMultithread() {
     // TODO: when using vector, this is not needed
#ifdef HASH_TABLE
     set_free_.clear();
       set_occ_.clear();
#endif
     int tt = ++tot_;
     timing::Timer raycastingTimer("raycasting");

     if (parameters_.ray_cast_num_thread_==0) {
          RaycastProcess(0, cloud_.points.size(), tt);
     } else {
          int part = cloud_.points.size()/parameters_.ray_cast_num_thread_;
          std::list<std::thread> integration_threads = std::list<std::thread>();
          for (size_t i = 0; i < parameters_.ray_cast_num_thread_; ++i) {
               integration_threads.emplace_back(&FusionMapNode::RaycastProcess, this, i, part, tt);
          }
          for (std::thread &thread : integration_threads) {
               thread.join();
          }
     }
     raycastingTimer.Stop();
} */

#endif // PROBABILISTIC

void FusionMapNode::UpdateEsdfEvent() {

     //cur_pos_ = sync_pos_;

/* #ifndef PROBABILISTIC
     timing::Timer handlePCTimer("handlePointCloud");
       pcl::fromROSMsg(*sync_pc_, cloud_);

       esdf_map_->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_, false);
       esdf_map_->SetAway();

       Eigen::Vector3i tmp_vox;
       Eigen::Vector3d tmp_pos;
       for (int i = 0; i < cloud_.size(); i++) {
         tmp_pos = Eigen::Vector3d(cloud_[i].x, cloud_[i].y, cloud_[i].z);
         esdf_map_->SetOccupancy(tmp_pos, 1);
       }
       esdf_map_->SetBack();
       handlePCTimer.Stop();
#endif */
     esdf_cnt_++;
     log_file_ << "Running " << esdf_cnt_ << " updates." << std::endl;
//    ros::Time t1 = ros::Time::now();
     if (esdf_map_->CheckUpdate()) {
          timing::Timer update_esdf_timer("UpdateESDF");
          if (parameters_.global_update_)
               esdf_map_->SetOriginalRange();
          else
               esdf_map_->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_);
          esdf_map_->UpdateOccupancy(parameters_.global_update_);
          esdf_map_->UpdateESDF();
#ifdef SIGNED_NEEDED
          // TODO: Complete this SIGNED_NEEDED
            inv_esdf_map_->UpdateOccupancy();
            inv_esdf_map_->UpdateESDF();
#endif
          update_esdf_timer.Stop();
          timing::Timing::Print(std::cout);
     }
//    ros::Time t2 = ros::Time::now();

//    std::string text = "Fiesta\nCurrent update Time\n"
//                       + timing::Timing::SecondsToTimeString((t2 - t1).toSec() * 1000)
//                       + " ms\n" + "Average update Time\n" +
//                       timing::Timing::SecondsToTimeString(timing::Timing::GetMeanSeconds("UpdateESDF") * 1000)
//                       + " ms";

     //if (parameters_.visualize_every_n_updates_!=0 && esdf_cnt_%parameters_.visualize_every_n_updates_==0) {
//        std::thread(Visualization, esdf_map_, text).detach();
        //feista::ESDFMap *esdf_map = esdf_map_;
          Visualization(parameters_.global_vis_, "");
     //}
//    else {
//        std::thread(Visualization, nullptr, text).detach();
//        Visualization(nullptr, globalVis, "");
//    }
}

void FusionMapNode::Visualization(bool global_vis, const std::string &text) {
    if (esdf_map_ != nullptr) {
        log_file_ << "Visualization" << std::endl;
        if (global_vis)
            esdf_map_->SetOriginalRange();
        else
            esdf_map_->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_, false);

        sensor_msgs::msg::PointCloud2 pc;
        esdf_map_->GetPointCloud2(pc, parameters_.vis_lower_bound_, parameters_.vis_upper_bound_);
        occupancy_pub_->publish(pc);

        visualization_msgs::msg::Marker slice_marker;
        esdf_map_->GetSliceMarker(slice_marker, parameters_.slice_vis_level_, 100,
                                Eigen::Vector4d(0, 1.0, 0, 1), parameters_.slice_vis_max_dist_);
        slice_pub_->publish(slice_marker);
    }

    if (!text.empty()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.id = 3456;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::MODIFY;

        marker.pose.position.x = 8.0;
        marker.pose.position.y = 2.0;
        marker.pose.position.z = 3.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.text = text;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.6;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        text_pub_->publish(marker);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
