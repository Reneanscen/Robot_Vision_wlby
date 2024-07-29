#include <fusion_map/fusion_map.h>




FusionMapNode::FusionMapNode() : Node("zme_fusion_map_node"), tf_broadcaster_(this) {
    initializeSubscribers();
}

FusionMapNode::~FusionMapNode() {}

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
    std::cout << "px1: " << px1 << std::endl;
    this->get_parameter("scan_to_baselink_transform.py", py1);
    this->get_parameter("scan_to_baselink_transform.pz", pz1);
    this->get_parameter("scan_to_baselink_transform.roll", roll1);
    this->get_parameter("scan_to_baselink_transform.pitch", pitch1);
    this->get_parameter("scan_to_baselink_transform.yaw", yaw1);
    transform_lidar_to_baselink = createTransformMatrix(px1, py1, pz1, roll1, pitch1, yaw1);
    
    float px2, py2, pz2, roll2, pitch2, yaw2;
    this->get_parameter("tof_to_baselink_transform.px", px2);
    std::cout << "px2: " << px2 << std::endl;
    this->get_parameter("tof_to_baselink_transform.py", py2);
    this->get_parameter("tof_to_baselink_transform.pz", pz2);
    this->get_parameter("tof_to_baselink_transform.roll", roll2);
    this->get_parameter("tof_to_baselink_transform.pitch", pitch2);
    this->get_parameter("tof_to_baselink_transform.yaw", yaw2);
    transform_tof_to_baselink = createTransformMatrix(px2, py2, pz2, roll2, pitch2, yaw2);

    float px3, py3, pz3, roll3, pitch3, yaw3;
    this->get_parameter("realsense_to_baselink_transform.px", px3);
    std::cout << "px3: " << px3 << std::endl;
    this->get_parameter("realsense_to_baselink_transform.py", py3);
    this->get_parameter("realsense_to_baselink_transform.pz", pz3);
    this->get_parameter("realsense_to_baselink_transform.roll", roll3);
    this->get_parameter("realsense_to_baselink_transform.pitch", pitch3);
    this->get_parameter("realsense_to_baselink_transform.yaw", yaw3);
    transform_realsense_to_baselink = createTransformMatrix(px3, py3, pz3, roll3, pitch3, yaw3);


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
    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fusion_map_pcl", 1);

    ground_sub_ = this->create_subscription<zme_msg_srv::msg::ObstacleavoidList>(
            "/Visual_obstacle_avoidance", 1, std::bind(&FusionMapNode::groundCallback, this, std::placeholders::_1));
    
    realsense_dep_sub_ = this->create_subscription<custom_image_msg::msg::Image4m>(
            "/head_cam_depth", qos, std::bind(&FusionMapNode::realsenseDepCallback, this, std::placeholders::_1));
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
    static double last_time;
    static sensor_msgs::msg::LaserScan last_msg;
    last_msg = *msg;
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
    for (size_t i = 0; i < last_msg.ranges.size(); i++)
    {
        if (i*0.0029183398000895977 > 0 && i*0.0029183398000895977 < 0.4 && last_msg.ranges[i] < 0.22)
        {
            last_msg.ranges[i] = NAN;
        }
        else if (i*0.0029183398000895977 < 1.57 && last_msg.ranges[i] < 0.33)
        {
            last_msg.ranges[i] = NAN;
        }
        
        
    }
    

    std::cout << "start laser" << std::endl;
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    // 处理激光雷达数据...
    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 cloud;
    projector_.projectLaser(last_msg, cloud);
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud, *pcl_cloud);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(pcl_cloud.makeShared());
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    
    lidar_data_ = *cloud_filtered;
    lidar_time_ = rclcpp::Time(last_msg.header.stamp);

    // 融合
    processData();
    //last_msg = *msg;
    std::cout << "stop laser" << std::endl;
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
    std::cout << "start tof" << std::endl;
    std::lock_guard<std::mutex> lock(sensor_mutex_);
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
    
    std::cout << "stop tof" << std::endl;
}


void FusionMapNode::groundCallback(const zme_msg_srv::msg::ObstacleavoidList::SharedPtr msg) {
    static zme_msg_srv::msg::ObstacleavoidList ground_msg;
    ground_msg = *msg;
    static int ground_n = 0;
    if (!ground_init)
    {
        ground_n ++;
        if (ground_n >= 10)
        {
            ground_init = true;
        }
        //ground_msg = *msg;
        return;
    }

    std::cout << "start ground" << std::endl;
    std::lock_guard<std::mutex> lock(sensor_mutex_);

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
    std::cout << "stop ground" << std::endl;
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
    std::cout << "start realsense" << std::endl;
    std::lock_guard<std::mutex> lock(sensor_mutex_);
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
    
    std::cout << "stop realsense" << std::endl;
}




void FusionMapNode::realsenseDepCallback(const custom_image_msg::msg::Image4m::SharedPtr msg) {
    static int fre = 0;
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

    fre ++;
    if (fre > 1)
    {
        fre = 0;
    }else{
        return;
    }
    
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
    // Depth image to point cloud  ////////////////////////////////////////////////////////////////////////////////
    // 相机内参（示例值，根据你的相机进行调整）
    float fx = 9.1271350097656250e+02; // Focal length in x
    float fy = 9.1257855224609375e+02; // Focal length in y
    float cx = realsense_dep_msg.width / 2.0f; // Optical center in x
    float cy = realsense_dep_msg.height / 2.0f; // Optical center in y

    pcl::PointCloud<pcl::PointXYZ> tempCloud;
    tempCloud.width = realsense_dep_msg.width;
    tempCloud.height = realsense_dep_msg.height;
    tempCloud.is_dense = false;
    tempCloud.points.resize(tempCloud.width * tempCloud.height);
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
            pcl::PointXYZ& pt = tempCloud.points[v * realsense_dep_msg.width + u];
            if (depthInMeters > 0) // 有效深度值
            {
                pt.x = (u - cx) * depthInMeters / fx;
                pt.y = (v - cy) * depthInMeters / fy;
                pt.z = depthInMeters;
            }
            else
            {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    // 转换为PCLPointCloud2
    pcl::toPCLPointCloud2(tempCloud, *pcl_cloud);


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "start realsense" << std::endl;
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    // 应用体素滤波...
    
    // pcl_conversions::toPCL(realsense_msg, *pcl_cloud);

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(pcl_cloud.makeShared());
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    realsense_data_ = *cloud_filtered;
    realsense_time_ = rclcpp::Time(realsense_dep_msg.header.stamp);
    //realsense_dep_msg = *msg;
    std::cout << "stop realsense" << std::endl;
}

void FusionMapNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::cout << "start odom" << std::endl;
    // 处理里程计数据...
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);
    //rclcpp::Duration additional_duration = rclcpp::Duration::from_seconds(0.15);
    //rclcpp::Time msg_time = msg_time1 + additional_duration;
    nav_msgs::msg::Odometry corr_msg = *msg;
    corr_msg.header.stamp = msg_time;
    // 将最新的里程计数据和时间戳存储到缓冲区中
    odom_buffer_.push_back(std::make_pair(corr_msg.header.stamp, corr_msg));
    // 移除超过1秒历史的数据
    while (!odom_buffer_.empty() && (msg_time - odom_buffer_.front().first).seconds() > 3.0) {
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
    std::cout << "stop odom" << std::endl;
}

void FusionMapNode::processData() {
    if (!(lidar_init && tof_init && realsense_init && odom_init))
    //if (!(lidar_init && realsense_init && odom_init))
    {
        std::cout << "!!!!!!not init" << std::endl;
        return;
    }
    


    // 获取雷达的时间戳
    //rclcpp::Time laser_time = ;

    // 获取TOF和realsense的时间戳
    //rclcpp::Time tof_time = ...;
    //rclcpp::Time realsense_time = ...;

    // 计算TOF和realsense时刻的位姿
    bool isget_lidar, isget_tof, isget_realsense, isget_ground;
    std::cout << "lidar" << std::endl;
    Eigen::Matrix4f lidar_pose = interpolatePose(lidar_time_, isget_lidar);
    std::cout << "tof" << std::endl;
    Eigen::Matrix4f tof_pose = interpolatePose(tof_time_, isget_tof);
    std::cout << "realsense" << std::endl;
    Eigen::Matrix4f realsense_pose = interpolatePose(realsense_time_, isget_realsense);
    std::cout << "ground" << std::endl;
    Eigen::Matrix4f ground_pose = interpolatePose(ground_time_, isget_ground);
    std::cout << "1" << std::endl;
    // 计算位姿差异
    //Eigen::Matrix4f tof_pose_diff = calculatePoseDifference(tof_pose, laser_time);
    //Eigen::Matrix4f realsense_pose_diff = calculatePoseDifference(realsense_pose, laser_time);

    // 将TOF和realsense数据变换到lidar坐标系下
    pcl::PCLPointCloud2 data_lidar, data_tof, data_realsense, data_ground;
    if(isget_lidar)
        data_lidar = applyTransform(lidar_data_,   lidar_pose * transform_lidar_to_baselink, false);
    std::cout << "2" << std::endl;
    /* sensor_msgs::msg::PointCloud2 cloud1;
    pcl_conversions::fromPCL(lidar_data_, cloud1);
    cloud1.header.frame_id = "base_link";
    pcl_pub_->publish(cloud1); */

    if(isget_tof)
        data_tof = applyTransform(tof_data_, tof_pose * transform_tof_to_baselink, false);
    /* sensor_msgs::msg::PointCloud2 cloud1;
    pcl_conversions::fromPCL(data_tof, cloud1);
    cloud1.header.frame_id = "base_link";
    pcl_pub_->publish(cloud1); */

    if(isget_realsense)
        data_realsense = applyTransform(realsense_data_, realsense_pose * transform_realsense_to_baselink, false);
    if(isget_ground)
        data_ground = applyTransform(ground_data_, ground_pose * transform_realsense_to_baselink, true);
    
    /* sensor_msgs::msg::PointCloud2 cloud1;
    pcl_conversions::fromPCL(data_realsense, cloud1);
    cloud1.header.frame_id = "base_link";
    pcl_pub_->publish(cloud1); */
    std::cout << "3" << std::endl;
    // 合并点云
    pcl::PCLPointCloud2 merged_cloud;
    mergePointClouds(merged_cloud, data_lidar, data_tof, data_realsense, data_ground);
    std::cout << "4" << std::endl;
    // 去除重复或相邻点 & outliers
    filterPointCloud(merged_cloud);
    std::cout << "5" << std::endl;
    // 压缩到二维
    flattenPointCloud(merged_cloud);
    std::cout << "6" << std::endl;
    // 投影到当前时刻的里程计坐标下
    projectPointCloud(merged_cloud);
    std::cout << "7" << std::endl;
    // 生成局部栅格地图
    generateGridMap(merged_cloud);

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

Eigen::Matrix4f FusionMapNode::interpolatePose(const rclcpp::Time& target_time, bool& isget) {

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
        std::cout << "[ " << target_time.seconds() << ", " << odom_buffer_.back().first.seconds() << " ]" << std::endl;
        //throw std::runtime_error("Target time is out of range of the odometry buffer.");
        isget = false;
        return Eigen::Matrix4f::Identity();
    }

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

//Eigen::Matrix4f FusionMapNode::calculatePoseDifference(const nav_msgs::msg::Odometry& pose, const rclcpp::Time& reference_time) {
    // TODO: 实现位姿差异的计算
//}

pcl::PCLPointCloud2 FusionMapNode::applyTransform(pcl::PCLPointCloud2& data, const Eigen::Matrix4f& transform, bool isground) {
    //std::cout << "Transform Matrix:" << std::endl << transform << std::endl;
    // TODO: 使用Eigen变换矩阵对点云数据进行变换
    // 将 pcl::PCLPointCloud2 转换为 pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
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
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    if (isground)
    {
        for (auto& point : *transformed_cloud) {
            point.z += 0.5;
        }
    }
    
    

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(transformed_cloud);
    pass.setFilterFieldName("z");  // 设置要过滤的维度为 Z 轴
    pass.setFilterLimits(0.0, 1.5); // 设置 z 值的过滤范围（在 0.05m 和 1.0m 之间）
    pass.filter(*filtered_cloud); // 应用过滤器

    // 将变换后的 pcl::PointCloud<pcl::PointXYZ> 转换回 pcl::PCLPointCloud2
    pcl::PCLPointCloud2 data1;
    pcl::toPCLPointCloud2(*filtered_cloud, data1);
    return data1;
}

void FusionMapNode::mergePointClouds(pcl::PCLPointCloud2& merged_cloud, pcl::PCLPointCloud2 data_lidar,
                                     pcl::PCLPointCloud2 data_tof, pcl::PCLPointCloud2 data_realsense,
                                     pcl::PCLPointCloud2 data_ground) {
    // 假设 lidar_data_, tof_data_ 和 realsense_data_ 是三个PCLPointCloud2对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tof(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_realsense(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromPCLPointCloud2(data_lidar, *cloud_lidar);
    pcl::fromPCLPointCloud2(data_tof, *cloud_tof);
    pcl::fromPCLPointCloud2(data_realsense, *cloud_realsense);
    pcl::fromPCLPointCloud2(data_ground, *cloud_ground);

    // 合并点云
    *cloud_lidar += *cloud_tof;
    *cloud_lidar += *cloud_realsense;
    *cloud_lidar += *cloud_ground;

    pcl::toPCLPointCloud2(*cloud_lidar, merged_cloud);
}

void FusionMapNode::filterPointCloud(pcl::PCLPointCloud2& cloud) {
    /* pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2(cloud));
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(temp_cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);  // 可以调整体素大小
    sor.filter(cloud); */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(cloud, *cloud_xyz);

    // 应用体素网格滤波
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_xyz);
    sor.setLeafSize(0.05f, 0.05f, 0.05f); // 可以调整体素大小
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    sor.filter(*temp_cloud);

    // 应用半径离群值移除滤波
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(temp_cloud);
    outrem.setRadiusSearch(0.1); // 设置搜索半径，可根据实际情况调整
    outrem.setMinNeighborsInRadius(2); // 设置半径内最小邻居数目，可根据实际情况调整
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    outrem.filter(*cloud_filtered);

    // 将滤波后的PointCloud<pcl::PointXYZ>转换回PCLPointCloud2
    pcl::toPCLPointCloud2(*cloud_filtered, cloud);

}

void FusionMapNode::flattenPointCloud(pcl::PCLPointCloud2& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(cloud, *cloud_xyz);

    for (auto& point : cloud_xyz->points) {
        point.z = 0;  // 将Z坐标设为0，压缩到二维平面
    }

    pcl::toPCLPointCloud2(*cloud_xyz, cloud);
}

void FusionMapNode::projectPointCloud(pcl::PCLPointCloud2& cloud) {
    // 获取当前时刻的里程计位姿
    bool isget;
    Eigen::Matrix4f current_odom_pose = interpolatePose(lidar_time_, isget);

    // 将点云转换到当前里程计位姿下
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(cloud, *cloud_xyz);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_xyz, *transformed_cloud, current_odom_pose.inverse());

    pcl::toPCLPointCloud2(*transformed_cloud, cloud);

    // TODO: 根据需要生成局部栅格地图
}

void FusionMapNode::generateGridMap(const pcl::PCLPointCloud2& cloud) {
    // 定义栅格地图的大小和分辨率
    // TODO: 创建和更新栅格地图
    sensor_msgs::msg::PointCloud2 cloud1;
    pcl_conversions::fromPCL(cloud, cloud1);
    cloud1.header.frame_id = "base_link";

    rclcpp::Time msg_time = rclcpp::Time(lidar_time_);
    cloud1.header.stamp = msg_time;
    
    // TODO: add lidar time to cloud1

    pcl_pub_->publish(cloud1);
}
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
