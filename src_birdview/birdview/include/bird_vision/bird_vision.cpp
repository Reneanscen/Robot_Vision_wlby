//
// Created by xu on 24-3-21.
//
#include "bird_vision.h"

BirdVision::BirdVision(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {

    declareParameters();

    getParameters();

    logParameters();

    // 定义动态调参的回调函数
    parameter_event_subscriber_ = node_->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
                auto result = rcl_interfaces::msg::SetParametersResult();
                result.successful = parameterCallback(parameters);
                return result;
            });

    // 初始化四路相机 鸟瞰图mask掩码
    initEffectArea(int(birdVisionParameter_.width_),
                   int(birdVisionParameter_.height_),
                   int(birdVisionParameter_.blind_length_ / (2 * birdVisionParameter_.resolution_)));

    // 制作重叠区域的加权mask掩码
    extrectOverLay(front_effect_areas_, left_effect_areas_, back_effect_areas_, right_effect_areas_);

    // 初始化线程池——鸟瞰图逻辑处理
    thread_pool_ptr_ = std::make_unique<zl::ThreadPool>(4);


    // ROS通信的QOS
    const rmw_qos_profile_t rmw_qos_profile_custom = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
    };
    const rclcpp::QoSInitialization qos_initialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2);
    rclcpp::QoS custome_qos(qos_initialization, rmw_qos_profile_custom);

    if(birdVisionParameter_.v4l2_stream_enable_){

        v4l2StreamUtil_vec_.push_back(std::make_shared<V4L2StreamUtil>(birdVisionParameter_.front_cam_dev_,
                                                                       birdVisionParameter_.stream_format_,
                                                                       birdVisionParameter_.stream_width_,
                                                                       birdVisionParameter_.stream_height_,
                                                                       birdVisionParameter_.stream_fps_));

        v4l2StreamUtil_vec_.push_back(std::make_shared<V4L2StreamUtil>(birdVisionParameter_.left_cam_dev_,
                                                                       birdVisionParameter_.stream_format_,
                                                                       birdVisionParameter_.stream_width_,
                                                                       birdVisionParameter_.stream_height_,
                                                                       birdVisionParameter_.stream_fps_));

        v4l2StreamUtil_vec_.push_back(std::make_shared<V4L2StreamUtil>(birdVisionParameter_.back_cam_dev_,
                                                                       birdVisionParameter_.stream_format_,
                                                                       birdVisionParameter_.stream_width_,
                                                                       birdVisionParameter_.stream_height_,
                                                                       birdVisionParameter_.stream_fps_));

        v4l2StreamUtil_vec_.push_back(std::make_shared<V4L2StreamUtil>(birdVisionParameter_.right_cam_dev_,
                                                                       birdVisionParameter_.stream_format_,
                                                                       birdVisionParameter_.stream_width_,
                                                                       birdVisionParameter_.stream_height_,
                                                                       birdVisionParameter_.stream_fps_));
    }
    else{
        // 初始化数据订阅对象
        front_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto front_option = rclcpp::SubscriptionOptions();
        front_option.callback_group = front_group_;

        left_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto left_option = rclcpp::SubscriptionOptions();
        left_option.callback_group = left_group_;

        back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto back_option = rclcpp::SubscriptionOptions();
        back_option.callback_group = back_group_;

        right_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto right_option = rclcpp::SubscriptionOptions();
        right_option.callback_group = right_group_;

        if (front_cam_.image_type_ == 0) {
            ros_sub_front_ptr_ = node_->create_subscription<ROS_IMAGE_TYPE>(front_cam_.topic_,
                                                                            rclcpp::SystemDefaultsQoS(),
                                                                            std::bind(&BirdVision::frontROS, this,
                                                                                      std::placeholders::_1),
                                                                            front_option);
        } else if (front_cam_.image_type_ == 1) {
            custome_sub_front_ptr_ = node_->create_subscription<CUSTOMIMAGE_TYPE>(front_cam_.topic_,
                                                                                  custome_qos,
                                                                                  std::bind(&BirdVision::frontCustome, this,
                                                                                            std::placeholders::_1),
                                                                                  front_option);
        } else {
            std::cout << "前视相机订阅初始化失败，检查配置文件的image_type." << std::endl;
        }

        if (left_cam_.image_type_ == 0) {
            ros_sub_left_ptr_ = node_->create_subscription<ROS_IMAGE_TYPE>(left_cam_.topic_,
                                                                           rclcpp::SystemDefaultsQoS(),
                                                                           std::bind(&BirdVision::leftROS, this,
                                                                                     std::placeholders::_1),
                                                                           left_option);
        } else if (left_cam_.image_type_ == 1) {
            custome_sub_left_ptr_ = node_->create_subscription<CUSTOMIMAGE_TYPE>(left_cam_.topic_,
                                                                                 custome_qos,
                                                                                 std::bind(&BirdVision::leftCustome, this,
                                                                                           std::placeholders::_1),
                                                                                 left_option);
        } else {
            std::cout << "左视相机订阅初始化失败，检查配置文件的image_type." << std::endl;
        }

        if (back_cam_.image_type_ == 0) {
            ros_sub_back_ptr_ = node_->create_subscription<ROS_IMAGE_TYPE>(back_cam_.topic_,
                                                                           rclcpp::SystemDefaultsQoS(),
                                                                           std::bind(&BirdVision::backROS, this,
                                                                                     std::placeholders::_1),
                                                                           back_option);
        } else if (back_cam_.image_type_ == 1) {
            custome_sub_back_ptr_ = node_->create_subscription<CUSTOMIMAGE_TYPE>(back_cam_.topic_,
                                                                                 custome_qos,
                                                                                 std::bind(&BirdVision::backCustome, this,
                                                                                           std::placeholders::_1),
                                                                                 back_option);
        } else {
            std::cout << "后视相机订阅初始化失败，检查配置文件的image_type." << std::endl;
        }

        if (right_cam_.image_type_ == 0) {
            ros_sub_right_ptr_ = node_->create_subscription<ROS_IMAGE_TYPE>(right_cam_.topic_,
                                                                            rclcpp::SystemDefaultsQoS(),
                                                                            std::bind(&BirdVision::rightROS, this,
                                                                                      std::placeholders::_1),
                                                                            right_option);
        } else if (right_cam_.image_type_ == 1) {
            custome_sub_right_ptr_ = node_->create_subscription<CUSTOMIMAGE_TYPE>(right_cam_.topic_,
                                                                                  custome_qos,
                                                                                  std::bind(&BirdVision::rightCustome, this,
                                                                                            std::placeholders::_1),
                                                                                  right_option);
        } else {
            std::cout << "右视相机订阅初始化失败，检查配置文件的image_type." << std::endl;
        }

    }


    // -------------------- 发布对象部分 -------------
    //  测试的发布对象
    if(birdVisionParameter_.is_pub_test_images_){
        pub_front_bird_vision_ptr_ = node_->create_publisher<ROS_IMAGE_TYPE>("/front_bird_vision", rclcpp::SystemDefaultsQoS());
        pub_left_bird_vision_ptr_ = node_->create_publisher<ROS_IMAGE_TYPE>("/left_bird_vision", rclcpp::SystemDefaultsQoS());
        pub_back_bird_vision_ptr_ = node_->create_publisher<ROS_IMAGE_TYPE>("/back_bird_vision", rclcpp::SystemDefaultsQoS());
        pub_right_bird_vision_ptr_ = node_->create_publisher<ROS_IMAGE_TYPE>("/right_bird_vision", rclcpp::SystemDefaultsQoS());
        pub_all_bird_vision_ptr_ = node_->create_publisher<ROS_IMAGE_TYPE>("/ros_bird_vision", rclcpp::SystemDefaultsQoS());
    }
    // 发布需要推流的图像
    pub_cutome_birdview_ptr_ = node_->create_publisher<CUSTOMIMAGE_TYPE>("/custom_bird_vision", custome_qos);


    // ------------------- 鸟瞰图处理线程 -------------------
    birdview_thread_ = std::thread(std::bind(&BirdVision::birdViewHandleThread, this));
}


// ROS参数声明
void BirdVision::declareParameters() {
    node_->declare_parameter<bool>("V4L2Stream.v4l2_stream_enable", false);
    node_->declare_parameter<std::string>("V4L2Stream.front_cam_dev", std::string(""));
    node_->declare_parameter<std::string>("V4L2Stream.left_cam_dev", std::string(""));
    node_->declare_parameter<std::string>("V4L2Stream.back_cam_dev", std::string(""));
    node_->declare_parameter<std::string>("V4L2Stream.right_cam_dev", std::string(""));
    node_->declare_parameter<int>("V4L2Stream.stream_format", 0);
    node_->declare_parameter<int>("V4L2Stream.stream_width", 0);
    node_->declare_parameter<int>("V4L2Stream.stream_height", 0);
    node_->declare_parameter<int>("V4L2Stream.stream_fps", 0);


    node_->declare_parameter<bool>("frontCamera.open", front_cam_.open_);
    node_->declare_parameter<int>("frontCamera.cam_type", front_cam_.cam_type_);
    node_->declare_parameter<int>("frontCamera.image_type", front_cam_.image_type_);
    node_->declare_parameter<std::string>("frontCamera.topic", front_cam_.topic_);
    node_->declare_parameter<std::vector<double>>("frontCamera.intrinsic_matrix", std::vector<double>(9, 0.0));
    node_->declare_parameter<std::vector<double>>("frontCamera.distort_matrix", std::vector<double>(4, 0.0));
    node_->declare_parameter<double>("frontCamera.T_baselink_camera.x", 0.0);
    node_->declare_parameter<double>("frontCamera.T_baselink_camera.y", 0.0);
    node_->declare_parameter<double>("frontCamera.T_baselink_camera.z", 0.0);
    node_->declare_parameter<double>("frontCamera.T_baselink_camera.yaw", 0.0);
    node_->declare_parameter<double>("frontCamera.T_baselink_camera.pitch", 0.0);
    node_->declare_parameter<double>("frontCamera.T_baselink_camera.roll", 0.0);

    node_->declare_parameter<bool>("leftCamera.open", left_cam_.open_);
    node_->declare_parameter<int>("leftCamera.cam_type", left_cam_.cam_type_);
    node_->declare_parameter<int>("leftCamera.image_type", left_cam_.image_type_);
    node_->declare_parameter<std::string>("leftCamera.topic", left_cam_.topic_);
    node_->declare_parameter<std::vector<double>>("leftCamera.intrinsic_matrix", std::vector<double>(9, 0.0));
    node_->declare_parameter<std::vector<double>>("leftCamera.distort_matrix", std::vector<double>(4, 0.0));
    node_->declare_parameter<double>("leftCamera.T_baselink_camera.x", 0.0);
    node_->declare_parameter<double>("leftCamera.T_baselink_camera.y", 0.0);
    node_->declare_parameter<double>("leftCamera.T_baselink_camera.z", 0.0);
    node_->declare_parameter<double>("leftCamera.T_baselink_camera.yaw", 0.0);
    node_->declare_parameter<double>("leftCamera.T_baselink_camera.pitch", 0.0);
    node_->declare_parameter<double>("leftCamera.T_baselink_camera.roll", 0.0);

    node_->declare_parameter<bool>("backCamera.open", back_cam_.open_);
    node_->declare_parameter<int>("backCamera.cam_type", back_cam_.cam_type_);
    node_->declare_parameter<int>("backCamera.image_type", back_cam_.image_type_);
    node_->declare_parameter<std::string>("backCamera.topic", back_cam_.topic_);
    node_->declare_parameter<std::vector<double>>("backCamera.intrinsic_matrix", std::vector<double>(9, 0.0));
    node_->declare_parameter<std::vector<double>>("backCamera.distort_matrix", std::vector<double>(4, 0.0));
    node_->declare_parameter<double>("backCamera.T_baselink_camera.x", 0.0);
    node_->declare_parameter<double>("backCamera.T_baselink_camera.y", 0.0);
    node_->declare_parameter<double>("backCamera.T_baselink_camera.z", 0.0);
    node_->declare_parameter<double>("backCamera.T_baselink_camera.yaw", 0.0);
    node_->declare_parameter<double>("backCamera.T_baselink_camera.pitch", 0.0);
    node_->declare_parameter<double>("backCamera.T_baselink_camera.roll", 0.0);

    node_->declare_parameter<bool>("rightCamera.open", right_cam_.open_);
    node_->declare_parameter<int>("rightCamera.cam_type", right_cam_.cam_type_);
    node_->declare_parameter<int>("rightCamera.image_type", right_cam_.image_type_);
    node_->declare_parameter<std::string>("rightCamera.topic", right_cam_.topic_);
    node_->declare_parameter<std::vector<double>>("rightCamera.intrinsic_matrix", std::vector<double>(9, 0.0));
    node_->declare_parameter<std::vector<double>>("rightCamera.distort_matrix", std::vector<double>(4, 0.0));
    node_->declare_parameter<double>("rightCamera.T_baselink_camera.x", 0.0);
    node_->declare_parameter<double>("rightCamera.T_baselink_camera.y", 0.0);
    node_->declare_parameter<double>("rightCamera.T_baselink_camera.z", 0.0);
    node_->declare_parameter<double>("rightCamera.T_baselink_camera.yaw", 0.0);
    node_->declare_parameter<double>("rightCamera.T_baselink_camera.pitch", 0.0);
    node_->declare_parameter<double>("rightCamera.T_baselink_camera.roll", 0.0);

    node_->declare_parameter<bool>("BirdParameters.is_save_images", false);
    node_->declare_parameter<bool>("BirdParameters.is_pub_test_images", false);
    node_->declare_parameter<double>("BirdParameters.height", 0.);
    node_->declare_parameter<double>("BirdParameters.width", 0.);
    node_->declare_parameter<double>("BirdParameters.resolution", 0.);
    node_->declare_parameter<double>("BirdParameters.blind_length", 0.);
    node_->declare_parameter<double>("BirdParameters.mask_reference_radio", 0.);
    node_->declare_parameter<double>("BirdParameters.HSV_V", 0.);
    node_->declare_parameter<int>("BirdParameters.fusion_plan", 0);
    node_->declare_parameter<double>("BirdParameters.sigmoid", 0.);



}

// ROS参数定义
void BirdVision::getParameters() {

    node_->get_parameter("V4L2Stream.v4l2_stream_enable", birdVisionParameter_.v4l2_stream_enable_); 
    node_->get_parameter("V4L2Stream.front_cam_dev", birdVisionParameter_.front_cam_dev_); 
    node_->get_parameter("V4L2Stream.left_cam_dev", birdVisionParameter_.left_cam_dev_); 
    node_->get_parameter("V4L2Stream.back_cam_dev", birdVisionParameter_.back_cam_dev_); 
    node_->get_parameter("V4L2Stream.right_cam_dev", birdVisionParameter_.right_cam_dev_);
    node_->get_parameter("V4L2Stream.stream_format", birdVisionParameter_.stream_format_);
    node_->get_parameter("V4L2Stream.stream_width", birdVisionParameter_.stream_width_);
    node_->get_parameter("V4L2Stream.stream_height", birdVisionParameter_.stream_height_);
    node_->get_parameter("V4L2Stream.stream_fps", birdVisionParameter_.stream_fps_);


    bool open;
    int cam_type;
    int image_type;
    std::string topic;
    std::vector<double> intrinsic, distort;
    double x, y, z, yaw, pitch, roll;

    // ================================================================ 设置Front参数 ====================================================
    node_->get_parameter("frontCamera.open", open);
    node_->get_parameter("frontCamera.cam_type", cam_type);
    node_->get_parameter("frontCamera.image_type", image_type);
    node_->get_parameter("frontCamera.topic", topic);
    node_->get_parameter("frontCamera.intrinsic_matrix", intrinsic);
    node_->get_parameter("frontCamera.distort_matrix", distort);
    node_->get_parameter("frontCamera.T_baselink_camera.x", x);
    node_->get_parameter("frontCamera.T_baselink_camera.y", y);
    node_->get_parameter("frontCamera.T_baselink_camera.z", z);
    node_->get_parameter("frontCamera.T_baselink_camera.yaw", yaw);
    node_->get_parameter("frontCamera.T_baselink_camera.pitch", pitch);
    node_->get_parameter("frontCamera.T_baselink_camera.roll", roll);
    
    front_cam_.open_ = open;
    front_cam_.cam_type_ = cam_type;
    front_cam_.image_type_ = image_type;
    front_cam_.topic_ = topic;
    front_cam_.intrinsic_matrix_ << intrinsic[0], intrinsic[1], intrinsic[2],
            intrinsic[3], intrinsic[4], intrinsic[5],
            intrinsic[6], intrinsic[7], intrinsic[8];
    if (front_cam_.cam_type_ == 0) {
        front_cam_.distort_matrix_.resize(1, 5);
        front_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3], distort[4];
    } else if (front_cam_.cam_type_ == 1) {
        front_cam_.distort_matrix_.resize(1, 4);
        front_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3];
    } else {
        std::cerr << "前视相机cam_type_参数不匹配(0:针孔 1:鱼眼)" << std::endl;
    }
    
    // 组合旋转矩阵，顺序是 R = Rz * Ry * Rx
    front_cam_.yaw_ = yaw;
    front_cam_.pitch_ = pitch;
    front_cam_.roll_ = roll;
    front_cam_.R_baselink_camera_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    front_cam_.t_baselink_camera_ = Eigen::Vector3d(x, y, z);

    

    // ================================================================ 设置Left参数 ====================================================
    node_->get_parameter("leftCamera.open", open);
    node_->get_parameter("leftCamera.cam_type", cam_type);
    node_->get_parameter("leftCamera.image_type", image_type);
    node_->get_parameter("leftCamera.topic", topic);
    node_->get_parameter("leftCamera.intrinsic_matrix", intrinsic);
    node_->get_parameter("leftCamera.distort_matrix", distort);
    node_->get_parameter("leftCamera.T_baselink_camera.x", x);
    node_->get_parameter("leftCamera.T_baselink_camera.y", y);
    node_->get_parameter("leftCamera.T_baselink_camera.z", z);
    node_->get_parameter("leftCamera.T_baselink_camera.yaw", yaw);
    node_->get_parameter("leftCamera.T_baselink_camera.pitch", pitch);
    node_->get_parameter("leftCamera.T_baselink_camera.roll", roll);

    left_cam_.open_ = open;
    left_cam_.cam_type_ = cam_type;
    left_cam_.image_type_ = image_type;
    left_cam_.topic_ = topic;
    left_cam_.intrinsic_matrix_ << intrinsic[0], intrinsic[1], intrinsic[2],
            intrinsic[3], intrinsic[4], intrinsic[5],
            intrinsic[6], intrinsic[7], intrinsic[8];
    if (left_cam_.cam_type_ == 0) {
        left_cam_.distort_matrix_.resize(1, 5);
        left_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3], distort[4];
    } else if (left_cam_.cam_type_ == 1) {
        left_cam_.distort_matrix_.resize(1, 4);
        left_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3];
    } else {
        std::cerr << "左视相机cam_type_参数不匹配(0:针孔 1:鱼眼)" << std::endl;
    }
    // 组合旋转矩阵，顺序是 R = Rz * Ry * Rx
    left_cam_.yaw_ = yaw;
    left_cam_.pitch_ = pitch;
    left_cam_.roll_ = roll;
    left_cam_.R_baselink_camera_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    left_cam_.t_baselink_camera_ = Eigen::Vector3d(x, y, z);

    // ================================================================ 设置Back参数 ====================================================
    node_->get_parameter("backCamera.open", open);
    node_->get_parameter("backCamera.cam_type", cam_type);
    node_->get_parameter("backCamera.image_type", image_type);
    node_->get_parameter("backCamera.topic", topic);
    node_->get_parameter("backCamera.intrinsic_matrix", intrinsic);
    node_->get_parameter("backCamera.distort_matrix", distort);
    node_->get_parameter("backCamera.T_baselink_camera.x", x);
    node_->get_parameter("backCamera.T_baselink_camera.y", y);
    node_->get_parameter("backCamera.T_baselink_camera.z", z);
    node_->get_parameter("backCamera.T_baselink_camera.yaw", yaw);
    node_->get_parameter("backCamera.T_baselink_camera.pitch", pitch);
    node_->get_parameter("backCamera.T_baselink_camera.roll", roll);

    back_cam_.open_ = open;
    back_cam_.cam_type_ = cam_type;
    back_cam_.image_type_ = image_type;
    back_cam_.topic_ = topic;
    back_cam_.intrinsic_matrix_ << intrinsic[0], intrinsic[1], intrinsic[2],
            intrinsic[3], intrinsic[4], intrinsic[5],
            intrinsic[6], intrinsic[7], intrinsic[8];
    if (back_cam_.cam_type_ == 0) {
        back_cam_.distort_matrix_.resize(1, 5);
        back_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3], distort[4];
    } else if (back_cam_.cam_type_ == 1) {
        back_cam_.distort_matrix_.resize(1, 4);
        back_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3];
    } else {
        std::cerr << "后视相机cam_type_参数不匹配(0:针孔 1:鱼眼)" << std::endl;
    }
    // 组合旋转矩阵，顺序是 R = Rz * Ry * Rx
    back_cam_.yaw_ = yaw;
    back_cam_.pitch_ = pitch;
    back_cam_.roll_ = roll;
    back_cam_.R_baselink_camera_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    back_cam_.t_baselink_camera_ = Eigen::Vector3d(x, y, z);

    // ================================================================ 设置Right参数 ====================================================
    node_->get_parameter("rightCamera.open", open);
    node_->get_parameter("rightCamera.cam_type", cam_type);
    node_->get_parameter("rightCamera.image_type", image_type);
    node_->get_parameter("rightCamera.topic", topic);
    node_->get_parameter("rightCamera.intrinsic_matrix", intrinsic);
    node_->get_parameter("rightCamera.distort_matrix", distort);
    node_->get_parameter("rightCamera.T_baselink_camera.x", x);
    node_->get_parameter("rightCamera.T_baselink_camera.y", y);
    node_->get_parameter("rightCamera.T_baselink_camera.z", z);
    node_->get_parameter("rightCamera.T_baselink_camera.yaw", yaw);
    node_->get_parameter("rightCamera.T_baselink_camera.pitch", pitch);
    node_->get_parameter("rightCamera.T_baselink_camera.roll", roll);

    right_cam_.open_ = open;
    right_cam_.cam_type_ = cam_type;
    right_cam_.image_type_ = image_type;
    right_cam_.topic_ = topic;
    right_cam_.intrinsic_matrix_ << intrinsic[0], intrinsic[1], intrinsic[2],
            intrinsic[3], intrinsic[4], intrinsic[5],
            intrinsic[6], intrinsic[7], intrinsic[8];
    if (right_cam_.cam_type_ == 0) {
        right_cam_.distort_matrix_.resize(1, 5);
        right_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3], distort[4];
    } else if (right_cam_.cam_type_ == 1) {
        right_cam_.distort_matrix_.resize(1, 4);
        right_cam_.distort_matrix_ << distort[0], distort[1], distort[2], distort[3];
    } else {
        std::cerr << "右视相机cam_type_参数不匹配(0:针孔 1:鱼眼)" << std::endl;
    }
    // 组合旋转矩阵，顺序是 R = Rz * Ry * Rx
    right_cam_.yaw_ = yaw;
    right_cam_.pitch_ = pitch;
    right_cam_.roll_ = roll;
    right_cam_.R_baselink_camera_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    right_cam_.t_baselink_camera_ = Eigen::Vector3d(x, y, z);

    // ================================================================ 获取鸟瞰图的参数 ====================================================
    node_->get_parameter("BirdParameters.is_save_images", birdVisionParameter_.is_save_images_);
    node_->get_parameter("BirdParameters.is_pub_test_images", birdVisionParameter_.is_pub_test_images_);
    node_->get_parameter("BirdParameters.height", birdVisionParameter_.height_);
    node_->get_parameter("BirdParameters.width", birdVisionParameter_.width_);
    node_->get_parameter("BirdParameters.resolution", birdVisionParameter_.resolution_);
    node_->get_parameter("BirdParameters.blind_length", birdVisionParameter_.blind_length_);
    node_->get_parameter("BirdParameters.mask_reference_radio", birdVisionParameter_.mask_reference_radio_);
    node_->get_parameter("BirdParameters.HSV_V", birdVisionParameter_.HSV_V_);
    birdVisionParameter_.assets_reference_path_ = std::string(ROOT_DIR) + "assets";
    node_->get_parameter("BirdParameters.fusion_plan", birdVisionParameter_.fusion_plan_);
    node_->get_parameter("BirdParameters.sigmoid", birdVisionParameter_.sigmoid_);
}

// ROS的参数打印
void BirdVision::logParameters() {

    std::cout << "V4L2Stream Parameters: " << std::endl;
    std::cout << "  v4l2_stream_enable: " << (birdVisionParameter_.v4l2_stream_enable_ ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  front_cam_dev: " << birdVisionParameter_.front_cam_dev_ << std::endl;
    std::cout << "  left_cam_dev: " << birdVisionParameter_.left_cam_dev_ << std::endl;
    std::cout << "  back_cam_dev: " << birdVisionParameter_.back_cam_dev_ << std::endl;
    std::cout << "  right_cam_dev: " << birdVisionParameter_.right_cam_dev_ << std::endl;

    std::cout << "  stream_format: " << birdVisionParameter_.stream_format_ << std::endl;
    std::cout << "  stream_width: " << birdVisionParameter_.stream_width_ << std::endl;
    std::cout << "  stream_height: " << birdVisionParameter_.stream_height_ << std::endl;
    std::cout << "  stream_fps: " << birdVisionParameter_.stream_fps_ << std::endl;



    std::cout << "frontCamera.open" << std::endl << (front_cam_.open_ ? "True" : "False") << std::endl;
    std::cout << "frontCamera.cam_type" << std::endl << (front_cam_.cam_type_ == 0 ? "针孔相机" : "鱼眼相机") << std::endl;
    std::cout << "frontCamera.image_type" << std::endl << (front_cam_.image_type_ == 0 ? "ROS图像格式" : "Image4M") << std::endl;
    std::cout << "frontCamera.topic" << std::endl << front_cam_.topic_ << std::endl;
    std::cout << "frontCamera.intrinsic_matrix: " << std::endl << front_cam_.intrinsic_matrix_ << std::endl;
    std::cout << "frontCamera.distort_matrix: " << std::endl << front_cam_.distort_matrix_ << std::endl;
    std::cout << "frontCamera.R_baselink_camera: " << std::endl << front_cam_.R_baselink_camera_.matrix() << std::endl;
    std::cout << "frontCamera.t_baselink_camera: " << std::endl << front_cam_.t_baselink_camera_.transpose()
              << std::endl;
    std::cout << std::endl;

    std::cout << "leftCamera.open" << std::endl << (left_cam_.open_ ? "True" : "False") << std::endl;
    std::cout << "leftCamera.cam_type" << std::endl << (left_cam_.cam_type_ == 0 ? "针孔相机" : "鱼眼相机")
              << std::endl;
    std::cout << "leftCamera.image_type" << std::endl << (left_cam_.image_type_ == 0 ? "ROS图像格式" : "Image4M")
              << std::endl;
    std::cout << "leftCamera.topic" << std::endl << left_cam_.topic_ << std::endl;
    std::cout << "leftCamera.intrinsic_matrix: " << std::endl << left_cam_.intrinsic_matrix_ << std::endl;
    std::cout << "leftCamera.distort_matrix: " << std::endl << left_cam_.distort_matrix_ << std::endl;
    std::cout << "leftCamera.R_baselink_camera: " << std::endl << left_cam_.R_baselink_camera_.matrix() << std::endl;
    std::cout << "leftCamera.t_baselink_camera: " << std::endl << left_cam_.t_baselink_camera_.transpose() << std::endl;
    std::cout << std::endl;

    std::cout << "backCamera.open" << std::endl << (back_cam_.open_ ? "True" : "False") << std::endl;
    std::cout << "backCamera.cam_type" << std::endl << (back_cam_.cam_type_ == 0 ? "针孔相机" : "鱼眼相机")
              << std::endl;
    std::cout << "backCamera.image_type" << std::endl << (back_cam_.image_type_ == 0 ? "ROS图像格式" : "Image4M")
              << std::endl;
    std::cout << "backCamera.topic" << std::endl << back_cam_.topic_ << std::endl;
    std::cout << "backCamera.intrinsic_matrix: " << std::endl << back_cam_.intrinsic_matrix_ << std::endl;
    std::cout << "backCamera.distort_matrix: " << std::endl << back_cam_.distort_matrix_ << std::endl;
    std::cout << "backCamera.R_baselink_camera: " << std::endl << back_cam_.R_baselink_camera_.matrix() << std::endl;
    std::cout << "backCamera.t_baselink_camera: " << std::endl << back_cam_.t_baselink_camera_.transpose() << std::endl;
    std::cout << std::endl;

    std::cout << "rightCamera.open" << std::endl << (right_cam_.open_ ? "True" : "False") << std::endl;
    std::cout << "rightCamera.cam_type" << std::endl << (right_cam_.cam_type_ == 0 ? "针孔相机" : "鱼眼相机")
              << std::endl;
    std::cout << "rightCamera.image_type" << std::endl << (right_cam_.image_type_ == 0 ? "ROS图像格式" : "Image4M")
              << std::endl;
    std::cout << "rightCamera.topic" << std::endl << right_cam_.topic_ << std::endl;
    std::cout << "rightCamera.intrinsic_matrix: " << std::endl << right_cam_.intrinsic_matrix_ << std::endl;
    std::cout << "rightCamera.distort_matrix: " << std::endl << right_cam_.distort_matrix_ << std::endl;
    std::cout << "rightCamera.R_baselink_camera: " << std::endl << right_cam_.R_baselink_camera_.matrix() << std::endl;
    std::cout << "rightCamera.t_baselink_camera: " << std::endl << right_cam_.t_baselink_camera_.transpose()
              << std::endl;
    std::cout << std::endl;

    std::cout << "birdVision.is_save_image: " << (birdVisionParameter_.is_save_images_ ? "True" : "False") << std::endl;
    std::cout << "birdVision.is_pub_test_images: " << (birdVisionParameter_.is_pub_test_images_ ? "True" : "False") << std::endl;
    std::cout << "birdVision.width_: " << birdVisionParameter_.width_ << std::endl;
    std::cout << "birdVision.height_: " << birdVisionParameter_.height_ << std::endl;
    std::cout << "birdVision.resolution_: " << birdVisionParameter_.resolution_ << std::endl;
    std::cout << "birdVision.blind_length_: " << birdVisionParameter_.blind_length_ << std::endl;
    std::cout << "birdVision.mask_reference_radio_: " << birdVisionParameter_.mask_reference_radio_ << std::endl;
    std::cout << "birdVision.HSV_V_: " << birdVisionParameter_.HSV_V_ << std::endl;
    std::cout << "assets reference path: " << birdVisionParameter_.assets_reference_path_ << std::endl;
    std::cout << "fusion_plan: " << birdVisionParameter_.fusion_plan_ << std::endl;
    std::cout << "sigmoid: " << birdVisionParameter_.sigmoid_ << std::endl;
    std::cout << std::endl;
}

// 动态传参——修改外参
bool BirdVision::parameterCallback(const vector<rclcpp::Parameter> &parameters) {

    // 尝试获取锁
    std::unique_lock<std::mutex> lock(param_mtx_, std::defer_lock);
    while (!lock.try_lock()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 等待 10 毫秒后重试
    }

    // 外参参数更新
    for (const auto &param: parameters) {
        if (param.get_name() == "frontCamera.T_baselink_camera.x") {
            std::cout << "front_cam(x): " << param.as_double() << std::endl;
            front_cam_.t_baselink_camera_.x() = param.as_double();
        } else if (param.get_name() == "frontCamera.T_baselink_camera.y") {
            std::cout << "front_cam(y): " << param.as_double() << std::endl;
            front_cam_.t_baselink_camera_.y() = param.as_double();
        } else if (param.get_name() == "frontCamera.T_baselink_camera.z") {
            std::cout << "front_cam(z): " << param.as_double() << std::endl;
            front_cam_.t_baselink_camera_.z() = param.as_double();
        } else if (param.get_name() == "frontCamera.T_baselink_camera.yaw") {
            std::cout << "front_cam(yaw): " << param.as_double() << std::endl;
            front_cam_.yaw_ = param.as_double();
        } else if (param.get_name() == "frontCamera.T_baselink_camera.pitch") {
            std::cout << "front_cam(pitch): " << param.as_double() << std::endl;
            front_cam_.pitch_ = param.as_double();
        } else if (param.get_name() == "frontCamera.T_baselink_camera.roll") {
            std::cout << "front_cam(roll): " << param.as_double() << std::endl;
            front_cam_.roll_ = param.as_double();
        }

        else if (param.get_name() == "leftCamera.T_baselink_camera.x") {
            std::cout << "left_cam(x): " << param.as_double() << std::endl;
            left_cam_.t_baselink_camera_.x() = param.as_double();
        } else if (param.get_name() == "leftCamera.T_baselink_camera.y") {
            std::cout << "left_cam(y): " << param.as_double() << std::endl;
            left_cam_.t_baselink_camera_.y() = param.as_double();
        } else if (param.get_name() == "leftCamera.T_baselink_camera.z") {
            std::cout << "left_cam(z): " << param.as_double() << std::endl;
            left_cam_.t_baselink_camera_.z() = param.as_double();
        } else if (param.get_name() == "leftCamera.T_baselink_camera.yaw") {
            std::cout << "left_cam(yaw): " << param.as_double() << std::endl;
            left_cam_.yaw_ = param.as_double();
        } else if (param.get_name() == "leftCamera.T_baselink_camera.pitch") {
            std::cout << "left_cam(pitch): " << param.as_double() << std::endl;
            left_cam_.pitch_ = param.as_double();
        } else if (param.get_name() == "leftCamera.T_baselink_camera.roll") {
            std::cout << "left_cam(roll): " << param.as_double() << std::endl;
            left_cam_.roll_ = param.as_double();
        }


        else if (param.get_name() == "backCamera.T_baselink_camera.x") {
            std::cout << "back_cam(x): " << param.as_double() << std::endl;
            back_cam_.t_baselink_camera_.x() = param.as_double();
        } else if (param.get_name() == "backCamera.T_baselink_camera.y") {
            std::cout << "back_cam(y): " << param.as_double() << std::endl;
            back_cam_.t_baselink_camera_.y() = param.as_double();
        } else if (param.get_name() == "backCamera.T_baselink_camera.z") {
            std::cout << "back_cam(z): " << param.as_double() << std::endl;
            back_cam_.t_baselink_camera_.z() = param.as_double();
        } else if (param.get_name() == "backCamera.T_baselink_camera.yaw") {
            std::cout << "back_cam(yaw): " << param.as_double() << std::endl;
            back_cam_.yaw_ = param.as_double();
        } else if (param.get_name() == "backCamera.T_baselink_camera.pitch") {
            std::cout << "back_cam(pitch): " << param.as_double() << std::endl;
            back_cam_.pitch_ = param.as_double();
        } else if (param.get_name() == "backCamera.T_baselink_camera.roll") {
            std::cout << "back_cam(roll): " << param.as_double() << std::endl;
            back_cam_.roll_ = param.as_double();
        }

        else if (param.get_name() == "rightCamera.T_baselink_camera.x") {
            std::cout << "right_cam(x): " << param.as_double() << std::endl;
            right_cam_.t_baselink_camera_.x() = param.as_double();
        } else if (param.get_name() == "rightCamera.T_baselink_camera.y") {
            std::cout << "right_cam(y): " << param.as_double() << std::endl;
            right_cam_.t_baselink_camera_.y() = param.as_double();
        } else if (param.get_name() == "rightCamera.T_baselink_camera.z") {
            std::cout << "right_cam(z): " << param.as_double() << std::endl;
            right_cam_.t_baselink_camera_.z() = param.as_double();
        } else if (param.get_name() == "rightCamera.T_baselink_camera.yaw") {
            std::cout << "right_cam(yaw): " << param.as_double() << std::endl;
            right_cam_.yaw_ = param.as_double();
        } else if (param.get_name() == "rightCamera.T_baselink_camera.pitch") {
            std::cout << "right_cam(pitch): " << param.as_double() << std::endl;
            right_cam_.pitch_ = param.as_double();
        } else if (param.get_name() == "rightCamera.T_baselink_camera.roll") {
            std::cout << "right_cam(roll): " << param.as_double() << std::endl;
            right_cam_.roll_ = param.as_double();
        }
    }

    front_cam_.R_baselink_camera_ = Eigen::AngleAxisd(front_cam_.yaw_, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(front_cam_.pitch_, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(front_cam_.roll_, Eigen::Vector3d::UnitX());

    left_cam_.R_baselink_camera_ = Eigen::AngleAxisd(left_cam_.yaw_, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(left_cam_.pitch_, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(left_cam_.roll_, Eigen::Vector3d::UnitX());

    back_cam_.R_baselink_camera_ = Eigen::AngleAxisd(back_cam_.yaw_, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(back_cam_.pitch_, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(back_cam_.roll_, Eigen::Vector3d::UnitX());

    right_cam_.R_baselink_camera_ = Eigen::AngleAxisd(right_cam_.yaw_, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(right_cam_.pitch_, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(right_cam_.roll_, Eigen::Vector3d::UnitX());

    system_init_flag = false; // 重置映射表标志位

    return true;
}

// 初始化认为的有效区域，减小鸟瞰图遍历的次数
void BirdVision::initEffectArea(int width, int height, int offset_pixels) {

    // =========================================== 生成鸟瞰子图mask ======================================
    // 生成front鸟瞰图的有效掩码
    front_effect_areas_ = cv::Mat(cv::Size(width, height / 2.0), CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Point front_vertices[7];
    front_vertices[0] = cv::Point(0, height / birdVisionParameter_.mask_reference_radio_); // 8
    front_vertices[1] = cv::Point(0, height / 2);
    front_vertices[2] = cv::Point(width, height / 2);
    front_vertices[3] = cv::Point(width, height / birdVisionParameter_.mask_reference_radio_); // 8
    front_vertices[4] = cv::Point(width / 2 + offset_pixels, height / 2 - offset_pixels);
    front_vertices[5] = cv::Point(width / 2 - offset_pixels, height / 2 - offset_pixels);
    front_vertices[6] = cv::Point(0, height / birdVisionParameter_.mask_reference_radio_); // 8

    const cv::Point *front_ppt[1] = {front_vertices};
    int front_npt[] = {sizeof(front_vertices) / sizeof(cv::Point)};
    fillPoly(front_effect_areas_, front_ppt, front_npt, 1, cv::Scalar(0, 0, 0));

    // 生成左视觉鸟瞰图有效区域掩码
    cv::rotate(front_effect_areas_, left_effect_areas_, cv::ROTATE_90_COUNTERCLOCKWISE);

    // 生成后视觉鸟瞰图有效区域掩码
    cv::rotate(front_effect_areas_, back_effect_areas_, cv::ROTATE_180);

    // 生成右视觉鸟瞰图有效区域掩码
    cv::rotate(front_effect_areas_, right_effect_areas_, cv::ROTATE_90_CLOCKWISE);

    // 保存的路径(测试用)
    if (birdVisionParameter_.is_save_images_) {
        std::string effect_areas_path = birdVisionParameter_.assets_reference_path_ + "/effect_areas/";
        std::cout << "EffectArea保存的路径: " << effect_areas_path << "*" << std::endl;
        cv::imwrite(effect_areas_path + "front_effect_areas_.jpg", front_effect_areas_);
        cv::imwrite(effect_areas_path + "left_effect_areas_.jpg", left_effect_areas_);
        cv::imwrite(effect_areas_path + "back_effect_areas_.jpg", back_effect_areas_);
        cv::imwrite(effect_areas_path + "right_effect_areas_.jpg", right_effect_areas_);
    }
}

// 在整个鸟瞰图上进行重合区域提取
void BirdVision::extrectOverLay(const cv::Mat &frontImage, const cv::Mat &leftImage,
                                const cv::Mat &backImage, const cv::Mat &rightImage) {
    // 图像二值化并基于轮廓去除轮廓内的噪点
    cv::Mat front_mask, left_mask, back_mask, right_mask;
    front_mask = BinaryAndNoiseRemoval(frontImage);
    left_mask = BinaryAndNoiseRemoval(leftImage);
    back_mask = BinaryAndNoiseRemoval(backImage);
    right_mask = BinaryAndNoiseRemoval(rightImage);

    cv::Mat front_inverted_image, left_inverted_image, back_inverted_image, right_inverted_image;
    cv::bitwise_not(front_mask, front_inverted_image);
    cv::bitwise_not(left_mask, left_inverted_image);
    cv::bitwise_not(back_mask, back_inverted_image);
    cv::bitwise_not(right_mask, right_inverted_image);

    // 提取重叠区域
    front_mid_ = front_mask.clone(); // 复制原始图像以防止修改原始图像

    cv::Rect roi_rect;
    roi_rect = cv::Rect(0, 0, front_mid_.size().height, front_mid_.size().height);
    left_inverted_image(cv::Rect(0, 0, left_inverted_image.size().width, left_inverted_image.size().width)).copyTo(
            front_mid_(roi_rect));

    roi_rect = cv::Rect(front_mid_.size().height, 0, front_mid_.size().height, front_mid_.size().height);
    right_inverted_image(cv::Rect(0, 0, right_inverted_image.size().width, right_inverted_image.size().width)).copyTo(
            front_mid_(roi_rect));
    cv::bitwise_and(front_mid_, front_mask, front_mid_);

    // 融合平衡的权重（空白区域到中心）
    cv::Mat front_mid_inverted_image;
    cv::bitwise_not(front_mid_, front_mid_inverted_image);

    cv::Mat distanceTransform_32F;
    cv::distanceTransform(front_mid_inverted_image, distanceTransform_32F, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::Mat distanceTransform_8U;
    distanceTransform_32F.convertTo(distanceTransform_8U, CV_8U);
    cv::Mat front_temp;
    cv::bitwise_and(front_mid_inverted_image, front_mask, front_temp);

    int pixel_max = 0, pixel_min = 255;
    for (int y = 0; y < front_temp.rows; ++y) {
        for (int x = 0; x < front_temp.cols; ++x) {

            if (front_temp.at<uchar>(y, x) == 0) {
                continue;
            }

            int pixel_value = static_cast<int>(distanceTransform_8U.at<uchar>(y, x));
            if (pixel_value > pixel_max) {
                pixel_max = pixel_value;
            }

            if (pixel_value < pixel_min) {
                pixel_min = pixel_value;
            }
        }
    }

    for (int y = 0; y < front_temp.rows; ++y) {
        for (int x = 0; x < front_temp.cols; ++x) {
            if (front_temp.at<uchar>(y, x) == 0) {
                continue;
            }

            int cur_pixel_value = pixel_max - int(distanceTransform_8U.at<uchar>(y, x)) + 1;
            int cur_pixel_norm = int((cur_pixel_value - pixel_min) * 255.0 / (pixel_max - pixel_min));
            front_temp.at<uchar>(y, x) = uchar(cur_pixel_norm);
        }
    }

    front_overlay_ = front_temp;

    // 生成 左视鸟瞰图 后视鸟瞰图 右视鸟瞰图
    cv::rotate(front_mid_, left_mid_, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(front_mid_, back_mid_, cv::ROTATE_180);
    cv::rotate(front_mid_, right_mid_, cv::ROTATE_90_CLOCKWISE);

    cv::rotate(front_overlay_, left_overlay_, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(front_overlay_, back_overlay_, cv::ROTATE_180);
    cv::rotate(front_overlay_, right_overlay_, cv::ROTATE_90_CLOCKWISE);

    if (birdVisionParameter_.is_save_images_) {
        std::string overlay_path = birdVisionParameter_.assets_reference_path_ + "/overlay/";
        std::cout << "overlay保存路径: " << overlay_path << "*" << std::endl;

        cv::imwrite(overlay_path + "front_mid_.png", front_mid_);
        cv::imwrite(overlay_path + "left_mid_.png", left_mid_);
        cv::imwrite(overlay_path + "back_mid_.png", back_mid_);
        cv::imwrite(overlay_path + "right_mid_.png", right_mid_);

        cv::imwrite(overlay_path + "front_overlay_.png", front_overlay_);
        cv::imwrite(overlay_path + "left_overlay_.png", left_overlay_);
        cv::imwrite(overlay_path + "back_overlay_.png", back_overlay_);
        cv::imwrite(overlay_path + "right_overlay_.png", right_overlay_);
    }
}


// 鸟瞰回调处理
void BirdVision::birdViewHandleThread() {

    std::cout << "BirdVision HandleThread Starting..." << std::endl;

    while (rclcpp::ok()){
        // 休眠
        std::this_thread::sleep_for(std::chrono::milliseconds(15));

        TicToc time_consume;

        // 获取数据
        static double last_front_timestamp = -1.0, last_left_timestamp = -1.0, last_back_timestamp = -1.0, last_right_timestamp= -1.0;
        cv::Mat front_origin_image, left_origin_image, back_origin_image, right_origin_image;
        double front_timestamp, left_timestamp, back_timestamp, right_timestamp;

        if(birdVisionParameter_.v4l2_stream_enable_){

            // 检查v4l2取流对象是否初始化成功
            if (v4l2StreamUtil_vec_.size() != 4 ||
                v4l2StreamUtil_vec_.at(0) == nullptr ||
                v4l2StreamUtil_vec_.at(1) == nullptr ||
                v4l2StreamUtil_vec_.at(2) == nullptr ||
                v4l2StreamUtil_vec_.at(3) == nullptr) {
                std::cout << " Error: v4l2取流工具4路对象未初始化成功..." << std::endl;
                continue;
            }

            // 获取数据进行显示
            std::shared_ptr<ImageWithStamp> front_image_stamp_ptr = nullptr, left_image_stamp_ptr = nullptr, back_image_stamp_ptr = nullptr, right_image_stamp_ptr = nullptr;
            v4l2StreamUtil_vec_.at(0)->getImageWithStamp(front_image_stamp_ptr);
            v4l2StreamUtil_vec_.at(1)->getImageWithStamp(left_image_stamp_ptr);
            v4l2StreamUtil_vec_.at(2)->getImageWithStamp(back_image_stamp_ptr);
            v4l2StreamUtil_vec_.at(3)->getImageWithStamp(right_image_stamp_ptr);

            // 检查数据是否获取成功
            if(front_image_stamp_ptr == nullptr || left_image_stamp_ptr == nullptr || back_image_stamp_ptr == nullptr || right_image_stamp_ptr == nullptr){
                std::cout << " Error: v4l2四路数据未获取成功..." << std::endl;
                continue;
            }

            front_origin_image = front_image_stamp_ptr->image_;
            left_origin_image = left_image_stamp_ptr->image_;
            back_origin_image = back_image_stamp_ptr->image_;
            right_origin_image = right_image_stamp_ptr->image_;

        }
        else{
            // 检查数据是否有效
            if (front_image_ptr_ == nullptr || left_image_ptr_ == nullptr || back_image_ptr_ == nullptr || right_image_ptr_ == nullptr) {
                continue;
            }


            {
                std::lock_guard<std::mutex> lock(front_mtx_);
                front_origin_image = front_image_ptr_->image_;
                front_timestamp = front_image_ptr_->timestamp_;
            }

            {
                std::lock_guard<std::mutex> lock(left_mtx_);
                left_origin_image = left_image_ptr_->image_;
                left_timestamp = left_image_ptr_->timestamp_;
            }

            {
                std::lock_guard<std::mutex> lock(back_mtx_);
                back_origin_image = back_image_ptr_->image_;
                back_timestamp = back_image_ptr_->timestamp_;
            }

            {
                std::lock_guard<std::mutex> lock(right_mtx_);
                right_origin_image = right_image_ptr_->image_;
                right_timestamp = right_image_ptr_->timestamp_;
            }

            // 数据同步检查
            auto delta_time = std::max({front_image_ptr_->timestamp_, left_image_ptr_->timestamp_, back_image_ptr_->timestamp_, right_image_ptr_->timestamp_}) -
                              std::min({front_image_ptr_->timestamp_, left_image_ptr_->timestamp_, back_image_ptr_->timestamp_, right_image_ptr_->timestamp_}) * 1000.0;
            if(delta_time >= 33.0){
                std::cout << std::fixed << std::setprecision(9) << "data not sync, max delta duration: " << delta_time << "ms." <<  std::endl;
            }

            // 时间戳更新
            last_front_timestamp = front_image_ptr_->timestamp_;
            last_left_timestamp = left_image_ptr_->timestamp_;
            last_back_timestamp = back_image_ptr_->timestamp_;
            last_right_timestamp = right_image_ptr_->timestamp_;

        }

        std::cout << front_origin_image.size() << std::endl;



        // effect_areas 为有效区域作为鸟瞰图填充的目标
        front_bird_vision_image_ = front_effect_areas_.clone();
        left_bird_vision_image_ = left_effect_areas_.clone();
        back_bird_vision_image_ = back_effect_areas_.clone();
        right_bird_vision_image_ = right_effect_areas_.clone();

        {
            std::lock_guard<std::mutex> lock(param_mtx_);

            std::promise<void> promise;
            std::future<void> future = promise.get_future();

            auto front_flag = thread_pool_ptr_->
                    add([this, front_origin_image](const std::string &camera_index) -> bool {
                birdImageHandle(front_origin_image,
                                camera_index,
                                front_cam_,
                                front_remap_,
                                front_bird_vision_image_);
                return true;
            }, "front");


            auto left_flag = thread_pool_ptr_->add([this, left_origin_image](const std::string &camera_index) -> bool {
                birdImageHandle(left_origin_image,
                                camera_index,
                                left_cam_,
                                left_remap_,
                                left_bird_vision_image_);
                return true;
            }, "left");


            auto back_flag = thread_pool_ptr_->add([this, back_origin_image](const std::string &camera_index) -> bool {
                birdImageHandle(back_origin_image,
                                camera_index,
                                back_cam_,
                                back_remap_,
                                back_bird_vision_image_);
                return true;
            }, "back");

            auto right_flag = thread_pool_ptr_->add([this, right_origin_image](const std::string &camera_index) -> bool {
                birdImageHandle(right_origin_image,
                                camera_index,
                                right_cam_,
                                right_remap_,
                                right_bird_vision_image_);
                return true;
            }, "right");


            // 等待四个任务完成
            front_flag.wait();
            left_flag.wait();
            back_flag.wait();
            right_flag.wait();

        }



        // 系统初始化
        if (!system_init_flag) {
            system_init_flag = true;
            HSVBrightBalance(front_bird_vision_image_, left_bird_vision_image_, back_bird_vision_image_, right_bird_vision_image_);
            std::cout << "鸟瞰图初始化成功......" << std::endl;
            continue;
        }

        // 发布四个鸟瞰子图
        if(birdVisionParameter_.is_pub_test_images_){
            pub_front_bird_vision_ptr_->publish(rosFromMat(front_bird_vision_image_, getRosTime()));
            pub_left_bird_vision_ptr_->publish(rosFromMat(left_bird_vision_image_, getRosTime()));
            pub_back_bird_vision_ptr_->publish(rosFromMat(back_bird_vision_image_, getRosTime()));
            pub_right_bird_vision_ptr_->publish(rosFromMat(right_bird_vision_image_, getRosTime()));
        }

        // 利用四个子图拼接成全景鸟瞰图
        if(birdVisionParameter_.fusion_plan_ == 0){
            // 方案一： 按照预设顺序进行填充并且对重合区域进行平滑
            all_bird_image = cv::Mat::zeros(int(birdVisionParameter_.height_), int(birdVisionParameter_.width_), CV_8UC3);

            front_bird_vision_image_.copyTo(all_bird_image(cv::Rect(0, 0, front_bird_vision_image_.cols, front_bird_vision_image_.rows)), front_mid_);
            left_bird_vision_image_.copyTo(all_bird_image(cv::Rect(0, 0, left_bird_vision_image_.cols, left_bird_vision_image_.rows)), left_mid_);
            back_bird_vision_image_.copyTo(all_bird_image(cv::Rect(0, int(birdVisionParameter_.height_ / 2.0), back_bird_vision_image_.cols, back_bird_vision_image_.rows)), back_mid_);
            right_bird_vision_image_.copyTo(all_bird_image(cv::Rect(int(birdVisionParameter_.width_ / 2.0), 0, right_bird_vision_image_.cols, right_bird_vision_image_.rows)), right_mid_);

            // 对四个重叠区域进行光滑处理
            // 左上角区域光滑
            for (int row = 0; row < front_overlay_.size().height; row++)
            {
                for (int col = 0; col < front_overlay_.size().width / 2; col++)
                {

                    if (front_overlay_.at<uchar>(row, col) == 0)
                    {
                        continue;
                    }

                    cv::Vec3b front_pixel = front_bird_vision_image_.at<cv::Vec3b>(row, col);
                    cv::Vec3b left_pixel = left_bird_vision_image_.at<cv::Vec3b>(row, col);

//                // 方案一：获取两个距离值分别计算加权的权重
//                auto front_pixle_weight = float(front_overlay_.at<uchar>(row, col));
//                auto left_pixle_weight = float(left_overlay_.at<uchar>(row, col));
//                auto sum_pixel_weight = front_pixle_weight * front_pixle_weight + left_pixle_weight * left_pixle_weight;
//                float front_weight = front_pixle_weight * front_pixle_weight / sum_pixel_weight;
//                float left_weight = 1 - front_weight;

                    // 方案二：利用sigmoid激活函数进行S曲线采样
                    float front_pixle_weight = float(front_overlay_.at<uchar>(row, col));
                    float front_weight = callsigmoid(front_pixle_weight, birdVisionParameter_.sigmoid_, 128.0);
                    float left_weight = 1 - front_weight;

                    cv::Vec3b blended_pixel;
                    blended_pixel[0] = front_weight * front_pixel[0] + left_weight * left_pixel[0];
                    blended_pixel[1] = front_weight * front_pixel[1] + left_weight * left_pixel[1];
                    blended_pixel[2] = front_weight * front_pixel[2] + left_weight * left_pixel[2];

                    all_bird_image.at<cv::Vec3b>(row, col) = blended_pixel;
                }
            }

            // 右上角区域光滑
            for (int row = 0; row < front_overlay_.size().height; row++)
            {
                for (int col = front_overlay_.size().width / 2 + 1; col < front_overlay_.size().width; col++)
                {

                    if (front_overlay_.at<uchar>(row, col) == 0)
                    {
                        continue;
                    }

                    cv::Vec3b front_pixel = front_bird_vision_image_.at<cv::Vec3b>(row, col);
                    cv::Vec3b right_pixel = right_bird_vision_image_.at<cv::Vec3b>(row, col - int(birdVisionParameter_.width_ / 2));

//                auto front_pixle_weight = float(front_overlay_.at<uchar>(row, col));
//                auto right_pixle_weight = float(right_overlay_.at<uchar>(row, col - int(birdVisionParameter_.width_ / 2)));
//                auto sum_pixel_weight = front_pixle_weight * front_pixle_weight + right_pixle_weight * right_pixle_weight;
//
//                float front_weight = front_pixle_weight * front_pixle_weight / sum_pixel_weight;
//                float right_weight = 1.0 - front_weight;


                    float front_pixle_weight = float(front_overlay_.at<uchar>(row, col));
                    float front_weight = callsigmoid(front_pixle_weight, birdVisionParameter_.sigmoid_, 128);
                    float right_weight = 1 - front_weight;

                    cv::Vec3b blended_pixel;
                    blended_pixel[0] = front_weight * front_pixel[0] + right_weight * right_pixel[0];
                    blended_pixel[1] = front_weight * front_pixel[1] + right_weight * right_pixel[1];
                    blended_pixel[2] = front_weight * front_pixel[2] + right_weight * right_pixel[2];

                    all_bird_image.at<cv::Vec3b>(row, col) = blended_pixel;
                }
            }

            // 左下角光滑处理
            for (int row = 0; row < back_overlay_.size().height; row++)
            {
                for (int col = 0; col < back_overlay_.size().width / 2; col++)
                {

                    if (back_overlay_.at<uchar>(row, col) == 0)
                    {
                        continue;
                    }

                    cv::Vec3b back_pixel = back_bird_vision_image_.at<cv::Vec3b>(row, col);
                    cv::Vec3b left_pixel = left_bird_vision_image_.at<cv::Vec3b>(row + int(birdVisionParameter_.width_ / 2), col);

//                auto back_pixle_weight = float(back_overlay_.at<uchar>(row, col));
//                auto left_pixle_weight = float(left_overlay_.at<uchar>(row + int(birdVisionParameter_.width_ / 2), col));
//                auto sum_pixel_weight = back_pixle_weight * back_pixle_weight + left_pixle_weight * left_pixle_weight;
//
//                float back_weight = back_pixle_weight * back_pixle_weight / sum_pixel_weight;
//                float left_weight = 1 - back_weight;


                    float back_pixle_weight = float(back_overlay_.at<uchar>(row, col));
                    float back_weight = callsigmoid(back_pixle_weight, birdVisionParameter_.sigmoid_, 128);
                    float left_weight = 1 - back_weight;

                    cv::Vec3b blended_pixel;
                    blended_pixel[0] = back_weight * back_pixel[0] + left_weight * left_pixel[0];
                    blended_pixel[1] = back_weight * back_pixel[1] + left_weight * left_pixel[1];
                    blended_pixel[2] = back_weight * back_pixel[2] + left_weight * left_pixel[2];

                    all_bird_image.at<cv::Vec3b>(row + int(birdVisionParameter_.width_ / 2), col) = blended_pixel;
                }
            }

            // 右下角光滑处理
            for (int row = 0; row < back_overlay_.size().height; row++)
            {
                for (int col = back_overlay_.size().width / 2; col < back_overlay_.size().width; col++)
                {

                    if (back_overlay_.at<uchar>(row, col) == 0)
                    {
                        continue;
                    }

                    cv::Vec3b back_pixel = back_bird_vision_image_.at<cv::Vec3b>(row, col);
                    cv::Vec3b right_pixel = right_bird_vision_image_.at<cv::Vec3b>(row + int(birdVisionParameter_.width_ / 2),
                                                                                   col - int(birdVisionParameter_.width_ / 2));

//                auto back_pixle_weight = float(back_overlay_.at<uchar>(row, col));
//                auto right_pixle_weight = float(right_overlay_.at<uchar>(row + int(birdVisionParameter_.width_ / 2),
//                                                                         col - int(birdVisionParameter_.width_ / 2)));
//
//                auto sum_pixel_weight = back_pixle_weight * back_pixle_weight + right_pixle_weight * right_pixle_weight;
//
//                float back_weight = back_pixle_weight * back_pixle_weight / sum_pixel_weight;
//                float right_weight = 1 - back_weight;

                    float back_pixle_weight = float(back_overlay_.at<uchar>(row, col));
                    float back_weight = callsigmoid(back_pixle_weight, birdVisionParameter_.sigmoid_, 128);
                    float right_weight = 1 - back_weight;

                    cv::Vec3b blended_pixel;
                    blended_pixel[0] = back_weight * back_pixel[0] + right_weight * right_pixel[0];
                    blended_pixel[1] = back_weight * back_pixel[1] + right_weight * right_pixel[1];
                    blended_pixel[2] = back_weight * back_pixel[2] + right_weight * right_pixel[2];

                    all_bird_image.at<cv::Vec3b>(row + int(birdVisionParameter_.width_ / 2), col) = blended_pixel;
                }
            }
        }
        else if(birdVisionParameter_.fusion_plan_ == 1){
            // 方案二: 对生成的四个鸟瞰图进行特征匹配拼接
            static cv::Mat H_front_left, H_front_right, H_right_back;
            if(H_front_left.empty() && H_front_right.empty() && H_right_back.empty()){
                // 使用特征检测与匹配找到两幅图像之间的变换关系
                Ptr<Feature2D> detector = ORB::create();
                vector<KeyPoint> keypoints1, keypoints2, keypoints3, keypoints4;
                Mat descriptors1, descriptors2, descriptors3, descriptors4;
                detector->detectAndCompute(front_bird_vision_image_, Mat(), keypoints1, descriptors1);
                detector->detectAndCompute(left_bird_vision_image_, Mat(), keypoints2, descriptors2);
                detector->detectAndCompute(back_bird_vision_image_, Mat(), keypoints3, descriptors3);
                detector->detectAndCompute(right_bird_vision_image_, Mat(), keypoints4, descriptors4);

                // 使用暴力匹配器进行特征匹配
                BFMatcher matcher(NORM_HAMMING);
                vector<DMatch> matches;
                matcher.match(descriptors1, descriptors2, matches);

                // 选择最佳匹配点
                double min_dist = DBL_MAX;
                for (int i = 0; i < descriptors1.rows; i++) {
                    double dist = matches[i].distance;
                    if (dist < min_dist) {
                        min_dist = dist;
                    }
                }

                vector<Point2f> points1, points2;
                for (int i = 0; i < descriptors1.rows; i++) {
                    if (matches[i].distance <= max(2 * min_dist, 30.0)) {
                        points1.push_back(keypoints1[matches[i].queryIdx].pt);
                        points2.push_back(keypoints2[matches[i].trainIdx].pt);
                    }
                }
                H_front_left = findHomography(points1, points2, RANSAC);
            }

            Mat result;
            warpPerspective(front_bird_vision_image_, all_bird_image, H_front_left, Size(all_bird_image.cols, all_bird_image.rows));
        }

        // 发布Image4M的图像进行推流
        static uint64_t count = 0;
        auto loanedMsg = pub_cutome_birdview_ptr_->borrow_loaned_message();
        setLoanedMessage(getRosTime(), std::to_string(count), all_bird_image, loanedMsg);
        count++;
        pub_cutome_birdview_ptr_->publish(std::move(loanedMsg));

        // 发布ROS上rviz可视化的鸟瞰图
        if(birdVisionParameter_.is_pub_test_images_){
            pub_all_bird_vision_ptr_->publish(rosFromMat(all_bird_image, getRosTime()));
        }

        std::cout << "耗时为：" << time_consume.Toc() << "ms." << std::endl << std::endl;
    }

}


// 鸟瞰图处理逻辑
void BirdVision::birdImageHandle(const cv::Mat &origin_image,
                                 const std::string& camera_index,
                                 const CamParameter& camParameter,
                                 std::vector<cv::Point2f> &map_table,
                                 cv::Mat &bird_image) const {

    if (camera_index == "front") {
        if (!system_init_flag) {
            // 初始化maptable大小
            map_table.clear();
            map_table.resize(bird_image.cols * bird_image.rows, cv::Point2f(-1.0, -1.0));

            // 鸟瞰图制作
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    // birdvision上的索引 转换成 baselink坐标下的三维点信息
                    Eigen::Vector2d bird_vision_pixel = Eigen::Vector2d(col, row);
                    double pw_x =
                            ((int(birdVisionParameter_.height_ / 2.0) - bird_vision_pixel.y()) - 0.5) * birdVisionParameter_.resolution_;
                    double pw_y =
                            ((int(birdVisionParameter_.height_ / 2.0) - bird_vision_pixel.x()) - 0.5) * birdVisionParameter_.resolution_;
                    Eigen::Vector3d p_w = Eigen::Vector3d(pw_x, pw_y, 0.0);

                    // 三维坐标到鱼眼相机像素平面内的投影
                    Eigen::Vector2d distort_pixel;
                    if(camParameter.cam_type_ == 0){
                        PinholeModel::pinhole_project_point(p_w,
                                                            camParameter.get_R_camera_baselink(),
                                                            camParameter.get_t_camera_baselink(),
                                                            camParameter.intrinsic_matrix_,
                                                            camParameter.distort_matrix_,
                                                            distort_pixel);
                    }else if(camParameter.cam_type_ == 1){
                        FishEyeModel::fish_project_point(p_w,
                                                         camParameter.get_R_camera_baselink(),
                                                         camParameter.get_t_camera_baselink(),
                                                         camParameter.intrinsic_matrix_,
                                                         camParameter.distort_matrix_,
                                                         distort_pixel);
                    }else{
                        std::cerr << " 相机类型错误... " << std::endl;
                    }

                    // 确定最终插值的像素点位置
                    int u = static_cast<int>(distort_pixel.x());
                    int v = static_cast<int>(distort_pixel.y());

                    // 进行鸟瞰图投影 这里计算第一帧，计算亮度平衡的参数
                    if (u < 0 || u >= origin_image.cols || v < 0 || v >= origin_image.rows) {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(-1.0, -1.0);
                        continue;
                    } else {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(distort_pixel.x(), distort_pixel.y());

                        // 用来计算亮度平衡系数
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }
        }
        else {
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    int index = row * bird_image.cols + col;
                    auto result = map_table.at(index);
                    if (result == cv::Point2f(-1, -1)) {
                        bird_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                    }else{
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }

//            cv::Mat hsvImage;
//            cv::cvtColor(bird_image, hsvImage, cv::COLOR_BGR2HSV);
//            std::vector<cv::Mat> channels;
//            cv::split(hsvImage, channels);
//            cv::Mat vChannel = channels[2]; // 明度通道
//            vChannel = vChannel * hsv[0];
//            channels[2] = vChannel;
//            cv::merge(channels, hsvImage);
//            cv::cvtColor(hsvImage, bird_image, cv::COLOR_HSV2BGR);

        }
    }

    if (camera_index == "left") {
        if (!system_init_flag) {
            // 初始化maptable大小
            map_table.clear();
            map_table.resize(bird_image.cols * bird_image.rows, cv::Point2f(-1.0, -1.0));

            // 鸟瞰图制作
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    // birdvision上的索引 转换成 baselink坐标下的三维点信息
                    Eigen::Vector2d bird_vision_pixel = Eigen::Vector2d(col, row);
                    double pw_x =
                            ((int(birdVisionParameter_.height_ / 2.0) - bird_vision_pixel.y()) - 0.5) * birdVisionParameter_.resolution_;
                    double pw_y =
                            ((int(birdVisionParameter_.height_ / 2.0) - bird_vision_pixel.x()) - 0.5) * birdVisionParameter_.resolution_;
                    Eigen::Vector3d p_w = Eigen::Vector3d(pw_x, pw_y, 0.0);

                    // 三维坐标到鱼眼相机像素平面内的投影
                    Eigen::Vector2d distort_pixel;
                    if(camParameter.cam_type_ == 0){
                        PinholeModel::pinhole_project_point(p_w,
                                                            camParameter.get_R_camera_baselink(),
                                                            camParameter.get_t_camera_baselink(),
                                                            camParameter.intrinsic_matrix_,
                                                            camParameter.distort_matrix_,
                                                            distort_pixel);
                    }else if(camParameter.cam_type_ == 1){
                        FishEyeModel::fish_project_point(p_w,
                                                         camParameter.get_R_camera_baselink(),
                                                         camParameter.get_t_camera_baselink(),
                                                         camParameter.intrinsic_matrix_,
                                                         camParameter.distort_matrix_,
                                                         distort_pixel);
                    }else{
                        std::cerr << " 相机类型错误... " << std::endl;
                    }

                    // 确定最终插值的像素点位置
                    int u = static_cast<int>(distort_pixel.x());
                    int v = static_cast<int>(distort_pixel.y());

                    // 进行鸟瞰图投影
                    if (u < 0 || u >= origin_image.cols || v < 0 || v >= origin_image.rows) {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(-1, -1);
                        continue;
                    } else {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(distort_pixel.x(), distort_pixel.y());

                        // 用来计算亮度平衡系数
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }
        } else {
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    int index = row * bird_image.cols + col;
                    auto result = map_table.at(index);

                    if (result == cv::Point2f(-1, -1)) {
                        bird_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                    }else{
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }

//            cv::Mat hsvImage;
//            cv::cvtColor(bird_image, hsvImage, cv::COLOR_BGR2HSV);
//            std::vector<cv::Mat> channels;
//            cv::split(hsvImage, channels);
//            cv::Mat vChannel = channels[2]; // 明度通道
//            vChannel = vChannel * hsv[1];
//            channels[2] = vChannel;
//            cv::merge(channels, hsvImage);
//            cv::cvtColor(hsvImage, bird_image, cv::COLOR_HSV2BGR);

        }
    }

    if (camera_index == "back") {
        if (!system_init_flag) {
            // 初始化maptable大小
            map_table.clear();
            map_table.resize(bird_image.cols * bird_image.rows, cv::Point2f(-1.0, -1.0));

            // 鸟瞰图制作
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    // birdvision上的索引 转换成 baselink坐标下的三维点信息
                    Eigen::Vector2d bird_vision_pixel = Eigen::Vector2d(col, row);
                    double pw_x = -1 * ((bird_vision_pixel.y() + 1) - 0.5) * birdVisionParameter_.resolution_;
                    double pw_y =
                            ((int(birdVisionParameter_.height_ / 2.0) - bird_vision_pixel.x()) - 0.5) * birdVisionParameter_.resolution_;
                    Eigen::Vector3d p_w = Eigen::Vector3d(pw_x, pw_y, 0.0);

                    // 三维坐标到鱼眼相机像素平面内的投影
                    Eigen::Vector2d distort_pixel;
                    if(camParameter.cam_type_ == 0){
                        PinholeModel::pinhole_project_point(p_w,
                                                            camParameter.get_R_camera_baselink(),
                                                            camParameter.get_t_camera_baselink(),
                                                            camParameter.intrinsic_matrix_,
                                                            camParameter.distort_matrix_,
                                                            distort_pixel);
                    }else if(camParameter.cam_type_ == 1){
                        FishEyeModel::fish_project_point(p_w,
                                                         camParameter.get_R_camera_baselink(),
                                                         camParameter.get_t_camera_baselink(),
                                                         camParameter.intrinsic_matrix_,
                                                         camParameter.distort_matrix_,
                                                         distort_pixel);
                    }else{
                        std::cerr << " 相机类型错误... " << std::endl;
                    }

                    // 确定最终插值的像素点位置
                    int u = static_cast<int>(distort_pixel.x());
                    int v = static_cast<int>(distort_pixel.y());

                    // 进行鸟瞰图投影
                    if (u < 0 || u >= origin_image.cols || v < 0 || v >= origin_image.rows) {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(-1, -1);
                        continue;
                    } else {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(distort_pixel.x(), distort_pixel.y());

                        // 用来计算亮度平衡系数
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }
        } else {
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    int index = row * bird_image.cols + col;
                    auto result = map_table.at(index);

                    if (result == cv::Point2f(-1, -1)) {
                        bird_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                    }else{
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }

//            cv::Mat hsvImage;
//            cv::cvtColor(bird_image, hsvImage, cv::COLOR_BGR2HSV);
//            std::vector<cv::Mat> channels;
//            cv::split(hsvImage, channels);
//            cv::Mat vChannel = channels[2]; // 明度通道
//            vChannel = vChannel * hsv[2];
//            channels[2] = vChannel;
//            cv::merge(channels, hsvImage);
//            cv::cvtColor(hsvImage, bird_image, cv::COLOR_HSV2BGR);

        }
    }

    if (camera_index == "right") {
        if (!system_init_flag) {
            // 初始化maptable大小
            map_table.clear();
            map_table.resize(bird_image.cols * bird_image.rows, cv::Point2f(-1.0, -1.0));

            // 鸟瞰图制作
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    // birdvision上的索引 转换成 baselink坐标下的三维点信息
                    Eigen::Vector2d bird_vision_pixel = Eigen::Vector2d(col, row);
                    double pw_x =
                            ((int(birdVisionParameter_.height_ / 2.0) - bird_vision_pixel.y()) - 0.5) * birdVisionParameter_.resolution_;
                    double pw_y = -1 * ((bird_vision_pixel.x() + 1) - 0.5) * birdVisionParameter_.resolution_;
                    Eigen::Vector3d p_w = Eigen::Vector3d(pw_x, pw_y, 0.0);

                    // 三维坐标到鱼眼相机像素平面内的投影
                    Eigen::Vector2d distort_pixel;
                    if(camParameter.cam_type_ == 0){
                        PinholeModel::pinhole_project_point(p_w,
                                                            camParameter.get_R_camera_baselink(),
                                                            camParameter.get_t_camera_baselink(),
                                                            camParameter.intrinsic_matrix_,
                                                            camParameter.distort_matrix_,
                                                            distort_pixel);
                    }else if(camParameter.cam_type_ == 1){
                        FishEyeModel::fish_project_point(p_w,
                                                         camParameter.get_R_camera_baselink(),
                                                         camParameter.get_t_camera_baselink(),
                                                         camParameter.intrinsic_matrix_,
                                                         camParameter.distort_matrix_,
                                                         distort_pixel);
                    }else{
                        std::cerr << " 相机类型错误... " << std::endl;
                    }

                    // 确定最终插值的像素点位置
                    int u = static_cast<int>(distort_pixel.x());
                    int v = static_cast<int>(distort_pixel.y());

                    // 进行鸟瞰图投影
                    if (u < 0 || u >= origin_image.cols || v < 0 || v >= origin_image.rows) {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(-1, -1);
                        continue;
                    } else {
                        size_t index = row * bird_image.cols + col;
                        map_table.at(index) = cv::Point2f(distort_pixel.x(), distort_pixel.y());

                        // 用来计算亮度平衡系数
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(distort_pixel.x(), distort_pixel.y()), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }
        } else {
            for (int row = 0; row < bird_image.rows; row++) {
                for (int col = 0; col < bird_image.cols; col++) {
                    // 判断是否为有效区域
                    if (bird_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0)) {
                        continue;
                    }

                    int index = row * bird_image.cols + col;
                    auto result = map_table.at(index);

                    if (result == cv::Point2f(-1, -1)) {
                        bird_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                    }else{
                        cv::Vec3b bgr;
                        if(camParameter.cam_type_ == 0){
                            PinholeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else if(camParameter.cam_type_ == 1){
                            FishEyeModel::bilinearInterpolation(origin_image, Eigen::Vector2d(result.x, result.y), bgr);
                        }else{
                            std::cerr << " 相机类型错误... " << std::endl;
                        }
                        bird_image.at<cv::Vec3b>(row, col) = bgr;
                    }
                }
            }

//            cv::Mat hsvImage;
//            cv::cvtColor(bird_image, hsvImage, cv::COLOR_BGR2HSV);
//            std::vector<cv::Mat> channels;
//            cv::split(hsvImage, channels);
//            cv::Mat vChannel = channels[2]; // 明度通道
//            vChannel = vChannel * hsv[3];
//            channels[2] = vChannel;
//            cv::merge(channels, hsvImage);
//            cv::cvtColor(hsvImage, bird_image, cv::COLOR_HSV2BGR);
        }
    }
}


// 基于RGB通道的亮度平衡
void BirdVision::RGBBrightBalance(cv::Mat &frontImg, cv::Mat &leftImg, cv::Mat &backImg, cv::Mat &rightImg) {
    // 计算每一张子图的三色通道的均值，并将四张子图三色通道均值设置成一致
    std::vector<cv::Mat> front_channels;
    cv::split(frontImg, front_channels);
    cv::Scalar front_mean_b = cv::mean(front_channels[0]);
    cv::Scalar front_mean_g = cv::mean(front_channels[1]);
    cv::Scalar front_mean_r = cv::mean(front_channels[2]);

    std::vector<cv::Mat> left_channels;
    cv::split(leftImg, left_channels);
    cv::Scalar left_mean_b = cv::mean(left_channels[0]);
    cv::Scalar left_mean_g = cv::mean(left_channels[1]);
    cv::Scalar left_mean_r = cv::mean(left_channels[2]);

    std::vector<cv::Mat> back_channels;
    cv::split(backImg, back_channels);
    cv::Scalar back_mean_b = cv::mean(back_channels[0]);
    cv::Scalar back_mean_g = cv::mean(back_channels[1]);
    cv::Scalar back_mean_r = cv::mean(back_channels[2]);

    std::vector<cv::Mat> right_channels;
    cv::split(rightImg, right_channels);
    cv::Scalar right_mean_b = cv::mean(right_channels[0]);
    cv::Scalar right_mean_g = cv::mean(right_channels[1]);
    cv::Scalar right_mean_r = cv::mean(right_channels[2]);

    // 计算四张子图的三通道的的均值，决定子图每个通道缩放系数
    double mean_b = (front_mean_b.val[0] + left_mean_b.val[0] + back_mean_b.val[0] + right_mean_b.val[0]) / 4.0;
    double mean_g = (front_mean_g.val[0] + left_mean_g.val[0] + back_mean_g.val[0] + right_mean_g.val[0]) / 4.0;
    double mean_r = (front_mean_r.val[0] + left_mean_r.val[0] + back_mean_r.val[0] + right_mean_r.val[0]) / 4.0;

    // 确定每张图片每个通道的缩放系数
    double front_b_focal = mean_b / front_mean_b.val[0];
    double front_g_focal = mean_g / front_mean_g.val[0];
    double front_r_focal = mean_r / front_mean_r.val[0];

    double left_b_focal = mean_b / left_mean_b.val[0];
    double left_g_focal = mean_g / left_mean_g.val[0];
    double left_r_focal = mean_r / left_mean_r.val[0];

    double back_b_focal = mean_b / back_mean_b.val[0];
    double back_g_focal = mean_g / back_mean_g.val[0];
    double back_r_focal = mean_r / back_mean_r.val[0];

    double right_b_focal = mean_b / right_mean_b.val[0];
    double right_g_focal = mean_g / right_mean_g.val[0];
    double right_r_focal = mean_r / right_mean_r.val[0];

    // 存储缩放倍数
    bgr[0] = front_b_focal;
    bgr[1] = front_g_focal;
    bgr[2] = front_r_focal;

    bgr[3] = left_b_focal;
    bgr[4] = left_g_focal;
    bgr[5] = left_r_focal;

    bgr[6] = back_b_focal;
    bgr[7] = back_g_focal;
    bgr[8] = back_r_focal;

    bgr[9] = right_b_focal;
    bgr[10] = right_g_focal;
    bgr[11] = right_r_focal;

    // 按照条件进行通道放大
    bool is_all_merged = false;
    if (is_all_merged) {
        // 全部放大
        front_channels[0] *= front_b_focal;
        front_channels[1] *= front_g_focal;
        front_channels[2] *= front_r_focal;


        left_channels[0] *= left_b_focal;
        left_channels[1] *= left_g_focal;
        left_channels[2] *= left_r_focal;


        back_channels[0] *= back_b_focal;
        back_channels[1] *= back_g_focal;
        back_channels[2] *= back_r_focal;


        right_channels[0] *= right_b_focal;
        right_channels[1] *= right_g_focal;
        right_channels[2] *= right_r_focal;
    } else {
        if (front_b_focal > 1.0 && front_g_focal > 1.0 && front_r_focal > 1.0) {
            front_channels[0] *= front_b_focal;
            front_channels[1] *= front_g_focal;
            front_channels[2] *= front_r_focal;
        }

        if (left_b_focal > 1.0 && left_g_focal > 1.0 && left_r_focal > 1.0) {
            left_channels[0] *= left_b_focal;
            left_channels[1] *= left_g_focal;
            left_channels[2] *= left_r_focal;
        }

        if (back_b_focal > 1.0 && back_g_focal > 1.0 && back_r_focal > 1.0) {
            back_channels[0] *= back_b_focal;
            back_channels[1] *= back_g_focal;
            back_channels[2] *= back_r_focal;
        }

        if (right_b_focal > 1.0 && right_g_focal > 1.0 && right_r_focal > 1.0) {
            right_channels[0] *= right_b_focal;
            right_channels[1] *= right_g_focal;
            right_channels[2] *= right_r_focal;
        }
    }

    // 合并三个通道成一个图像
    cv::merge(front_channels, frontImg);
    cv::merge(left_channels, leftImg);
    cv::merge(back_channels, backImg);
    cv::merge(right_channels, rightImg);
}

// 基于YUV通道的亮度平衡 表示暗亮程度的亮度信号(Luminance)Y，和两个表示颜色的色度信号(Chrominance)U 和 V
void BirdVision::YUVBrightBalance(cv::Mat &frontImg, cv::Mat &leftImg, cv::Mat &backImg, cv::Mat &rightImg) {
    // 将RGB图片转换成YUV图片
    cv::Mat yuv_front, yuv_left, yuv_back, yuv_right;
    cvtColor(frontImg, yuv_front, cv::COLOR_BGR2YUV);
    cvtColor(leftImg, yuv_left, cv::COLOR_BGR2YUV);
    cvtColor(backImg, yuv_back, cv::COLOR_BGR2YUV);
    cvtColor(rightImg, yuv_right, cv::COLOR_BGR2YUV);

    // 提取每个通道的 Y通道数据 U通道数据 V通道数据
    std::vector<cv::Mat> front_channels;
    cv::split(yuv_front, front_channels);
    cv::Scalar front_mean_y = cv::mean(front_channels[0]);
    cv::Scalar front_mean_u = cv::mean(front_channels[1]);
    cv::Scalar front_mean_v = cv::mean(front_channels[2]);

    std::vector<cv::Mat> left_channels;
    cv::split(yuv_left, left_channels);
    cv::Scalar left_mean_y = cv::mean(left_channels[0]);
    cv::Scalar left_mean_u = cv::mean(left_channels[1]);
    cv::Scalar left_mean_v = cv::mean(left_channels[2]);

    std::vector<cv::Mat> back_channels;
    cv::split(yuv_back, back_channels);
    cv::Scalar back_mean_y = cv::mean(back_channels[0]);
    cv::Scalar back_mean_u = cv::mean(back_channels[1]);
    cv::Scalar back_mean_v = cv::mean(back_channels[2]);

    std::vector<cv::Mat> right_channels;
    cv::split(yuv_right, right_channels);
    cv::Scalar right_mean_y = cv::mean(right_channels[0]);
    cv::Scalar right_mean_u = cv::mean(right_channels[1]);
    cv::Scalar right_mean_v = cv::mean(right_channels[2]);

    // 根据Y通道进行Y的调节
    auto mean_Y = (front_mean_y.val[0] + left_mean_y.val[0] + back_mean_y.val[0] + right_mean_y.val[0]) / 4.0;
    auto focal_front = front_mean_y.val[0] / mean_Y;
    auto focal_left = left_mean_y.val[0] / mean_Y;
    auto focal_back = back_mean_y.val[0] / mean_Y;
    auto focal_right = right_mean_y.val[0] / mean_Y;

    // 存储YUV的放大系数
    yuv[0] = focal_front;
    yuv[1] = focal_left;
    yuv[2] = focal_back;
    yuv[3] = focal_right;

    // 按照条件进行通道放大
    bool is_all_merged = false;
    if (is_all_merged) {
        // 全部放大
        front_channels[0] *= focal_front;
        front_channels[1] *= focal_front;
        front_channels[2] *= focal_front;

        left_channels[0] *= focal_left;
        left_channels[1] *= focal_left;
        left_channels[2] *= focal_left;

        back_channels[0] *= focal_back;
        back_channels[1] *= focal_back;
        back_channels[2] *= focal_back;

        right_channels[0] *= focal_right;
        right_channels[1] *= focal_right;
        right_channels[2] *= focal_right;
    } else {
        if (focal_front > 1.0) {
            front_channels[0] *= focal_front;
            front_channels[1] *= focal_front;
            front_channels[2] *= focal_front;
        }

        if (focal_left > 1.0) {
            left_channels[0] *= focal_left;
            left_channels[1] *= focal_left;
            left_channels[2] *= focal_left;
        }

        if (focal_back > 1.0) {
            back_channels[0] *= focal_back;
            back_channels[1] *= focal_back;
            back_channels[2] *= focal_back;
        }

        if (focal_right > 1.0) {
            right_channels[0] *= focal_right;
            right_channels[1] *= focal_right;
            right_channels[2] *= focal_right;
        }
    }

    // 合并三个通道成一个图像
    cv::merge(front_channels, yuv_front);
    cv::merge(left_channels, yuv_left);
    cv::merge(back_channels, yuv_back);
    cv::merge(right_channels, yuv_right);

    // YUV转换成RGB
    cvtColor(yuv_front, frontImg, cv::COLOR_YUV2BGR);
    cvtColor(yuv_left, leftImg, cv::COLOR_YUV2BGR);
    cvtColor(yuv_back, backImg, cv::COLOR_YUV2BGR);
    cvtColor(yuv_right, rightImg, cv::COLOR_YUV2BGR);

}

// HSV通道亮度平衡 H，S，V 这三个通道分别代表着色相(Hue)，饱和度(Saturation)和明度(Value)。
void BirdVision::HSVBrightBalance(cv::Mat &frontImg, cv::Mat &leftImg, cv::Mat &backImg, cv::Mat &rightImg) {

    // 创建每个子图的掩码-用于亮度平衡计算省略黑色部分的计算
    cv::Mat front_mask, left_mask, back_mask, right_mask;
    cvtColor(frontImg, front_mask, cv::COLOR_BGR2GRAY);
    threshold(front_mask, front_mask, 1, 255, cv::THRESH_BINARY);

    cvtColor(leftImg, left_mask, cv::COLOR_BGR2GRAY);
    threshold(left_mask, left_mask, 1, 255, cv::THRESH_BINARY);

    cvtColor(backImg, back_mask, cv::COLOR_BGR2GRAY);
    threshold(back_mask, back_mask, 1, 255, cv::THRESH_BINARY);

    cvtColor(rightImg, right_mask, cv::COLOR_BGR2GRAY);
    threshold(right_mask, right_mask, 1, 255, cv::THRESH_BINARY);


    // 转换为HSV颜色空间，分离H、S、V通道，计算V通道的平均值，调整V通道的亮度，将调整后的V通道放回原始图像，将图像转换回BGR颜色空间
    const float targetMean = birdVisionParameter_.HSV_V_;

    cv::Mat front_hsvImage;
    cv::cvtColor(frontImg, front_hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> front_channels;
    cv::split(front_hsvImage, front_channels);
    cv::Mat front_vChannel = front_channels[2];
    cv::Mat front_maskedVChannel;
    front_vChannel.copyTo(front_maskedVChannel, front_mask);
    cv::Scalar front_meanValue = cv::mean(front_maskedVChannel);
    hsv[0] = targetMean / front_meanValue[0];
    front_vChannel *= hsv[0];
    front_channels[2] = front_vChannel;
    cv::merge(front_channels, front_hsvImage);
    cv::cvtColor(front_hsvImage, frontImg, cv::COLOR_HSV2BGR);


    cv::Mat left_hsvImage;
    cv::cvtColor(leftImg, left_hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> left_channels;
    cv::split(left_hsvImage, left_channels);
    cv::Mat left_vChannel = left_channels[2];
    cv::Mat left_maskedVChannel;
    left_vChannel.copyTo(left_maskedVChannel, left_mask);
    cv::Scalar left_meanValue = cv::mean(left_maskedVChannel);
    hsv[1] = targetMean / left_meanValue[0];
    left_vChannel *= hsv[0];
    left_channels[2] = left_vChannel;
    cv::merge(left_channels, left_hsvImage);
    cv::cvtColor(left_hsvImage, leftImg, cv::COLOR_HSV2BGR);


    cv::Mat back_hsvImage;
    cv::cvtColor(backImg, back_hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> back_channels;
    cv::split(back_hsvImage, back_channels);
    cv::Mat back_vChannel = back_channels[2];
    cv::Mat back_maskedVChannel;
    back_vChannel.copyTo(back_maskedVChannel, back_mask);
    cv::Scalar back_meanValue = cv::mean(back_maskedVChannel);
    hsv[2] = targetMean / back_meanValue[0];
    back_vChannel *= hsv[0];
    back_channels[2] = back_vChannel;
    cv::merge(back_channels, back_hsvImage);
    cv::cvtColor(back_hsvImage, backImg, cv::COLOR_HSV2BGR);


    cv::Mat right_hsvImage;
    cv::cvtColor(rightImg, right_hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> right_channels;
    cv::split(right_hsvImage, right_channels);
    cv::Mat right_vChannel = right_channels[2];
    cv::Mat right_maskedVChannel;
    right_vChannel.copyTo(right_maskedVChannel, right_mask);
    cv::Scalar right_meanValue = cv::mean(right_maskedVChannel);
    hsv[3] = targetMean / right_meanValue[0];
    right_vChannel *= hsv[0];
    right_channels[2] = right_vChannel;
    cv::merge(right_channels, right_hsvImage);
    cv::cvtColor(right_hsvImage, rightImg, cv::COLOR_HSV2BGR);

    // 亮度均衡后的结果可视化
    cv::Mat bright_balance = cv::Mat::zeros(int(birdVisionParameter_.height_), int(birdVisionParameter_.width_),
                                            CV_8UC3);
    frontImg.copyTo(bright_balance(cv::Rect(0, 0, frontImg.cols, frontImg.rows)), front_mask);
    leftImg.copyTo(bright_balance(cv::Rect(0, 0, leftImg.cols, leftImg.rows)), left_mask);
    backImg.copyTo(bright_balance(cv::Rect(0, int(birdVisionParameter_.height_ / 2.0), backImg.cols, backImg.rows)),
                   back_mask);
    rightImg.copyTo(bright_balance(cv::Rect(int(birdVisionParameter_.width_ / 2.0), 0, rightImg.cols, rightImg.rows)),
                    right_mask);


    // 写出色彩平衡的mask范围的掩码 以及 平衡后的图像
    if (birdVisionParameter_.is_save_images_) {
        std::string color_balance_path = birdVisionParameter_.assets_reference_path_ + "/color_balance/";
        cv::imwrite(color_balance_path + "front_mask.png", frontImg);
        cv::imwrite(color_balance_path + "left_mask.png", leftImg);
        cv::imwrite(color_balance_path + "back_mask.png", backImg);
        cv::imwrite(color_balance_path + "right_mask.png", rightImg);
        cv::imwrite(color_balance_path + "bright_balance.png", bright_balance);
    }

}

cv::Mat BirdVision::BinaryAndNoiseRemoval(const Mat &inputImg) {
    cv::Mat binary_mask;

    if (inputImg.channels() == 3) {
        cvtColor(inputImg, binary_mask, cv::COLOR_BGR2GRAY);
    }
    threshold(binary_mask, binary_mask, 1, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat front_whiteBg = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
    cv::drawContours(front_whiteBg, contours, -1, cv::Scalar(255), cv::FILLED);
    return front_whiteBg;
}

builtin_interfaces::msg::Time BirdVision::getRosTime() {
    struct timespec thread_cpu_time{};
    clock_gettime(CLOCK_REALTIME, &thread_cpu_time);
    double cur_timestamp = thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;

    int32_t seconds = static_cast<int32_t>(std::floor(cur_timestamp));
    uint32_t nanosec = static_cast<int32_t>((cur_timestamp - seconds) * 1e9);
    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = seconds;
    msg_time.nanosec = nanosec;
    return msg_time;
}

cv::Mat BirdVision::matFromCustome(const CUSTOMIMAGE_TYPE::SharedPtr &source) {
    int source_type = CV_8UC3;
    int byte_depth = 1;
    int num_channels = 3;

    if (source->step < source->width * byte_depth * num_channels)
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " << source->step << " != " << source->width << " * " << byte_depth << " * " << num_channels;
        throw std::runtime_error(ss.str());
    }

    if (source->height * source->step > CUSTOMIMAGE_TYPE::DATA_MAX_SIZE)
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: height * step > size  or  " << source->height << " * " << source->step << " > " << CUSTOMIMAGE_TYPE::DATA_MAX_SIZE;
        throw std::runtime_error(ss.str());
    }

    // If the endianness is the same as locally, share the data
    cv::Mat mat(source->height, source->width, source_type, const_cast<uchar *>(&source->data[0]),
                source->step);

    // if ((rcpputils::endian::native == rcpputils::endian::big && source->is_bigendian) || (rcpputils::endian::native == rcpputils::endian::little && !source->is_bigendian) || byte_depth == 1)
    // {

    //     return mat;
    // }

    // Otherwise, reinterpret the data as bytes and switch the channels accordingly
    mat = cv::Mat(source->height, source->width, CV_MAKETYPE(CV_8U, num_channels * byte_depth),
                  const_cast<uchar *>(&source->data[0]), source->step);
    cv::Mat mat_swap(source->height, source->width, mat.type());

    std::vector<int> fromTo;
    fromTo.reserve(num_channels * byte_depth);
    for (int i = 0; i < num_channels; ++i)
    {
        for (int j = 0; j < byte_depth; ++j)
        {
            fromTo.push_back(byte_depth * i + j);
            fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
        }
    }
    cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

    // Interpret mat_swap back as the proper type
    mat_swap.reshape(num_channels);

    return mat_swap;
}

void BirdVision::setLoanedMessage(const builtin_interfaces::msg::Time &tm,
                                  const string &frame_id, const Mat &img,
                                  rclcpp::LoanedMessage<CUSTOMIMAGE_TYPE> &loanedMsg){

    CUSTOMIMAGE_TYPE &ros_image = loanedMsg.get();

    ros_image.header.stamp = tm;

    uint8_t frame_id_size = frame_id.size();
    ros_image.header.frame_id.size = frame_id_size;
    std::memcpy(ros_image.header.frame_id.data.data(), frame_id.data(), frame_id_size);

    ros_image.height = img.rows;
    ros_image.width = img.cols;

    std::string encoding_str = "bgr8";
    uint8_t encoding_str_size = encoding_str.size();
    ros_image.encoding.size = encoding_str_size;
    std::memcpy(ros_image.encoding.data.data(), encoding_str.data(), encoding_str_size);

    ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);

    ros_image.step = img.cols * img.elemSize();

    size_t size = ros_image.step * img.rows;
    if (size > CUSTOMIMAGE_TYPE::DATA_MAX_SIZE) {
        std::stringstream ss;
        ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " << ros_image.step
           << " > " << CUSTOMIMAGE_TYPE::DATA_MAX_SIZE;
        throw std::runtime_error(ss.str());
    }

    if (img.isContinuous()) {
        memcpy(reinterpret_cast<char *>(&ros_image.data[0]), img.data, size);
    } else {
        // Copy by row by row
        uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
        uchar *cv_data_ptr = img.data;
        for (int i = 0; i < img.rows; ++i) {
            memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
            ros_data_ptr += ros_image.step;
            cv_data_ptr += img.step;
        }
    }
}

ROS_IMAGE_TYPE BirdVision::rosFromMat(const Mat &mat, const builtin_interfaces::msg::Time &msg_time){
    ROS_IMAGE_TYPE msg;
    try {
        // 将OpenCV图像转换为ROS图像消息
        cv_bridge::CvImage cv_image;
        cv_image.image = mat;
        if (mat.channels() == 1) {
            cv_image.encoding = "mono8";
        } else {
            cv_image.encoding = "bgr8";
        }
        msg = *cv_image.toImageMsg();
    }
    catch (cv_bridge::Exception &e) {
        // 如果无法将OpenCV图像转换为ROS图像消息，则会抛出异常
        printf("Could not convert from 'bgr8' to '%s'.", msg.encoding.c_str());
    }
    msg.header.stamp = msg_time;
    return msg;
}

cv::Mat BirdVision::matFromROS(const ROS_IMAGE_TYPE &msg){
    cv::Mat transformed_image;
    try {
        // 将ROS图像消息转换为OpenCV图像格式
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 在这里可以对OpenCV图像进行操作
        transformed_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e) {
        // 如果无法将ROS图像消息转换为OpenCV图像格式，则会抛出异常
        printf("Could not convert from '%s' to 'bgr8'.", msg.encoding.c_str());
    }

    return transformed_image;
}
















