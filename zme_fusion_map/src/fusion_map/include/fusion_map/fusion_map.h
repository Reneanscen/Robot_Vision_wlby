#ifndef FUSION_MAP_H
#define FUSION_MAP_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "zme_msg_srv/msg/obstacleavoid.hpp"
#include "zme_msg_srv/msg/obstacleavoid_list.hpp"

#include "custom_image_msg/msg/image4m.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <list>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>


#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <esdf/ESDFMap.h>
//#include <esdf/parameters.h>
#include <esdf/raycast.h>
#include <esdf/timing.h>


class FusionMapNode : public rclcpp::Node 
{
public:

  FusionMapNode();

  ~FusionMapNode();

  struct Parameters {
    // resolution
    double resolution_;
    // hash table implementation only
    int reserved_size_;
    // array implementation only
    Eigen::Vector3d l_cornor_, r_cornor_, map_size_;
    // intrinsic parameters of camera, only used if your input is depth image
    double center_x_, center_y_, focal_x_, focal_y_;
    // parameters of probabilistic occupancy updating
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
    //depth filter
    bool use_depth_filter_;
    double depth_filter_max_dist_, depth_filter_min_dist_, depth_filter_tolerance_;
    int depth_filter_margin_;
    // ray cast parameters
    double min_ray_length_, max_ray_length_;
    // visualization
    double slice_vis_max_dist_;
    int slice_vis_level_, vis_lower_bound_, vis_upper_bound_;
    // frequency of updating
    double update_esdf_every_n_sec_;
    // frequency of visualization
    int visualize_every_n_updates_;
    // number of thread used
    int ray_cast_num_thread_;
    // local map
    bool global_vis_, global_update_, global_map_;
    Eigen::Vector3d radius_;
    // transforms
    Eigen::Matrix4d T_B_C_, T_D_B_;

  };

private:

  void initializeSubscribers();
  void SetParameters();
  void createLogFile();
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void tofCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void realsenseCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void realsenseDepCallback(const custom_image_msg::msg::Image4m::SharedPtr msg);
  void groundCallback(const zme_msg_srv::msg::ObstacleavoidList::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void processData();
  Eigen::Matrix4f interpolatePose(const rclcpp::Time& target_time, bool& isget);

  //Eigen::Matrix4f calculatePoseDifference(const nav_msgs::msg::Odometry& pose, const rclcpp::Time& reference_time);

  pcl::PointCloud<pcl::PointXYZ>::Ptr applyTransform(pcl::PCLPointCloud2& data, const Eigen::Matrix4f& transform, bool isground);

  void mergePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& merged_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr data_lidar,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr data_tof, pcl::PointCloud<pcl::PointXYZ>::Ptr data_realsense,
                                                                        pcl::PointCloud<pcl::PointXYZ>::Ptr data_ground);
  void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void flattenPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void projectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void generateGridMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  Eigen::Matrix4f createTransformMatrix(float x, float y, float z, float roll, float pitch, float yaw);

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr realsense_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<zme_msg_srv::msg::ObstacleavoidList>::SharedPtr ground_sub_;
  rclcpp::Subscription<custom_image_msg::msg::Image4m>::SharedPtr realsense_dep_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr slice_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_pub_;

  std::thread processing_thread;
  std::mutex sensor_mutex_;
  bool running_;
  bool esdf_mode;
  std::ofstream log_file_; 

  // 里程计数据缓冲区
  std::list<std::pair<rclcpp::Time, nav_msgs::msg::Odometry>> odom_buffer_;
  pcl::PCLPointCloud2 lidar_data_;
  pcl::PCLPointCloud2 tof_data_;
  pcl::PCLPointCloud2 realsense_data_;
  pcl::PCLPointCloud2 ground_data_;
  rclcpp::Time lidar_time_;
  rclcpp::Time tof_time_;
  rclcpp::Time realsense_time_;
  rclcpp::Time ground_time_;
  bool odom_init;
  bool lidar_init;
  bool tof_init;
  bool realsense_init;
  bool ground_init;
  Eigen::Matrix4f transform_lidar_to_baselink;  
  Eigen::Matrix4f transform_tof_to_baselink;  
  Eigen::Matrix4f transform_realsense_to_baselink; 
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  
  laser_geometry::LaserProjection projector_;

  // esdf
  fiesta::ESDFMap *esdf_map_;
  Parameters parameters_;

#ifdef SIGNED_NEEDED
    fiesta::ESDFMap *inv_esdf_map_;
#endif
    bool new_msg_ = false;
    //pcl::PointCloud<pcl::PointXYZ> cloud_;
#ifndef PROBABILISTIC
    sensor_msgs::msg::PointCloud2::ConstPtr sync_pc_;
#endif
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    //ros::Publisher slice_pub_, occupancy_pub_, text_pub_;
    //ros::Subscriber transform_sub_, depth_sub_;
    //ros::Timer update_mesh_timer_;
    Eigen::Vector3d sync_pos_, raycast_origin_, cur_pos_;
    Eigen::Quaterniond sync_q_;

    //std::queue<std::tuple<ros::Time, Eigen::Vector3d, Eigen::Quaterniond>> transform_queue_;
    //std::queue<DepthMsgType> depth_queue_;
    //DepthMsgType sync_depth_;

    //cv::Mat img_[2];
    Eigen::Matrix4d transform_, last_transform_;
    uint image_cnt_ = 0, esdf_cnt_ = 0, tot_ = 0;
#ifdef HASH_TABLE
    std::unordered_set<int> set_free_, set_occ_;
#else
    std::vector<int> set_free_, set_occ_;
#endif
    void Visualization(bool global_vis, const std::string &text);
#ifdef PROBABILISTIC
    void RaycastProcess(int i, int part, int tt, pcl::PointCloud<pcl::PointXYZ> cloud_);
    //void Visualization(fiesta::ESDFMap *esdf_map, bool global_vis, const std::string &text);
    //void RaycastMultithread();
#endif

    //double GetInterpolation(const cv::Mat &img, double u, double v);

    //void DepthConversion();

    //void SynchronizationAndProcess();

    //void DepthCallback(const DepthMsgType &depth_map);

    //void PoseCallback(const PoseMsgType &msg);

    void UpdateEsdfEvent();

};


#endif  // FUSION_MAP_H