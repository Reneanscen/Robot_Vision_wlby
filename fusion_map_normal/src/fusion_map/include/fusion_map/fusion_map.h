#ifndef FUSION_MAP_H
#define FUSION_MAP_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
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

#include <Eigen/Dense>
#include <Eigen/Geometry>


class FusionMapNode : public rclcpp::Node 
{

public:

  FusionMapNode();

  ~FusionMapNode();

private:

  void initializeSubscribers();
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void tofCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void realsenseCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void realsenseDepCallback(const custom_image_msg::msg::Image4m::SharedPtr msg);
  void groundCallback(const zme_msg_srv::msg::ObstacleavoidList::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void processData();
  Eigen::Matrix4f interpolatePose(const rclcpp::Time& target_time, bool& isget);

  //Eigen::Matrix4f calculatePoseDifference(const nav_msgs::msg::Odometry& pose, const rclcpp::Time& reference_time);

  pcl::PCLPointCloud2 applyTransform(pcl::PCLPointCloud2& data, const Eigen::Matrix4f& transform, bool isground);

  void mergePointClouds(pcl::PCLPointCloud2& merged_cloud, pcl::PCLPointCloud2 data_lidar,
                                     pcl::PCLPointCloud2 data_tof, pcl::PCLPointCloud2 data_realsense,
                                                                        pcl::PCLPointCloud2 data_ground);
  void filterPointCloud(pcl::PCLPointCloud2& cloud);
  void flattenPointCloud(pcl::PCLPointCloud2& cloud);
  void projectPointCloud(pcl::PCLPointCloud2& cloud);
  void generateGridMap(const pcl::PCLPointCloud2& cloud);

  Eigen::Matrix4f createTransformMatrix(float x, float y, float z, float roll, float pitch, float yaw);

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr realsense_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<zme_msg_srv::msg::ObstacleavoidList>::SharedPtr ground_sub_;
  rclcpp::Subscription<custom_image_msg::msg::Image4m>::SharedPtr realsense_dep_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

  // 里程计数据缓冲区
  std::list<std::pair<rclcpp::Time, nav_msgs::msg::Odometry>> odom_buffer_;
  std::mutex sensor_mutex_;
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

};


#endif  // FUSION_MAP_H