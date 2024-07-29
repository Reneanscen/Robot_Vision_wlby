#ifndef CHARGING_INDENTIFI_H
#define CHARGING_INDENTIFI_H

#include <charging_indentifi/dock_candidate.h>
#include <charging_indentifi/laser_processor.h>
#include <charging_indentifi/linear_pose_filter_2d.h>
#include <charging_indentifi/icp_2d.h>

#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "zme_msg_srv/srv/dock_pose.hpp"
#include "zme_msg_srv/srv/charger_location.hpp"
#include "app_msgs/srv/charger_location.hpp"
#include "visualization_msgs/msg/marker.hpp"

//#include <tf/tf.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <vector>
#include <deque>
#include <string>
#include <boost/thread.hpp>
#include <list>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <cmath>

#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>




class ChargInden
{

public:

  

  struct precandidate {
    std::vector<geometry_msgs::msg::Point> points;
    double dist;  // 线段长度
    double k;     // 曲率
    double angle_; //法向量角度
    double radius_;  // 直线到观测点距离
  };

  ChargInden(rclcpp::Node::SharedPtr node);

  ~ChargInden();

  bool start(const geometry_msgs::msg::PoseStamped& pose);

  bool stop();

private:

  void createLogFile();

  double getPoseDistance(const geometry_msgs::msg::Pose a,
                       const geometry_msgs::msg::Pose b);

  void callback(const sensor_msgs::msg::LaserScan scan);

  DockCandidatePtr extract(laser_processor::SampleSet* cluster);
  DockCandidatePtr extract(laser_processor::SampleSet* cluster, 
                            std::vector<DockCandidatePtr>& candidates1);

  double calculateCurvature(const geometry_msgs::msg::Point& prev, 
                          const geometry_msgs::msg::Point& current, 
                          const geometry_msgs::msg::Point& next);

  void rdpSegmentation(const std::vector<geometry_msgs::msg::Point>& points, 
                     double epsilon, 
                     std::vector<ChargInden::precandidate>& segments);

  double calculateDistanceToLine(const geometry_msgs::msg::Point& p0, 
                               const geometry_msgs::msg::Point& p1, 
                               const geometry_msgs::msg::Point& p2);

  std::vector<precandidate> processCandidates(const precandidate& input);

  void ToMarkerMsg(std::vector<ChargInden::precandidate> precand, 
                                           visualization_msgs::msg::Marker &marker_msg);

  double fit(const DockCandidatePtr& candidate, geometry_msgs::msg::Pose& pose);
  
  static bool isValid(const tf2::Quaternion& q);

  void handle_service_request(const std::shared_ptr<app_msgs::srv::ChargerLocation::Request> request,
                                std::shared_ptr<app_msgs::srv::ChargerLocation::Response> response);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  
  std::ofstream log_file_;
  bool running_;  
  bool debug_;  
  LinearPoseFilter2DPtr dock_pose_filter_;  
  laser_processor::params param_;

  std::string tracking_frame_;
  std::string lidar_topic_;
  geometry_msgs::msg::PoseStamped dock_;
  geometry_msgs::msg::PoseStamped dock_1;
  std::mutex dock_mutex_;
  bool found_dock_;
  bool init_dock;
  rclcpp::Time dock_stamp_;
  double max_alignment_error_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_points2_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_lines_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr StationPose_publisher_;
  rclcpp::Service<app_msgs::srv::ChargerLocation>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::CallbackGroup::SharedPtr laser_cb_group_;

  std::vector<geometry_msgs::msg::Point> ideal_cloud_;
  std::vector<geometry_msgs::msg::Point> front_cloud_;
  rclcpp::Node::SharedPtr node_;

  double abort_distance_; 
};

#endif  // CHARGING_INDENTIFI_H