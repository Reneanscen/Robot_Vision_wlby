//
// Created by jazzey on 2023/11/14.
//

#ifndef SEMANTIC_MAPPING_SEMANTIC_MAPPING_H
#define SEMANTIC_MAPPING_SEMANTIC_MAPPING_H

#include <mutex>
#include <iostream>
#include <thread>
#include <functional>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>

#include <pcl/filters/voxel_grid.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.hpp>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "std_msgs/msg/header.hpp"

#include "segment_msg/msg/result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "control_msg/srv/control.hpp"

#include "message_filters/subscriber.h"

#include "parameter.h"
#include "common/sleep.h"
#include "common/time_utils.h"
#include "PoseUtils/Pose2DDataStruct.h"
#include "PoseUtils/Pose3DDataStruct.h"

#include "depth_segmentation.h"
#include "semantic_integrator.h"

#include "message_report/message_report.h"

#include "check/check.h"
#include "logger/my_logger.h"

namespace wlby{


    class SemanticMappingNode {
    public:
        explicit SemanticMappingNode(rclcpp::Node::SharedPtr node) : node_(std::move(node)),
                                                                     camera_info_ready_(false){

            //申明参数
            declareParameter();
            //从配置中设置参数
            setParameter();
            //存储log
            configValueLogSave();

            int ret = readNavMap(Parameter::CommonParams::nav_map_path);

            if(!ret){
                log__save("SemanticMapping", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "read nav map success!");
            }else{
                log__save("SemanticMapping", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "read nav map failed!");
                MessageReport::reportErrorAndAssert(1001,"read nav map failed!");
            }

            marker_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            depth_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_, Parameter::TopicParams::depth_image_topic);
            //depth_image_sub_->registerCallback(std::bind(&SemanticMappingNode::depthImageCallback, this, std::placeholders::_1));

            depth_info_sub_= std::make_shared< message_filters::Subscriber<sensor_msgs::msg::CameraInfo> >(node_, Parameter::TopicParams::depth_camera_info_topic);
            depth_info_sub_->registerCallback(std::bind(&SemanticMappingNode::cameraInfoCallback, this, std::placeholders::_1));

            instance_segmentation_sub_ = std::make_shared< message_filters::Subscriber<segment_msg::msg::Result> >(node_, Parameter::TopicParams::segmentation_result_topic);

            tracked_pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(node_,Parameter::TopicParams::tracked_pose_topic);
            //tracked_pose_sub_->registerCallback(std::bind(&SemanticMappingNode::poseStampedCallback, this, std::placeholders::_1));

            CHECK_NOT_NULL(depth_image_sub_);
            CHECK_NOT_NULL(depth_info_sub_);
            CHECK_NOT_NULL(instance_segmentation_sub_);
            CHECK_NOT_NULL(tracked_pose_sub_);

            constexpr int kQueueSize = 300;
            image_segmentation_sync_policy_ptr_ = std::make_shared< message_filters::Synchronizer<ImageSegmentationSyncPolicy> >
                    (ImageSegmentationSyncPolicy(kQueueSize), *depth_image_sub_, *instance_segmentation_sub_, *tracked_pose_sub_);
            image_segmentation_sync_policy_ptr_->setMaxIntervalDuration(rclcpp::Duration(0,50000000));

            image_segmentation_sync_policy_ptr_->registerCallback(
                    std::bind(&SemanticMappingNode::depth_result_pose_callback, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));


            calib_odo_to_cam_ = Localization::Pose3D(Parameter::CalibParams::camera_in_odo_x,Parameter::CalibParams::camera_in_odo_y,Parameter::CalibParams::camera_in_odo_z,
                                                     Parameter::CalibParams::camera_in_odo_yaw,Parameter::CalibParams::camera_in_odo_pitch,Parameter::CalibParams::camera_in_odo_roll);

            segmented_pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_point", 10);
            instance_pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/instance_point", 10);

            semantic_integrator::VoxPosConvertTools vct(Localization::Pos3D(-25,-25,-25),0.025,Localization::Pos3D(50,50,50));
            semantic_integrator_ = std::make_shared<semantic_integrator::SemanticIntegrator>(vct);
            semantic_integrator_->setMaxMin(max_,min_);

            // 创建服务
            control_service_ = node_->create_service<control_msg::srv::Control>(
                    "control_services",
                    std::bind(&SemanticMappingNode::control_callback, this, std::placeholders::_1, std::placeholders::_2));

            // 创建第一个 Publisher 发布第一个 Marker 消息
            marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("/obb_box", 10);

            marker_publisher_timer_ = node_->create_wall_timer(std::chrono::milliseconds(int(0.1 * 1000)), std::bind(&SemanticMappingNode::markerVecPublish,this), marker_group_);

            std::thread t1(&SemanticMappingNode::dataHandleLoop, this);
            t1.detach();

        }

        void declareParameter(){
            node_->declare_parameter<std::string>("topic_params.depth_image_topic", Parameter::TopicParams::depth_image_topic);
            node_->declare_parameter<std::string>("topic_params.depth_camera_info_topic", Parameter::TopicParams::depth_camera_info_topic);
            node_->declare_parameter<std::string>("topic_params.segmentation_result_topic", Parameter::TopicParams::segmentation_result_topic);
            node_->declare_parameter<std::string>("topic_params.tracked_pose_topic", Parameter::TopicParams::tracked_pose_topic);

            node_->declare_parameter<double>("calib_params.camera_in_odo_x", Parameter::CalibParams::camera_in_odo_x);
            node_->declare_parameter<double>("calib_params.camera_in_odo_y", Parameter::CalibParams::camera_in_odo_y);
            node_->declare_parameter<double>("calib_params.camera_in_odo_z", Parameter::CalibParams::camera_in_odo_z);
            node_->declare_parameter<double>("calib_params.camera_in_odo_yaw", Parameter::CalibParams::camera_in_odo_yaw);
            node_->declare_parameter<double>("calib_params.camera_in_odo_pitch", Parameter::CalibParams::camera_in_odo_pitch);
            node_->declare_parameter<double>("calib_params.camera_in_odo_roll", Parameter::CalibParams::camera_in_odo_roll);

            node_->declare_parameter<bool>("depth_discontinuity_map_params.use_discontinuity", Parameter::DepthDiscontinuityMapParams::use_discontinuity);
            node_->declare_parameter<int>("depth_discontinuity_map_params.kernel_size", Parameter::DepthDiscontinuityMapParams::kernel_size);
            node_->declare_parameter<double>("depth_discontinuity_map_params.discontinuity_ratio", Parameter::DepthDiscontinuityMapParams::discontinuity_ratio);

            node_->declare_parameter<int>("final_edge_map_params.morphological_opening_size", Parameter::FinalEdgeMapParams::morphological_opening_size);
            node_->declare_parameter<int>("final_edge_map_params.morphological_closing_size", Parameter::FinalEdgeMapParams::morphological_closing_size);
            node_->declare_parameter<bool>("final_edge_map_params.use_morphological_opening", Parameter::FinalEdgeMapParams::use_morphological_opening);
            node_->declare_parameter<bool>("final_edge_map_params.use_morphological_closing", Parameter::FinalEdgeMapParams::use_morphological_closing);

            node_->declare_parameter<int>("label_map_params.label_map_method", (int)Parameter::LabelMapParams::method);
            node_->declare_parameter<int>("label_map_params.min_size", Parameter::LabelMapParams::min_size);

            node_->declare_parameter<bool>("min_convexity_map_params.use_min_convexity", Parameter::MinConvexityMapParams::use_min_convexity);
            node_->declare_parameter<int>("min_convexity_map_params.morphological_opening_size", Parameter::MinConvexityMapParams::morphological_opening_size);
            node_->declare_parameter<int>("min_convexity_map_params.window_size", Parameter::MinConvexityMapParams::window_size);
            node_->declare_parameter<int>("min_convexity_map_params.step_size", Parameter::MinConvexityMapParams::step_size);
            node_->declare_parameter<bool>("min_convexity_map_params.use_morphological_opening", Parameter::MinConvexityMapParams::use_morphological_opening);
            node_->declare_parameter<bool>("min_convexity_map_params.use_threshold", Parameter::MinConvexityMapParams::use_threshold);
            node_->declare_parameter<double>("min_convexity_map_params.threshold", Parameter::MinConvexityMapParams::threshold);
            node_->declare_parameter<double>("min_convexity_map_params.mask_threshold", Parameter::MinConvexityMapParams::mask_threshold);
            node_->declare_parameter<bool>("min_convexity_map_params.if_use_gpu", Parameter::MinConvexityMapParams::if_use_gpu);

            node_->declare_parameter<double>("normal_map_params.distance_factor_threshold", Parameter::NormalMapParams::distance_factor_threshold);
            node_->declare_parameter<int>("normal_map_params.window_size", Parameter::NormalMapParams::window_size);
            node_->declare_parameter<bool>("normal_map_params.if_use_gpu", Parameter::NormalMapParams::if_use_gpu);

            node_->declare_parameter<bool>("semantic_instance_segmentation_params.if_show_window", Parameter::SemanticInstanceSegmentationParams::if_show_window);
            node_->declare_parameter<double>("semantic_instance_segmentation_params.overlap_threshold", Parameter::SemanticInstanceSegmentationParams::overlap_threshold);

            node_->declare_parameter<double>("common_params.cluster_distance", Parameter::CommonParams::cluster_distance);
            node_->declare_parameter<std::string>("common_params.save_path", Parameter::CommonParams::save_path);
            node_->declare_parameter<std::string>("common_params.nav_map_path", Parameter::CommonParams::nav_map_path);

            node_->declare_parameter<double>("semantic_integrator_params.score_limit", Parameter::SemanticIntegratorParams::score_limit);
            node_->declare_parameter<int>("semantic_integrator_params.nums_limit", Parameter::SemanticIntegratorParams::nums_limit);
            node_->declare_parameter<double>("semantic_integrator_params.fusion_ratio_limit", Parameter::SemanticIntegratorParams::fusion_ratio_limit);
            node_->declare_parameter<int>("semantic_integrator_params.obs_pass_limit", Parameter::SemanticIntegratorParams::obs_pass_limit);
            node_->declare_parameter<double>("semantic_integrator_params.obbbox_fusion_ratio_limit", Parameter::SemanticIntegratorParams::obbbox_fusion_ratio_limit);

            std::string semantic_label_str = "sink,toilet";
            node_->declare_parameter<std::string>("semantic_integrator_params.semantic_label_vec", semantic_label_str);
        }

        void setParameter(){
            node_->get_parameter("topic_params.depth_image_topic", Parameter::TopicParams::depth_image_topic);
            node_->get_parameter("topic_params.depth_camera_info_topic", Parameter::TopicParams::depth_camera_info_topic);
            node_->get_parameter("topic_params.segmentation_result_topic", Parameter::TopicParams::segmentation_result_topic);
            node_->get_parameter("topic_params.tracked_pose_topic", Parameter::TopicParams::tracked_pose_topic);

            node_->get_parameter("calib_params.camera_in_odo_x", Parameter::CalibParams::camera_in_odo_x);
            node_->get_parameter("calib_params.camera_in_odo_y", Parameter::CalibParams::camera_in_odo_y);
            node_->get_parameter("calib_params.camera_in_odo_z", Parameter::CalibParams::camera_in_odo_z);
            node_->get_parameter("calib_params.camera_in_odo_yaw", Parameter::CalibParams::camera_in_odo_yaw);
            node_->get_parameter("calib_params.camera_in_odo_pitch", Parameter::CalibParams::camera_in_odo_pitch);
            node_->get_parameter("calib_params.camera_in_odo_roll", Parameter::CalibParams::camera_in_odo_roll);

            node_->get_parameter("depth_discontinuity_map_params.use_discontinuity", Parameter::DepthDiscontinuityMapParams::use_discontinuity);
            node_->get_parameter("depth_discontinuity_map_params.kernel_size", Parameter::DepthDiscontinuityMapParams::kernel_size);
            node_->get_parameter("depth_discontinuity_map_params.discontinuity_ratio", Parameter::DepthDiscontinuityMapParams::discontinuity_ratio);

            node_->get_parameter("final_edge_map_params.morphological_opening_size", Parameter::FinalEdgeMapParams::morphological_opening_size);
            node_->get_parameter("final_edge_map_params.morphological_closing_size", Parameter::FinalEdgeMapParams::morphological_closing_size);
            node_->get_parameter("final_edge_map_params.use_morphological_opening", Parameter::FinalEdgeMapParams::use_morphological_opening);
            node_->get_parameter("final_edge_map_params.use_morphological_closing", Parameter::FinalEdgeMapParams::use_morphological_closing);

            node_->get_parameter("label_map_params.label_map_method",  Parameter::LabelMapParams::method);
            node_->get_parameter("label_map_params.min_size", Parameter::LabelMapParams::min_size);

            node_->get_parameter("min_convexity_map_params.use_min_convexity", Parameter::MinConvexityMapParams::use_min_convexity);
            node_->get_parameter("min_convexity_map_params.morphological_opening_size", Parameter::MinConvexityMapParams::morphological_opening_size);
            node_->get_parameter("min_convexity_map_params.window_size", Parameter::MinConvexityMapParams::window_size);
            node_->get_parameter("min_convexity_map_params.step_size", Parameter::MinConvexityMapParams::step_size);
            node_->get_parameter("min_convexity_map_params.use_morphological_opening", Parameter::MinConvexityMapParams::use_morphological_opening);
            node_->get_parameter("min_convexity_map_params.use_threshold", Parameter::MinConvexityMapParams::use_threshold);
            node_->get_parameter("min_convexity_map_params.threshold", Parameter::MinConvexityMapParams::threshold);
            node_->get_parameter("min_convexity_map_params.mask_threshold", Parameter::MinConvexityMapParams::mask_threshold);
            node_->get_parameter("min_convexity_map_params.if_use_gpu", Parameter::MinConvexityMapParams::if_use_gpu);

            node_->get_parameter("normal_map_params.distance_factor_threshold", Parameter::NormalMapParams::distance_factor_threshold);
            node_->get_parameter("normal_map_params.window_size", Parameter::NormalMapParams::window_size);
            node_->get_parameter("normal_map_params.if_use_gpu", Parameter::NormalMapParams::if_use_gpu);

            node_->get_parameter("semantic_instance_segmentation_params.if_show_window", Parameter::SemanticInstanceSegmentationParams::if_show_window);
            node_->get_parameter("semantic_instance_segmentation_params.overlap_threshold", Parameter::SemanticInstanceSegmentationParams::overlap_threshold);

            node_->get_parameter("common_params.cluster_distance", Parameter::CommonParams::cluster_distance);
            node_->get_parameter("common_params.save_path", Parameter::CommonParams::save_path);
            node_->get_parameter("common_params.nav_map_path", Parameter::CommonParams::nav_map_path);

            node_->get_parameter("semantic_integrator_params.score_limit", Parameter::SemanticIntegratorParams::score_limit);
            node_->get_parameter("semantic_integrator_params.nums_limit", Parameter::SemanticIntegratorParams::nums_limit);
            node_->get_parameter("semantic_integrator_params.fusion_ratio_limit", Parameter::SemanticIntegratorParams::fusion_ratio_limit);
            node_->get_parameter("semantic_integrator_params.obs_pass_limit", Parameter::SemanticIntegratorParams::obs_pass_limit);
            node_->get_parameter("semantic_integrator_params.obbbox_fusion_ratio_limit", Parameter::SemanticIntegratorParams::obbbox_fusion_ratio_limit);


            std::string semantic_label_str_tmp;
            node_->get_parameter("semantic_integrator_params.semantic_label_vec", semantic_label_str_tmp);

            std::vector<std::string> semantic_label_str_vec;
            SplitString(semantic_label_str_tmp,",",semantic_label_str_vec);
            for(const auto& item:semantic_label_str_vec){
                Parameter::SemanticIntegratorParams::semantic_label_vec.push_back(item);
            }
        }

        static void configValueLogSave(){
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"topic_params.depth_image_topic: %s",Parameter::TopicParams::depth_image_topic.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"topic_params.depth_camera_info_topic: %s",Parameter::TopicParams::depth_camera_info_topic.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"topic_params.segmentation_result_topic: %s",Parameter::TopicParams::segmentation_result_topic.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"topic_params.tracked_pose_topic: %s",Parameter::TopicParams::tracked_pose_topic.c_str());

            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"calib_params.camera_in_odo_x: %f",Parameter::CalibParams::camera_in_odo_x);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"calib_params.camera_in_odo_y: %f",Parameter::CalibParams::camera_in_odo_y);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"calib_params.camera_in_odo_z: %f",Parameter::CalibParams::camera_in_odo_z);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"calib_params.camera_in_odo_yaw: %f",Parameter::CalibParams::camera_in_odo_yaw);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"calib_params.camera_in_odo_pitch: %f",Parameter::CalibParams::camera_in_odo_pitch);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"calib_params.camera_in_odo_roll: %f",Parameter::CalibParams::camera_in_odo_roll);

            std::string use_discontinuity_str = "false";
            if(Parameter::DepthDiscontinuityMapParams::use_discontinuity){
                use_discontinuity_str = "true";
            }

            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"depth_discontinuity_map_params.use_discontinuity: %s",use_discontinuity_str.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"depth_discontinuity_map_params.kernel_size: %d",Parameter::DepthDiscontinuityMapParams::kernel_size);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"depth_discontinuity_map_params.discontinuity_ratio: %f",Parameter::DepthDiscontinuityMapParams::discontinuity_ratio);

            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"final_edge_map_params.morphological_opening_size: %d",Parameter::FinalEdgeMapParams::morphological_opening_size);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"final_edge_map_params.morphological_closing_size: %d",Parameter::FinalEdgeMapParams::morphological_closing_size);
            std::string use_morphological_opening_str = "false";
            if(Parameter::FinalEdgeMapParams::use_morphological_opening){
                use_morphological_opening_str = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"final_edge_map_params.use_morphological_opening: %s",use_morphological_opening_str.c_str());
            std::string use_morphological_closing_str = "false";
            if(Parameter::FinalEdgeMapParams::use_morphological_closing){
                use_morphological_closing_str = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"final_edge_map_params.use_morphological_closing: %s",use_morphological_closing_str.c_str());

            std::string method = "kFloodFill";
            if(Parameter::LabelMapParams::method == 1){
                method = "kContour";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"label_map_params.label_map_method: %s",method.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"label_map_params.min_size: %d", Parameter::LabelMapParams::min_size);


            std::string use_min_convexity_str = "false";
            if(Parameter::MinConvexityMapParams::use_min_convexity){
                use_min_convexity_str = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.use_min_convexity: %s", use_min_convexity_str.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.morphological_opening_size: %d", Parameter::MinConvexityMapParams::morphological_opening_size);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.window_size: %d", Parameter::MinConvexityMapParams::window_size);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.step_size: %d", Parameter::MinConvexityMapParams::step_size);

            std::string use_morphological_opening_str2 = "false";
            if(Parameter::MinConvexityMapParams::use_morphological_opening){
                use_morphological_opening_str2 = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.use_morphological_opening: %s",use_morphological_opening_str2.c_str());

            std::string use_threshold_str = "false";
            if(Parameter::MinConvexityMapParams::use_threshold){
                use_threshold_str = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.use_threshold: %s",use_threshold_str.c_str());

            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.threshold: %f", Parameter::MinConvexityMapParams::threshold);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.mask_threshold: %f", Parameter::MinConvexityMapParams::mask_threshold);

            std::string if_use_gpu_str = "false";
            if(Parameter::MinConvexityMapParams::if_use_gpu){
                if_use_gpu_str = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"min_convexity_map_params.if_use_gpu: %s", if_use_gpu_str.c_str());

            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"normal_map_params.distance_factor_threshold: %f", Parameter::NormalMapParams::distance_factor_threshold);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"normal_map_params.window_size: %d", Parameter::NormalMapParams::window_size);
            std::string if_use_gpu_str2 = "false";
            if(Parameter::NormalMapParams::if_use_gpu){
                if_use_gpu_str2 = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"normal_map_params.if_use_gpu: %s", if_use_gpu_str2.c_str());


            std::string if_show_window_str = "false";
            if(Parameter::SemanticInstanceSegmentationParams::if_show_window){
                if_show_window_str = "true";
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_instance_segmentation_params.if_show_window: %s", if_show_window_str.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_instance_segmentation_params.overlap_threshold: %f", Parameter::SemanticInstanceSegmentationParams::overlap_threshold);

            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"common_params.cluster_distance: %f", Parameter::CommonParams::cluster_distance);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"common_params.save_path: %s", Parameter::CommonParams::save_path.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"common_params.nav_map_path: %s", Parameter::CommonParams::nav_map_path.c_str());


            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_integrator_params.score_limit: %f", Parameter::SemanticIntegratorParams::score_limit);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_integrator_params.nums_limit: %d", Parameter::SemanticIntegratorParams::nums_limit);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_integrator_params.fusion_ratio_limit: %f", Parameter::SemanticIntegratorParams::fusion_ratio_limit);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_integrator_params.obs_pass_limit: %d", Parameter::SemanticIntegratorParams::obs_pass_limit);
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_integrator_params.obbbox_fusion_ratio_limit: %f", Parameter::SemanticIntegratorParams::obbbox_fusion_ratio_limit);

            std::string semantic_str;
            for(const auto& item:Parameter::SemanticIntegratorParams::semantic_label_vec){
                semantic_str += (item + ",");
            }
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"semantic_integrator_params.semantic_label_vec: %s", semantic_str.c_str());

        }

        //像素坐标系转换到cartographer地图坐标系
        Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point){
            return Eigen::Array2i(
                    RoundToInt((max_.y() - point.y()) / resolution_ - 0.5 ),
                    RoundToInt((max_.x() - point.x()) / resolution_ - 0.5 ));
        }

        //检测是否包含在limits里面
        bool Contains(const Eigen::Array2i& cell_index){
            return (Eigen::Array2i(0, 0) <= cell_index).all() &&
                   (cell_index < Eigen::Array2i(num_x_cells_, num_y_cells_)).all();
        }




        void markerVecPublish();

        static void createMarkerText(const std::vector<semantic_integrator::OBBBox>& obbbox_vec, std::vector<std::shared_ptr<visualization_msgs::msg::Marker>>& marker_msg_vec);

        static void createMarkerLineStripMsg(const std::vector<semantic_integrator::OBBBox>& obbbox_vec, std::vector<std::shared_ptr<visualization_msgs::msg::Marker>>& marker_msg_vec);

        static int saveSematicMap(const std::string& file_path,const std::vector<semantic_integrator::OBBBox>& obbbox_vec);

        int changeMapWithOccpuiedValue(const std::string& file_path,const std::vector<semantic_integrator::OBBBox>& obb_box_vec);

        int readNavMap(const std::string& file_path);

        void control_callback(control_msg::srv::Control::Request::SharedPtr request, control_msg::srv::Control::Response::SharedPtr response);

        void depth_result_pose_callback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                                        const segment_msg::msg::Result::ConstSharedPtr &segmentation_msg,
                                        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_stamped_msg){
            set_depth_segment_posestamped(depth_msg,segmentation_msg,pose_stamped_msg);
        }

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depth_camera_info_msg);

        void set_depth_segment_posestamped(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg_ptr,
                                                     const segment_msg::msg::Result::ConstSharedPtr& result_msg_ptr,
                                                     const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_stamped_ptr){
            std::lock_guard<std::mutex> lock(data_mutx);
            current_depth_msg_ptr_ = depth_msg_ptr;
            current_instance_segmentation_msg_ptr_ = result_msg_ptr;
            current_pose_stamped_msg_ptr_ = pose_stamped_ptr;
        }

        void get_depth_segment_posestamped(sensor_msgs::msg::Image::ConstSharedPtr& depth_msg_ptr,
                                                     segment_msg::msg::Result::ConstSharedPtr& result_msg_ptr,
                                                     geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_stamped_ptr){
            std::lock_guard<std::mutex> lock(data_mutx);
            depth_msg_ptr = current_depth_msg_ptr_;
            result_msg_ptr = current_instance_segmentation_msg_ptr_;
            pose_stamped_ptr = current_pose_stamped_msg_ptr_;
        }

        // 预处理操作
        /*
            depth_msg：传入的深度msg
            rescaled_depth：转换成以m为单位的浮点数深度图矩阵
        */
        static void depthMatFromRosMsg(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg, cv::Mat *rescaled_depth);

        //解析位姿信息
        static void PoseStampedFromRosMsg(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_stamped_ptr, Localization::Pose2D& pose);

        //解析
        static void semanticInstanceSegmentationFromRosMsg(const segment_msg::msg::Result::ConstSharedPtr &segmentation_msg,
                                                    depth_segmentation::SemanticInstanceSegmentation *semantic_instance_segmentation);

        /*
         * rescaled_depth：深度图(nan值已经转成0) depth_map ： 点云数据  normal_map ： 法向量图  edge_map ： 边缘图
         */
        void computeEdgeMap(cv::Mat &rescaled_depth, cv::Mat *depth_map, cv::Mat *normal_map, cv::Mat *edge_map);

        void dataHandleLoop();

        static void sendPointCloud2(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_ptr,
                             const std_msgs::msg::Header& header);

        // 发布分割
        /*void publish_segments(const std::vector<depth_segmentation::Segment> &segments, const std_msgs::msg::Header &header,const Localization::Pose3D& odo_pose_3d) {
            CHECK_GT(segments.size(), 0u);

            static pcl::PointCloud<pcl::PointXYZRGB>::Ptr segemented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (depth_segmentation::Segment segment: segments) {
                if (*(segment.instance_label.begin()) == 0) {
                    continue;
                }
                CHECK_GT(segment.points.size(), 0u);

                for (std::size_t i = 0u; i < segment.points.size(); ++i) {
                    uint8_t semantic_label = 0u;
                    uint32_t instance_label = 0u;
                    float score = 0.0f;
                    if (segment.instance_label.size() > 0u) {
                        // 取当前分割类的label
                        instance_label = *(segment.instance_label.begin());
                        semantic_label = *(segment.semantic_label.begin());
                        score = *(segment.scores.begin());
                    }

                    Localization::Pos3D pos = Localization::Pos3D(segment.points[i](0),segment.points[i](1),segment.points[i](2));
                    Localization::Pos3D trans_pos = odo_pose_3d * calib_odo_to_cam_ * pos;

                    pcl::PointXYZRGB pt;
                    pt.x = trans_pos._xyz[0];
                    pt.y = trans_pos._xyz[1];
                    pt.z = trans_pos._xyz[2];
                    pt.r = 255;
                    pt.g = 255;
                    pt.b = 255;
                    segemented_cloud->push_back(pt);

                }



            }

            us_filter_.setInputCloud(segemented_cloud);
            us_filter_.filter(*segemented_cloud);

            std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud2_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*segemented_cloud, *pointcloud2_ptr);

            pointcloud2_ptr->header = header;
            pointcloud2_ptr->header.frame_id = "map";
            segmented_pointcloud_publisher_->publish(*pointcloud2_ptr);

        }*/

        void segmentationObjectReceive(const std::vector<depth_segmentation::Segment>& segments,
                                       const Localization::Pose3D& global_pose,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& semantic_cloud_ptr,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr& instance_cloud_ptr);

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> depth_info_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_image_sub_;
        std::shared_ptr<message_filters::Subscriber<segment_msg::msg::Result>> instance_segmentation_sub_;
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> tracked_pose_sub_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pointcloud_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr instance_pointcloud_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;


        // 时间同步 depth result 适配器所需类型
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, segment_msg::msg::Result, geometry_msgs::msg::PoseStamped> ImageSegmentationSyncPolicy;
        // 时间同步器指针
        std::shared_ptr<message_filters::Synchronizer<ImageSegmentationSyncPolicy>> image_segmentation_sync_policy_ptr_;

        std::mutex data_mutx;
        sensor_msgs::msg::Image::ConstSharedPtr current_depth_msg_ptr_;
        double last_depth_msg_time_ = -1;

        segment_msg::msg::Result::ConstSharedPtr current_instance_segmentation_msg_ptr_;
        double last_segmentation_msg_time_ = -1;

        geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_stamped_msg_ptr_;
        double last_pose_stamped_msg_time_ = -1;

        Localization::Pose3D calib_odo_to_cam_;

        depth_segmentation::DepthCamera depth_camera_;
        std::shared_ptr<depth_segmentation::DepthSegmenter> depth_segmenter_;

        bool camera_info_ready_;

        std::shared_ptr<semantic_integrator::SemanticIntegrator> semantic_integrator_;

        rclcpp::Service<control_msg::srv::Control>::SharedPtr control_service_;
        //tacked pose 定时器
        rclcpp::TimerBase::SharedPtr marker_publisher_timer_;

        std::mutex marker_mutex_;
        std::vector<std::shared_ptr<visualization_msgs::msg::Marker>> marker_msg_vec_;

        rclcpp::CallbackGroup::SharedPtr marker_group_;


        double resolution_ = 0;      //分辨率
        Eigen::Vector2f max_;    // cartographer地图坐标系左上角为坐标系的坐标的最大值
        Eigen::Vector2f min_;
        int num_x_cells_ = 0;        //x方向的格子数
        int num_y_cells_ = 0;        //y方向的格子数

    };


}

#endif //SEMANTIC_MAPPING_SEMANTIC_MAPPING_H