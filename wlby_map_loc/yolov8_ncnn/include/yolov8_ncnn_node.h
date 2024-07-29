//
// Created by jazzey on 2023/11/14.
//

#ifndef YOLOV8_NCNN_YOLOV8_NCNN_NODE_H
#define YOLOV8_NCNN_YOLOV8_NCNN_NODE_H

#include <mutex>
#include <iostream>
#include <thread>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "segment_msg/msg/result.hpp"

#include "parameter.h"

#include "yolo.h"

#include "common/sleep.h"
#include "common/time_utils.h"

#include "check/check.h"
#include "logger/my_logger.h"

namespace wlby::yolo{

    class YoloV8NcnnNode {
    public:
        explicit YoloV8NcnnNode(rclcpp::Node::SharedPtr node) : node_(std::move(node)){

            //申明参数
            declareParameter();
            //从配置中设置参数
            setParameter();
            //存储参数
            config_value_log_save();

            rgb_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(Parameter::TopicParams::rgb_image_topic, 10,
                                                                                 std::bind(&YoloV8NcnnNode::rgbImageCallback, this, std::placeholders::_1));

            pub_result_ = node_->create_publisher<segment_msg::msg::Result>(Parameter::TopicParams::segmentation_result_topic, 1);
            CHECK_NOT_NULL(rgb_image_sub_);
            CHECK_NOT_NULL(pub_result_);

            float mean[3] = {0.0, 0.0, 0.0};
            float norm[3] = {1/255.0, 1/255.0, 1/255.0};
            yolo_ptr_ = std::make_shared<Yolo>(Parameter::WeightParams::weight_param_path,Parameter::WeightParams::weight_bin_path,640, mean, norm, true);

            std::thread t1(&YoloV8NcnnNode::dataHandleLoop, this);
            t1.detach();


        }


    private:
        void declareParameter(){
            node_->declare_parameter<std::string>("topic_params.rgb_image_topic", Parameter::TopicParams::rgb_image_topic);
            node_->declare_parameter<std::string>("topic_params.segmentation_result_topic", Parameter::TopicParams::segmentation_result_topic);
            node_->declare_parameter<std::string>("weight_params.weight_param_path", Parameter::WeightParams::weight_param_path);
            node_->declare_parameter<std::string>("weight_params.weight_bin_path", Parameter::WeightParams::weight_bin_path);
        }

        void setParameter(){
            node_->get_parameter("topic_params.rgb_image_topic", Parameter::TopicParams::rgb_image_topic);
            node_->get_parameter("topic_params.segmentation_result_topic", Parameter::TopicParams::segmentation_result_topic);
            node_->get_parameter("weight_params.weight_param_path", Parameter::WeightParams::weight_param_path);
            node_->get_parameter("weight_params.weight_bin_path", Parameter::WeightParams::weight_bin_path);
        }

        static void config_value_log_save(){
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"topic_params.rgb_image_topic: %s",Parameter::TopicParams::rgb_image_topic.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"topic_params.segmentation_result_topic: %s",Parameter::TopicParams::segmentation_result_topic.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"weight_params.weight_param_path: %s",Parameter::WeightParams::weight_param_path.c_str());
            log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"weight_params.weight_bin_path: %s",Parameter::WeightParams::weight_bin_path.c_str());
        }

        void setRGBImagePtr(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_ptr){
            std::lock_guard<std::mutex> lock(rgb_mutx);
            current_rgb_image_msg_ = rgb_image_ptr;
        }

        void getRGBImagePtr(sensor_msgs::msg::Image::ConstSharedPtr& rgb_image_ptr){
            std::lock_guard<std::mutex> lock(rgb_mutx);
            rgb_image_ptr = current_rgb_image_msg_;
        }

        void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
            setRGBImagePtr(msg);
        }

        void dataHandleLoop();

    private:
        rclcpp::Node::SharedPtr node_;

        std::mutex rgb_mutx;
        sensor_msgs::msg::Image::ConstSharedPtr current_rgb_image_msg_;
        double last_rgb_image_msg_time_ = -1;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;

        std::shared_ptr<Yolo> yolo_ptr_;

        rclcpp::Publisher<segment_msg::msg::Result>::SharedPtr pub_result_;

    };


}

#endif //YOLOV8_NCNN_YOLOV8_NCNN_NODE_H

