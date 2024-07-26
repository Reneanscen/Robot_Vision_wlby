#pragma once

#include "rclcpp/rclcpp.hpp"
//#include "robot_cv_msg/msg/CVinstructions.hpp"
#include "cv_msgs/msg/c_vinstructions.hpp"


class VisionCommunication  : public rclcpp::Node {
public:
    VisionCommunication();
    ~VisionCommunication();



    cv_msgs::msg::CVinstructions GetLatestCmdCondition();

private:
    void commandCallback(const cv_msgs::msg::CVinstructions::SharedPtr msg);

    rclcpp::Subscription<cv_msgs::msg::CVinstructions>::SharedPtr cmd_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv_msgs::msg::CVinstructions current_cmd_;
};
