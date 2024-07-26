#include "VisionCommunication.h"

VisionCommunication::VisionCommunication()  : Node("vision_communication_node"){
    cmd_subscription_ = this->create_subscription<cv_msgs::msg::CVinstructions>(
        "vision_command_topic",  // Replace with the actual topic name
        10,                 // Queue size
        std::bind(&VisionCommunication::commandCallback, this, std::placeholders::_1)
    );
}

VisionCommunication::~VisionCommunication() {

}


cv_msgs::msg::CVinstructions VisionCommunication::GetLatestCmdCondition()
    {
        return current_cmd_;
    }

void VisionCommunication::commandCallback(const cv_msgs::msg::CVinstructions::SharedPtr msg) {
    current_cmd_ = *msg;
    //CheckShotCondition(msg);
}
