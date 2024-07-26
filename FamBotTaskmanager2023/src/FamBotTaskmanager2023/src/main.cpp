#include "FamBotTaskmanager.h"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);


    // 使用智能指针实例化VisionCommunication对象
    std::shared_ptr<VisionCommunication> vision_communication = std::make_shared<VisionCommunication>();

    FamBotTaskmanager task_manager(vision_communication);
    task_manager.SyncTaskRun();


    rclcpp::shutdown();
    return 0;
}