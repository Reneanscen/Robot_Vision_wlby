//
// Created by jazzey on 2023/11/14.
//

#include "yolov8_ncnn_node.h"

namespace wlby{
    void RunYoloV8(){

        //创建运行的节点
        rclcpp::Node::SharedPtr yolov8_node = rclcpp::Node::make_shared("yolov8_node");

        auto node = std::make_shared<yolo::YoloV8NcnnNode>(yolov8_node);

        rclcpp::spin(yolov8_node);
    }
}

int main(int argc, char **argv)
{
    log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"YoloV8 Version: 0.0.0");

    rclcpp::init(argc, argv);

    wlby::RunYoloV8();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
