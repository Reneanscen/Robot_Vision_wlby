//
// Created by xu on 24-3-21.
//
#include "bird_vision/bird_vision.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("BirdVision");

    auto BirdVisionNode = std::make_shared<BirdVision>(node);

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;

}