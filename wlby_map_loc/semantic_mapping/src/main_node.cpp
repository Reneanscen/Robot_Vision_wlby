//
// Created by jazzey on 2023/11/14.
//

#include "semantic_mapping_node.h"

namespace wlby{
    void RunSemanticMapping(){

        //创建运行的节点
        rclcpp::Node::SharedPtr semantic_mapping_node = rclcpp::Node::make_shared("semantic_mapping_node");
        auto node = std::make_shared<SemanticMappingNode>(semantic_mapping_node);
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(semantic_mapping_node);
        executor.spin();
    }
}

int main(int argc, char **argv)
{
    log__save("ParamRecord",kLogLevel_Info,kLogTarget_Stdout|kLogTarget_Filesystem,"Semantic Mapping Version: 0.0.0");

    rclcpp::init(argc, argv);

    wlby::RunSemanticMapping();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}


