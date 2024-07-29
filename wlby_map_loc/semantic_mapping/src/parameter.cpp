#include "parameter.h"


namespace Parameter {

    namespace TopicParams {
        std::string depth_camera_info_topic = "/camera/aligned_depth_to_color/camera_info";
        std::string depth_image_topic = "/camera/aligned_depth_to_color/image_raw";
        std::string segmentation_result_topic = "/yolov8/result";
        std::string tracked_pose_topic = "/tracked_pose";
    }

    namespace CalibParams{
        double camera_in_odo_x = 0.035140;
        double camera_in_odo_y = 0.028017;
        double camera_in_odo_z = 1.26;
        double camera_in_odo_yaw = -91.9603;
        double camera_in_odo_pitch = -0.201452;
        double camera_in_odo_roll = -122.733;
    }

    namespace DepthDiscontinuityMapParams {
        //是否使用边缘检测
        bool use_discontinuity = true;
        int kernel_size = 3;
        double discontinuity_ratio = 0.025;
    }

    namespace FinalEdgeMapParams{
        //窗口大小
        int morphological_opening_size = 1;
        int morphological_closing_size = 1;

        //对最终的凸性图是否进行开运算
        bool use_morphological_opening = true;
        //对最终的不连续图是否进行闭运算
        bool use_morphological_closing = false;
    }

    namespace  LabelMapParams {
        //填充使用的模式
        int method = 0;
        //小于此阈值会丢弃分割
        int min_size = 500u;
    }

    namespace MinConvexityMapParams {
        bool use_min_convexity = true;
        int morphological_opening_size = 1;
        int window_size = 5;
        int step_size = 1;
        bool use_morphological_opening = true;
        bool use_threshold = true;
        double threshold = 0.95;
        double mask_threshold = -0.0005;
        bool if_use_gpu = true;
    }

    namespace NormalMapParams{
        bool if_use_gpu = true;
        int window_size = 13;
        double distance_factor_threshold = 0.05;
    }

    namespace SemanticInstanceSegmentationParams {
        bool if_show_window = false;
        double overlap_threshold = 0.7f;
    }

    namespace CommonParams {
        double cluster_distance = 0.4;
        std::string save_path = "/home/jazzey/ros2_ws/WLBYMapLoc_ws/data/semantic_map.data";
        std::string nav_map_path = "/home/jazzey/ros2_ws/WLBYMapLoc_ws/data/map_new.data";
    }

    namespace SemanticIntegratorParams{
        //低于此阈值的类别会pass
        double score_limit = 0.3f;

        //低于此阈值的格子数类别会pass
        int nums_limit = 200;

        //超过此ratio的会融合
        double fusion_ratio_limit = 0.15f;

        //观测次数：小于此观测次数的将会pass
        int obs_pass_limit = 2;

        double obbbox_fusion_ratio_limit = 0.5f;

        //要建图的语义标签
        std::vector<std::string> semantic_label_vec;

    }


}