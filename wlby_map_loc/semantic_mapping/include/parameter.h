#ifndef _WLBY_PARAMETER_H_
#define _WLBY_PARAMETER_H_
/*******************************************************************************
* @brief 参数设置
* @author jazzey
* @date 2023-11-14
********************************************************************************/
#include <string>
#include <vector>

namespace Parameter {

    //订阅的话题
    namespace TopicParams {
        extern std::string depth_camera_info_topic;
        extern std::string depth_image_topic;
        extern std::string segmentation_result_topic;
        extern std::string tracked_pose_topic;
    }

    namespace CalibParams{
        extern double camera_in_odo_x;
        extern double camera_in_odo_y;
        extern double camera_in_odo_z;
        extern double camera_in_odo_yaw;
        extern double camera_in_odo_pitch;
        extern double camera_in_odo_roll;
    }

    namespace DepthDiscontinuityMapParams {
        //是否使用边缘检测
        extern bool use_discontinuity;
        extern int kernel_size;
        extern double discontinuity_ratio;
    }

    namespace FinalEdgeMapParams{
        //窗口大小
        extern int morphological_opening_size;
        extern int morphological_closing_size;
        //对最终的凸性图是否进行开运算
        extern bool use_morphological_opening;
        //对最终的不连续图是否进行闭运算
        extern bool use_morphological_closing;
    }

    namespace LabelMapParams {
        //填充使用的模式
        extern int method;
        //小于此阈值会丢弃分割
        extern int min_size;
    }

    namespace MinConvexityMapParams{
        extern bool use_min_convexity;
        extern int morphological_opening_size;
        extern int window_size;
        extern int step_size;
        extern bool use_morphological_opening;
        extern bool use_threshold;
        extern double threshold;
        extern double mask_threshold;
        extern bool if_use_gpu;
    }

    namespace NormalMapParams{
        extern bool if_use_gpu;
        extern int window_size;
        extern double distance_factor_threshold;
    }

    namespace SemanticInstanceSegmentationParams {
        extern bool if_show_window;
        extern double overlap_threshold;
    }

    namespace CommonParams {
        extern double cluster_distance;
        extern std::string save_path;
        extern std::string nav_map_path;
    }

    namespace SemanticIntegratorParams{
        //低于此阈值的类别会pass
        extern double score_limit;

        //低于此阈值的格子数类别会pass
        extern int nums_limit;

        //超过此ratio的会融合
        extern double fusion_ratio_limit;

        //观测次数：小于此观测次数的将会pass
        extern int obs_pass_limit;

        extern std::vector<std::string> semantic_label_vec;

        extern double obbbox_fusion_ratio_limit;

    }

}



#endif