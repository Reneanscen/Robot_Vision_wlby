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
    

    namespace TopicParams {
        
        //是否定位模式 or 建图模式
        extern std::string rgb_image_topic;
        extern std::string segmentation_result_topic;
    }

    namespace WeightParams {
        extern std::string weight_param_path;
        extern std::string weight_bin_path;
    }
    

}



#endif