#include "parameter.h"


namespace Parameter {

    namespace TopicParams {
        std::string rgb_image_topic = "/camera/color/image_raw";
        std::string segmentation_result_topic = "/yolov8/result";
    }

    namespace WeightParams {
        std::string weight_param_path = "./yolo.param";
        std::string weight_bin_path = "./yolo.bin";
    }

}