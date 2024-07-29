//
// Created by jazzey on 2023/11/14.
//
#ifndef YOLOV8_NCNN_YOLO_H
#define YOLOV8_NCNN_YOLO_H

#include <iostream>

#include <opencv2/core/core.hpp>

#include <ncnn/net.h>

#include "variables.h"

#include "logger/my_logger.h"

namespace wlby::yolo{

    struct Object
    {
        cv::Rect_<float> rect;
        int label_int;
        std::string label_str;
        float prob;
        cv::Mat mask;
        std::vector<float> mask_feat;
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    class Yolo
    {
    public:
        Yolo(const std::string& param_path, const std::string& bin_path, int target_size, const float* mean_vals, const float* norm_vals, bool use_gpu = true){

            yolo_.clear();
            blob_pool_allocator_.clear();
            workspace_pool_allocator_.clear();

            blob_pool_allocator_.set_size_compare_ratio(0.f);
            workspace_pool_allocator_.set_size_compare_ratio(0.f);
            param_path_ = param_path;
            bin_path_ = bin_path;

            log__save("Yolo", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem,"load param path %s", param_path_.c_str());
            log__save("Yolo", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem,"load bin path %s", bin_path_.c_str());

            yolo_.opt = ncnn::Option();
            yolo_.opt.use_vulkan_compute = use_gpu;
            yolo_.opt.num_threads = 2;
            yolo_.opt.blob_allocator = &blob_pool_allocator_;
            yolo_.opt.workspace_allocator = &workspace_pool_allocator_;

            yolo_.load_param(param_path_.c_str());
            yolo_.load_model(bin_path_.c_str());

            target_size_ = target_size;
            mean_vals_[0] = mean_vals[0];
            mean_vals_[1] = mean_vals[1];
            mean_vals_[2] = mean_vals[2];
            norm_vals_[0] = norm_vals[0];
            norm_vals_[1] = norm_vals[1];
            norm_vals_[2] = norm_vals[2];

        }

        int detect(const cv::Mat& rgb, std::vector<Object>& objects, float prob_threshold = 0.4f, float nms_threshold = 0.5f);

        int draw(cv::Mat& rgb, const std::vector<Object>& objects);

    private:
        ncnn::Net yolo_;

        int target_size_ = 640;
        float mean_vals_[3] = {0};
        float norm_vals_[3] = {0};

        std::string param_path_;
        std::string bin_path_;

        ncnn::UnlockedPoolAllocator blob_pool_allocator_;
        ncnn::PoolAllocator workspace_pool_allocator_;
    };


}


#endif //YOLOV8_NCNN_YOLO_H
