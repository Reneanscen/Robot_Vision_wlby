//
// Created by xu on 24-5-21.
//

#ifndef ZME_BIRDVISION_PARAMETER_H
#define ZME_BIRDVISION_PARAMETER_H

#include "Eigen/Core"

class CamParameter{
public:

    CamParameter()=default;
    ~CamParameter()=default;

    Eigen::Matrix3d get_R_camera_baselink() const {
        auto result = R_baselink_camera_.inverse();
        return result;
    }

    Eigen::Vector3d get_t_camera_baselink() const {
        auto result = -1 * R_baselink_camera_.inverse() * t_baselink_camera_;
        return result;
    }

    bool open_{};
    int cam_type_{};
    int image_type_{};
    std::string topic_;
    Eigen::Matrix<double, 3, 3> intrinsic_matrix_;
    Eigen::Matrix<double, 1, Eigen::Dynamic> distort_matrix_; // 初始化的时候需要resize大小
    Eigen::Matrix<double, 3, 1> t_baselink_camera_;
    Eigen::Matrix<double, 3, 3> R_baselink_camera_;
    double x_, y_, z_;
    double roll_, pitch_, yaw_;
};

class BirdVisionParameter{
public:
    bool v4l2_stream_enable_;
    std::string front_cam_dev_;
    std::string left_cam_dev_;
    std::string back_cam_dev_;
    std::string right_cam_dev_;
    int stream_format_;
    int stream_width_;
    int stream_height_;
    int stream_fps_;


    bool is_save_images_;
    bool is_pub_test_images_;
    float height_;
    float width_;
    float resolution_;
    float blind_length_;
    float mask_reference_radio_;
    float HSV_V_;
    std::string assets_reference_path_;

    // 测试使用:
    int fusion_plan_;
    double sigmoid_;

};

#endif //ZME_BIRDVISION_PARAMETER_H
