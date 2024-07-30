//
// Created by xu on 24-3-30.
//

#ifndef BIRDVIEW_FISHEYE_MODEL_H
#define BIRDVIEW_FISHEYE_MODEL_H

#include "opencv2/opencv.hpp"
#include "Eigen/Core"

namespace FishEyeModel {

    // 3D点投影到鱼眼相机内
    inline void fish_project_point(const Eigen::Vector3d &point3d,
                                   const Eigen::Matrix3d &R_cam_world,
                                   const Eigen::Vector3d &t_cam_world,
                                   const Eigen::Matrix3d &intrinsic,
                                   const Eigen::Matrix<double, 1, 4> &distort_coff,
                                   Eigen::Vector2d &distort_pixel) {

        const double fx = intrinsic(0, 0) * 1.0;
        const double fy = intrinsic(1, 1) * 1.0;
        const double cx = intrinsic(0, 2);
        const double cy = intrinsic(1, 2);

        const double k1 = distort_coff(0, 0);
        const double k2 = distort_coff(0, 1);
        const double k3 = distort_coff(0, 2);
        const double k4 = distort_coff(0, 3);

        // 3D点转换到相机坐标系下
        Eigen::Vector3d point3d_cam = R_cam_world * point3d + t_cam_world;

        // 转换到归一化平面坐标系
        Eigen::Vector3d point3d_cam_norm = Eigen::Vector3d(point3d_cam.x() / point3d_cam.z(),
                                                           point3d_cam.y() / point3d_cam.z(), 1.);

        // 归一化平面到鱼眼相机像素平面映射 （畸变正映射 + 内参矩阵映射）
        const double x2_plus_y2 =
                point3d_cam_norm.x() * point3d_cam_norm.x() + point3d_cam_norm.y() * point3d_cam_norm.y();
        const double theta = atan2f(sqrtf(x2_plus_y2), point3d_cam_norm.z());
        const double psi = atan2f(point3d_cam_norm.y(), point3d_cam_norm.x());

        const double theta2 = theta * theta;
        const double theta3 = theta * theta2;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        const double theta9 = theta7 * theta2;

        const double r = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;

        distort_pixel.x() = fx * r * cos(psi) + cx;
        distort_pixel.y() = fy * r * sin(psi) + cy;

    }

// 鱼眼相机去畸变
    inline void fish_undistort(const Eigen::Vector2i &undistort_pixel,
                               const Eigen::Matrix3d &intrinsic,
                               const Eigen::Matrix<double, 1, 4> &distort_coffess,
                               Eigen::Vector2d &distort_pixel) {
        const double fx = intrinsic(0, 0);
        const double fy = intrinsic(1, 1);
        const double cx = intrinsic(0, 2);
        const double cy = intrinsic(1, 2);

        const double k1 = distort_coffess(0, 0);
        const double k2 = distort_coffess(0, 1);
        const double k3 = distort_coffess(0, 2);
        const double k4 = distort_coffess(0, 3);

        // 当前去畸变点进行归一化平面投影
        Eigen::Vector2d pixel_normal;
        pixel_normal.x() = (undistort_pixel.x() - cx) / fx;
        pixel_normal.y() = (undistort_pixel.y() - cy) / fy;

        // 归一化平面到鱼眼相机像素平面映射 （畸变正映射 + 内参矩阵映射）
        const double x2_plus_y2 = pixel_normal.x() * pixel_normal.x() + pixel_normal.y() * pixel_normal.y();
        const double theta = atan2f(sqrtf(x2_plus_y2), 1);
        const double psi = atan2f(pixel_normal.y(), pixel_normal.x());

        const double theta2 = theta * theta;
        const double theta3 = theta * theta2;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        const double theta9 = theta7 * theta2;

        const double r = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;

        distort_pixel.x() = fx * r * cos(psi) + cx;
        distort_pixel.y() = fy * r * sin(psi) + cy;
    }

// 双线性插值
    inline void bilinearInterpolation(const cv::Mat &origin_image,
                                      const Eigen::Vector2d &distort_pixel,
                                      cv::Vec3b &bgr) {
        // 计算双线性插值的rgb颜色
        // 领域插值（直接int取整），但是会出现阶梯状
        int u_floor = static_cast<int>(std::floor(distort_pixel.x()));
        int v_floor = static_cast<int>(std::floor(distort_pixel.y()));

        int u_ceil = static_cast<int>(std::ceil(distort_pixel.x()));
        int v_ceil = static_cast<int>(std::ceil(distort_pixel.y()));

        if (u_floor >= 0 && u_ceil < origin_image.cols && v_floor >= 0 && v_ceil < origin_image.rows) {
            // Calculate the fractional parts for bilinear interpolation
            double u_frac = distort_pixel.x() - u_floor;
            double v_frac = distort_pixel.y() - v_floor;

            for (int i = 0; i < 3; ++i) {
                // Interpolation along the x-axis (u direction)
                double val_floor = (1 - u_frac) * origin_image.at<cv::Vec3b>(v_floor, u_floor)[i] +
                                   u_frac * origin_image.at<cv::Vec3b>(v_floor, u_ceil)[i];
                double val_ceil = (1 - u_frac) * origin_image.at<cv::Vec3b>(v_ceil, u_floor)[i] +
                                  u_frac * origin_image.at<cv::Vec3b>(v_ceil, u_ceil)[i];

                // Interpolation along the y-axis (v direction)
                double final_val = (1 - v_frac) * val_floor + v_frac * val_ceil;

                // Assign the interpolated value to the corresponding color channel
                bgr[i] = static_cast<uchar>(final_val);
            }
        }
    }

}


#endif //BIRDVIEW_FISHEYE_MODEL_H
