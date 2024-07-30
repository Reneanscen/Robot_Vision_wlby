//
// Created by xu on 24-5-14.
//

#ifndef BIRDVIEW_PINHOLE_H
#define BIRDVIEW_PINHOLE_H

#include "opencv2/opencv.hpp"
#include "Eigen/Core"

namespace PinholeModel {

    // 3D点投影到针孔相机内
    inline void pinhole_project_point(const Eigen::Vector3d &point3d,
                                      const Eigen::Matrix3d &R_cam_world,
                                      const Eigen::Vector3d &t_cam_world,
                                      const Eigen::Matrix3d &intrinsic,
                                      const Eigen::Matrix<double, 1, 5> &distort_coff,
                                      Eigen::Vector2d &distort_pixel) {

        const double fx = intrinsic(0, 0) * 1.0;
        const double fy = intrinsic(1, 1) * 1.0;
        const double cx = intrinsic(0, 2);
        const double cy = intrinsic(1, 2);

        const double k1 = distort_coff(0, 0);
        const double k2 = distort_coff(0, 1);
        const double p1 = distort_coff(0, 2);
        const double p2 = distort_coff(0, 3);
        const double k3 = distort_coff(0, 4);

        // 3D点转换到相机坐标系下
        Eigen::Vector3d point3d_cam = R_cam_world * point3d + t_cam_world;

        // 转换到归一化平面坐标系
        Eigen::Vector3d point3d_cam_norm = Eigen::Vector3d(point3d_cam.x() / point3d_cam.z(),
                                                           point3d_cam.y() / point3d_cam.z(),
                                                           1.);

        // 归一化平面上进行畸变的正映射（畸变正映射）
        double r2 = point3d_cam_norm.x() * point3d_cam_norm.x() + point3d_cam_norm.y() * point3d_cam_norm.y();

        //  径向畸变部分
        const double radial_distortion = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        double x_radial = point3d_cam_norm.x() * radial_distortion;
        double y_radial = point3d_cam_norm.y() * radial_distortion;

        //  切向畸变部分
        double x_tangential = 2 * p1 * point3d_cam_norm.x() * point3d_cam_norm.y() +
                              p2 * (r2 + 2 * point3d_cam_norm.x() * point3d_cam_norm.x());
        double y_tangential = p1 * (r2 + 2 * point3d_cam_norm.y() * point3d_cam_norm.y()) +
                              2 * p2 * point3d_cam_norm.x() * point3d_cam_norm.y();

        Eigen::Vector2d normal_plane_distorted = Eigen::Vector2d(x_radial + x_tangential, y_radial + y_tangential);

        // 归一化平面到针孔相机像素平面映射(内参矩阵映射)
        distort_pixel.x() = fx * normal_plane_distorted.x() + cx;
        distort_pixel.y() = fy * normal_plane_distorted.y() + cy;

    }

    // 针孔相机去畸变
    /**
     * @brief 输入去畸变像素坐标，内参矩阵，畸变系数，输出畸变像素坐标
     */
    inline void pinhole_undistort(const Eigen::Vector2i &undistort_pixel,
                                  const Eigen::Matrix3d &intrinsic,
                                  const Eigen::Matrix<double, 1, 5> &distort_coffess,
                                  Eigen::Vector2d &distort_pixel) {

        const double fx = intrinsic(0, 0);
        const double fy = intrinsic(1, 1);
        const double cx = intrinsic(0, 2);
        const double cy = intrinsic(1, 2);

        const double k1 = distort_coffess(0, 0);
        const double k2 = distort_coffess(0, 1);
        const double p1 = distort_coffess(0, 2);
        const double p2 = distort_coffess(0, 3);
        const double k3 = distort_coffess(0, 4);

        // 当前去畸变点进行归一化平面投影
        Eigen::Vector2d pixel_normal;
        pixel_normal.x() = (undistort_pixel.x() - cx) / fx;
        pixel_normal.y() = (undistort_pixel.y() - cy) / fy;

        // 归一化平面进行畸变正映射
        // 归一化平面上进行畸变的正映射（畸变正映射）
        double r2 = pixel_normal.x() * pixel_normal.x() + pixel_normal.y() * pixel_normal.y();

        //  径向畸变部分
        const double radial_distortion = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        double x_radial = pixel_normal.x() * radial_distortion;
        double y_radial = pixel_normal.y() * radial_distortion;

        //  切向畸变部分
        double x_tangential = 2 * p1 * pixel_normal.x() * pixel_normal.y() +
                              p2 * (r2 + 2 * pixel_normal.x() * pixel_normal.x());
        double y_tangential = p1 * (r2 + 2 * pixel_normal.y() * pixel_normal.y()) +
                              2 * p2 * pixel_normal.x() * pixel_normal.y();

        Eigen::Vector2d normal_plane_distorted = Eigen::Vector2d(x_radial + x_tangential, y_radial + y_tangential);


        // 归一化平面上进行畸变的正映射（畸变正映射）
        distort_pixel.x() = fx * normal_plane_distorted.x() + cx;
        distort_pixel.y() = fy * normal_plane_distorted.y() + cy;
    }

    // 双线性插值 求解BGR参数
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

#endif //BIRDVIEW_PINHOLE_H
