//
// Created by jazzey on 2023/11/18.
//

#ifndef SEMANTIC_MAPPING_DEPTH_SEGMENTATION_H
#define SEMANTIC_MAPPING_DEPTH_SEGMENTATION_H

#include <opencv2/core.hpp>

#include "common/common_type_define.h"
#include "PoseUtils/Pose3DDataStruct.h"
#include "parameter.h"
#include "depseg_kernel.cuh"

#include "dbscan.h"

#include "variables.h"

#include "check/check.h"

namespace wlby::depth_segmentation{

    class DepthCamera {
    public:
        DepthCamera() = default;
        inline void setCameraMatrix(const cv::Mat& camera_matrix) {
            CHECK_TRUE(!camera_matrix.empty());
            camera_matrix_ = camera_matrix;
        }
        inline void setHeight(int height) {
            height_ = height;
        }
        inline int getHeight() const {
            return height_;
        }
        inline void setWidth(int width) {
            width_ = width;
        }
        inline int getWidth() const {
            return width_;
        }
        inline cv::Mat getCameraMatrix() const { return camera_matrix_; }
    private:
        cv::Mat camera_matrix_;
        int height_ = 480;
        int width_ = 640;
    };

    struct SemanticInstanceSegmentation {
        std::vector<cv::Mat> masks;
        std::vector<int> labels;
        std::vector<float> scores;
    };

    struct Segment {
        std::vector<cv::Vec3f> points;
        std::set<size_t> instance_label;
        std::set<size_t> semantic_label;
        std::vector<float> scores;
        cv::Vec3f center;
    };

    class SegmentWithIdx {

    public:
        SegmentWithIdx(const Segment& segment,const Localization::Pose3D& global_pose){

            //赋值点的坐标
            for(int i = 0; i < segment.points.size(); i++){
                Localization::Pos3D point(segment.points[i](0), segment.points[i](1), segment.points[i](2));
                Localization::Pos3D trans = global_pose * point;
                points_.push_back(trans);
            }

            semantic_label_ = *segment.semantic_label.begin();
            instance_label_ = *segment.instance_label.begin();
            score_ = *segment.scores.begin();

        }

        uint8_t semantic_label_;
        uint32_t instance_label_;
        double score_;
        std::vector<Localization::Pos3D> points_;
        std::set<uint64> idx_set;
    };

    class DepthSegmenter {
    public:
        explicit DepthSegmenter(const DepthCamera &depth_camera):depth_camera_(depth_camera){
            fresh_instance_label_ = 1;
        };

        inline uint32_t getFreshInstanceLabel(){
            return fresh_instance_label_++;
        }

        //解析深度图，返回点云数据
        void computeDepthMap(const cv::Mat& depth_image, cv::Mat* depth_map) {
            CHECK_TRUE(!depth_image.empty());
            CHECK_EQ(depth_image.type(), CV_32FC1);
            CHECK_NOT_NULL(depth_map);
            CHECK_EQ(depth_image.size(), depth_map->size());
            CHECK_EQ(depth_map->type(), CV_32FC3);
            CHECK_TRUE(!depth_camera_.getCameraMatrix().empty());

            //原始深度图depth_image
            //标定矩阵 depth_camera_.getCameraMatrix()
            //输出的点云数据 depth_map
            cv::rgbd::depthTo3d(depth_image, depth_camera_.getCameraMatrix(), *depth_map);

            // 遍历矩阵中的每个像素点 , 将小于0.2大于2m的点变成0
            for (cv::MatIterator_<cv::Vec3f> it = depth_map->begin<cv::Vec3f>(), end = depth_map->end<cv::Vec3f>(); it != end; ++it) {
                cv::Vec3f& pixel = *it;
                double r = pixel[0] * pixel[0] + pixel[1] * pixel[1] + pixel[2] * pixel[2];
                if (r > 5.0 || r < 0.04) {
                    pixel[0] = 0;
                    pixel[1] = 0;
                    pixel[2] = 0; // 将 z 大于 1.0 的像素点的 z 值赋为 0
                }
            }
        }


        //计算深度不连续图
        static void computeDepthDiscontinuityMap(const cv::Mat& depth_image, cv::Mat* depth_discontinuity_map) {
            CHECK_TRUE(!depth_image.empty());
            CHECK_EQ(depth_image.type(), CV_32FC1);
            CHECK_NOT_NULL(depth_discontinuity_map);
            CHECK_EQ(depth_discontinuity_map->type(), CV_32FC1);

            constexpr size_t kMaxValue = 1u;
            constexpr double kNanThreshold = 0.0;

            //可用于构造一个特定大小和形状的结构元素，用于图像形态学处理。
            cv::Size image_size(depth_image.cols, depth_image.rows);
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(Parameter::DepthDiscontinuityMapParams::kernel_size,
                                                                                 Parameter::DepthDiscontinuityMapParams::kernel_size));
            //第一个参数是要进行阈值操作的输入图像 depth_image
            //第二个参数是输出图像 depth_without_nans
            //第三个参数是阈值 kNanThreshold，表示小于该值的像素点将被认为是无效值（NaN），被赋值为 0。
            //第四个参数是当像素值大于或等于阈值时所赋的新值，这里是 kMaxValue，即 1。
            //最后一个参数 cv::THRESH_TOZERO 则表示当像素值小于阈值时设置为 0。

            //用代码表示此函数的意义：cv::THRESH_TOZERO
            //destination = （src[i,j] >kNanThreshold)? src[i,j] : 0;
            //也就是，遍历灰度图的每一个像素点，如果遍历到的像素值大于这个阈值，那么保持不变，如果小于等于这个阈值，那么不好意思，直接为0。
            //去除nan点，并将其赋值成0
            cv::Mat depth_without_nans(image_size, CV_32FC1);
            cv::threshold(depth_image, depth_without_nans, kNanThreshold, kMaxValue, cv::THRESH_TOZERO);

            //膨胀操作
            cv::Mat dilate_image(image_size, CV_32FC1);
            cv::dilate(depth_without_nans, dilate_image, element);
            dilate_image -= depth_without_nans;

            //腐蚀操作
            cv::Mat erode_image(image_size, CV_32FC1);
            cv::erode(depth_without_nans, erode_image, element);
            erode_image = depth_without_nans - erode_image;

            //获取dilate_image和erode_image对应像素的最大值
            cv::Mat max_image(image_size, CV_32FC1);
            cv::max(dilate_image, erode_image, max_image);

            //============== 图片做除法
            // 输入数组1 (分子)
            // 输入数组1 (分母)
            // 输出数组 (scale*src1/src2)
            // 乘数因子
            // 输出数组类型, -1表示与src2相同。
            cv::Mat ratio_image(image_size, CV_32FC1);
            cv::divide(max_image, depth_without_nans, ratio_image);

            //destination = (src[i,j]>threshold)? maxval : 0;
            //也就是：灰度图的在[i,j]位置的像素值是否大于阈值，如果大于的话，那么它的值就是maxval,如果不是的话（哪怕是等于也不行），那么就等于0。
            cv::threshold(ratio_image, *depth_discontinuity_map,
                          Parameter::DepthDiscontinuityMapParams::discontinuity_ratio, kMaxValue,
                          cv::THRESH_BINARY);

            //此时，depth_discontinuity_map的值非0即1
        }

        void computeOwnNormals(const cv::Mat& depth_map, cv::Mat* normals) {
            CHECK_TRUE(!depth_map.empty());
            CHECK_EQ(depth_map.type(), CV_32FC3);
            CHECK_NOT_NULL(normals);
            CHECK_EQ(depth_map.size(), normals->size());

            cv::Mat neighborhood = cv::Mat::zeros(3, 13 * 13, CV_32FC1);
            cv::Mat eigenvalues;
            cv::Mat eigenvectors;
            cv::Mat covariance(3, 3, CV_32FC1);
            covariance = cv::Mat::zeros(3, 3, CV_32FC1);
            cv::Vec3f mean;
            cv::Vec3f mid_point;

            constexpr float float_nan = std::numeric_limits<float>::quiet_NaN();
#pragma omp parallel for private(neighborhood, eigenvalues, eigenvectors, covariance, mean, mid_point)
            for (size_t y = 0; y < depth_map.rows; ++y) {
                for (size_t x = 0; x < depth_map.cols; ++x) {
                    mid_point = depth_map.at<cv::Vec3f>(y, x);
                    // Skip point if z value is nan.
                    if (cvIsNaN(mid_point[0]) || cvIsNaN(mid_point[1]) ||
                        cvIsNaN(mid_point[2]) || (mid_point[2] == 0.0)) {
                        normals->at<cv::Vec3f>(y, x) =
                                cv::Vec3f(float_nan, float_nan, float_nan);
                        continue;
                    }
                    const float max_distance = 0.05 * mid_point[2];
                    mean = cv::Vec3f(0.0f, 0.0f, 0.0f);

                    const size_t neighborhood_size = findNeighborhood(depth_map, 13, max_distance, x, y, &neighborhood, &mean);

                    if (neighborhood_size > 1u) {
                        computeCovariance(neighborhood, mean, neighborhood_size, &covariance);
                        // Compute Eigen vectors.
                        cv::eigen(covariance, eigenvalues, eigenvectors);
                        // Get the Eigenvector corresponding to the smallest Eigenvalue.
                        constexpr size_t n_th_eigenvector = 2u;
                        for (size_t coordinate = 0u; coordinate < 3u; ++coordinate) {
                            normals->at<cv::Vec3f>(y, x)[coordinate] =
                                    eigenvectors.at<float>(n_th_eigenvector, coordinate);
                        }
                        // Re-Orient normals to point towards camera.
                        if (normals->at<cv::Vec3f>(y, x)[2] > 0.0f) {
                            normals->at<cv::Vec3f>(y, x) = -normals->at<cv::Vec3f>(y, x);
                        }
                    } else {
                        normals->at<cv::Vec3f>(y, x) =
                                cv::Vec3f(float_nan, float_nan, float_nan);
                    }
                }
            }
        }

        void computeCovariance(const cv::Mat& neighborhood, const cv::Vec3f& mean,
                               const size_t neighborhood_size, cv::Mat* covariance) {
            CHECK_TRUE(!neighborhood.empty());
            CHECK_EQ(neighborhood.rows, 3u);
            CHECK_GT(neighborhood_size, 0u);
            CHECK_LE(neighborhood_size, neighborhood.cols);
            CHECK_NOT_NULL(covariance);

            *covariance = cv::Mat::zeros(3, 3, CV_32F);

            for (size_t i = 0; i < neighborhood_size; ++i) {
                cv::Vec3f point;
                for (size_t row = 0; row < neighborhood.rows; ++row) {
                    point[row] = neighborhood.at<float>(row, i) - mean[row];
                }

                covariance->at<float>(0, 0) += point[0] * point[0];
                covariance->at<float>(0, 1) += point[0] * point[1];
                covariance->at<float>(0, 2) += point[0] * point[2];
                covariance->at<float>(1, 1) += point[1] * point[1];
                covariance->at<float>(1, 2) += point[1] * point[2];
                covariance->at<float>(2, 2) += point[2] * point[2];
            }
            // Assign the symmetric elements of the covariance matrix.
            covariance->at<float>(1, 0) = covariance->at<float>(0, 1);
            covariance->at<float>(2, 0) = covariance->at<float>(0, 2);
            covariance->at<float>(2, 1) = covariance->at<float>(1, 2);
        }

        size_t findNeighborhood(const cv::Mat& depth_map, const size_t window_size,
                                const float max_distance, const size_t x,
                                const size_t y, cv::Mat* neighborhood,
                                cv::Vec3f* mean) {
            CHECK_TRUE(!depth_map.empty());
            CHECK_GT(window_size, 0u);
            CHECK_EQ(window_size % 2u, 1u);
            CHECK_GE(max_distance, 0.0f);
            CHECK_GE(x, 0u);
            CHECK_GE(y, 0u);
            CHECK_LT(x, depth_map.cols);
            CHECK_LT(y, depth_map.rows);
            CHECK_NOT_NULL(neighborhood);
            CHECK_NOT_NULL(mean);

            size_t neighborhood_size = 0u;
            *neighborhood = cv::Mat::zeros(3, window_size * window_size, CV_32FC1);
            cv::Vec3f mid_point = depth_map.at<cv::Vec3f>(y, x);
            for (size_t y_idx = 0; y_idx < window_size; ++y_idx) {
                const int y_filter_idx = y + y_idx - window_size / 2u;
                if (y_filter_idx < 0 || y_filter_idx >= depth_map.rows) {
                    continue;
                }
                CHECK_GE(y_filter_idx, 0u);
                CHECK_LT(y_filter_idx, depth_map.rows);
                for (size_t x_idx = 0; x_idx < window_size; ++x_idx) {
                    const int x_filter_idx = x + x_idx - window_size / 2u;
                    if (x_filter_idx < 0 || x_filter_idx >= depth_map.cols) {
                        continue;
                    }
                    CHECK_GE(x_filter_idx, 0u);
                    CHECK_LT(x_filter_idx, depth_map.cols);

                    cv::Vec3f filter_point =
                            depth_map.at<cv::Vec3f>(y_filter_idx, x_filter_idx);

                    // Compute Euclidean distance between filter_point and mid_point.
                    const cv::Vec3f difference = mid_point - filter_point;
                    const float euclidean_dist = cv::sqrt(difference.dot(difference));
                    if (euclidean_dist < max_distance) {
                        // Add the filter_point to neighborhood set.
                        for (size_t coordinate = 0u; coordinate < 3u; ++coordinate) {
                            neighborhood->at<float>(coordinate, neighborhood_size) =
                                    filter_point[coordinate];
                        }
                        ++neighborhood_size;
                        *mean += filter_point;
                    }
                }
            }
            CHECK_GE(neighborhood_size, 1u);
            CHECK_LE(neighborhood_size, window_size * window_size);
            *mean /= static_cast<float>(neighborhood_size);
            return neighborhood_size;
        }

        void computeNormalMap(const cv::Mat &depth_map, cv::Mat *normal_map)
        {
            CHECK_TRUE(!depth_map.empty());
            CHECK_NOT_NULL(normal_map);
            if (Parameter::NormalMapParams::if_use_gpu)
            {
                // 改写:使用GPU加速
                int Heiget = depth_map.size().height;
                int Width = depth_map.size().width;
                cv::cuda::GpuMat d_a(Heiget, Width, CV_32FC3);
                cv::cuda::GpuMat d_s(Heiget, Width, CV_32FC3);
                d_a.upload(depth_map);
                ComputeNormal_gpu(Heiget, Width, d_a, d_s);
                cudaDeviceSynchronize();
                d_s.download(*normal_map);
            }
            else
            {
                // 源代码写法，未使用GPU加速
                // compute normals will go there ...
                computeOwnNormals(depth_map, normal_map);
            }
        }

        static float& getRefFromLIJ(float* &input, int l,int i,int j,int a1,int a2){
            return input[l*a1*a2+i*a2+j];
        }

        void MallocMat(float* &mat,const int k,const int i,const int j){
            mat= new float[k*i*j]();
        }

        static void FreeMat(float* &mat){
            delete[] mat;
            mat = nullptr;
        }

        void getPaddedMat(float* &mat_channel,int rows,int cols, float* &padded_mat, int padding_rows,int padding_cols){


            int padding_size = padding_rows - rows;

            //填充中间部分
            int count = padding_size / 2;
            for (int l = 0; l < 3; l++)
            {
                for (int i = 0; i < rows; i++)
                {
                    for (int j = 0; j < cols; j++)
                    {
                        int s = i + count, k = j + count;
                        getRefFromLIJ(padded_mat,l,s,k,padding_rows,padding_cols) = getRefFromLIJ(mat_channel, l, i, j, rows,cols);
                    }
                }




                for (int i = 0; i < count; i++)
                {
                    for (int j = 0; j < cols; j++)
                    {
                        getRefFromLIJ(padded_mat,l,i,j+2,padding_rows,padding_cols) = getRefFromLIJ(mat_channel, l, count - i, j, rows,cols);
                        getRefFromLIJ(padded_mat,l,padding_rows - 1 - i,j+2,padding_rows,padding_cols) = getRefFromLIJ(mat_channel, l, rows - 1 - count + i, j, rows,cols);
                    }

                    for (int j = 0; j < rows; j++)
                    {
                        getRefFromLIJ(padded_mat,l,j+2,i,padding_rows,padding_cols) = getRefFromLIJ(mat_channel, l, j, count - i, rows,cols);
                        getRefFromLIJ(padded_mat,l,j+2,padding_cols - 1 - i,padding_rows,padding_cols) = getRefFromLIJ(mat_channel, l, j, rows - 2 - count + i, rows,cols);

                    }
                }

                // 四个角的加速
                for (int i = 0; i < count; i++)
                {
                    for (int j = 0; j < count; j++)
                    {
                        getRefFromLIJ(padded_mat,l,i,j,padding_rows,padding_cols) = getRefFromLIJ(padded_mat,l,i,2 * count - j,padding_rows,padding_cols);
                        getRefFromLIJ(padded_mat,l,padding_rows - 1 - i,j,padding_rows,padding_cols) = getRefFromLIJ(padded_mat,l,padding_rows - 1 - i,2 * count - j,padding_rows,padding_cols);
                        getRefFromLIJ(padded_mat,l,i,padding_cols - 1 - j,padding_rows,padding_cols) = getRefFromLIJ(padded_mat,l,i,padding_cols - 1 - 2 * count + j,padding_rows,padding_cols);
                        getRefFromLIJ(padded_mat,l,padding_rows - 1 - i,padding_cols - 1 - j,padding_rows,padding_cols) = getRefFromLIJ(padded_mat,l,padding_rows - 1 - i,padding_cols - 1 - 2 * count + j,padding_rows,padding_cols);
                    }
                }
            }
        }


        void computeMinConvexityMap_gpu_version(const cv::Mat& depth_map, const cv::Mat& normal_map, cv::Mat* min_convexity_map){

            const int rows = depth_map.rows;
            const int cols = depth_map.cols;

            min_convexity_map->setTo(cv::Scalar(10.0f));

            float *depth_map_mat = nullptr;
            MallocMat(depth_map_mat, 3, rows, cols);
            for (int k = 0; k < 3; k++)
            {
                for (int i = 0; i < rows; ++i)
                {
                    for (int j = 0; j < cols; ++j)
                    {
                        getRefFromLIJ(depth_map_mat, k, i, j, rows, cols) = depth_map.at<cv::Vec3f>(i, j)[k];
                    }
                }
            }

            float *normal_map_mat = nullptr;
            MallocMat(normal_map_mat, 3, rows, cols);
            for (int k = 0; k < 3; k++)
            {
                for (int i = 0; i < rows; ++i)
                {
                    for (int j = 0; j < cols; ++j)
                    {
                        getRefFromLIJ(normal_map_mat, k, i, j, rows, cols) = normal_map.at<cv::Vec3f>(i, j)[k];
                    }
                }
            }


            //固定kernel_size == 5 ...
            int kernel_size = 5;
            // 首先对input进行镜像扩容
            int padding_size = kernel_size - 1;

            const int padding_rows = rows + padding_size;
            const int padding_cols = cols + padding_size;

            float *padding_mat = nullptr;
            MallocMat(padding_mat,3,padding_rows,padding_cols);
            getPaddedMat(depth_map_mat, rows, cols, padding_mat,padding_rows,padding_cols);

            float *padding_mat_normal = nullptr;
            MallocMat(padding_mat_normal,3,padding_rows,padding_cols);
            getPaddedMat(normal_map_mat, rows, cols, padding_mat_normal,padding_rows,padding_cols);


            float* deviceArray; // 在设备上分配内存的指针
            cudaMalloc((void**)&deviceArray, 3 * padding_rows * padding_cols * sizeof(float)); // 分配设备内存
            cudaMemcpy(deviceArray, padding_mat, 3 * padding_rows * padding_cols * sizeof(float), cudaMemcpyHostToDevice); // 将主机上的数组复制到设备上

            float* deviceArray_normal;
            cudaMalloc((void**)&deviceArray_normal, 3 * padding_rows * padding_cols * sizeof(float)); // 分配设备内存
            cudaMemcpy(deviceArray_normal, padding_mat_normal, 3 * padding_rows * padding_cols * sizeof(float), cudaMemcpyHostToDevice); // 将主机上的数组复制到设备上

            int Heiget = depth_map.size().height;
            int Width = depth_map.size().width;
            auto mask_threshold = (float)Parameter::MinConvexityMapParams::mask_threshold;
            auto threshold = (float)Parameter::MinConvexityMapParams::threshold;
            cv::cuda::GpuMat d_a(Heiget, Width, CV_32FC3);
            cv::cuda::GpuMat d_s(Heiget, Width, CV_32FC3);
            cv::cuda::GpuMat d_out(Heiget,Width,CV_32FC1);
            d_a.upload(depth_map);
            d_s.upload(normal_map);
            d_out.upload(*min_convexity_map);
            ComputeMinConvexityMap_gpu(Heiget, Width, deviceArray, deviceArray_normal, d_a, d_s,d_out,mask_threshold,threshold);
            cudaDeviceSynchronize();
            d_out.download(*min_convexity_map);

            if (Parameter::MinConvexityMapParams::use_morphological_opening)
            {
                cv::Mat element = cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(2 * Parameter::MinConvexityMapParams::morphological_opening_size + 1,
                                 2 * Parameter::MinConvexityMapParams::morphological_opening_size + 1),
                        cv::Point(Parameter::MinConvexityMapParams::morphological_opening_size,
                                  Parameter::MinConvexityMapParams::morphological_opening_size));
                cv::morphologyEx(*min_convexity_map, *min_convexity_map, cv::MORPH_OPEN, element);
            }

            cudaFree(deviceArray);
            cudaFree(deviceArray_normal);

            FreeMat(depth_map_mat);
            FreeMat(normal_map_mat);
            FreeMat(padding_mat);
            FreeMat(padding_mat_normal);

        }

        void computeMinConvexityMap_cpu_version(const cv::Mat &depth_map,
                                                                const cv::Mat &normal_map,
                                                                cv::Mat *min_convexity_map)
        {

            min_convexity_map->setTo(cv::Scalar(10.0f));

            // kernel_size == 5
            const size_t kernel_size = Parameter::MinConvexityMapParams::window_size + (Parameter::MinConvexityMapParams::step_size - 1u) *
                    (Parameter::MinConvexityMapParams::window_size - 1u);

            // n_kernels == 24
            const size_t n_kernels = Parameter::MinConvexityMapParams::window_size * Parameter::MinConvexityMapParams::window_size - 1u;

            // Define the n point-wise distance kernels and compute the filtered images.
            // The kernels for i look as follows
            //(e.g. window_size = 5, i = 6):        (e.g. window_size = 5, i = 5):
            //     0  0  0  0  0                     0  0  0  0  0
            //     0  1  0  0  0                     1  0  0  0  0
            //     0  0 -1  0  0                     0  0 -1  0  0
            //     0  0  0  0  0                     0  0  0  0  0
            //     0  0  0  0  0                     0  0  0  0  0
            for (size_t i = 0; i < n_kernels + 1; i += Parameter::MinConvexityMapParams::step_size)
            {

                // i += static_cast<size_t>(i % kernel_size == kernel_size) * kernel_size + params_.min_convexity.step_size

                if (i == n_kernels / 2u)
                {
                    continue;
                }

                cv::Mat difference_kernel = cv::Mat::zeros(kernel_size, kernel_size, CV_32FC1);
                difference_kernel.at<float>(i) = 1.0f;
                difference_kernel.at<float>(n_kernels / 2u) = -1.0f;

                // Compute the filtered images.
                // 用于对输入图像进行卷积操作
                cv::Mat difference_map(depth_map.size(), CV_32FC3);
                cv::filter2D(depth_map, difference_map, CV_32FC3, difference_kernel);

                // Calculate the dot product over the three channels of difference_map and normal_map.
                // 计算Difference_map和normal_map三个通道上的点积。
                cv::Mat difference_times_normal(depth_map.size(), CV_32FC3);
                difference_times_normal = difference_map.mul(-normal_map);

                // 将difference_times_normal通道分离，并将三通道合成一通道
                std::vector<cv::Mat> channels(3);
                cv::split(difference_times_normal, channels);
                cv::Mat vector_projection(depth_map.size(), CV_32FC1);
                vector_projection = channels[0] + channels[1] + channels[2];

                cv::Mat concavity_mask(depth_map.size(), CV_32FC1);
                cv::Mat convexity_mask(depth_map.size(), CV_32FC1);

                // Split the projected vector images into convex and concave
                // regions/masks.
                constexpr float kMaxBinaryValue = 1.0f;
                cv::threshold(vector_projection, convexity_mask,
                              Parameter::MinConvexityMapParams::mask_threshold, kMaxBinaryValue,
                              cv::THRESH_BINARY);
                cv::threshold(vector_projection, concavity_mask,
                              Parameter::MinConvexityMapParams::mask_threshold, kMaxBinaryValue,
                              cv::THRESH_BINARY_INV);

                cv::Mat normal_kernel = cv::Mat::zeros(kernel_size, kernel_size, CV_32FC1);
                normal_kernel.at<float>(i) = 1.0f;

                cv::Mat filtered_normal_image = cv::Mat::zeros(normal_map.size(), CV_32FC3);
                cv::filter2D(normal_map, filtered_normal_image, CV_32FC3, normal_kernel);

                normal_map.copyTo(filtered_normal_image, filtered_normal_image != filtered_normal_image);

                // TODO(ff): Create a function for this mulitplication and projections.
                cv::Mat normal_times_filtered_normal(depth_map.size(), CV_32FC3);
                normal_times_filtered_normal = normal_map.mul(filtered_normal_image);
                filtered_normal_image.copyTo(normal_times_filtered_normal, normal_times_filtered_normal != normal_times_filtered_normal);

                std::vector<cv::Mat> normal_channels(3);
                cv::split(normal_times_filtered_normal, normal_channels);
                cv::Mat normal_vector_projection(depth_map.size(), CV_32FC1);

                normal_vector_projection = normal_channels[0] + normal_channels[1] + normal_channels[2];
                normal_vector_projection = concavity_mask.mul(normal_vector_projection);

                cv::Mat convexity_map = cv::Mat::zeros(depth_map.size(), CV_32FC1);
                convexity_map = convexity_mask + normal_vector_projection;
                cv::min(*min_convexity_map, convexity_map, *min_convexity_map);
            }

            if (Parameter::MinConvexityMapParams::use_threshold)
            {
                constexpr float kMaxBinaryValue = 1.0f;
                cv::threshold(*min_convexity_map, *min_convexity_map,
                              Parameter::MinConvexityMapParams::threshold, kMaxBinaryValue,
                              cv::THRESH_BINARY);
            }

            if (Parameter::MinConvexityMapParams::use_morphological_opening)
            {
                cv::Mat element = cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(2u * Parameter::MinConvexityMapParams::morphological_opening_size + 1u,
                                 2u * Parameter::MinConvexityMapParams::morphological_opening_size + 1u),
                        cv::Point(Parameter::MinConvexityMapParams::morphological_opening_size,
                                  Parameter::MinConvexityMapParams::morphological_opening_size));
                cv::morphologyEx(*min_convexity_map, *min_convexity_map, cv::MORPH_OPEN, element);
            }
        }

        void computeMinConvexityMap(const cv::Mat& depth_map,
                                    const cv::Mat& normal_map,
                                    cv::Mat* min_convexity_map) {
            CHECK_TRUE(!depth_map.empty());
            CHECK_TRUE(!normal_map.empty());
            CHECK_EQ(depth_map.type(), CV_32FC3);
            CHECK_EQ(normal_map.type(), CV_32FC3);
            CHECK_EQ(depth_map.size(), normal_map.size());
            CHECK_NOT_NULL(min_convexity_map);
            CHECK_EQ(min_convexity_map->type(), CV_32FC1);
            CHECK_EQ(depth_map.size(), min_convexity_map->size());
            // Check if window_size is odd.
            CHECK_EQ(Parameter::MinConvexityMapParams::window_size % 2, 1u);

            bool if_use_gpu = Parameter::MinConvexityMapParams::if_use_gpu;
            if(if_use_gpu){
                computeMinConvexityMap_gpu_version(depth_map, normal_map, min_convexity_map);
            }else{
                computeMinConvexityMap_cpu_version(depth_map, normal_map, min_convexity_map);
            }
        }


        static void computeFinalEdgeMap(const cv::Mat& convexity_map,
                                                 const cv::Mat& discontinuity_map,
                                                 cv::Mat* edge_map) {
            CHECK_TRUE(!convexity_map.empty());
            CHECK_TRUE(!discontinuity_map.empty());
            CHECK_EQ(convexity_map.type(), CV_32FC1);
            CHECK_EQ(discontinuity_map.type(), CV_32FC1);
            CHECK_EQ(convexity_map.size(), discontinuity_map.size());
            CHECK_NOT_NULL(edge_map);

            if (Parameter::FinalEdgeMapParams::use_morphological_opening) {
                cv::Mat element = cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(2 * Parameter::FinalEdgeMapParams::morphological_opening_size + 1,
                                 2 * Parameter::FinalEdgeMapParams::morphological_opening_size + 1),
                        cv::Point(Parameter::FinalEdgeMapParams::morphological_opening_size,
                                  Parameter::FinalEdgeMapParams::morphological_opening_size));

                cv::morphologyEx(convexity_map, convexity_map, cv::MORPH_OPEN, element);
            }

            if (Parameter::FinalEdgeMapParams::use_morphological_closing) {
                cv::Mat element = cv::getStructuringElement(
                        cv::MORPH_RECT,
                        cv::Size(2 * Parameter::FinalEdgeMapParams::morphological_closing_size + 1,
                                 2 * Parameter::FinalEdgeMapParams::morphological_closing_size + 1),
                        cv::Point(Parameter::FinalEdgeMapParams::morphological_closing_size,
                                  Parameter::FinalEdgeMapParams::morphological_closing_size));
                // TODO(ntonci): Consider making a separate parameter for discontinuity_map.
                cv::morphologyEx(discontinuity_map, discontinuity_map, cv::MORPH_CLOSE,
                                 element);
            }
            *edge_map = convexity_map - discontinuity_map;
        }

        void findBlobs(const cv::Mat& binary, std::vector<std::vector<cv::Point2i>>* labels) {
            CHECK_TRUE(!binary.empty());
            CHECK_EQ(binary.type(), CV_32FC1);
            CHECK_NOT_NULL(labels);
            labels->clear();
            cv::Mat label_image;
            binary.convertTo(label_image, CV_32SC1);

            // label_image 非 0 即 1
            // 0 边缘线 是 1 是未标注
            // 填充后，填充的值设为2
            size_t label_count = 2u;
            for (size_t y = 0; y < label_image.rows; ++y) {
                for (size_t x = 0; x < label_image.cols; ++x) {

                    //对于边缘线略过
                    if (label_image.at<int>(y, x) != 1) {
                        continue;
                    }

                    /*
                     *
                        InputOutputArray类型的image，输入输出图像。
                        [可选]InputOutputArray类型的mask，掩膜区，掩膜区非零区域不被填充，且掩膜区的宽高要比输入图像各多两个像素，输入图像中(x,y)位置对应掩膜区(x+1,y+1)位置，掩膜区的具体使用将在下方进行举例演示。
                        Point类型的seedPoint，漫水填充的起点。
                        Scalar类型的newVal，像素点被填充后呈现的颜色。
                        Rect*类型的rect，漫水填充后重绘区域的最小边界矩形区域。
                        Scalar类型的loDiff，当前观测像素与种子像素颜色负差的最大值。通俗地讲，就是观测像素各个通道的值都要比种子像素小，且小的数值要低于loDiff，满足该条件才会被填充。具体使用将在下方进行举例演示。
                        Scalar类型的upDiff，当前观测像素与种子像素颜色正差的最大值。通俗地讲，就是观测像素各个通道的值都要比种子像素大，且大的数值要低于upDiff，满足该条件才会被填充。具体使用将在下方进行举例演示。
                        int类型的flags，操作标识符，是32位二进制数。低八位表示算法连通性，一般取4或者8,4为上下左右，8加上对角四点；高八位可选两种标识符，分别为FLOODFILL_FIXED_RANGE和FLOODFILL_MASK_ONLY，也可以用or（|）组合它们；中八位是填充掩膜的数值，搭配FLOODFILL_MASK_ONLY使用。
                     *
                     */
                    cv::Rect rect;
                    cv::floodFill(label_image, cv::Point(x, y), label_count, &rect, 0, 0, cv::FLOODFILL_FIXED_RANGE);
                    std::vector<cv::Point2i> blob;
                    size_t rect_size_y = rect.y + rect.height;
                    size_t rect_size_x = rect.x + rect.width;

                    for (size_t i = rect.y; i < rect_size_y; ++i) {
                        for (size_t j = rect.x; j < rect_size_x; ++j) {
                            if (label_image.at<int>(i, j) != label_count) {
                                continue;
                            }
                            blob.push_back(cv::Point2i(j, i));
                        }
                    }

                    if (blob.size() > 1u) {
                        labels->push_back(blob);
                        ++label_count;
                    }

                }
            }
        }

        void generateRandomColorsAndLabels(size_t contours_size, std::vector<cv::Scalar>* colors,
                std::vector<int>* labels) {
            CHECK_GE(contours_size, 0u);
            CHECK_NOT_NULL(colors);
            colors->clear();
            CHECK_NOT_NULL(labels);
            labels->clear();
            CHECK_EQ(colors_.size(), labels_.size());

            if (colors_.size() < contours_size) {
                colors_.reserve(contours_size);
                for (size_t i = colors_.size(); i < contours_size; ++i) {
                    colors_.push_back(cv::Scalar(255 * (rand() / static_cast<float>(RAND_MAX)),
                                                 255 * (rand() / static_cast<float>(RAND_MAX)),
                                                 255 * (rand() / static_cast<float>(RAND_MAX))));
                    labels_.push_back(i);
                }
            }
            *colors = colors_;
            *labels = labels_;
        }

        void labelMap(const cv::Mat& depth_map, const cv::Mat& edge_map,
                      const cv::Mat& normal_map, cv::Mat* labeled_map,
                      std::vector<cv::Mat>* segment_masks,
                      std::vector<Segment>* segments) {
            CHECK_TRUE(!edge_map.empty());
            CHECK_EQ(edge_map.type(), CV_32FC1);
            CHECK_EQ(depth_map.type(), CV_32FC3);
            CHECK_EQ(normal_map.type(), CV_32FC3);
            CHECK_NOT_NULL(labeled_map);
            CHECK_NOT_NULL(segment_masks);
            CHECK_NOT_NULL(segments);
            segments->clear();

            constexpr size_t kMaskValue = 255u;

            //显示的output
            cv::Mat output = cv::Mat::zeros(depth_map.size(), CV_8UC3);

            std::vector<int> point_nums_vec;
            switch (Parameter::LabelMapParams::method) {
                /*case 1: {
                    // TODO(ff): Move to method.
                    std::vector<std::vector<cv::Point>> contours;
                    std::vector<cv::Vec4i> hierarchy;

                    //===============  对边缘图进行轮廓提取
                    //将图片变成8通道
                    cv::Mat edge_map_8u;
                    edge_map.convertTo(edge_map_8u, CV_8U);
                    static const cv::Point kContourOffset = cv::Point(0, 0);

                    auto start_time = std::chrono::steady_clock::now();

                      image: 输入的二值图像，可以是 8-bit 单通道图像，也可以是浮点型单通道图像。
                      contours: 输出的轮廓数组，每个轮廓都存储为一个矢量形式的点序列，可以通过 vector<vector<Point>> 来定义。
                      hierarchy: 输出的轮廓层级信息，可以通过 vector<Vec4i> 定义。
                      mode: 轮廓检索模式，有以下两种可选：
                          CV_RETR_EXTERNAL：只检索最外层的轮廓。
                          CV_RETR_TREE：检索所有轮廓，并重构轮廓之间的整个层次结构。
                      method: 轮廓逼近方法，有以下四种可选：
                          CV_CHAIN_APPROX_NONE: 存储所有的轮廓点。
                          CV_CHAIN_APPROX_SIMPLE: 压缩水平、垂直和斜的部分，只保留其端点。
                          CV_CHAIN_APPROX_TC89_L1: 应用 Teh-Chin 链逼近算法中的 L1 范数方法。
                          CV_CHAIN_APPROX_TC89_KCOS: 应用 Teh-Chin 链逼近算法中的 k cos θ 方法。
                      offset: 轮廓点坐标的偏移量，默认为 Point()。

                    //此函数输入8通道的图片
                    cv::findContours(edge_map_8u, contours, hierarchy,
                                     cv::RETR_TREE,
                                     CV_CHAIN_APPROX_NONE, kContourOffset);
                    auto end_time = std::chrono::steady_clock::now();
                    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    std::cout << "findContours时间 " << elapsed_time.count()/1000.0 << "s" << std::endl;

                    //生成随机的colors 和 labels数组
                    //若第1张深度图分割后，产生8个分类实例  colors 和 labels 数量 8 个
                    //第2张深度图分割后，产生6个分割实例   colors 和 labels 数量 14 个
                    //以此类推。。。
                    std::vector<cv::Scalar> colors;
                    std::vector<int> labels;
                    generateRandomColorsAndLabels(contours.size(), &colors, &labels);

                    start_time = std::chrono::steady_clock::now();
                    //对每个分割出来的孔洞和噪声进行处理
                    for (size_t i = 0u; i < contours.size(); ++i) {
                        //计算提取的轮廓的面积
                        const double area = cv::contourArea(contours[i]);
                        constexpr int kNoParentContour = -1;
                        //大部分是空洞型轮廓,小轮廓空洞的处理
                        if (area < Parameter::LabelMapParams::min_size) {

                            //hierarchy有四维，分别是当前轮廓的后一个轮廓、前一个轮廓、子轮廓，父轮廓的索引编号
                            const int parent_contour = hierarchy[i][3];
                            if (parent_contour == kNoParentContour) {

                                // 表明: 此轮廓下没有父轮廓了, 没有父轮廓，则轮廓画成黑色
                                colors[i] = cv::Scalar(0, 0, 0);
                                labels[i] = -1;
                                //以填充模式画黑色轮廓
                                drawContours(edge_map_8u, contours, i, cv::Scalar(0u), cv::FILLED, 8, hierarchy);

                            } else {

                                //若有父轮廓
                                if (hierarchy[i][0] == -1 && hierarchy[i][1] == -1) {
                                    //父轮廓里只有自己,赋值和父轮廓一样的标签和颜色
                                    colors[i] = colors[parent_contour];
                                    labels[i] = labels[parent_contour];

                                } else {
                                    //父轮廓内不止一个，赋值成黑色
                                    colors[i] = cv::Scalar(0, 0, 0);
                                    labels[i] = -1;
                                    drawContours(edge_map_8u, contours, i, cv::Scalar(0u), cv::FILLED, 8, hierarchy);
                                }

                            }
                        }
                    }

                    end_time = std::chrono::steady_clock::now();
                    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    std::cout << "噪声处理时间 " << elapsed_time.count()/1000.0 << "s" << std::endl;

                    //edge_map_8u : 仅有0和1构成，0是提取出来的边缘
                    //output : all 0

                    start_time = std::chrono::steady_clock::now();
                    cv::Mat output_labels = cv::Mat(depth_map.size(), CV_32SC1, cv::Scalar(0));
                    for (size_t i = 0; i < contours.size(); ++i) {
                        //根据颜色进行轮廓的填充
                        drawContours(output, contours, i, cv::Scalar(colors[i]), cv::FILLED, 8, hierarchy);
                        drawContours(output_labels, contours, i, cv::Scalar(labels[i]), cv::FILLED, 8, hierarchy);

                        //对边缘点进行线型填充
                        drawContours(edge_map_8u, contours, i, cv::Scalar(0u), 1, 8, hierarchy);
                    }
                    end_time = std::chrono::steady_clock::now();
                    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    std::cout << "填充时间 " << elapsed_time.count()/1000.0 << "s" << std::endl;

                    //对边缘点的特别处理，颜色变成黑色，label变成-1
                    output.setTo(cv::Scalar(0, 0, 0), edge_map_8u == 0u);
                    output_labels.setTo(-1, edge_map_8u == 0u);

                    //=========== 此时大部分已经分割好，但仍有在大块色块中的黑色空洞 ===============

                    //===== 创建一个labels_map，键是label的值，值是index
                    std::map<size_t, size_t> labels_map;
                    size_t value = 0u;
                    for (size_t i = 0; i < labels.size(); ++i) {
                        if (labels[i] >= 0) {
                            // Create a new map if label is not yet in keys.
                            if (labels_map.find(labels[i]) == labels_map.end()) {
                                labels_map[labels[i]] = value;
                                ++value;
                            }
                        }
                    }
                    //

                    //=============   分割出的众多实例
                    segments->resize(labels_map.size());
                    segment_masks->resize(labels_map.size());
                    //

                    //============= 将掩码设置成 0
                    for (cv::Mat& segment_mask : *segment_masks) {
                        segment_mask = cv::Mat(depth_map.size(), CV_8UC1, cv::Scalar(0));
                    }

                    start_time = std::chrono::steady_clock::now();

                    int Height = depth_map.size().height;
                    int Width = depth_map.size().width;

                    cv::Mat output_tmp = cv::Mat::zeros(Height,Width, CV_32SC1);
                    cv::cuda::GpuMat d_output_labels(Height, Width, CV_32SC1);
                    cv::cuda::GpuMat d_output(Height, Height, CV_8UC1);
                    cv::cuda::GpuMat d_edge_map_8u(Height, Width, CV_8UC1);
                    cv::cuda::GpuMat d_depth_map(Height, Width, CV_32FC3);

                    d_output_labels.upload(output_labels);
                    d_output.upload(output_tmp);
                    d_edge_map_8u.upload(edge_map_8u);
                    d_depth_map.upload(depth_map);

                    ComputeOutPut_gpu(Height,Width,d_output_labels,d_output,d_edge_map_8u,d_depth_map);

                    cudaDeviceSynchronize();
                    d_output.download(output_tmp);

                    end_time = std::chrono::steady_clock::now();
                    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    std::cout << "填充细小空洞时间 " << elapsed_time.count()/1000.0 << "s" << std::endl;


                    for(int i = 0; i < output_tmp.rows; ++i){
                        for(int j = 0; j < output_tmp.cols; ++j){
                            int32_t label = output_tmp.at<int32_t>(i, j);
                            if(label > 0){
                                output.at<cv::Vec3b>(i, j) = cv::Vec3b(colors[label][0], colors[label][1], colors[label][2]);

                                // 对分割出的点进行处理
                                cv::Vec3f point = depth_map.at<cv::Vec3f>(i, j);
                                (*segments)[labels_map.at(label)].points.push_back(point);
                                (*segment_masks)[labels_map.at(label)].at<uint8_t>(i, j) = kMaskValue;
                            }
                        }
                    }

                    CHECK_EQ(segments->size(), labels_map.size());
                    break;
                }*/
                case 0: {

                    cv::Mat binary_edge_map;
                    constexpr float kEdgeMapThresholdValue = 0.0f;
                    constexpr float kMaxBinaryValue = 1.0f;
                    cv::threshold(edge_map, binary_edge_map, kEdgeMapThresholdValue, kMaxBinaryValue, cv::THRESH_BINARY);

                    //深度图分割出来的第i个区域: labeled_segments[i]
                    //深度图分割出来的第i个区域第j个点: labeled_segments[i][j]
                    std::vector<std::vector<cv::Point2i>> labeled_segments;
                    findBlobs(binary_edge_map, &labeled_segments);

                    std::vector<cv::Scalar> colors;
                    std::vector<int> labels;
                    generateRandomColorsAndLabels(labeled_segments.size(), &colors, &labels);
                    segments->resize(labeled_segments.size());

                    int point_nums = 0;
                    // Assign the colors and labels to the segments.
                    for (size_t i = 0; i < labeled_segments.size(); ++i) {

                        cv::Vec3b color;
                        if (labeled_segments[i].size() < Parameter::LabelMapParams::min_size) {
                            color = cv::Vec3b(0, 0, 0);
                        }
                        else {
                            color = cv::Vec3b(colors[i][0], colors[i][1], colors[i][2]);
                        }

                        //给深度图分割出来的区域上色。。。
                        cv::Mat segment_mask = cv::Mat(depth_map.size(), CV_8UC1, cv::Scalar(0));
                        cv::Vec3f centerPoint(0,0,0);
                        for (size_t j = 0; j < labeled_segments[i].size(); ++j) {
                            const size_t x = labeled_segments[i][j].x;
                            const size_t y = labeled_segments[i][j].y;
                            //对同一个区域内的点上色
                            output.at<cv::Vec3b>(y, x) = color;

                            cv::Vec3f point = depth_map.at<cv::Vec3f>(y, x);
                            //(*segments)[i].points.push_back(point);
                            ++point_nums;

                            segment_mask.at<uint8_t>(y, x) = kMaskValue;

                            centerPoint += point;
                        }
                        //将当前分割的中心点算出来
                        point_nums_vec.push_back(point_nums);
                        (*segments)[i].center = centerPoint / point_nums;
                        segment_masks->push_back(segment_mask);
                    }
                    break;
                }
            }

            CHECK_EQ(segments->size(), segment_masks->size());
            CHECK_EQ(segments->size(), point_nums_vec.size());

            // Remove small segments from segments vector.
            // 移除面积较小的分割
            for (size_t i = 0; i < segments->size();) {
                if (point_nums_vec[i] < Parameter::LabelMapParams::min_size) {
                    segments->erase(segments->begin() + i);
                    segment_masks->erase(segment_masks->begin() + i);
                } else {
                    ++i;
                }
            }

            *labeled_map = output;
        }

        //与maskrcnn结果结合的labelmap
        void LabelMap(const SemanticInstanceSegmentation &instance_segmentation,
                      const cv::Mat &depth_map, const cv::Mat &edge_map,
                      const cv::Mat &normal_map, cv::Mat *labeled_map,
                      std::vector<cv::Mat> *segment_masks, std::vector<Segment> *segments){

            for(int i = 0; i < instance_segmentation.masks.size(); i++){
                std::string class_name = yolo::Class_names[instance_segmentation.labels[i]];
                log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"Get %d Yolo_Segment %s confidence %f", i, class_name.c_str(), instance_segmentation.scores[i]);
            }

            // 深度图分割出来的实例...
            std::vector<depth_segmentation::Segment> segments_vec;

            // 分割出了一个个实例
            labelMap(depth_map, edge_map, normal_map, labeled_map, segment_masks, &segments_vec);

            //key是语义标号，value是该语义下深度图的segment标号
            std::map<size_t, std::vector<size_t>> merged_map;

            // 图像分割分割出的实例 与 yolo预测的mask进行重叠度比较
            // 原则是：在同一个yolo预测的mask内有多个图像分割分割出的实例符合，会检测每个图像分割分割出的实例的质心
            // 会根据质心进行聚类，最后选出点数最多的一类，其他的丢弃

            for (size_t i = 0; i < segments_vec.size(); ++i)
            {
                log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"============ Depth_Segment %d size %d ============", i, segments_vec[i].points.size());
                // For each DS segment identify the corresponding
                // maximally overlapping mask, if any.
                size_t maximally_overlapping_mask_index = 0u;
                int max_overlap_size = 0;
                int x = 0, y = 0;

                // 深度图分割出来的实例, 计算非零元素的数量
                int segment_size = cv::countNonZero((*segment_masks)[i]);
                static const std::string WindowNameResult = "Result";

                // 遍历maskrcnn分割出的实例
                for (size_t j = 0; j < instance_segmentation.masks.size(); ++j)
                {

                    // 将maskrcnn的预测实例的mask 和 分割实例 的 mask 做and运算，重叠部分不为0
                    cv::Mat mask_overlap;
                    cv::bitwise_and((*segment_masks)[i], instance_segmentation.masks[j], mask_overlap);

                    int overlap_size = cv::countNonZero(mask_overlap);

                    // 比率 = 重叠部分面积 / 分割部分面积
                    float normalized_overlap = (float)overlap_size / (float)segment_size;

                    // 当 重叠部分面积 > 某个阈值 且 比率 > 某个阈值，此实例分割结果 = 当前的分割类别
                    if (overlap_size > max_overlap_size && normalized_overlap > Parameter::SemanticInstanceSegmentationParams::overlap_threshold)
                    {
                        log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"check Yolo_Segment %d overlap_area %d overlap_ratio %f limits %d",j,overlap_size,normalized_overlap,max_overlap_size);
                        maximally_overlapping_mask_index = j;
                        max_overlap_size = overlap_size;
                        log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"pass.... max_overlap_size change %d",max_overlap_size);

                        segments_vec[i].points.clear();
                        for(int k = 0; k < mask_overlap.rows; k++){
                            for(int l = 0; l < mask_overlap.cols; l++){
                                if(mask_overlap.at<uint8>(k,l) != 0){
                                    cv::Vec3f point = depth_map.at<cv::Vec3f>(k, l);
                                    segments_vec[i].points.push_back(point);
                                }
                            }
                        }

                        cv::Moments m = cv::moments((*segment_masks)[i], true);
                        cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
                        x = center.x;
                        y = center.y;
                    }
                }

                if (max_overlap_size > 0)
                {

                    merged_map[maximally_overlapping_mask_index].push_back(i);

                    segments_vec[i].semantic_label.insert(instance_segmentation.labels[maximally_overlapping_mask_index]);

                    segments_vec[i].instance_label.insert(getFreshInstanceLabel());

                    segments_vec[i].scores.push_back(instance_segmentation.scores[maximally_overlapping_mask_index]);

                    if (Parameter::SemanticInstanceSegmentationParams::if_show_window)
                    {
                        const std::string class_name = yolo::Class_names[instance_segmentation.labels[maximally_overlapping_mask_index]];
                        cv::namedWindow(WindowNameResult, cv::WINDOW_NORMAL);
                        cv::putText(*labeled_map, class_name, cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                        imshow(WindowNameResult, *labeled_map);
                        cv::waitKey(1);
                    }

                }
            }

            /*for(const auto &it : merged_map){
                log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"语义标号 %d 融合分割图标号 %d", it.first, it.second.size());
                for (const auto &idx : it.second)
                {
                    log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"分割图标号: %d 分割图点数 %d", idx, segments_vec[idx].points.size());
                }
            }*/

            std::map<size_t, std::vector<size_t>> merged_map_handled;
            // 对一个mask内的深度segment进行质心聚类，聚类出那个类最多的点，其余抛弃
            for (const auto &it : merged_map)
            {
                //同一个语义下的分割进行遍历
                std::vector<DBSCANPoint> points;
                for (const auto &idx : it.second)
                {
                    DBSCANPoint point;
                    Segment &segment = segments_vec[idx];
                    point.x = segment.center[0];
                    point.y = segment.center[1];
                    point.z = segment.center[2];
                    point.clusterID = UNCLASSIFIED;
                    point.PointNum = segment.points.size();
                    points.push_back(point);
                }

                // 最小的聚类数量
                int MINIMUM_POINTS = 1;
                // 聚类阈值： 距离的平方
                auto EPSILON = static_cast<float>(Parameter::CommonParams::cluster_distance * Parameter::CommonParams::cluster_distance);
                DBSCAN ds(MINIMUM_POINTS, EPSILON, points);
                ds.run();

                // 获取聚类点数最多的一类的标号
                std::vector<int> vec;
                if (ds.getMaxNumIdx(vec))
                {
                    for (int i = 0; i < vec.size(); ++i)
                    {
                        //it.second 是点云
                        merged_map_handled[it.first].push_back(it.second[vec[i]]);
                    }
                }
            }

            /*log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"聚类后....");
            for(const auto &it : merged_map_handled){
                log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"语义标号 %d 融合分割图标号 %d", it.first, it.second.size());
                for (const auto &idx : it.second)
                {
                    log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"分割图标号: %d 分割图点数 %d", idx, segments_vec[idx].points.size());
                }
            }*/

            // 新增：根据实例分割的mask标号 融合
            for (const auto &it : merged_map_handled)
            {

                Segment segment_tmp;
                // 将以下的深度图分割出来的segments归为一类
                for (const auto &idx : it.second)
                {
                    Segment &segment = segments_vec[idx];
                    segment_tmp.instance_label.insert(segment.instance_label.begin(), segment.instance_label.end());
                    segment_tmp.semantic_label.insert(segment.semantic_label.begin(), segment.semantic_label.end());
                    segment_tmp.points.insert(segment_tmp.points.end(), segment.points.begin(), segment.points.end());
                    segment_tmp.scores.insert(segment_tmp.scores.end(), segment.scores.begin(), segment.scores.end());
                }
                segments->push_back(segment_tmp);
            }

            for(int i = 0; i < segments->size(); i++){
                std::string class_name = yolo::Class_names[*((*segments)[i].semantic_label.begin())];
                log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"final send object %s size %d", class_name.c_str(), (*segments)[i].points.size());
            }
            log__save("LabelMap",kLogLevel_Info, kLogTarget_Filesystem,"\n");
        }


    private:
        //深度相机
        const DepthCamera& depth_camera_;
        uint32 fresh_instance_label_;

        std::vector<cv::Scalar> colors_;
        std::vector<int> labels_;
    };
}

#endif //SEMANTIC_MAPPING_DEPTH_SEGMENTATION_H
