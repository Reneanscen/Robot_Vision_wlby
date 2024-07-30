//
// Created by xu on 24-3-21.
//

#ifndef BIRDVIEW_USB_CIRCLE_FISHEYE_H
#define BIRDVIEW_USB_CIRCLE_FISHEYE_H

#include <utility>
#include "vector"
#include "algorithm"
#include <deque>
#include <mutex>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "rclcpp/rclcpp.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "rcpputils/endian.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <message_filters/subscriber.h>
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <custom_image_msg/msg/image4m.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#include "parameter/parameter.h"
#include "camera_model/fisheye_model.h" // 鱼眼相机工具
#include "camera_model/pinhole_model.h" // 针孔相机工具
#include "time_utils/tictoc.h"          // 计时工具
#include "thread_pool/thread_pool.h"    // 线程池工具


// v4l2取流头文件
#include "v4l2_stream/v4l2_capture/v4l2_capture.h"

using namespace cv;
using namespace std;

using ROS_IMAGE_TYPE = sensor_msgs::msg::Image ;
using CUSTOMIMAGE_TYPE = custom_image_msg::msg::Image4m;

// 自定义图像数据结构
struct ImageWithStamp {
    ImageWithStamp() = default;

    ~ImageWithStamp() = default;

    ImageWithStamp(const cv::Mat &image, double timestamp) {
        image_ = image;
        timestamp_ = timestamp;
    }

    // 拷贝构造函数
    ImageWithStamp(const ImageWithStamp &other) : image_(other.image_), timestamp_(other.timestamp_) {}

    // 拷贝赋值运算符
    ImageWithStamp& operator=(const ImageWithStamp &other) {
        if (this != &other) {
            image_ = other.image_;
            timestamp_ = other.timestamp_;
        }
        return *this;
    }

    cv::Mat image_;
    double timestamp_{};
};

// v4l2相机对象类
class V4L2StreamUtil{
public:
    V4L2StreamUtil(const std::string &dev, int format, int width, int height, int fps) : dev_(dev), format_(format), width_(width), height_(height), fps_(fps) {
        // 设置v4l2取流的参数
        v4L2Capture_.set_device(dev_);
        v4L2Capture_.set_resolution(width_, height_);
        v4L2Capture_.set_frame_rate(fps_);

        if(v4L2Capture_.init(format_) == 0){
            log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Successfully init v4l2 camera.", dev_.c_str());
        }else{
            log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Unsuccessfully init v4l2 camera.", dev_.c_str());
        }

        if(v4L2Capture_.camera_capture_start() == 0){
            log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Successfully start v4l2 camera.", dev_.c_str());
        }else{
            log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s Unsuccessfully start v4l2 camera.", dev_.c_str());
        }

        log__save("V4L2StreamUtil", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%s loopUnit...", dev_.c_str());
        thread_ = std::thread(&V4L2StreamUtil::loopUnit, this);
        thread_.detach();
    }

    void loopUnit(){
        while (rclcpp::ok() && !cancel_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));

            cv::Mat rgbImage;

            buffer_ = v4L2Capture_.get_one_frame(); // 获取数据

            if(format_ == 0){
                // Decode YUYV
                cv::Mat img = cv::Mat(cv::Size(width_, height_), CV_8UC2, (uint8_t *) buffer_.data);
                cv::cvtColor(img, rgbImage, cv::COLOR_YUV2RGB_YVYU);
            }else if(format_ == 1){
                // Decode MJPEG
                std::vector<uint8_t> mjpg_data((uint8_t *)buffer_.data, (uint8_t *)buffer_.data + buffer_.length);
                rgbImage = cv::imdecode(mjpg_data, cv::IMREAD_COLOR);
            }else if(format_ == 2){
                // todo: BGR24 decode
            }else{
                // log
            }

            // 向外发送数据
            setImageWithStamp(std::make_shared<ImageWithStamp>(rgbImage, getSystemTimestamp()));

            v4L2Capture_.clear_one_frame(); // 清除缓存
        }
    }

    void setImageWithStamp(const std::shared_ptr<ImageWithStamp> imageWithStamp_ptr){
        std::lock_guard<std::mutex> lock(image_timestamp_mtx_);
        imageWithStamp_ptr_ = imageWithStamp_ptr;
    }

    void getImageWithStamp(std::shared_ptr<ImageWithStamp>& imageWithStamp_ptr){
        std::lock_guard<std::mutex> lock(image_timestamp_mtx_);
        imageWithStamp_ptr = imageWithStamp_ptr_;
    }

    // ***************************** 工具部分 ***************************** //
    // YUV422转Mat
    void yuyv422_to_mat(const uint8_t * yuyv_data, int width, int height, cv::Mat& mat){
        mat.create(height, width, CV_8UC3);
        int yuyv_index = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x += 2) {
                int y1 = yuyv_data[yuyv_index++];
                int u = yuyv_data[yuyv_index++];
                int y2 = yuyv_data[yuyv_index++];
                int v = yuyv_data[yuyv_index++];

                int r1 = y1 + 1.13983 * (v - 128);
                int g1 = y1 - 0.39465 * (u - 128) - 0.58060 * (v - 128);
                int b1 = y1 + 2.03211 * (u - 128);

                int r2 = y2 + 1.13983 * (v - 128);
                int g2 = y2 - 0.39465 * (u - 128) - 0.58060 * (v - 128);
                int b2 = y2 + 2.03211 * (u - 128);

                mat.at<cv::Vec3b>(y, x)[0] = cv::saturate_cast<unsigned char>(b1);
                mat.at<cv::Vec3b>(y, x)[1] = cv::saturate_cast<unsigned char>(g1);
                mat.at<cv::Vec3b>(y, x)[2] = cv::saturate_cast<unsigned char>(r1);

                mat.at<cv::Vec3b>(y, x + 1)[0] = cv::saturate_cast<unsigned char>(b2);
                mat.at<cv::Vec3b>(y, x + 1)[1] = cv::saturate_cast<unsigned char>(g2);
                mat.at<cv::Vec3b>(y, x + 1)[2] = cv::saturate_cast<unsigned char>(r2);
            }
        }
    }

    // YUV2(YUYV)转I420
    void YUY2toI420(int inWidth, int inHeight, uint8_t *pSrc, uint8_t *pDest){
        int i, j;
        //首先对I420的数据整体布局指定
        uint8_t *u = pDest + (inWidth * inHeight);
        uint8_t *v = u + (inWidth * inHeight) / 4;

        for (i = 0; i < inHeight / 2; i++) {
            /*采取的策略是:在外层循环里面，取两个相邻的行*/
            uint8_t *src_l1 = pSrc + inWidth * 2 * 2 * i;//因为4:2:2的原因，所以占用内存，相当一个像素占2个字节，2个像素合成4个字节
            uint8_t *src_l2 = src_l1 + inWidth * 2;//YUY2的偶数行下一行
            uint8_t *y_l1 = pDest + inWidth * 2 * i;//偶数行
            uint8_t *y_l2 = y_l1 + inWidth;//偶数行的下一行
            for (j = 0; j < inWidth / 2; j++)//内层循环
            {
                // two pels in one go//一次合成两个像素
                //偶数行，取完整像素;Y,U,V;偶数行的下一行，只取Y
                *y_l1++ = src_l1[0];//Y
                *u++ = src_l1[1];//U
                *y_l1++ = src_l1[2];//Y
                *v++ = src_l1[3];//V
                //这里只有取Y
                *y_l2++ = src_l2[0];
                *y_l2++ = src_l2[2];
                //YUY2,4个像素为一组
                src_l1 += 4;
                src_l2 += 4;
            }
        }
    }

    double getSystemTimestamp(){
        struct timespec thread_cpu_time{};
        clock_gettime(CLOCK_REALTIME, &thread_cpu_time);
        double cur_timestamp = thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
        return cur_timestamp;
    }

private:
    std::string dev_;
    int format_;
    int width_;
    int height_;
    int fps_;

    V4L2Capture v4L2Capture_;
    Buffer buffer_{};

    std::atomic<bool> cancel_ {false}; // 原子变量，可以确保操作是不会被其他线程影响
    std::thread thread_;

    std::mutex image_timestamp_mtx_;
    std::shared_ptr<ImageWithStamp> imageWithStamp_ptr_ = nullptr;
};


class BirdVision{
public:
    explicit BirdVision(rclcpp::Node::SharedPtr node);

private:
    // ROS参数配置
    void declareParameters();

    void getParameters();

    void logParameters();

    bool parameterCallback(const std::vector<rclcpp::Parameter> &parameters);

    // 数据获取方式———通过ROS订阅的方式获取数据
    //  way1: Custom message
    void frontCustome(const CUSTOMIMAGE_TYPE::ConstSharedPtr &image){
        std::lock_guard<std::mutex> lock(front_mtx_);
        front_image_ptr_ = std::make_shared<ImageWithStamp>(matFromCustome(std::make_shared<CUSTOMIMAGE_TYPE>(*image)),
                                                            image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }

    void leftCustome(const CUSTOMIMAGE_TYPE::ConstSharedPtr &image) {
        std::lock_guard<std::mutex> lock(left_mtx_);
        left_image_ptr_ = std::make_shared<ImageWithStamp>(matFromCustome(std::make_shared<CUSTOMIMAGE_TYPE>(*image)),
                                                            image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }

    void backCustome(const CUSTOMIMAGE_TYPE::ConstSharedPtr &image) {
        std::lock_guard<std::mutex> lock(back_mtx_);
        back_image_ptr_ = std::make_shared<ImageWithStamp>(matFromCustome(std::make_shared<CUSTOMIMAGE_TYPE>(*image)),
                                                            image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }

    void rightCustome(const CUSTOMIMAGE_TYPE::ConstSharedPtr &image){
        std::lock_guard<std::mutex> lock(right_mtx_);
        right_image_ptr_ = std::make_shared<ImageWithStamp>(matFromCustome(std::make_shared<CUSTOMIMAGE_TYPE>(*image)),
                                                            image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }

    //  way2: ROS message
    void frontROS(const ROS_IMAGE_TYPE ::ConstSharedPtr &image) {
        std::lock_guard<std::mutex> lock(front_mtx_);
        front_image_ptr_ = std::make_shared<ImageWithStamp>(matFromROS(*image),
                                                            image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }

    void leftROS(const ROS_IMAGE_TYPE::ConstSharedPtr &image) {
        std::lock_guard<std::mutex> lock(left_mtx_);
        left_image_ptr_ = std::make_shared<ImageWithStamp>(matFromROS(*image),
                                                           image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }

    void backROS(const ROS_IMAGE_TYPE::ConstSharedPtr &image) {
        std::lock_guard<std::mutex> lock(back_mtx_);
        back_image_ptr_ = std::make_shared<ImageWithStamp>(matFromROS(*image),
                                                            image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }

    void rightROS(const ROS_IMAGE_TYPE::ConstSharedPtr &image) {
        std::lock_guard<std::mutex> lock(right_mtx_);
        right_image_ptr_ = std::make_shared<ImageWithStamp>(matFromROS(*image),
                                                            image->header.stamp.sec + image->header.stamp.nanosec * 1e-9);
    }


    // 鸟瞰图处理主线程框架
    void birdViewHandleThread();

    // 鸟瞰图处理核心逻辑
    void birdImageHandle(const cv::Mat& origin_image, const std::string& camera_index, const CamParameter& camParameter,
                         std::vector<cv::Point2f> &map_table,  cv::Mat &bird_image) const;


    // Sigmoid函数共视区域的图像融合
    static float callsigmoid(float x, float k, float x0) {
        return 1.0 / (1.0 + exp(-k * (x - x0)));
    }

    // ---------------------------------------- 初始化有效区域以及重合区域 ----------------------------
    // 初始化四路相机鸟瞰图有效区域
    void initEffectArea(int width, int height, int offset_pixels);

    // 在整个鸟瞰图上进行重合区域提取
    void extrectOverLay(const cv::Mat& frontImage, const cv::Mat& leftImage,
                        const cv::Mat& backImage, const cv::Mat& rightImage);

    // 图像二值化并基于轮廓去除轮廓内的噪点
    static cv::Mat BinaryAndNoiseRemoval(const cv::Mat& inputImg);

    // ---------------------------------------- 3A调节 -----------------------------------
    // RGB通道亮度平衡
    void RGBBrightBalance(cv::Mat &frontImg, cv::Mat &leftImg, cv::Mat &backImg, cv::Mat &rightImg);

    // YUV通道亮度平衡 表示暗亮程度的亮度信号(Luminance)Y，和两个表示颜色的色度信号(Chrominance)U 和 V
    void YUVBrightBalance(cv::Mat &frontImg, cv::Mat &leftImg, cv::Mat &backImg, cv::Mat &rightImg);

    // HSV通道亮度平衡 H，S，V 这三个通道分别代表着色相(Hue)，饱和度(Saturation)和明度(Value)。
    void HSVBrightBalance(cv::Mat &frontImg, cv::Mat &leftImg, cv::Mat &backImg, cv::Mat &rightImg);

    // 根据灰度、Gamma归一化亮度  转到LAB，统一色度
    cv::Mat GraygammaAndLAB(const cv::Mat& image){

        cv::Mat image_py;
        image.copyTo(image_py);

        cv::Mat m_gray;
        cv::cvtColor(image_py, m_gray, cv::COLOR_BGR2GRAY);
        float Gamma = log(128.0 / 255.0) / log(cv::mean(m_gray)[0] / 255.0);
        cv::Mat lookUpTable = cv::Mat::zeros(cv::Size(1, 256), CV_8UC1);
        for (int i = 0; i < 256; i++)
            lookUpTable.at<uchar>(0, i) = pow(i / 255.0, Gamma) * 255.0;
        cv::LUT(image_py, lookUpTable, image_py);

        cv::Mat m_lab;
        cv::cvtColor(image_py, m_lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> split_lab;
        cv::split(m_lab, split_lab);
        cv::convertScaleAbs(split_lab[1], split_lab[1], 128.0 / cv::mean(split_lab[1])[0], 0.0);
        cv::convertScaleAbs(split_lab[2], split_lab[2], 128.0 / cv::mean(split_lab[2])[0], 0.0);

        cv::Mat result;
        cv::merge(split_lab, result);
        cv::cvtColor(result, result, cv::COLOR_Lab2BGR);
        return result;
    }

    // ---------------------------------------- ROS消息转换 -------------------------------------

    // 获取系统时间
    static builtin_interfaces::msg::Time getRosTime();

    // image4M 转 cv::Mat
    static cv::Mat matFromCustome(const CUSTOMIMAGE_TYPE::SharedPtr &source);

    // ROS_IMAGE_TYPE 转 cv::Mat
    static cv::Mat matFromROS(const ROS_IMAGE_TYPE &msg);

    // cv::Mat 转 ROS_IMAGE_TYPE
    static ROS_IMAGE_TYPE rosFromMat(const cv::Mat& mat, const builtin_interfaces::msg::Time& msg_time);

    // cv::Mat 转 CUSTOMIMAGE_TYPE
    static void setLoanedMessage(const builtin_interfaces::msg::Time &tm, const std::string &frame_id,
                                 const cv::Mat &img, rclcpp::LoanedMessage<CUSTOMIMAGE_TYPE> &loanedMsg);

private:
    rclcpp::Node::SharedPtr node_;

    // 动态调参
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_event_subscriber_;  //OnSetParametersCallbackHandle 变量
    std::mutex param_mtx_;

    // 参数部分
    CamParameter front_cam_, left_cam_, back_cam_, right_cam_;
    BirdVisionParameter birdVisionParameter_;


    // ----------------------- 数据获取部分 -------------------
    // v4l2取流线程
    std::vector<std::shared_ptr<V4L2StreamUtil>> v4l2StreamUtil_vec_;       // v4l2取流工具对象队列 前 左 后 右的顺序初始化取流对象

    // 订阅四路相机图像
    rclcpp::CallbackGroup::SharedPtr front_group_, left_group_, back_group_, right_group_;
    rclcpp::Subscription<ROS_IMAGE_TYPE>::SharedPtr ros_sub_front_ptr_, ros_sub_left_ptr_, ros_sub_back_ptr_, ros_sub_right_ptr_;
    rclcpp::Subscription<CUSTOMIMAGE_TYPE>::SharedPtr custome_sub_front_ptr_, custome_sub_left_ptr_, custome_sub_back_ptr_, custome_sub_right_ptr_;
    std::mutex front_mtx_, left_mtx_, back_mtx_, right_mtx_;
    std::shared_ptr<ImageWithStamp> front_image_ptr_, left_image_ptr_, back_image_ptr_, right_image_ptr_;
    std::thread birdview_thread_; // 鸟瞰图处理线程

    // 发布处理结果
    rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_front_bird_vision_ptr_;   // 前视鸟瞰图融合效果
    rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_left_bird_vision_ptr_;    // 左视鸟瞰图融合效果
    rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_back_bird_vision_ptr_;    // 后视鸟瞰图融合效果
    rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_right_bird_vision_ptr_;   // 右视鸟瞰图融合效果
    rclcpp::Publisher<ROS_IMAGE_TYPE>::SharedPtr pub_all_bird_vision_ptr_;     // 全景鸟瞰图融合效果
    rclcpp::Publisher<CUSTOMIMAGE_TYPE>::SharedPtr pub_cutome_birdview_ptr_;   // 发布hhy需要的推流图

    // 环视相机鸟瞰图处理线程池
    std::unique_ptr<zl::ThreadPool> thread_pool_ptr_;

    // 人工设置鸟瞰子图有效区域（initEffectArea函数生成的有效区域）
    cv::Mat front_effect_areas_, left_effect_areas_, back_effect_areas_, right_effect_areas_;
    // 重叠区域的计算(整个鸟瞰图上的重叠区域计算）
    cv::Mat front_mid_, left_mid_, back_mid_, right_mid_;
    cv::Mat front_overlay_, left_overlay_, back_overlay_, right_overlay_;

    // 系统初始化（mask掩码以及映射表初始化）
    bool system_init_flag = false;
    std::vector<cv::Point2f> front_remap_, left_remap_, back_remap_, right_remap_;

    // 前视、左视、后视、右视相机鸟瞰子图（会先从人工设置的有效区域内进行填充鸟瞰，可以降低计算量）
    cv::Mat front_bird_vision_image_, left_bird_vision_image_, back_bird_vision_image_, right_bird_vision_image_;
    cv::Mat all_bird_image;  // 整个鸟瞰图

    // 图像平衡处理变量
    double bgr[12]{};
    double yuv[4]{};
    double hsv[4] = {0.0};
};

#endif //BIRDVIEW_USB_CIRCLE_FISHEYE_H
