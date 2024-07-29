#include "sensor_data/sensor_data_parser.h"
#include <algorithm>

#include <opencv2/rgbd.hpp>
#include <cv_bridge/cv_bridge.h>
#include "check/check.h"
#include "common/time_utils.h"
#include "PoseUtils/angle_math_utils.h"
#include "PoseUtils/Pose3DDataStruct.h"

using namespace Localization;

namespace wlby{

    std::shared_ptr<ScanSensorData> SensorDataParser::parse(const sensor_msgs::msg::LaserScan &msg){

        CHECK_GE(msg.range_min, 0.f);
        CHECK_GE(msg.range_max, msg.range_min);

        if (msg.angle_increment > 0.f) {
            CHECK_GT(msg.angle_max, msg.angle_min);
        } else {
            CHECK_GT(msg.angle_min, msg.angle_max);
        }

        if (msg.intensities.size() > 0) {
            CHECK_EQ(msg.intensities.size(), msg.ranges.size());
        }

        static uint64 scan_data_index = 0;
        std::shared_ptr<ScanSensorData> scan_sensor_data_ptr = std::make_shared<ScanSensorData>(0.0);
        scan_sensor_data_ptr->_dataIndex = scan_data_index;
        ++scan_data_index;

        float angle_rad = msg.angle_min;
        double angle_deg = angle_math_utils::rad_to_deg(angle_rad);
        angle_deg = angle_math_utils::normalize_deg(angle_deg);

        std::vector<ScanSensorData::Beam> beam_vec;
        for (size_t i = 0; i < msg.ranges.size(); ++i) {

            const auto& range = msg.ranges[i];
            if(range < used_scan_range_min_ || range > used_scan_range_max_) {
                
                angle_deg += angle_math_utils::rad_to_deg(msg.angle_increment);
                angle_deg = angle_math_utils::normalize_deg(angle_deg);
                
                continue;
            }

            //删除两个柱子
            if((0 < angle_deg && angle_deg < 11) || (169 < angle_deg && angle_deg < 180) ){

                angle_deg += angle_math_utils::rad_to_deg(msg.angle_increment);
                angle_deg = angle_math_utils::normalize_deg(angle_deg);

                continue;
            }

            if(used_scan_deg_start_ <= angle_deg && angle_deg <= used_scan_deg_end_) {

                if (msg.range_min <= range && range <= msg.range_max) {

                    double point_time =  i * msg.time_increment;
                    ScanSensorData::Beam beam(angle_deg,range,point_time,msg.intensities[i],true);

                    //std::cout << std::fixed << std::setprecision(6)<<"range: "<<range<<" angle: "<< angle_deg<<"  "<<beam._pos._x<<","<<beam._pos._y <<std::endl;


                    beam_vec.push_back(beam);

                }

            }


            angle_deg += angle_math_utils::rad_to_deg(msg.angle_increment);
            angle_deg = angle_math_utils::normalize_deg(angle_deg);
        }
        std::sort(beam_vec.begin(), beam_vec.end(), [](const ScanSensorData::Beam& a, ScanSensorData::Beam& b) {
            return a._polar._deg < b._polar._deg;
        });
        scan_sensor_data_ptr->_beamVec = beam_vec;

        //修改时间戳
        double timestamp = TimeUtils::RosTimeToAbsTime(msg.header.stamp);
        if (scan_sensor_data_ptr->_beamVec.size() != 0) {
            const double duration = scan_sensor_data_ptr->_beamVec.back()._posTime;
            for (auto& point : scan_sensor_data_ptr->_beamVec) {
                point._posTime -= duration;
            }
        }

        scan_sensor_data_ptr->_localTime = timestamp;

        CHECK_GT(scan_sensor_data_ptr->_localTime, 0);


        return scan_sensor_data_ptr;

 }  

    std::shared_ptr<OdoSensorData> SensorDataParser::parse(const nav_msgs::msg::Odometry &msg){

        double timestamp = TimeUtils::RosTimeToAbsTime(msg.header.stamp);
    
        static uint64 odo_data_index = 0;
        std::shared_ptr<OdoSensorData> odo_sensor_data_ptr = std::make_shared<OdoSensorData>(timestamp);
        odo_sensor_data_ptr->_dataIndex = odo_data_index;
        ++odo_data_index;

        CHECK_GT(odo_sensor_data_ptr->_localTime, 0);


        double quat[4] = {msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z};
        Orient3D orient(quat);
        orient.uniformByQuat();

        odo_sensor_data_ptr->_odoPose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, orient._eulerDeg[0]);
        odo_sensor_data_ptr->_speedX = msg.twist.twist.linear.x;
        odo_sensor_data_ptr->_speedY = msg.twist.twist.linear.y;
        odo_sensor_data_ptr->_angleSpeedDeg = angle_math_utils::rad_to_deg(msg.twist.twist.angular.z);

        return odo_sensor_data_ptr;

    }

    std::shared_ptr<RGBCameraSensorData> SensorDataParser::parse(const sensor_msgs::msg::Image &msg){
        double timestamp = TimeUtils::RosTimeToAbsTime(msg.header.stamp);
        static uint64 rgb_camera_data_index = 0;
        std::shared_ptr<RGBCameraSensorData> rgb_camera_sensor_data_ptr = std::make_shared<RGBCameraSensorData>(timestamp);
        rgb_camera_sensor_data_ptr->_dataIndex = rgb_camera_data_index;
        ++rgb_camera_data_index;
        CHECK_GT(rgb_camera_sensor_data_ptr->_localTime, 0);
        cv_bridge::CvImagePtr cv_RGB_image(new cv_bridge::CvImage);
        if(msg.encoding == sensor_msgs::image_encodings::BGR8){
            cv_RGB_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            rgb_camera_sensor_data_ptr->_rgb_mat = cv_RGB_image->image;
        }else if(msg.encoding == sensor_msgs::image_encodings::RGB8){
            cv_RGB_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            rgb_camera_sensor_data_ptr->_rgb_mat = cv_RGB_image->image;
        }
        return rgb_camera_sensor_data_ptr;
    }
    std::shared_ptr<DepthCameraSensorData> SensorDataParser::parseDepthMsg(const sensor_msgs::msg::Image &msg){
        double timestamp = TimeUtils::RosTimeToAbsTime(msg.header.stamp);
        static uint64 depth_camera_data_index = 0;
        std::shared_ptr<DepthCameraSensorData> depth_camera_sensor_data_ptr = std::make_shared<DepthCameraSensorData>(timestamp);
        depth_camera_sensor_data_ptr->_dataIndex = depth_camera_data_index;
        ++depth_camera_data_index;
        CHECK_GT(depth_camera_sensor_data_ptr->_localTime, 0);
        cv_bridge::CvImagePtr cv_depth_image(new cv_bridge::CvImage);
        if(msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1){
            cv_depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_camera_sensor_data_ptr->_depth_mat = cv::Mat::zeros(cv_depth_image->image.size(), CV_32FC1);
            cv::rgbd::rescaleDepth(cv_depth_image->image, CV_32FC1, depth_camera_sensor_data_ptr->_depth_mat );
        }else if(msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            cv_depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_camera_sensor_data_ptr->_depth_mat = cv_depth_image->image;
        }
        return depth_camera_sensor_data_ptr;
    }
    std::shared_ptr<RGBDCameraSensorData> SensorDataParser::parse(const sensor_msgs::msg::Image &rgb_msg, const sensor_msgs::msg::Image &depth_msg){
        std::shared_ptr<RGBCameraSensorData> rgb_camera_sensor_data = parse(rgb_msg);
        std::shared_ptr<DepthCameraSensorData> depth_camera_sensor_data = parseDepthMsg(depth_msg);
        double timestamp = (rgb_camera_sensor_data->_localTime + depth_camera_sensor_data->_localTime) / 2;
        static uint64 rgbd_camera_data_index = 0;
        std::shared_ptr<RGBDCameraSensorData> rgbd_camera_sensor_data_ptr = std::make_shared<RGBDCameraSensorData>(timestamp);
        rgbd_camera_sensor_data_ptr->_dataIndex = rgbd_camera_data_index;
        ++rgbd_camera_data_index;
        rgbd_camera_sensor_data_ptr->_rgb_sensor_data = *rgb_camera_sensor_data;
        rgbd_camera_sensor_data_ptr->_depth_sensor_data = *depth_camera_sensor_data;
        return rgbd_camera_sensor_data_ptr;
    }
}
