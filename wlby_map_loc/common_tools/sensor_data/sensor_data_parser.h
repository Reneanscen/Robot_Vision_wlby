#ifndef _SCAN_PARSER_H_
#define _SCAN_PARSER_H_

/*******************************************************************************
* @brief scan解析类
* @author jazzey
* @date 2023-8-10
********************************************************************************/
#include <memory>
#include "sensor_data/sensor_data.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace wlby {

class SensorDataParser{

public:
    SensorDataParser(){};

    std::shared_ptr<ScanSensorData> parse(const sensor_msgs::msg::LaserScan &msg);

    void setUseScanRangeMin(double used_scan_range_min){
        used_scan_range_min_ = used_scan_range_min;
    }

    void setUseScanRangeMax(double used_scan_range_max){
        used_scan_range_max_ = used_scan_range_max;
    }

    void setUseScanDegStart(double used_scan_deg_start){
        used_scan_deg_start_ = used_scan_deg_start;
    }

    void setUseScanDegEnd(double used_scan_deg_end){
        used_scan_deg_end_ = used_scan_deg_end;
    }

    std::shared_ptr<OdoSensorData> parse(const nav_msgs::msg::Odometry &msg);

    std::shared_ptr<RGBCameraSensorData> parse(const sensor_msgs::msg::Image &msg);
    std::shared_ptr<DepthCameraSensorData> parseDepthMsg(const sensor_msgs::msg::Image &msg);
    std::shared_ptr<RGBDCameraSensorData> parse(const sensor_msgs::msg::Image &rgb_msg, const sensor_msgs::msg::Image &depth_msg);
private:
    double used_scan_range_min_ = 0;
    double used_scan_range_max_ = 12.0;
    double used_scan_deg_start_ = -180.0;
    double used_scan_deg_end_ = 180.0;

};   
}

#endif