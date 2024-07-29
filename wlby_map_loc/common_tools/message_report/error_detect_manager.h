#ifndef _ERROR_DETECT_MANAGER_H_
#define _ERROR_DETECT_MANAGER_H_
/*******************************************************************************
* @brief 错误检测上报类
* @author jazzey
* @date 2023-8-8
********************************************************************************/
#include <thread>
#include <mutex>
#include <iostream>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "common/time_utils.h"
#include "common/sleep.h"

#include "logger/my_logger.h"

namespace wlby{

    class ErrorDetectManager{

    public:
        ErrorDetectManager()= default;

        void checkIfReceiveAllSensorData(){

            log__save("LocInit",kLogLevel_Info,kLogTarget_Filesystem, "Check If Receive Data Loop Start ...");
            std::thread t(&ErrorDetectManager::check_func,this);
            t.detach();
        }

        void checkScanSensorData(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg){

            if_receive_scan_ = true;
            check_scan_timeout_func(msg);
            check_scan_normal_func(msg);

        }

        void checkOdoSensorData(const nav_msgs::msg::Odometry::ConstSharedPtr &msg){

            if_receive_odo_ = true;
            check_odo_timeout_func(msg);
            check_odo_normal_func(msg);
        }

    private:

        void check_func();
        void check_scan_timeout_func(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg);
        void check_odo_timeout_func(const nav_msgs::msg::Odometry::ConstSharedPtr &odo_msg);
        static void check_odo_normal_func(const nav_msgs::msg::Odometry::ConstSharedPtr &odo_msg);
        static void check_scan_normal_func(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg);

    private:

        double last_scan_time_ = -1;

        double last_odo_time_ = -1;

        int odo_sensor_receive_timeout_count = 150;
        int scan_sensor_receive_timeout_count = 150;
        int start_odo_time_out_count_ = 0;
        int start_scan_time_out_count_ = 0;
        bool if_receive_scan_ = false;
        bool if_receive_odo_ = false;


        double scan_sensor_timeout_ = 1.0;
        double odo_sensor_timeout_ = 0.3;

    };

}


#endif