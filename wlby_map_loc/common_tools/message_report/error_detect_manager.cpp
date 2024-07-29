#include "message_report/error_detect_manager.h"
#include <cmath>
#include <iostream>

#include "common/sleep.h"
#include "common/time_utils.h"
#include "message_report/message_report.h"


namespace wlby{

    void ErrorDetectManager::check_func(){
        while(1){
            SleepUtils::Sleep(100);

            if(!if_receive_scan_) {
                ++start_scan_time_out_count_;
            }else{
                log__save("LocInit",kLogLevel_Info,kLogTarget_Filesystem, "Check If Receive Data Loop End ...");
                break;
            }
            if(start_scan_time_out_count_ == scan_sensor_receive_timeout_count){
                MESSAGE_REPORT_ERROR(1006,"Long Time Not Receive Scan Data.....");
            }


            if(!if_receive_odo_) {
                ++start_odo_time_out_count_;
            }else{
                break;
            }
            if(start_odo_time_out_count_ == odo_sensor_receive_timeout_count){
                MESSAGE_REPORT_ERROR(1005,"Long Time Not Receive Odo Data.....");
            }

        }
    }

    void ErrorDetectManager::check_scan_timeout_func(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg){

        if(scan_msg == nullptr){
            return;
        }

        double scan_time = TimeUtils::RosTimeToAbsTime(scan_msg->header.stamp);

        if(last_scan_time_ < 0){
            last_scan_time_ = scan_time;
            return;
        }

        double scan_gap_time = scan_time - last_scan_time_;
        if(scan_gap_time < 0){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "last scan time: %f, cur scan time %f",last_scan_time_,scan_time);
            MESSAGE_REPORT_ERROR(1009,"Last Scan Time Is Newer Than Current Scan Time.....");
        }else if(scan_gap_time >= scan_sensor_timeout_){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "last scan time: %f, cur scan time %f",last_scan_time_,scan_time);
            MESSAGE_REPORT_ERROR(1010,"Scan Time Out...");
        }
        last_scan_time_ = scan_time;
    }

    void ErrorDetectManager::check_odo_timeout_func(const nav_msgs::msg::Odometry::ConstSharedPtr &odo_msg) {
        if(odo_msg == nullptr){
            return;
        }

        double odo_time = TimeUtils::RosTimeToAbsTime(odo_msg->header.stamp);

        if(last_odo_time_ < 0){
            last_odo_time_ = odo_time;
            return;
        }
        double gap_time = odo_time - last_odo_time_;
        if(gap_time < 0){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "last odo time: %f, cur odo time %f",last_odo_time_,odo_time);
            MESSAGE_REPORT_ERROR(1007,"Last Odo Time Is Newer Than Current Odo Time.....");
        }else if(gap_time >= odo_sensor_timeout_){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "last odo time: %f, cur odo time %f",last_odo_time_,odo_time);
            MESSAGE_REPORT_ERROR(1008,"Odo Time Out...");
        }
        last_odo_time_ = odo_time;
    }

    void ErrorDetectManager::check_odo_normal_func(const nav_msgs::msg::Odometry::ConstSharedPtr &odo_msg){

        double pos_x = odo_msg->pose.pose.position.x;
        double pos_y = odo_msg->pose.pose.position.y;
        double w = odo_msg->pose.pose.orientation.w;
        double x = odo_msg->pose.pose.orientation.x;
        double y = odo_msg->pose.pose.orientation.y;
        double z = odo_msg->pose.pose.orientation.z;
        if( std::isnan(pos_x) || std::isnan(pos_y)){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "Odo x y Data Is NAN, x: %f, y: %f",pos_x,pos_y);
            MESSAGE_REPORT_ERROR(1014,"Odo x y Data Is NAN");
        }
        if( std::isinf(pos_x) || std::isinf(pos_y)){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "Odo x y Data Is INF, x: %f, y: %f",pos_x,pos_y);
            MESSAGE_REPORT_ERROR(1015,"Odo x y Data Is INF");
        }
        if(std::isnan(w) || std::isnan(x) || std::isnan(y) || std::isnan(z)){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "Odo deg Data Is NAN, w: %f x: %f, y: %f, z: %f",w,x,y,z);
            MESSAGE_REPORT_ERROR(1016,"Odo Deg Data Is NAN");
        }
        if(std::isinf(w) || std::isinf(x) || std::isinf(y) || std::isinf(z)){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "Odo deg Data Is NAN, w: %f x: %f, y: %f, z: %f",w,x,y,z);
            MESSAGE_REPORT_ERROR(1017,"Odo Deg Data Is INF");
        }
    }
    void ErrorDetectManager::check_scan_normal_func(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg){
        int n = scan_msg->ranges.size();
        if(n == 0){
            MESSAGE_REPORT_ERROR(1018,"2D Laser Data Error! Range Size 0");
        }
        if(n < 100){
            log__save("Error",kLogLevel_Info,kLogTarget_Filesystem, "2D Laser Data Too Small! Range Size %d", n);
            MESSAGE_REPORT_WARNING(2008,"2D Laser Data Too Small!");
        }
        bool flag = true;
        for(int i = 0; i < n; i++){
            if(!std::isnan(scan_msg->ranges[i]) || !std::isinf(scan_msg->ranges[i])){
                flag = false;
                break;
            }
        }
        if(flag){
            MESSAGE_REPORT_ERROR(1019,"2D Laser Data Error! Data is NAN OR INF");
        }
    }
}