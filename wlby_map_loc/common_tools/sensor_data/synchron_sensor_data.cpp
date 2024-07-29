#include "sensor_data/synchron_sensor_data.h"
#include "message_report/message_report.h"

namespace wlby{

    void SynchronSensorData::unDistort(std::shared_ptr<ScanSensorData>& scan_sensor_data){
        auto Pn = scan_sensor_data->_odoData._odoPose;
        for (size_t i = 0; i < scan_sensor_data->_beamVec.size(); ++i) {
            Pose2D Pi = scan_sensor_data->_beamVec[i]._odoPose;
            Pos2D Xi = scan_sensor_data->_beamVec[i]._pos_in_odo;
            Pose2D trans = Pi.getInvPose() * Pn;
            Pos2D Xn = (trans * Xi);
            //这里去畸变后，坐标依旧是雷达系
            scan_sensor_data->_beamVec[i]._undistortionPos_in_odo = Xn;
        }

    }

    void SynchronSensorData::Run(){

        while (1) {
            SleepUtils::Sleep(50);

            std::shared_ptr<BaseSensorData> cur_sensor_data_ptr;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                cur_sensor_data_ptr = current_sensor_data_;
            }
            if(cur_sensor_data_ptr == nullptr){
                continue;
            }
            if(last_sensor_data_ != nullptr && last_sensor_data_ == cur_sensor_data_ptr){
                continue;
            }
            last_sensor_data_ = cur_sensor_data_ptr;

            if(cur_sensor_data_ptr->_sensorType == SENSOR_TYPE::SCAN){
                std::shared_ptr<ScanSensorData> cur_scan_data_ptr = std::dynamic_pointer_cast<ScanSensorData>(cur_sensor_data_ptr);
            //将数据从雷达系转换到车体系
            for (auto &beam : cur_scan_data_ptr->_beamVec) {
                beam._pos_in_odo = laser_to_odo_*beam._pos;
            }
           
            //给每个点根据odo插值赋值位姿
            if(!SensorDataBuffer::instance()->assignScanDataPointsOdoPose(cur_scan_data_ptr)){
                continue;
            }

            //去畸变
            unDistort(cur_scan_data_ptr);
            }
            else if(cur_sensor_data_ptr->_sensorType == SENSOR_TYPE::RGB_CAMERA){
                std::shared_ptr<RGBCameraSensorData> cur_rgb_camera_data_ptr = std::dynamic_pointer_cast<RGBCameraSensorData>(cur_sensor_data_ptr);
                if(!SensorDataBuffer::instance()->assignRGBOdoPose(cur_rgb_camera_data_ptr)){
                    continue;
                }
            }
            else if(cur_sensor_data_ptr->_sensorType == SENSOR_TYPE::DEPTH_CAMERA){
                std::shared_ptr<DepthCameraSensorData> cur_depth_camera_data_ptr = std::dynamic_pointer_cast<DepthCameraSensorData>(cur_sensor_data_ptr);
                if(!SensorDataBuffer::instance()->assignDepthOdoPose(cur_depth_camera_data_ptr)){
                    continue;
                }
            }
            else if(cur_sensor_data_ptr->_sensorType == SENSOR_TYPE::RGBD_CAMERA){
                std::shared_ptr<RGBDCameraSensorData> cur_rgbd_camera_data_ptr = std::dynamic_pointer_cast<RGBDCameraSensorData>(cur_sensor_data_ptr);
                if(!SensorDataBuffer::instance()->assignRGBDOdoPose(cur_rgbd_camera_data_ptr)){
                    continue;
                }
            }
            else{
                MESSAGE_REPORT_ERROR(1020,"Unknown Sensor Type ...");
            }

            if (callback_) {
                callback_(cur_sensor_data_ptr);
            }
            
        }


    }



}