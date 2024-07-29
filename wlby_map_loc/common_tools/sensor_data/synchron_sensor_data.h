#ifndef _SYNCHRON_SENSOR_DATA_H_
#define _SYNCHRON_SENSOR_DATA_H_

/*******************************************************************************
* @brief 同步传感器数据类
* @author jazzey
* @date 2023-8-9
********************************************************************************/

#include <mutex>
#include <memory>
#include <functional>
#include <thread>
#include <iostream>

#include "common/sleep.h"

#include "sensor_data/sensor_data.h"
#include "sensor_data/sensor_data_buffer.h"

namespace wlby{

class SynchronSensorData {
public:
    SynchronSensorData(){
        slam_running_thread_ = std::thread(&SynchronSensorData::Run, this);
    }
    ~SynchronSensorData(){

    }

    void setLaserToOdo(const Pose2D& laser_to_odo){
        laser_to_odo_ = laser_to_odo;
    }

    void addSensorData(const std::shared_ptr<BaseSensorData> &sensor_data_ptr){
        std::lock_guard<std::mutex> lock(mtx_);
        current_sensor_data_ = sensor_data_ptr;
    }
    
    void addOdoData(const std::shared_ptr<OdoSensorData> &odo_data_ptr){
        SensorDataBuffer::instance()->Insert(odo_data_ptr);
    }

    void registerBaseSensor(std::function<void(std::shared_ptr<BaseSensorData>&)> callback){
        callback_ = callback;
    }

private:
    void unDistort(std::shared_ptr<ScanSensorData>& scan_sensor_data);

    void Run();

    std::mutex mtx_;
    std::thread slam_running_thread_;
    std::function<void(std::shared_ptr<BaseSensorData>&)> callback_;

    std::shared_ptr<BaseSensorData> current_sensor_data_;
    std::shared_ptr<BaseSensorData> last_sensor_data_;

    Pose2D laser_to_odo_;

};

}



#endif