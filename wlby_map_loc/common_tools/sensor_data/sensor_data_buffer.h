#ifndef _SENSOR_DATA_BUFFER_H_
#define _SENSOR_DATA_BUFFER_H_
/*******************************************************************************
* @brief 传感器插值类
* @author jazzey
* @date 2023-8-8
********************************************************************************/
#include <iostream>
#include <iomanip>
#include <mutex>
#include <memory>
#include "sensor_data/sensor_data.h"
#include "check/check.h"
#include "common/common_type_define.h"

namespace wlby{


class SensorDataBuffer{

    public:

        static SensorDataBuffer* instance()
	    {
		    static SensorDataBuffer ins;
		    return &ins;
	    }

        void Insert(const std::shared_ptr<BaseSensorData> &sensor_data);

        //输入一个时间，查询这个时间最近的数据
        std::shared_ptr<BaseSensorData> Search(double time,bool if_print = false);

        //插值函数
        void Interpolate(const std::shared_ptr<OdoSensorData> &start_data, const std::shared_ptr<OdoSensorData> &end_data,
            double query_time, std::shared_ptr<OdoSensorData> &interpolate_data);

        void Reset();

        bool assignScanDataPointsOdoPose(std::shared_ptr<ScanSensorData>& scan_sensor_data);
        bool assignRGBOdoPose(std::shared_ptr<RGBCameraSensorData>& rgb_camera_sensor_data);
        bool assignDepthOdoPose(std::shared_ptr<DepthCameraSensorData>& depth_camera_sensor_data);
        bool assignRGBDOdoPose(std::shared_ptr<RGBDCameraSensorData>& rgbd_camera_sensor_data);
        
        bool getLatestTimeOdoPose(double& time , Localization::Pose2D& pose);
    private:
        SensorDataBuffer(){}

        std::mutex mtx_;
        uint64 index_ = 0;
        bool is_ready = false;
        static constexpr int buffer_size_ = 128;
        // 创建一个大小为 2048 的 TimestampedTransform2d 类型的 std::array 对象
        std::array<std::shared_ptr<BaseSensorData>, buffer_size_> buffer_;

};

}

#endif