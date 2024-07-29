#include "sensor_data/sensor_data_buffer.h"

#include <iostream>

namespace wlby
{

    void SensorDataBuffer::Insert(const std::shared_ptr<BaseSensorData> &sensor_data)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        buffer_[index_ % buffer_size_] = sensor_data;
        ++index_;

        // index_即队列的实际数据数量

        if (index_ == 1)
            is_ready = true;
    }

    // 输入一个时间，查询这个时间最近的数据
    std::shared_ptr<BaseSensorData> SensorDataBuffer::Search(double time, bool if_print)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        // 当队列只有一个数据时，直接返回,将查询时间和队列的odo时间的时间差 乘以 速度后返回
        if (index_ == 1)
        {

            std::shared_ptr<BaseSensorData> sensor_data = buffer_[0];
            if (sensor_data->_sensorType == SENSOR_TYPE::ODOM)
            {
                std::shared_ptr<OdoSensorData> start_odo_data = std::dynamic_pointer_cast<OdoSensorData>(sensor_data);

                std::shared_ptr<OdoSensorData> interpolate_data = std::make_shared<OdoSensorData>(time);
                interpolate_data->_dataIndex = start_odo_data->_dataIndex;
                interpolate_data->_speedX = start_odo_data->_speedX;
                interpolate_data->_speedY = start_odo_data->_speedY;
                interpolate_data->_angleSpeedDeg = start_odo_data->_angleSpeedDeg;
                interpolate_data->_odoPose = start_odo_data->_odoPose;

                if (time > sensor_data->_localTime)
                {
                    double delta_t = time - sensor_data->_localTime;
                    Localization::Pose2D dr_pose = Localization::Pose2D(start_odo_data->_speedX * delta_t, start_odo_data->_speedY * delta_t, start_odo_data->_angleSpeedDeg * delta_t);
                    interpolate_data->_odoPose = start_odo_data->_odoPose * dr_pose;
                }
                else if (time < sensor_data->_localTime)
                {
                    double delta_t = sensor_data->_localTime - time;
                    Localization::Pose2D dr_pose = Localization::Pose2D(start_odo_data->_speedX * delta_t, start_odo_data->_speedY * delta_t, start_odo_data->_angleSpeedDeg * delta_t);
                    interpolate_data->_odoPose = start_odo_data->_odoPose * dr_pose.getInvPose();
                }
                return interpolate_data;
            }
            else if (sensor_data->_sensorType == SENSOR_TYPE::IMU)
            {
            }
            return nullptr;
        }

        // 将查询时间转换成int64格式
        uint64 start;
        uint64 end;
        // 数据指针标号：start指向第一个数据, end指向最后一个数据
        // index_ <= buffer_size_ 。。。 index_-1 是最后一个数据在队列的索引
        if (index_ - 1 < buffer_size_)
        {
            start = 0;
            end = index_ - 1;
        }
        else
        {
            start = index_ - buffer_size_;
            end = index_ - 1;
        }

        // 获取当前队列的首尾数据
        std::shared_ptr<BaseSensorData> start_sensor_data = buffer_[start % buffer_size_];
        std::shared_ptr<BaseSensorData> end_sensor_data = buffer_[end % buffer_size_];

        if (start_sensor_data->_sensorType == SENSOR_TYPE::ODOM && end_sensor_data->_sensorType == SENSOR_TYPE::ODOM)
        {

            std::shared_ptr<OdoSensorData> interpolate_data = std::make_shared<OdoSensorData>(time);

            if (time < start_sensor_data->_localTime)
            {
                std::shared_ptr<OdoSensorData> start_odo_data = std::dynamic_pointer_cast<OdoSensorData>(start_sensor_data);
                std::shared_ptr<OdoSensorData> end_odo_data = std::dynamic_pointer_cast<OdoSensorData>(end_sensor_data);

                // 1. 若查询的时间比start的时间还小
                double delta_t = start_sensor_data->_localTime - time;
                Localization::Pose2D dr_pose = Localization::Pose2D(start_odo_data->_speedX * delta_t, start_odo_data->_speedY * delta_t, start_odo_data->_angleSpeedDeg * delta_t);
                interpolate_data->_odoPose = start_odo_data->_odoPose * dr_pose.getInvPose();
                interpolate_data->_dataIndex = start_odo_data->_dataIndex;
                interpolate_data->_speedX = start_odo_data->_speedX;
                interpolate_data->_speedY = start_odo_data->_speedY;
                interpolate_data->_angleSpeedDeg = start_odo_data->_angleSpeedDeg;
            }
            else if (time > end_sensor_data->_localTime)
            {

                std::shared_ptr<OdoSensorData> start_odo_data = std::dynamic_pointer_cast<OdoSensorData>(start_sensor_data);
                std::shared_ptr<OdoSensorData> end_odo_data = std::dynamic_pointer_cast<OdoSensorData>(end_sensor_data);

                // 1. 若查询的时间比end的时间还大
                double delta_t = time - end_sensor_data->_localTime;
                Localization::Pose2D dr_pose = Localization::Pose2D(end_odo_data->_speedX * delta_t, end_odo_data->_speedY * delta_t, end_odo_data->_angleSpeedDeg * delta_t);
                interpolate_data->_odoPose = end_odo_data->_odoPose * dr_pose;
                interpolate_data->_dataIndex = end_odo_data->_dataIndex;
                interpolate_data->_speedX = end_odo_data->_speedX;
                interpolate_data->_speedY = end_odo_data->_speedY;
                interpolate_data->_angleSpeedDeg = end_odo_data->_angleSpeedDeg;
            }
            else
            {

                // 3. 查询的时间在start和end之间，二分查找
                while (end - start > 1)
                {
                    auto mid = (start + end) / 2;
                    if (time < buffer_[mid % buffer_size_]->_localTime)
                    {
                        end = mid;
                    }
                    else if (time > buffer_[mid % buffer_size_]->_localTime)
                    {
                        start = mid;
                    }
                    else
                    {
                        return buffer_[mid % buffer_size_];
                    }
                }

                std::shared_ptr<OdoSensorData> start_odo_data = std::dynamic_pointer_cast<OdoSensorData>(buffer_[start % buffer_size_]);
                std::shared_ptr<OdoSensorData> end_odo_data = std::dynamic_pointer_cast<OdoSensorData>(buffer_[end % buffer_size_]);

                // if(if_print){
                //     log__save("Data", kLogLevel_Error, kLogTarget_Filesystem,"start time: %f   %f %f %f",start_odo_data->_localTime,start_odo_data->_odoPose._pos._x,start_odo_data->_odoPose._pos._y,start_odo_data->_odoPose._orient._deg);
                //     log__save("Data", kLogLevel_Error, kLogTarget_Filesystem,"end time: %f   %f %f %f",end_odo_data->_localTime,end_odo_data->_odoPose._pos._x,end_odo_data->_odoPose._pos._y,end_odo_data->_odoPose._orient._deg);
                // }

                Interpolate(start_odo_data, end_odo_data, time, interpolate_data);
            }

            return interpolate_data;
        }
        else if (start_sensor_data->_sensorType == SENSOR_TYPE::IMU && end_sensor_data->_sensorType == SENSOR_TYPE::IMU)
        {
            // to do ......
        }
        return nullptr;
    }

    // 插值函数
    void SensorDataBuffer::Interpolate(const std::shared_ptr<OdoSensorData> &start_data, const std::shared_ptr<OdoSensorData> &end_data,
                                       double query_time, std::shared_ptr<OdoSensorData> &interpolate_data)
    {
        CHECK_LE(start_data->_localTime, query_time);
        CHECK_GE(end_data->_localTime, query_time);
        CHECK_NOT_NULL(start_data);
        CHECK_NOT_NULL(end_data);
        CHECK_NOT_NULL(interpolate_data);

        const double duration = end_data->_localTime - start_data->_localTime;
        const double factor = (query_time - start_data->_localTime) / duration;

        Pose2D dr_pose = end_data->_odoPose / start_data->_odoPose;
        interpolate_data->_odoPose = start_data->_odoPose * Pose2D(dr_pose._pos._x * factor, dr_pose._pos._y * factor, dr_pose._orient._deg * factor);
        interpolate_data->_localTime = query_time;
        interpolate_data->_speedX = start_data->_speedX + (end_data->_speedX - start_data->_speedX) * factor;
        interpolate_data->_speedY = start_data->_speedY + (end_data->_speedY - start_data->_speedY) * factor;
        interpolate_data->_angleSpeedDeg = start_data->_angleSpeedDeg + (end_data->_angleSpeedDeg - start_data->_angleSpeedDeg) * factor;
        interpolate_data->_dataIndex = factor >= 0.5 ? end_data->_dataIndex : start_data->_dataIndex;
    }

    bool SensorDataBuffer::assignScanDataPointsOdoPose(std::shared_ptr<ScanSensorData> &scan_sensor_data)
    {

        CHECK_NOT_NULL(scan_sensor_data);

        std::vector<double> points_time;
        points_time.reserve(scan_sensor_data->_beamVec.size());
        for (auto &beam : scan_sensor_data->_beamVec)
        {
            points_time.emplace_back(beam._posTime + scan_sensor_data->_localTime);
        }

        if (points_time.size() == 0)
        {
            return false;
        }

        if (!is_ready)
        {
            return false;
        }

        // log__save("Data", kLogLevel_Error, kLogTarget_Filesystem,"query: %f",scan_sensor_data->_localTime);
        std::shared_ptr<BaseSensorData> base_data = Search(scan_sensor_data->_localTime);
        CHECK_NOT_NULL(base_data);

        std::shared_ptr<OdoSensorData> odo_data = std::dynamic_pointer_cast<OdoSensorData>(base_data);

        scan_sensor_data->_odoData = *odo_data;

        // log__save("Data", kLogLevel_Error, kLogTarget_Filesystem,"query: %f odo_pose: %f %f %f",scan_sensor_data->_localTime,odo_data->_odoPose._pos._x,odo_data->_odoPose._pos._y,odo_data->_odoPose._orient._deg);

        for (size_t i = 0; i < points_time.size(); ++i)
        {
            std::shared_ptr<BaseSensorData> base_data = Search(points_time[i]);
            CHECK_NOT_NULL(base_data);
            std::shared_ptr<OdoSensorData> odo_data = std::dynamic_pointer_cast<OdoSensorData>(base_data);
            scan_sensor_data->_beamVec[i]._odoPose = odo_data->_odoPose;
        }
        return true;
    }

    bool SensorDataBuffer::assignDepthOdoPose(std::shared_ptr<DepthCameraSensorData>& depth_camera_sensor_data){
        CHECK_NOT_NULL(depth_camera_sensor_data);
        if (!is_ready)
        {
            return false;
        }
        std::shared_ptr<BaseSensorData> base_data = Search(depth_camera_sensor_data->_localTime);
        CHECK_NOT_NULL(base_data);
        std::shared_ptr<OdoSensorData> odo_data = std::dynamic_pointer_cast<OdoSensorData>(base_data);
        depth_camera_sensor_data->_odoData = *odo_data;
        return true;
    }
    bool SensorDataBuffer::assignRGBOdoPose(std::shared_ptr<RGBCameraSensorData>& rgb_camera_sensor_data){
        CHECK_NOT_NULL(rgb_camera_sensor_data);
        if (!is_ready)
        {
            return false;
        }
        std::shared_ptr<BaseSensorData> base_data = Search(rgb_camera_sensor_data->_localTime);
        CHECK_NOT_NULL(base_data);
        std::shared_ptr<OdoSensorData> odo_data = std::dynamic_pointer_cast<OdoSensorData>(base_data);
        rgb_camera_sensor_data->_odoData = *odo_data;
        return true;
    }
    bool SensorDataBuffer::assignRGBDOdoPose(std::shared_ptr<RGBDCameraSensorData>& rgbd_camera_sensor_data){
        CHECK_NOT_NULL(rgbd_camera_sensor_data);
        if (!is_ready)
        {
            return false;
        }
        std::shared_ptr<BaseSensorData> base_data = Search(rgbd_camera_sensor_data->_localTime);
        CHECK_NOT_NULL(base_data);
        std::shared_ptr<OdoSensorData> odo_data = std::dynamic_pointer_cast<OdoSensorData>(base_data);
        rgbd_camera_sensor_data->_odoData = *odo_data;
        return true;
    }
    bool SensorDataBuffer::getLatestTimeOdoPose(double &time, Localization::Pose2D &pose)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        if (!is_ready)
        {
            return false;
        }

        uint64 end = index_ - 1;

        // 获取当前队列的首尾数据
        std::shared_ptr<BaseSensorData> end_sensor_data = buffer_[end % buffer_size_];
        if (end_sensor_data->_sensorType == SENSOR_TYPE::ODOM)
        {
            std::shared_ptr<OdoSensorData> end_odo_data = std::dynamic_pointer_cast<OdoSensorData>(end_sensor_data);
            pose = end_odo_data->_odoPose;
            time = end_odo_data->_localTime;
            return true;
        }
        else
        {
            return false;
        }
    }

    void SensorDataBuffer::Reset()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        index_ = 0;
        is_ready = false;
        buffer_.fill(nullptr);
    }

}