#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_
/*******************************************************************************
* @brief 传感器数据类
* @author jazzey
* @date 2023-8-9
********************************************************************************/
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "common/common_type_define.h"
#include "PoseUtils/Pose2DDataStruct.h"

using namespace Localization;

namespace wlby {

class BaseSensorData {
    public:
        BaseSensorData(SENSOR_TYPE sensor_type = SENSOR_TYPE::NONE,double local_time = 0) {
			_localTime = local_time;
			_state = true;
			_sensorType = sensor_type;
			_dataIndex = 0;
		}

        BaseSensorData& operator=(const BaseSensorData& other);
	
    public:
        //设备类型
        SENSOR_TYPE _sensorType;
        //数据索引
        uint64 _dataIndex;
        //数据状态
        bool _state;
        //数据时间
        double _localTime;

    virtual ~BaseSensorData(){

    }
};

class OdoSensorData : public BaseSensorData {

public:

    OdoSensorData(double local_time = 0): BaseSensorData(SENSOR_TYPE::ODOM, local_time) {
			_speedX = _speedY = _angleSpeedDeg = 0;
	}
    OdoSensorData& operator=(const OdoSensorData &other);

public:
    // odom递推累计位姿
    Pose2D _odoPose;
    // x方向的速度、y方向的速度、角速度
    double _speedX, _speedY, _angleSpeedDeg;

};

class ScanSensorData : public BaseSensorData {
public:
	class Beam {
		public:
            //点坐标，极坐标 ploar 形式
			Polar2D _polar;
            //点坐标，x y 形式
			Pos2D _pos;
            Pos2D _pos_in_odo;
            //去畸变后的点坐标
			Pos2D _undistortionPos_in_odo;
            //反射强度
			float _rssi;
            //当前点的时间
            double _posTime;
            //里程数据
            Pose2D _odoPose;

            //是否有效
			bool _validity;
			Beam(double angle_deg = 0, double dis = 0, double pos_time = 0, float rssi = 0, bool validity = true) :_polar(angle_deg, dis) {
				_pos = _polar.toPos();
                _posTime = pos_time;
				_rssi = rssi;
				_validity = validity;
			}

			Beam& operator=(const Beam& other) {
				_polar = other._polar;
				_pos = other._pos;
                _pos_in_odo = other._pos_in_odo;
				_undistortionPos_in_odo = other._undistortionPos_in_odo;
				_rssi = other._rssi;
                _posTime = other._posTime;
                _odoPose = other._odoPose;
                _validity = other._validity;
				return *this;
			}
	};	


    ScanSensorData(double local_time = 0):BaseSensorData(SENSOR_TYPE::SCAN, local_time) {
        _deviceId = 0;
	}

	ScanSensorData &operator=(const ScanSensorData& other);

    void saveToPCDFile_pos(const std::string &filename);
    void saveToPCDFile_pos_in_odo(const std::string &filename);
    void saveToPCDFile_undistortionPos_in_odo(const std::string &filename);

public:
    //激光设备号
    int _deviceId;
    //激光的点集合
    std::vector<Beam> _beamVec;

    OdoSensorData _odoData;
    
};

class RGBCameraSensorData : public BaseSensorData{
public:
    RGBCameraSensorData(double local_time = 0):BaseSensorData(SENSOR_TYPE::RGB_CAMERA, local_time){

    }
    RGBCameraSensorData &operator=(const RGBCameraSensorData& other);
public:
    cv::Mat _rgb_mat;
    OdoSensorData _odoData;
};
class DepthCameraSensorData : public BaseSensorData{
public:
    DepthCameraSensorData(double local_time = 0):BaseSensorData(SENSOR_TYPE::DEPTH_CAMERA, local_time) {
    }
    DepthCameraSensorData &operator=(const DepthCameraSensorData& other);
public:
    cv::Mat _depth_mat;
    OdoSensorData _odoData;
};
class RGBDCameraSensorData : public BaseSensorData{
public:
    RGBDCameraSensorData(double local_time = 0):BaseSensorData(SENSOR_TYPE::RGBD_CAMERA, local_time) {
    }
    RGBDCameraSensorData &operator=(const RGBDCameraSensorData& other);
public:
    RGBCameraSensorData _rgb_sensor_data;
    DepthCameraSensorData _depth_sensor_data;
    OdoSensorData _odoData;
};
}
#endif