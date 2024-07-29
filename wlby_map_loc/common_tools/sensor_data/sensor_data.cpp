#include "sensor_data/sensor_data.h"



#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace wlby {


BaseSensorData& BaseSensorData::operator=(const BaseSensorData& other) {
		_localTime = other._localTime;
		_state = other._state;
		_sensorType = other._sensorType;
		_dataIndex = other._dataIndex;

		return *this;
	}


OdoSensorData& OdoSensorData::operator=(const OdoSensorData &other) {
		_localTime = other._localTime;
		_state = other._state;
		_sensorType = other._sensorType;
		_dataIndex = other._dataIndex;
		_odoPose = other._odoPose;
		_speedX = other._speedX;  
        _speedY = other._speedY;  
        _angleSpeedDeg = other._angleSpeedDeg;
		return *this;
	}

ScanSensorData& ScanSensorData::operator=(const ScanSensorData& other){

        _localTime = other._localTime;
		_state = other._state;
		_sensorType = other._sensorType;
		_dataIndex = other._dataIndex;
        _deviceId = other._deviceId;
        _beamVec = other._beamVec;
		_odoData = other._odoData;
        return *this;
}

void ScanSensorData::saveToPCDFile_pos(const std::string &filename){

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	for(size_t i = 0; i < _beamVec.size(); i++){
		pcl::PointXYZI pt;
		pt.x = _beamVec[i]._pos._x;
		pt.y = _beamVec[i]._pos._y;
		pt.z = 0;
		pt.intensity = _beamVec[i]._rssi;
		cloud->push_back(pt);
	}

	if(cloud->points.size() > 0){
		pcl::io::savePCDFileASCII(filename, *cloud);
	}
}

void ScanSensorData::saveToPCDFile_pos_in_odo(const std::string &filename){

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	for(size_t i = 0; i < _beamVec.size(); i++){
		pcl::PointXYZI pt;
		pt.x = _beamVec[i]._pos_in_odo._x;
		pt.y = _beamVec[i]._pos_in_odo._y;
		pt.z = 0;
		pt.intensity = _beamVec[i]._rssi;
		cloud->push_back(pt);
	}

	if(cloud->points.size() > 0){
		pcl::io::savePCDFileASCII(filename, *cloud);
	}

}

void ScanSensorData::saveToPCDFile_undistortionPos_in_odo(const std::string &filename){

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	for(size_t i = 0; i < _beamVec.size(); i++){
		pcl::PointXYZI pt;
		pt.x = _beamVec[i]._undistortionPos_in_odo._x;
		pt.y = _beamVec[i]._undistortionPos_in_odo._y;
		pt.z = 0;
		pt.intensity = _beamVec[i]._rssi;
		cloud->push_back(pt);
	}

	if(cloud->points.size() > 0){
		pcl::io::savePCDFileASCII(filename, *cloud);
	}


}

    RGBCameraSensorData &RGBCameraSensorData::operator=(const RGBCameraSensorData &other) {

        _localTime = other._localTime;
        _state = other._state;
        _sensorType = other._sensorType;
        _dataIndex = other._dataIndex;

        _rgb_mat = other._rgb_mat.clone();
        _odoData = other._odoData;

        return *this;
    }
    DepthCameraSensorData &DepthCameraSensorData::operator=(const DepthCameraSensorData& other){
        _localTime = other._localTime;
        _state = other._state;
        _sensorType = other._sensorType;
        _dataIndex = other._dataIndex;
        _depth_mat = other._depth_mat.clone();
        _odoData = other._odoData;
        return *this;
    }
    RGBDCameraSensorData &RGBDCameraSensorData::operator=(const RGBDCameraSensorData& other){
        _localTime = other._localTime;
        _state = other._state;
        _sensorType = other._sensorType;
        _dataIndex = other._dataIndex;
        _rgb_sensor_data = other._rgb_sensor_data;
        _depth_sensor_data = other._depth_sensor_data;
        _odoData = other._odoData;
    }
}

