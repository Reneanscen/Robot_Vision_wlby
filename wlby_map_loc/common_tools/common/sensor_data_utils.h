#ifndef _SENSOR_DATA_UTILS_
#define _SENSOR_DATA_UTILS_

#include <map>
#include <string>
#include "common/common_type_define.h"

namespace wlby{
class SensorDataUtils{

private:
    SensorDataUtils() {
        _sensorTypeNameStr.clear();
		_sensorTypeNameStr[wlby::SENSOR_TYPE::NONE] = "NONE";
		_sensorTypeNameStr[wlby::SENSOR_TYPE::ODOM] = "ODO";
		_sensorTypeNameStr[wlby::SENSOR_TYPE::IMU] = "IMU";
		_sensorTypeNameStr[wlby::SENSOR_TYPE::SCAN] = "SCAN";
    }
    std::map<SENSOR_TYPE, std::string> _sensorTypeNameStr;

public:

    static SensorDataUtils* instance()
	{
		static SensorDataUtils ins;
		return &ins;
	}	

    std::string getNameStrBySensorType(SENSOR_TYPE sensor_type) {
		return _sensorTypeNameStr[sensor_type];
	}
};

#define SENSOR_DATA_UTILS SensorDataUtils::instance()

}

#endif