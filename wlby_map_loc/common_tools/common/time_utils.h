#ifndef _TIME_UTILS_H_
#define _TIME_UTILS_H_

/*******************************************************************************
* @brief 时间转换类
* @author jazzey
* @date 2023-8-10
********************************************************************************/

#include "builtin_interfaces/msg/time.hpp"

namespace wlby{

class TimeUtils{
public:
    static double GetThreadCpuTimeSeconds();
    //时间转换类 将ROS2时间转换成double类型，可能有精度的损失，损失在1e-6左右，满足要求
    static double RosTimeToAbsTime(const builtin_interfaces::msg::Time& ros_time);
    static builtin_interfaces::msg::Time AbsTimeToRosTime(double abs_time);
    static builtin_interfaces::msg::Time GetThreadCpuRosTime();
};


}


#endif