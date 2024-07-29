#include <time.h>
#include "common/time_utils.h"
#include "check/check.h"

namespace wlby{


double TimeUtils::GetThreadCpuTimeSeconds() {
  struct timespec thread_cpu_time;
  CHECK_EQ(clock_gettime(CLOCK_REALTIME, &thread_cpu_time),0)
  return thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
}

double TimeUtils::RosTimeToAbsTime(const builtin_interfaces::msg::Time& ros_time){
    return ros_time.sec + ros_time.nanosec * 1e-9;
}

builtin_interfaces::msg::Time TimeUtils::AbsTimeToRosTime(double abs_time){

    int32_t seconds = static_cast<int32_t>(std::floor(abs_time));
    uint32_t nanosec = static_cast<int32_t>((abs_time - seconds)*1e9);

    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = seconds;
    msg_time.nanosec = nanosec;
    return msg_time;
}

builtin_interfaces::msg::Time TimeUtils::GetThreadCpuRosTime(){
  double time = GetThreadCpuTimeSeconds();
  return AbsTimeToRosTime(time);
}

}