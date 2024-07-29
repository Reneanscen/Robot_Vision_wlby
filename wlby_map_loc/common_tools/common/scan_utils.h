#ifndef _SCAN_UTILS_UTILS_
#define _SCAN_UTILS_UTILS_
/*******************************************************************************
* @brief 点云工具类
* @author jazzey
* @date 2023-8-9
********************************************************************************/

#include <memory>
#include <vector>
#include "PoseUtils/Pose2DDataStruct.h"
#include "sensor_data/sensor_data.h"


namespace wlby{
class ScanUtils{

public:

    //给定点云的指针，返回均匀采样的点云数组
    static void getPointVec(const std::shared_ptr<ScanSensorData> &scan_sensor_data_ptr,
                    int limit_nums, std::vector<Localization::Pos2D> &used_points);
    //旋转点云。。
    static void RotatePointCloud(const std::vector<Localization::Pos2D>& point_cloud, const Localization::Orient2D& init_orient,
      std::vector<Localization::Pos2D>& rotated_point_cloud);

    //返回全部的点云
    static void getPointVec(const std::shared_ptr<ScanSensorData> &scan_sensor_data_ptr, std::vector<Localization::Pos2D> &used_points);
};

}

#endif