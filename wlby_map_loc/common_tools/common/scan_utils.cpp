#include "common/scan_utils.h"

namespace wlby{

    void ScanUtils::getPointVec(const std::shared_ptr<ScanSensorData> &scan_sensor_data_ptr,
                    int limit_nums, std::vector<Localization::Pos2D> &used_points){

        std::vector<Localization::Pos2D> used_points_tmp;
        for(const auto& beam : scan_sensor_data_ptr->_beamVec){
            Localization::Pos2D pos = beam._undistortionPos_in_odo;
            
            if(!used_points_tmp.empty()){
                double gap = (used_points_tmp.back() - pos).norm();
                double r_min = beam._polar._radius;
                double gap_margin = r_min < 2 ? 0.02 : 0.04;
                if (gap < gap_margin) {
                    continue;
                }
            }
            used_points_tmp.push_back(pos);
        }

        double ratio = 1.0*(int)used_points_tmp.size() / limit_nums;
		double denominator = 0;
		for (size_t i = 0; i < used_points_tmp.size(); ++i) {
			if (i >= denominator) {
				used_points.push_back(used_points_tmp[i]);
				denominator += ratio;
			}
		}

    }


    //旋转点云
    void ScanUtils::RotatePointCloud(const std::vector<Localization::Pos2D>& point_cloud, const Localization::Orient2D& init_orient,
      std::vector<Localization::Pos2D>& rotated_point_cloud){
        for(const auto & point : point_cloud){
            Localization::Pos2D pos = init_orient*point;
            rotated_point_cloud.push_back(pos);
        }
    }

    void ScanUtils::getPointVec(const std::shared_ptr<ScanSensorData> &scan_sensor_data_ptr, std::vector<Localization::Pos2D> &used_points){
        for(const auto & beam : scan_sensor_data_ptr->_beamVec) {
            Localization::Pos2D pos = beam._undistortionPos_in_odo;
            used_points.push_back(pos);
        }
    }

}