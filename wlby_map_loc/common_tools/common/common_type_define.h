#ifndef _COMMON_TYPE_DEFINE_H_
#define _COMMON_TYPE_DEFINE_H_

/*******************************************************************************
* @brief 存一些通用定义
* @author jazzey
* @date 2023-8-9
********************************************************************************/
#include <chrono>
#include <ratio>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <vector>

#include "Eigen/Core"

#include "PoseUtils/Pose2DDataStruct.h"

namespace wlby{

    //SENSOR_TYPE定义 里程计、IMU、2D激光类型
    typedef enum {
		NONE = 0,
		ODOM = 1,
		IMU,
        SCAN,
        RGB_CAMERA,
        DEPTH_CAMERA,
        RGBD_CAMERA
	} SENSOR_TYPE;

	//定义定位状态 目前三种状态，定位未初始化、混合定位模式、盲走模式
	typedef enum{
		NOT_INITED = 0,
		MIX_LOC = 1,
		BLIND_WALK = 2
	}LOC_State;

	typedef enum{
		Nav = 0,
		Odo = 1,
		Reg = 2
	}Pose_Category;

	typedef enum{
		CONFIG_WAY = 0,
		LAST_WAY = 1,
		GLOBAL_SEARCH_WAY = 2
	}Reloc_Ways;

    class PoseWithStamped{
    public:
        PoseWithStamped(double time,const Localization::Pose2D& pose){
            time_ = time;
            pose_ = pose;
        }
        double time_;
        Localization::Pose2D pose_;
    };

    struct OBBBox{
        std::vector<Localization::Pos2D> obb_box;
        std::vector<Eigen::Array2i> semantic_point_vec;
        uint8_t semantic_label;
        float r;
        float g;
        float b;
    };

    class PoseSet{

    public:
        PoseSet() = default;
        PoseSet(const Localization::Pose2D& nav_pose,const Localization::Pose2D& odo_pose,const Localization::Pose2D& reg_pose){
            nav_pose_ = nav_pose;
            odo_pose_ = odo_pose;
            reg_pose_ = reg_pose;
        }
        PoseSet& operator=(const PoseSet &other)= default;

        Localization::Pose2D nav_pose_;
        Localization::Pose2D odo_pose_;
        Localization::Pose2D reg_pose_;
        //to do : imu_pose
    };

	using int8 = int8_t;
	using int16 = int16_t;
	using int32 = int32_t;
	using int64 = int64_t;
	using uint8 = uint8_t;
	using uint16 = uint16_t;
	using uint32 = uint32_t;
	using uint64 = uint64_t;

	inline int RoundToInt(const float x) { return (int)std::lround(x); }
	inline int RoundToInt(const double x) { return (int)std::lround(x); }
	inline int64 RoundToInt64(const float x) { return std::lround(x); }
	inline int64 RoundToInt64(const double x) { return std::lround(x); }

	// Clamps 'value' to be in the range ['min', 'max'].
	template <typename T>
	T Clamp(const T value, const T min, const T max) {
	if (value > max) {
		return max;
	}
	if (value < min) {
		return min;
	}
		return value;
	}

	template <typename T>
	constexpr T Power(T base, int exponent) {
		return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
	}

	// Calculates a^2.
	template <typename T>
	constexpr T Pow2(T a) {
		return Power(a, 2);
	}

	int SplitString(const std::string &input, const std::string &delimiter, std::vector<std::string> &results);

}


#endif
