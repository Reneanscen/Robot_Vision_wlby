#ifndef _ANGLE_MATH_UTILS_H_
#define _ANGLE_MATH_UTILS_H_

#include <vector>

namespace Localization {

	namespace angle_math_utils {
		extern const double pi;
		extern const double rad_to_deg_ratio;
		extern const double deg_to_rad_ratio;

		double rad_to_deg(double angle);
		double deg_to_rad(double angle);
		double normalize_deg(double deg);
		double normalize_rad(double rad);

		double sind(double deg);
		double cosd(double deg);
		double tand(double deg);
		double cotd(double deg);
		double asind(double val);
		double acosd(double val);
		double atand(double val);

		int amax(int a, int b);
		int amin(int a, int b);
		float amax(float a, float b);
		float amin(float a, float b);
		double amax(double a, double b);
		double amin(double a, double b);
		double minus_deg(double a_deg, double b_deg);//a_deg - b_deg
		double minus_rad(double a_rad, double b_rad);//a_rad - b_rad
		double mean_deg(const std::vector<double>& deg_vec);
		double mean_rad(const std::vector<double>& rad_vec);
	}

}

#endif