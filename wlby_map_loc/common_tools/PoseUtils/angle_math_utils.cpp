#include "angle_math_utils.h"
#include <math.h>


namespace Localization {

	namespace angle_math_utils {
		const double pi = 3.1415926535;
		const double rad_to_deg_ratio = 180 / pi;
		const double deg_to_rad_ratio = pi / 180;

		double rad_to_deg(double angle) {
			angle *= rad_to_deg_ratio;
			return angle;
		}

		double deg_to_rad(double angle) {
			angle *= deg_to_rad_ratio;
			return angle;
		}

		double normalize_deg(double deg) {
			while (deg < -180) {
				deg += 360;
			}
			while (deg >= 180) {
				deg -= 360;
			}
			return deg;
		}

		double normalize_rad(double rad) {
			while (rad < -pi) {
				rad += 2 * pi;
			}
			while (rad >= pi) {
				rad -= 2 * pi;
			}
			return rad;
		}

		double sind(double deg) {
			return sin(deg*deg_to_rad_ratio);
		}

		double cosd(double deg) {
			return cos(deg*deg_to_rad_ratio);
		}

		double tand(double deg) {
			return tan(deg*deg_to_rad_ratio);
		}

		double cotd(double deg) {
			return 1. / tan(deg*deg_to_rad_ratio);
		}

		double asind(double val) {
			if (val < -1) val = -1;
			if (val > 1) val = 1;
			return asin(val)*rad_to_deg_ratio;
		}

		double acosd(double val) {
			if (val < -1) val = -1;
			if (val > 1) val = 1;
			return acos(val)*rad_to_deg_ratio;
		}

		double atand(double val) {
			return atan(val)*rad_to_deg_ratio;
		}

		int amax(int a, int b) {
			return a > b ? a : b;
		}

		int amin(int a, int b) {
			return a < b ? a : b;
		}

		float amax(float a, float b) {
			return a > b ? a : b;
		}

		float amin(float a, float b) {
			return a < b ? a : b;
		}

		double amax(double a, double b) {
			return a > b ? a : b;
		}

		double amin(double a, double b) {
			return a < b ? a : b;
		}
		double minus_deg(double a_deg, double b_deg) {
			return normalize_deg(a_deg - b_deg);
		}
		double minus_rad(double a_rad, double b_rad) {
			return normalize_rad(a_rad - b_rad);
		}
		double mean_deg(const std::vector<double>& deg_vec) {
			if (deg_vec.size() == 0) {
				return 0;
			}
			double sum_cha = 0;
			for (int i = 1; i < deg_vec.size(); ++i) {
				double cha = minus_deg(deg_vec[i], deg_vec[0]);
				sum_cha += cha;
			}
			sum_cha /= deg_vec.size();
			return normalize_deg(sum_cha + deg_vec[0]);
		}
		double mean_rad(const std::vector<double>& rad_vec) {
			if (rad_vec.size() == 0) {
				return 0;
			}
			double sum_cha = 0;
			for (int i = 1; i < rad_vec.size(); ++i) {
				double cha = minus_rad(rad_vec[i], rad_vec[0]);
				sum_cha += cha;
			}
			sum_cha /= rad_vec.size();
			return normalize_rad(sum_cha + rad_vec[0]);
		}
	}

}