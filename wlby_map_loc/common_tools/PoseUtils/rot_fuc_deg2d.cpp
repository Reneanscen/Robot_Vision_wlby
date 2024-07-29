#include "rot_fuc_deg2d.h"
#include "angle_math_utils.h"
#include <math.h>
#include "Matrix.h"


using namespace Localization::angle_math_utils;

namespace Localization {

	namespace rot_fuc_deg2d {
		void deg_to_rotmat(double deg, double rotmat[4]) {
			rotmat[0] = cosd(deg);   rotmat[1] = -sind(deg);
			rotmat[2] = sind(deg);   rotmat[3] = cosd(deg);
		}

		void rotmat_to_deg(double rotmat[4], double& deg) {
			double c = rotmat[0];
			double s = rotmat[2];
			double rad = atan2(s, c);
			deg = rad_to_deg(rad);
		}

		void rotate_xy(double x, double y, double rot_deg, double &result_x, double &result_y) {
			result_x = x*cosd(rot_deg) - y*sind(rot_deg);
			result_y = x*sind(rot_deg) + y*cosd(rot_deg);
		}

		/*��ʱ����תrot_deg���õ�ԭ����ϵ�е�ʸ������ת������ϵ�еı�ʾ*/
		void rotate_xy_inv(double x, double y, double rot_deg, double &result_x, double &result_y) {
			result_x = x*cosd(rot_deg) + y*sind(rot_deg);
			result_y = -x*sind(rot_deg) + y*cosd(rot_deg);
		}

		double get_grad_deg(double x, double y) {
			double gradRad = 0;
			/*if (fabs(x) < 0.0001) {
				gradRad = (y>0 ? 1 : -1) * pi / 2;
			}
			else {
				gradRad = atan(y / x);
				if (x < 0) {
					if (y >= 0) {
						gradRad = pi + gradRad;
					}
					else {
						gradRad -= pi;
					}
				}
			}*/

			gradRad = atan2(y, x);

			return rad_to_deg(gradRad);
		}

		bool get_delta_pose_of_two_axis(double xin1[], double yin1[], double xin2[], double yin2[], int point_size, double &x2in1, double &y2in1, double &deg2to1) {
			CMatrix H(2 * point_size, 4);
			CMatrix Z(2 * point_size, 1);

			for (int i = 0; i<point_size; ++i) {
				int index = 2 * i;
				H(index, 0) = 1;     H(index, 1) = 0;     H(index, 2) = xin2[i];     H(index, 3) = -yin2[i];
				H(index + 1, 0) = 0;   H(index + 1, 1) = 1;   H(index + 1, 2) = yin2[i];   H(index + 1, 3) = xin2[i];

				Z(index, 0) = xin1[i];
				Z(index + 1, 0) = yin1[i];
			}

			CMatrix HT = H.Transpose();
			CMatrix HTH = HT*H;
			if (HTH.InvertGaussJordan() == false) {
				return false;
			}

			CMatrix X = HTH*HT*Z;

			double dx = X(0, 0);
			double dy = X(1, 0);
			double cos_angle = X(2, 0);
			double sin_angle = X(3, 0);
			double one_t = sqrt(cos_angle*cos_angle + sin_angle*sin_angle);
			cos_angle /= one_t;
			sin_angle /= one_t;

			x2in1 = dx;
			y2in1 = dy;

			deg2to1 = atan2(sin_angle, cos_angle) * 180 / 3.1415926535;

			return true;
		}

		bool get_delta_pose_of_two_axis(std::vector<double> xin1, std::vector<double> yin1, std::vector<double> xin2, std::vector<double> yin2, double &x2in1, double &y2in1, double &deg2to1) {
			int point_size = xin1.size();

			CMatrix H(2 * point_size, 4);
			CMatrix Z(2 * point_size, 1);

			for (int i = 0; i<point_size; ++i) {
				int index = 2 * i;
				H(index, 0) = 1;     H(index, 1) = 0;     H(index, 2) = xin2[i];     H(index, 3) = -yin2[i];
				H(index + 1, 0) = 0;   H(index + 1, 1) = 1;   H(index + 1, 2) = yin2[i];   H(index + 1, 3) = xin2[i];

				Z(index, 0) = xin1[i];
				Z(index + 1, 0) = yin1[i];
			}

			CMatrix HT = H.Transpose();
			CMatrix HTH = HT*H;
			if (HTH.InvertGaussJordan() == false) {
				return false;
			}

			CMatrix X = HTH*HT*Z;

			double dx = X(0, 0);
			double dy = X(1, 0);
			double cos_angle = X(2, 0);
			double sin_angle = X(3, 0);
			double one_t = sqrt(cos_angle*cos_angle + sin_angle*sin_angle);
			cos_angle /= one_t;
			sin_angle /= one_t;

			x2in1 = dx;
			y2in1 = dy;

			deg2to1 = atan2(sin_angle, cos_angle) * 180 / 3.1415926535;

			return true;
		}
	}

}