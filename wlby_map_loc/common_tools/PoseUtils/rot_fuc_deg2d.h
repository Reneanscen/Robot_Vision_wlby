#ifndef _ROT_FUN_DEG_2D_
#define _ROT_FUN_DEG_2D_

#include <vector>


namespace Localization {

	namespace rot_fuc_deg2d {
		void deg_to_rotmat(double deg, double rotmat[4]);
		void rotmat_to_deg(double rotmat[4], double& deg);
		void rotate_xy(double x, double y, double rot_deg, double &result_x, double &result_y);
		void rotate_xy_inv(double x, double y, double rot_deg, double &result_x, double &result_y);

		double get_grad_deg(double x, double y);

		bool get_delta_pose_of_two_axis(double xin1[], double yin1[], double xin2[], double yin2[], int point_size, double &x2in1, double &y2in1, double &deg2to1);

		bool get_delta_pose_of_two_axis(std::vector<double> xin1, std::vector<double> yin1, std::vector<double> xin2, std::vector<double> yin2, double &x2in1, double &y2in1, double &deg2to1);

	}

}

#endif

