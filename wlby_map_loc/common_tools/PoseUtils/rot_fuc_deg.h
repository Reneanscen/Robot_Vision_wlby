#ifndef _ROT_FUC_DEG_
#define _ROT_FUC_DEG_


namespace Localization {

	namespace rot_fuc_deg {

		void euler_to_rotmat(double yaw, double pitch, double roll, double R[9]);
		void euler_to_rotmat(double euler[3], double R[9]);
		void rotmat_to_euler_old(double R[9], double euler_result[3]);//�����±��Ǵ�0��ʼ��
		void rotmat_to_euler_new(const double R[9], double euler_result[3]);
		void rotmat_to_euler_new_old(const double R[9], double euler_result[3]);
		void euler_to_quat(double yaw, double pitch, double roll, double qut[4]);
		void euler_to_quat(double euler[3], double qut[4]);
		void quat_to_euler(double Q[4], double euler[3]);
		void quat_to_rotmat(double Q[4], double R[9]);
		void rotmat_to_quat(double R[9], double qut[4]);

		void quat_multi(double Q1[4], double Q2[4], double qut[4]);
		void d_quat_by_wt(double wx, double wy, double wz, double t, double qut[4]);
		void d_quat_by_wt(double w[3], double t, double qut[4]);
		void quat_change_by_wt(double wx, double wy, double wz, double t, double quat_ori[4], double quat_end[4]);
		void quat_change_by_wt(double w[3], double t, double quat_ori[4], double quat_end[4]);
		void quat_recursion(double quat_ori[4], double qut_delta[4], double quat_end[4]);
		void quat_mat_by_wt(double wx, double wy, double wz, double t, double quat_mat_r[16]);
		void quat_mat_by_wt(double w[3], double t, double quat_mat_r[16]);
		void quat_mat_left(double quat[4], double quat_mat[16]);
		void quat_mat_right(double quat[4], double quat_mat[16]);
		void quat_unit_inv(const double quat[4], double quat_inv[4]);
		void error_by_d_quat(double d_quat[4], double err_result[3]);
		void d_quat_by_error(double err[3], double d_quat[4]);
		void normalize_quat(double quat[4]);
		void quat_mat_by_wt_first_order(double w_k_1_deg[3], double w_k_deg[3], double dt, double mat[16]);//to do

		void rotmat_multi(const double rot1[9], const double rot2[9], double rot3[9]);
		void d_rotmat_by_wt(double wx, double wy, double wz, double t, double rot[9]);
		void d_rotmat_by_wt(double w[3], double t, double rot[9]);
		void rotmat_change_by_wt(double wx, double wy, double wz, double t, double rot_ori[9], double rot_end[9]);
		void rotmat_change_by_wt(double w[3], double t, double rot_ori[9], double rot_end[9]);
		void rotmat_recursion(const double rot_ori[9], const double rot_delta[9], double rot_end[9]);
		void sw_by_w(double wx, double wy, double wz, double sw[9]);
		void sw_by_w(double w[3], double sw[9]);
		void rotmat_inv(const double rot[9], double rot_inv[9]);
		void rot_vector(const double rot[9], const double vec_ori[3], double vec_end[3]);

		void mat_add(double *left, double *right, double *result, int row, int col);
		void mat_min(double *left, double *right, double *result, int row, int col);
		bool mat_multi(double *left, double *right, double *result, int row1, int col1, int col2);
		void mat_multi_in_diff_out(double *left, double *right, double *result, int row1, int col1, int col2);
		void mat_multi_scale(double *in_mat, double scale, double *result, int row, int col);

		void array_copy(const double *src, double *dst, int length);
		void set_euler(double euler[3], double yaw, double pitch, double roll);
		void set_pos(double pos[3], double x, double y, double z);
		void set_quat(double quat[4], double q0, double q1, double q2, double q3);

		void get_pos_rotmat_back(double rot_a_to_b[9], double pos_a_in_b[3], double pos_b_in_a[3]);
		void get_pitch_roll_by_acc(double ax, double ay, double az, double &pitch, double &roll);
		void get_pitch_roll_by_acc(double acc[3], double &pitch, double &roll);
	}

}
#endif