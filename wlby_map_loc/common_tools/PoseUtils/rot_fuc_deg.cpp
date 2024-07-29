/****************************************************************
*欧拉角、旋转矩阵、四元数两两之间的转换
by huanghong   20151208
所有输入输出的角度、角速度的单位都用deg
****************************************************************/


#include"rot_fuc_deg.h"
#include<math.h>
#include<iostream>
#include "angle_math_utils.h"

using namespace std;
using namespace Localization::angle_math_utils;

namespace Localization {

	namespace rot_fuc_deg {
		/*function [R]=RotMat(yaw,pitch,roll)
		  坐标系L经yaw/pitch/roll后得到新的坐标系L'，则此处的R可得到L'中的向量在原L中的坐标
		  R=[cos(pitch)*cos(yaw),sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw),cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);...
		  cos(pitch)*sin(yaw),sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw),cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);...
		  -sin(pitch),sin(roll)*cos(pitch),cos(roll)*cos(pitch)];
		  Rz*Ry*Rx
		  = cos(yaw) -sin(yaw) 0     cos(pitch)  0  sin(pitch)    1         0          0
		  sin(yaw)  cos(yaw) 0  *          0   1          0  *  0  cos(roll) -sin(roll)
		  0         0  1     -sin(pitch) 0  cos(pitch)    0  sin(roll)  cos(roll)
		  =
		  [ cos(dy)*cos(dz), sin(dx)*sin(dy)*cos(dz)-cos(dx)*sin(dz), cos(dx)*sin(dy)*cos(dz)+sin(dx)*sin(dz)]
		  [ cos(dy)*sin(dz), sin(dx)*sin(dy)*sin(dz)+cos(dx)*cos(dz), cos(dx)*sin(dy)*sin(dz)-sin(dx)*cos(dz)]
		  [        -sin(dy),                         sin(dx)*cos(dy),                         cos(dx)*cos(dy)]
		  */
		//欧拉角变旋转矩阵
		void euler_to_rotmat(double yaw, double pitch, double roll, double R[9])
		{
			yaw = deg_to_rad(yaw);
			pitch = deg_to_rad(pitch);
			roll = deg_to_rad(roll);
			R[0] = cos(pitch)*cos(yaw);	R[1] = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);	R[2] = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
			R[3] = cos(pitch)*sin(yaw);   R[4] = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);  R[5] = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
			R[6] = -sin(pitch);           R[7] = sin(roll)*cos(pitch);                              R[8] = cos(roll)*cos(pitch);
		}
		//欧拉角变旋转矩阵
		void euler_to_rotmat(double euler[3], double R[9])
		{
			double yaw = deg_to_rad(euler[0]);
			double pitch = deg_to_rad(euler[1]);
			double roll = deg_to_rad(euler[2]);
			R[0] = cos(pitch)*cos(yaw);	R[1] = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);	R[2] = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
			R[3] = cos(pitch)*sin(yaw);   R[4] = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);  R[5] = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
			R[6] = -sin(pitch);           R[7] = sin(roll)*cos(pitch);                              R[8] = cos(roll)*cos(pitch);
		}

		/*function [yaw,pitch,roll]=rot_to_ola(R)
		%由旋转矩阵得到欧拉角，与RotMat是逆过程
		%yaw(-180°~180°)，pitch(-90°~90°),roll(-180°~180°),这里单位是弧度
		% R=[cos(pitch)*cos(yaw),sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw),cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);...
		%    cos(pitch)*sin(yaw),sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw),cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);...
		%    -sin(pitch),sin(roll)*cos(pitch),cos(roll)*cos(pitch)];
		*/
		//现在一般使用下面那个版本rotmat_to_ola_new
		void rotmat_to_euler_old(double R[9], double euler_result[3])//矩阵下标是从0开始的
		{
			//double yaw,pitch,roll;
			double euler[3];
			double sroll, croll, cyaw, syaw, cpitch;
			euler[1] = -asin(R[6]);
			if (cos(euler[1]) == 0) cpitch = 0.000001;// yaw=NaN;roll=NaN;
			else               cpitch = cos(euler[1]);

			sroll = R[7] / cpitch; croll = R[8] / cpitch;
			cyaw = R[0] / cpitch;  syaw = R[3] / cpitch;

			if (sroll > 0)
				euler[2] = acos(croll);
			if (sroll<0)
				euler[2] = -acos(croll);
			if (sroll == 0) {
				if (croll>0) euler[2] = 0;
				if (croll<0) euler[2] = pi;
				//if(croll==0) roll=NaN;                
			}
			if (syaw>0)
				euler[0] = acos(cyaw);
			if (syaw<0)
				euler[0] = -acos(cyaw);
			if (syaw == 0) {
				if (cyaw>0) euler[0] = 0;
				if (cyaw < 0) euler[0] = pi;
				//if(cyaw==0) yaw=NaN;
			}
			euler_result[0] = rad_to_deg(euler[0]);
			euler_result[1] = rad_to_deg(euler[1]);
			euler_result[2] = rad_to_deg(euler[2]);
		}
		// R=[cos(pitch)*cos(yaw),   sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw),   cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);...
		//	% cos(pitch)*sin(yaw),   sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw),   cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);...
		//	% -sin(pitch),           sin(roll)*cos(pitch),                               cos(roll)*cos(pitch)];
		void rotmat_to_euler_new(const double R[9], double euler_result[3]) {
			double euler[3];
			double cp = sqrt(R[0] * R[0] + R[3] * R[3]);
			if (cp == 0) {
				euler[0] = 0;
				euler[2] = 0;
				if (R[6] >= 0) euler[1] = -pi / 2;
				else        euler[1] = pi / 2;
			} else {
				euler[0] = atan2(R[3], R[0]);
				euler[1] = atan2(-R[6], cp);
				euler[2] = atan2(R[7], R[8]);
			}
			euler_result[0] = rad_to_deg(euler[0]);
			euler_result[1] = rad_to_deg(euler[1]);
			euler_result[2] = rad_to_deg(euler[2]);
		}

		void rotmat_to_euler_new_old(const double R[9], double euler_result[3])
		{//yaw(-pi pi]  pitch[-pi/2 pi/2] roll(-pi pi]
			double euler[3];
			double cp = sqrt(R[0] * R[0] + R[3] * R[3]);
			if (cp == 0)
			{
				euler[0] = 0;
				if (R[6] >= 0) euler[1] = -pi / 2;
				else        euler[1] = pi / 2;
				if (R[4] != 0)
				{
					if (euler[1] > 0) euler[2] = atan(R[1] / R[4]);
					else euler[2] = -atan(R[1] / R[4]);
					if (R[4] < 0 && euler[2] <= 0) euler[2] += pi;
					else if (R[4]<0 && euler[2]>0) euler[2] -= pi;
				}
				else
				{
					if (R[1] >= 0) euler[2] = euler[1];
					else        euler[2] = -euler[1];
				}
			}
			else
			{
				euler[1] = atan(-R[6] / cp);
				if (R[0] != 0)
				{
					euler[0] = atan(R[3] / R[0]);
					if (R[0] < 0 && R[3] >= 0) euler[0] += pi;
					if (R[0] < 0 && R[3] < 0) euler[0] -= pi;
				}
				else
				{
					if (R[3] >= 0) euler[0] = pi / 2;
					else        euler[0] = -pi / 2;
				}
				if (R[8] != 0)
				{
					euler[2] = atan(R[7] / R[8]);
					if (R[8] < 0 && R[7] >= 0) euler[2] += pi;
					if (R[8] < 0 && R[7] < 0) euler[2] -= pi;
				}
				else
				{
					if (R[7] >= 0) euler[2] = pi / 2;
					else        euler[2] = -pi / 2;
				}
			}
			euler_result[0] = rad_to_deg(euler[0]);
			euler_result[1] = rad_to_deg(euler[1]);
			euler_result[2] = rad_to_deg(euler[2]);
		}

		/*由欧拉角得到四元数q0,q1,q2,q3
		  R=[2*q0*q0+2*q1*q1-1,2*q1*q2-2*q0*q3,2*q1*q3+2*q0*q2;2*q1*q2+2*q0*q3,2*q0*q0+2*q2*q2-1,2*q2*q3-2*q0*q1;2*q1*q3-2*q0*q2,2*q2*q3+2*q0*q1,2*q0*q0+2*q3*q3-1]
		  坐标系L经yaw/pitch/roll后得到新的坐标系L'，则此处的qOL'Oq_可得到L'中的向量在原L中的坐标

		  qx=[cos(roll/2),sin(roll/2),0,0]';	    qx_=[cos(roll/2),-sin(roll/2),0,0]';
		  qy=[cos(pitch/2),0,sin(pitch/2),0]';	qy_=[cos(pitch/2),0,-sin(pitch/2),0]';
		  qz=[cos(yaw/2),0,0,sin(yaw/2)]';	    qz_=[cos(yaw/2),0,0,-sin(yaw/2)]';
		  ori=[0,x,y,z]';
		  qxOoriOqx_=Rx*ori  qyOoriOqy_=Ry*ori  qzOoriOqz_=Rz*ori
		  qzqyqxOoriOqx_qy_qz_
		  = Rz*Ry*Rx*ori
		  = cos(yaw) -sin(yaw) 0     cos(pitch)  0  sin(pitch)    1         0          0     x
		  sin(yaw)  cos(yaw) 0  *          0   1          0  *  0  cos(roll) -sin(roll) *  y
		  0         0  1     -sin(pitch) 0  cos(pitch)    0  sin(roll)  cos(roll)    z
		  */
		void euler_to_quat(double yaw, double pitch, double roll, double qut[4])
		{
			yaw = deg_to_rad(yaw);
			pitch = deg_to_rad(pitch);
			roll = deg_to_rad(roll);
			qut[0] = cos(yaw / 2)*cos(pitch / 2)*cos(roll / 2) + sin(yaw / 2)*sin(pitch / 2)*sin(roll / 2);
			qut[1] = cos(yaw / 2)*cos(pitch / 2)*sin(roll / 2) - sin(yaw / 2)*sin(pitch / 2)*cos(roll / 2);
			qut[2] = cos(yaw / 2)*sin(pitch / 2)*cos(roll / 2) + sin(yaw / 2)*cos(pitch / 2)*sin(roll / 2);
			qut[3] = sin(yaw / 2)*cos(pitch / 2)*cos(roll / 2) - cos(yaw / 2)*sin(pitch / 2)*sin(roll / 2);
		}
		void euler_to_quat(double euler[3], double qut[4])
		{
			double yaw = deg_to_rad(euler[0]);
			double pitch = deg_to_rad(euler[1]);
			double roll = deg_to_rad(euler[2]);
			qut[0] = cos(yaw / 2)*cos(pitch / 2)*cos(roll / 2) + sin(yaw / 2)*sin(pitch / 2)*sin(roll / 2);
			qut[1] = cos(yaw / 2)*cos(pitch / 2)*sin(roll / 2) - sin(yaw / 2)*sin(pitch / 2)*cos(roll / 2);
			qut[2] = cos(yaw / 2)*sin(pitch / 2)*cos(roll / 2) + sin(yaw / 2)*cos(pitch / 2)*sin(roll / 2);
			qut[3] = sin(yaw / 2)*cos(pitch / 2)*cos(roll / 2) - cos(yaw / 2)*sin(pitch / 2)*sin(roll / 2);
		}
		/*四元数转换成欧拉角*/
		void quat_to_euler(double Q[4], double euler[3])
		{
			double R[9];
			quat_to_rotmat(Q, R);
			rotmat_to_euler_new(R, euler);
		}
		/*
		四元数转换成旋转矩阵
		function [R]=qu_to_rot(q0,q1,q2,q3)
		R=[2*q0*q0+2*q1*q1-1,2*q1*q2-2*q0*q3,2*q1*q3+2*q0*q2;...
		2*q1*q2+2*q0*q3,2*q0*q0+2*q2*q2-1,2*q2*q3-2*q0*q1;...
		2*q1*q3-2*q0*q2,2*q2*q3+2*q0*q1,2*q0*q0+2*q3*q3-1];
		*/
		//原理是
		//[q0,q1,q2,q3]O[0,x,y,z]O[q0,-q1,-q2,-q3]
		//=     0
		//	Q1^2*x+2*Q1*Q2*y+2*Q1*Q3*z+Q0^2*x+2*Q0*Q2*z-2*Q0*Q3*y-Q3^2*x-Q2^2*x
		//	2*Q2*Q1*x+Q2^2*y+2*Q2*Q3*z+Q0^2*y-2*Q0*Q1*z+2*Q0*Q3*x-Q3^2*y-Q1^2*y
		//	2*Q3*Q1*x+2*Q3*Q2*y+Q3^2*z+Q0^2*z+2*Q0*Q1*y-2*Q0*Q2*x-Q2^2*z-Q1^2*z
		//=[0 R]*[0,x,y,z]T
		void quat_to_rotmat(double Q[4], double R[9])
		{
			R[0] = 2 * Q[0] * Q[0] + 2 * Q[1] * Q[1] - 1;  R[1] = 2 * Q[1] * Q[2] - 2 * Q[0] * Q[3];    R[2] = 2 * Q[1] * Q[3] + 2 * Q[0] * Q[2];
			R[3] = 2 * Q[1] * Q[2] + 2 * Q[0] * Q[3];    R[4] = 2 * Q[0] * Q[0] + 2 * Q[2] * Q[2] - 1;  R[5] = 2 * Q[2] * Q[3] - 2 * Q[0] * Q[1];
			R[6] = 2 * Q[1] * Q[3] - 2 * Q[0] * Q[2];    R[7] = 2 * Q[2] * Q[3] + 2 * Q[0] * Q[1];    R[8] = 2 * Q[0] * Q[0] + 2 * Q[3] * Q[3] - 1;
		}
		/*旋转矩阵转换成四元数*/
		void rotmat_to_quat(double R[9], double qut[4])//
		{
			double ola[3];
			rotmat_to_euler_new(R, ola);
			euler_to_quat(ola, qut);
		}

		/*
		function [Q0,Q1,Q2,Q3]=qua_multi(q0,q1,q2,q3,r0,r1,r2,r3)
		%两个四元数的乘法
		*/
		void quat_multi(double Q1[4], double Q2[4], double qut[4])
		{
			double qu0, qu1, qu2, qu3;
			qu0 = Q1[0] * Q2[0] - Q1[1] * Q2[1] - Q1[2] * Q2[2] - Q1[3] * Q2[3];
			qu1 = Q1[0] * Q2[1] + Q2[0] * Q1[1] + Q1[2] * Q2[3] - Q1[3] * Q2[2];
			qu2 = Q1[0] * Q2[2] + Q2[0] * Q1[2] - Q1[1] * Q2[3] + Q1[3] * Q2[1];
			qu3 = Q1[0] * Q2[3] + Q2[0] * Q1[3] + Q1[1] * Q2[2] - Q1[2] * Q2[1];
			qut[0] = qu0; qut[1] = qu1; qut[2] = qu2; qut[3] = qu3;
		}

		void d_quat_by_wt(double wx, double wy, double wz, double t, double qut[4]) {
			wx = deg_to_rad(wx);
			wy = deg_to_rad(wy);
			wz = deg_to_rad(wz);
			double w = sqrt(wx*wx + wy*wy + wz*wz);
			double wt = w*t / 2;

			if (wt == 0) {
				qut[0] = 1; qut[1] = 0; qut[2] = 0; qut[3] = 0;
				return;
			}

			qut[0] = cos(wt);
			qut[1] = sin(wt)*wx / w;
			qut[2] = sin(wt)*wy / w;
			qut[3] = sin(wt)*wz / w;
		}

		void d_quat_by_wt(double w[3], double t, double qut[4]) {
            d_quat_by_wt(w[0], w[1], w[2], t, qut);
		}

		void quat_change_by_wt(double wx, double wy, double wz, double t, double quat_ori[4], double quat_end[4]) {
			double d_quat[4];
			d_quat_by_wt(wx, wy, wz, t, d_quat);
			quat_multi(quat_ori, d_quat, quat_end);
		}

		void quat_change_by_wt(double w[3], double t, double quat_ori[4], double quat_end[4]) {
			double d_quat[4];
			d_quat_by_wt(w, t, d_quat);
			quat_multi(quat_ori, d_quat, quat_end);
		}

		void quat_recursion(double quat_ori[4], double qut_delta[4], double quat_end[4]) {
			quat_multi(quat_ori, qut_delta, quat_end);
		}

		void quat_mat_by_wt(double wx, double wy, double wz, double t, double quat_mat_r[16]) {
			double d_quat[4];
			d_quat_by_wt(wx, wy, wz, t, d_quat);
			quat_mat_right(d_quat, quat_mat_r);
		}

		void quat_mat_by_wt(double w[3], double t, double quat_mat_r[16]) {
			double d_quat[4];
			d_quat_by_wt(w, t, d_quat);
			quat_mat_right(d_quat, quat_mat_r);
		}

		void quat_mat_left(double quat[4], double quat_mat[16]) {
			quat_mat[0] = quat[0];   quat_mat[1] = -quat[1];   quat_mat[2] = -quat[2];  quat_mat[3] = -quat[3];
			quat_mat[4] = quat[1];   quat_mat[5] = quat[0];    quat_mat[6] = -quat[3];  quat_mat[7] = quat[2];
			quat_mat[8] = quat[2];   quat_mat[9] = quat[3];    quat_mat[10] = quat[0];  quat_mat[11] = -quat[1];
			quat_mat[12] = quat[3];  quat_mat[13] = -quat[2];  quat_mat[14] = quat[1];  quat_mat[15] = quat[0];
		}

		void quat_mat_right(double quat[4], double quat_mat[16]) {
			quat_mat[0] = quat[0];   quat_mat[1] = -quat[1];  quat_mat[2] = -quat[2];   quat_mat[3] = -quat[3];
			quat_mat[4] = quat[1];   quat_mat[5] = quat[0];   quat_mat[6] = quat[3];    quat_mat[7] = -quat[2];
			quat_mat[8] = quat[2];   quat_mat[9] = -quat[3];  quat_mat[10] = quat[0];   quat_mat[11] = quat[1];
			quat_mat[12] = quat[3];  quat_mat[13] = quat[2];  quat_mat[14] = -quat[1];  quat_mat[15] = quat[0];
		}

		void quat_unit_inv(const double quat[4], double quat_inv[4]) {
			quat_inv[0] = quat[0];
			quat_inv[1] = -quat[1];
			quat_inv[2] = -quat[2];
			quat_inv[3] = -quat[3];
		}

		void error_by_d_quat(double d_quat[4], double err_result[3]) {
			double err[3];
			err[0] = 2 * d_quat[1];
			err[1] = 2 * d_quat[2];
			err[2] = 2 * d_quat[3];
			err_result[0] = rad_to_deg(err[0]);
			err_result[1] = rad_to_deg(err[1]);
			err_result[2] = rad_to_deg(err[2]);
		}

		void d_quat_by_error(double err[3], double d_quat[4]) {
			double err_rad[3] = { deg_to_rad(err[0]), deg_to_rad(err[1]), deg_to_rad(err[2]) };
			double dq1 = err_rad[0] / 2;
			double dq2 = err_rad[1] / 2;
			double dq3 = err_rad[2] / 2;
			double dqq = dq1*dq1 + dq2*dq2 + dq3*dq3;

			d_quat[1] = dq1;
			d_quat[2] = dq2;
			d_quat[3] = dq3;

			if (dqq > 1) {
				d_quat[0] = 1 / sqrt(1 + dqq);
				d_quat[1] /= d_quat[0];
				d_quat[2] /= d_quat[0];
				d_quat[3] /= d_quat[0];
			}
			else {
				d_quat[0] = sqrt(1 - dqq);
			}
		}

		void normalize_quat(double quat[4]) {
			double model = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
			quat[0] /= model;   quat[1] /= model;   quat[2] /= model;   quat[3] /= model;
		}

		//exp(1/2[w]dt) + 1/48([w_k+1]*[w_k]-[w_k]*[w_k+1])*dt*dt
		void quat_mat_by_wt_first_order(double w_k_1_deg[3], double w_k_deg[3], double dt, double mat[16]) {
			double w_k_1[3] = { deg_to_rad(w_k_1_deg[0]), deg_to_rad(w_k_1_deg[1]), deg_to_rad(w_k_1_deg[2]) };
			double w_k[3] = { deg_to_rad(w_k_deg[0]), deg_to_rad(w_k_deg[1]), deg_to_rad(w_k_deg[2]) };

			double w_m[3] = { (w_k_1[0] + w_k[0]) / 2, (w_k_1[1] + w_k[1]) / 2, (w_k_1[2] + w_k[2]) / 2 };
			double d_quat[4];
			d_quat_by_wt(w_m[0], w_m[1], w_m[2], dt, d_quat);
			quat_mat_right(d_quat, mat);//

			//1/48scale
			double dt_dt_48 = dt*dt / 48;

			double mw_k_1[16], mw_k[16];
			double q_w_k_1[4] = { 0, w_k_1[0], w_k_1[1], w_k_1[2] }, q_w_k[4] = { 0, w_k[0], w_k[1], w_k[2] };
			quat_mat_left(q_w_k_1, mw_k_1);//
			quat_mat_left(q_w_k, mw_k);//

			double multi_tmp1[16], multi_tmp2[16];
			mat_multi(mw_k, mw_k_1, multi_tmp1, 4, 4, 4);
			mat_multi(mw_k_1, mw_k, multi_tmp2, 4, 4, 4);
			double min_tmp[16], scale_tmp[16];
			mat_min(multi_tmp1, multi_tmp2, min_tmp, 4, 4);
			mat_multi_scale(min_tmp, dt_dt_48, scale_tmp, 4, 4);

			//combine  //-/+
			mat[0] = mat[0] + scale_tmp[0];     mat[1] = mat[1] + scale_tmp[1];     mat[2] = mat[2] + scale_tmp[2];     mat[3] = mat[3] + scale_tmp[3];
			mat[4] = mat[4] + scale_tmp[4];     mat[5] = mat[5] + scale_tmp[5];     mat[6] = mat[6] - scale_tmp[6];     mat[7] = mat[7] - scale_tmp[7];
			mat[8] = mat[8] + scale_tmp[8];     mat[9] = mat[9] - scale_tmp[9];     mat[10] = mat[10] + scale_tmp[10];  mat[11] = mat[11] - scale_tmp[11];
			mat[12] = mat[12] + scale_tmp[12];  mat[13] = mat[13] - scale_tmp[13];  mat[14] = mat[14] - scale_tmp[14];  mat[15] = mat[15] + scale_tmp[15];
		}

		void rotmat_multi(const double rot1[9], const double rot2[9], double rot3[9]) {
			double rot[9];
			rot[0] = rot1[0] * rot2[0] + rot1[1] * rot2[3] + rot1[2] * rot2[6];  rot[1] = rot1[0] * rot2[1] + rot1[1] * rot2[4] + rot1[2] * rot2[7];  rot[2] = rot1[0] * rot2[2] + rot1[1] * rot2[5] + rot1[2] * rot2[8];
			rot[3] = rot1[3] * rot2[0] + rot1[4] * rot2[3] + rot1[5] * rot2[6];  rot[4] = rot1[3] * rot2[1] + rot1[4] * rot2[4] + rot1[5] * rot2[7];  rot[5] = rot1[3] * rot2[2] + rot1[4] * rot2[5] + rot1[5] * rot2[8];
			rot[6] = rot1[6] * rot2[0] + rot1[7] * rot2[3] + rot1[8] * rot2[6];  rot[7] = rot1[6] * rot2[1] + rot1[7] * rot2[4] + rot1[8] * rot2[7];  rot[8] = rot1[6] * rot2[2] + rot1[7] * rot2[5] + rot1[8] * rot2[8];

			array_copy(rot, rot3, 9);
		}

		//I+sin(wt)*s(w)/w+(1-cos(wt))*s(w)*s(w)/ww
		void d_rotmat_by_wt(double wx, double wy, double wz, double t, double rot[9]) {
			wx = deg_to_rad(wx);
			wy = deg_to_rad(wy);
			wz = deg_to_rad(wz);
			double ww = wx*wx + wy*wy + wz*wz;
			double w = sqrt(ww);
			if (w == 0) {
				rot[0] = 1; rot[1] = 0; rot[2] = 0;
				rot[3] = 0; rot[4] = 1; rot[5] = 0;
				rot[6] = 0; rot[7] = 0; rot[8] = 1;
				return;
			}

			double sinwt_w = sin(w*t) / w;
			double one_cos_ww = (1 - cos(w*t)) / ww;

			rot[0] = 1 + 0 + one_cos_ww*(-wy*wy - wz*wz);    rot[1] = 0 - sinwt_w*wz + one_cos_ww*wx*wy;    rot[2] = 0 + sinwt_w*wy + one_cos_ww*wx*wz;
			rot[3] = 0 + sinwt_w*wz + one_cos_ww*wx*wy;    rot[4] = 1 + 0 + one_cos_ww*(-wx*wx - wz*wz);    rot[5] = 0 - sinwt_w*wx + one_cos_ww*wy*wz;
			rot[6] = 0 - sinwt_w*wy + one_cos_ww*wx*wz;    rot[7] = 0 + sinwt_w*wx + one_cos_ww*wy*wz;    rot[8] = 1 + 0 + one_cos_ww*(-wx*wx - wy*wy);
		}

		void d_rotmat_by_wt(double w[3], double t, double rot[9]) {
			d_rotmat_by_wt(w[0], w[1], w[2], t, rot);
		}

		void rotmat_change_by_wt(double wx, double wy, double wz, double t, double rot_ori[9], double rot_end[9]) {
			double d_rot[9];
			d_rotmat_by_wt(wx, wy, wz, t, d_rot);
			rotmat_multi(rot_ori, d_rot, rot_end);
		}

		void rotmat_change_by_wt(double w[3], double t, double rot_ori[9], double rot_end[9]) {
			double d_rot[9];
			d_rotmat_by_wt(w, t, d_rot);
			rotmat_multi(rot_ori, d_rot, rot_end);
		}

		void rot_recursion(const double rot_ori[9], const double rot_delta[9], double rot_end[9]) {
			rotmat_multi(rot_ori, rot_delta, rot_end);
		}

		//dR = I + S(wt)
		void sw_by_w(double wx, double wy, double wz, double sw[9]) {
			wx = deg_to_rad(wx);
			wy = deg_to_rad(wy);
			wz = deg_to_rad(wz);
			sw[0] = 0;     sw[1] = -wz;    sw[2] = wy;
			sw[3] = wz;    sw[4] = 0;      sw[5] = -wx;
			sw[6] = -wy;   sw[7] = wx;     sw[8] = 0;
		}

		void sw_by_w(double w[3], double sw[9]) {
			sw_by_w(w[0], w[1], w[2], sw);
		}

		void rotmat_inv(const double rot[9], double rot_inv[9]) {
			double r_inv[9];
			r_inv[0] = rot[0];  r_inv[1] = rot[3];  r_inv[2] = rot[6];
			r_inv[3] = rot[1];  r_inv[4] = rot[4];  r_inv[5] = rot[7];
			r_inv[6] = rot[2];  r_inv[7] = rot[5];  r_inv[8] = rot[8];
			array_copy(r_inv, rot_inv, 9);
		}

		void rot_vector(const double rot[9], const double vec_ori[3], double vec_end[3]) {
			double vec_result[3];
			vec_result[0] = rot[0] * vec_ori[0] + rot[1] * vec_ori[1] + rot[2] * vec_ori[2];
			vec_result[1] = rot[3] * vec_ori[0] + rot[4] * vec_ori[1] + rot[5] * vec_ori[2];
			vec_result[2] = rot[6] * vec_ori[0] + rot[7] * vec_ori[1] + rot[8] * vec_ori[2];
			array_copy(vec_result, vec_end, 3);
		}

		void mat_add(double *left, double *right, double *result, int row, int col) {
			for (int i = 0; i < row; ++i) {
				for (int j = 0; j < col; ++j) {
					result[i*col + j] = left[i*col + j] + right[i*col + j];
				}
			}
		}

		void mat_min(double *left, double *right, double *result, int row, int col) {
			for (int i = 0; i < row; ++i) {
				for (int j = 0; j < col; ++j) {
					result[i*col + j] = left[i*col + j] - right[i*col + j];
				}
			}
		}

		bool mat_multi(double *left, double *right, double *result, int row1, int col1, int col2) {
			double *temp = new double[row1, col2];
			if (temp == NULL) {
				return false;
			}

			double	value;
			for (int i = 0; i < row1; ++i) {
				for (int j = 0; j < col2; ++j) {
					value = 0.0;
					for (int k = 0; k < col1; ++k) {
						value += left[i*col1 + k] * right[k*col2 + j];//GetElement(i, k) * other.GetElement(k, j) ;
					}
					temp[i*col2 + j] = value;
				}
			}

			array_copy(temp, result, row1*col2);
			delete[]temp;
			return true;
		}

		void mat_multi_in_diff_out(double *left, double *right, double *result, int row1, int col1, int col2) {
			double	value;
			for (int i = 0; i < row1; ++i) {
				for (int j = 0; j < col2; ++j) {
					value = 0.0;
					for (int k = 0; k < col1; ++k) {
						value += left[i*col1 + k] * right[k*col2 + j];//GetElement(i, k) * other.GetElement(k, j) ;
					}
					result[i*col2 + j] = value;
				}
			}
		}

		void mat_multi_scale(double *in_mat, double scale, double *result, int row, int col) {
			for (int i = 0; i < row; ++i) {
				for (int j = 0; j < col; ++j) {
					result[i*col + j] = scale * in_mat[i*col + j];
				}
			}
		}

		void array_copy(const double *src, double *dst, int length) {
			for (int i = 0; i < length; i++) {
				dst[i] = src[i];
			}
		}

		void set_euler(double euler[3], double yaw, double pitch, double roll) {
			euler[0] = yaw;  euler[1] = pitch;  euler[2] = roll;
		}

		void set_pos(double pos[3], double x, double y, double z) {
			pos[0] = x;   pos[1] = y;   pos[2] = z;
		}

		void set_quat(double quat[4], double q0, double q1, double q2, double q3) {
			quat[0] = q0;  quat[1] = q1; quat[2] = q2;  quat[3] = q3;
		}

		void get_pos_rotmat_back(double rot_a_to_b[9], double pos_a_in_b[3], double pos_b_in_a[3]) {
			double rot_b_to_a[9];
			rotmat_inv(rot_a_to_b, rot_b_to_a);

			rot_vector(rot_b_to_a, pos_a_in_b, pos_b_in_a);
			pos_b_in_a[0] *= -1;
			pos_b_in_a[1] *= -1;
			pos_b_in_a[2] *= -1;
		}

		void get_pitch_roll_by_acc(double ax, double ay, double az, double &pitch, double &roll) {
			//-sin(pitch)
			//sin(roll)*cos(pitch)
			//cos(roll)*cos(pitch)

			double g = sqrt(ax*ax + ay*ay + az*az);
			pitch = asin(-ax / g);

			double cos_pitch = cos(pitch);
			if (cos_pitch == 0) {
				roll = 0;
				return;
			}

			if (az == 0) {
				roll = (pi / 2)*(ay > 0 ? 1 : (-1));
				return;
			}

			roll = atan(ay / az);
			if (roll<0 && ay>0) {
				roll += pi;
			}
			if (roll > 0 && ay < 0) {
				roll -= pi;
			}

			pitch = rad_to_deg(pitch);
			roll = rad_to_deg(roll);
		}

		void get_pitch_roll_by_acc(double acc[3], double &pitch, double &roll) {
			get_pitch_roll_by_acc(acc[0], acc[1], acc[2], pitch, roll);
		}

		//角速度积分算四元素的时候，算的q使qOL'Oq到L，则每单位时间刻的delta_q要右乘q_1Odelta_q=q_

	}

}