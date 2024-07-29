#include "Pose2DDataStruct.h"
#include "angle_math_utils.h"
#include "rot_fuc_deg2d.h"
#include "Matrix.h"
#include <math.h>
#include <iostream>


using namespace Localization::angle_math_utils;
using namespace Localization::rot_fuc_deg2d;

namespace Localization {

	Pos2D Polar2D::toPos() const {
		return Pos2D(_radius*cosd(_deg), _radius*sind(_deg));
	}


	double Polar2D::getEuclideanDis(const Polar2D& polar1, const Polar2D& polar2) {
		Pos2D pos1 = polar1.toPos();
		Pos2D pos2 = polar2.toPos();
		return Pos2D::getEuclideanDis(pos1, pos2);
	}

	void Polar2D::getPolarErr(const Polar2D& polar1, const Polar2D& polar2, double &radius_err, double &deg_err) {
		radius_err = fabs(polar1._radius - polar2._radius);
		deg_err = fabs(normalize_deg(polar1._deg - polar2._deg));
	}

	std::vector<Pos2D> Polar2D::polarVecToPosVec(const std::vector<Polar2D>& polar_vec) {
		std::vector<Pos2D> pos_vec;

		for (int i = 0; i < polar_vec.size(); ++i) {
			pos_vec.push_back(polar_vec[i].toPos());
		}

		return pos_vec;
	}

	const Pos2D& Pos2D::operator = (const Pos2D& pos) {
		_x = pos._x;   _y = pos._y;
		return *this;
	}

	Pos2D Pos2D::operator+(const Pos2D& pos) const {
		Pos2D result;
		result._x = _x + pos._x;
		result._y = _y + pos._y;
		return result;
	}

	Pos2D Pos2D::operator-(const Pos2D& pos) const {
		Pos2D result;
		result._x = _x - pos._x;
		result._y = _y - pos._y;
		return result;
	}

	Pos2D Pos2D::operator*(double scale) const {
		Pos2D result;
		result._x = _x*scale;
		result._y = _y*scale;
		return result;
	}

	double Pos2D::getEuclideanDis(const Pos2D& pos1, const Pos2D& pos2) {
		double dx = pos1._x - pos2._x;
		double dy = pos1._y - pos2._y;
		return sqrt(dx*dx + dy*dy);
	}

	double Pos2D::norm() const{
		return sqrt(_x*_x + _y*_y);
	}

	double Pos2D::gradDeg() {
		double deg = rad_to_deg(atan2(_y, _x));
		return deg;
	}

	void Pos2D::print() {
		std::cout << "Pos2D: " << _x << " " << _y << std::endl;
	}

	Polar2D Pos2D::toPolar() const {
		double r = sqrt(_x*_x + _y*_y);
		double deg = rad_to_deg(atan2(_y, _x));
		return Polar2D(deg, r);
	}

	Orient2D::Orient2D(double deg) {
		_deg = deg;
	}

	Orient2D::Orient2D(const Orient2D& other) {
		_deg = other._deg;
	}

	const Orient2D& Orient2D::operator=(const Orient2D& other) {
		_deg = other._deg;
		return *this;
	}

	Orient2D Orient2D::operator*(const Orient2D& orient) const {
		double result_deg = normalize_deg(_deg + orient._deg);
		return Orient2D(result_deg);
	}

	Pos2D Orient2D::operator*(const Pos2D& pos) const {
		double result_x = pos._x*cosd(_deg) - pos._y*sind(_deg);
		double result_y = pos._x*sind(_deg) + pos._y*cosd(_deg);
		return Pos2D(result_x, result_y);
	}

	Polar2D Orient2D::operator*(const Polar2D& polar) const {
		double result_deg = normalize_deg(polar._deg + _deg);
		return Polar2D(result_deg, polar._radius);
	}

	Orient2D Orient2D::operator/(const Orient2D& orient) const {
		double result_deg = normalize_deg(_deg - orient._deg);
		return Orient2D(result_deg);
	}

	Pos2D Orient2D::rotatePos2D(const Pos2D& pos, double deg) {
		double result_x = pos._x*cosd(deg) - pos._y*sind(deg);
		double result_y = pos._x*sind(deg) + pos._y*cosd(deg);
		return Pos2D(result_x, result_y);
	}

	Orient2D Orient2D::getInvOrient() const {
		return Orient2D(-_deg);
	}

	const Pose2D& Pose2D::operator = (const Pose2D& other) {
		_pos = other._pos;
		_orient = other._orient;
		return *this;
	}

	Pose2D Pose2D::operator*(const Pose2D& pose) const {
		Pos2D pos = _pos + _orient*pose._pos;
		Orient2D orient = _orient*pose._orient;

		return Pose2D(pos, orient);
	}

	Pos2D Pose2D::operator*(const Pos2D& pos) const {
		return _pos + _orient*pos;
	}

	Pos2D Pose2D::operator*(const Polar2D& polar) const {
		return (*this)*polar.toPos();
	}

	Pose2D Pose2D::operator/(const Pose2D& posei) const {
		Orient2D inv_orienti = posei._orient.getInvOrient();
		Pos2D pos = inv_orienti*(_pos - posei._pos);
		Orient2D orient = inv_orienti*_orient;

		return Pose2D(pos, orient);
	}

	Pose2D Pose2D::getInvPose() const {
		Orient2D orient_inv = _orient.getInvOrient();
		Pos2D pos = (orient_inv*_pos)*-1;
		return Pose2D(pos, orient_inv);
	}

	Pose2D Pose2D::devideCalibPose(const Pose2D& calib_pose) const {
		Orient2D odo_orient = _orient / calib_pose._orient;
		Pos2D odo_pos = _pos - odo_orient * calib_pose._pos;
		return Pose2D(odo_pos, odo_orient);
	}

	void Pose2D::print() const {
		std::cout<<"Pose2D: " << _pos._x << " " << _pos._y << " " << _orient._deg << std::endl;
	}

	Pose2D Pose2D::getRelativePoseOfTwoPose(const Pose2D& posei, const Pose2D& posej) {//relative defined in i
		return posej / posei;
	}

	Pose2D Pose2D::getRecursionPose(const Pose2D& posei, const Pose2D& delta_pose) {//delta_pose defined in i
		return posei*delta_pose;
	}

	Pose2D Pose2D::getInvDeltaPose(const Pose2D& delta_pose) {
		return delta_pose.getInvPose();
	}

	Pos2D Pose2D::getRelativePos(const Pose2D& g_pose, const Pos2D& g_pos) {
		Pos2D temp_pos = g_pos - g_pose._pos;
		Orient2D inv_orient = g_pose._orient.getInvOrient();
		return inv_orient * temp_pos;
	}

	Polar2D Pose2D::getRelativePolar(const Pose2D& g_pose, const Pos2D& g_pos) {
		Pos2D pos = getRelativePos(g_pose, g_pos);
		return pos.toPolar();
	}

	void Pose2D::relativePoseToRotationTranslation(const Pose2D& pose, double rot[9], double trans[3]) {
		trans[0] = pose._pos._x;
		trans[1] = pose._pos._y;
		trans[2] = 0;

		double yaw_deg = pose._orient._deg;
		rot[0] = cosd(yaw_deg);   rot[1] = -sind(yaw_deg);   rot[2] = 0;
		rot[3] = sind(yaw_deg);   rot[4] = cosd(yaw_deg);    rot[5] = 0;
		rot[6] = 0;               rot[7] = 0;                rot[8] = 1;
	}

	void Pose2D::rotationTranslationToRelativePose(double rot[9], double trans[3], Pose2D& pose) {
		pose._pos._x = trans[0];
		pose._pos._y = trans[1];

		double c = rot[0];
		double s = rot[3];
		double model = sqrt(c*c + s*s);
		c /= model;
		s /= model;
		double rad = 0;
		if (s > 0) {
			rad = acos(c);
		}
		else if (s < 0) {
			rad = -acos(c);
		}
		else {
			if (c > 0) {
				rad = 0;
			}
			else {
				rad = 3.1415926535;
			}
		}

		pose._orient._deg = rad_to_deg(rad);
	}

	double Pose2D::getEuclideanDis(const Pose2D& posei, const Pose2D& posej) {
		double dx = posei._pos._x - posej._pos._x;
		double dy = posei._pos._y - posej._pos._y;

		double dis = sqrt(dx*dx + dy*dy);
		return dis;
	}

	double Pose2D::getRelativeDegOfTwoPose(const Pose2D& posei, const Pose2D& posej) {
		double deg = normalize_deg(posej._orient._deg - posei._orient._deg);
		return deg;
	}

	bool Pose2D::getDeltaPoseOfTwoAxis(const std::vector<Pos2D>& pos_in1_vec, const std::vector<Pos2D>& pos_in2_vec, Pose2D& pose_2in1) {
		if (pos_in1_vec.size() != pos_in2_vec.size()) {
			return false;
		}

		int point_size = pos_in1_vec.size();
		if (point_size < 2) {
			return false;
		}

		CMatrix H(2 * point_size, 4);
		CMatrix Z(2 * point_size, 1);

		for (int i = 0; i < point_size; ++i) {
			int index = 2 * i;
			H(index, 0) = 1;     H(index, 1) = 0;     H(index, 2) = pos_in2_vec[i]._x;     H(index, 3) = -pos_in2_vec[i]._y;
			H(index + 1, 0) = 0;   H(index + 1, 1) = 1;   H(index + 1, 2) = pos_in2_vec[i]._y;   H(index + 1, 3) = pos_in2_vec[i]._x;

			Z(index, 0) = pos_in1_vec[i]._x;
			Z(index + 1, 0) = pos_in1_vec[i]._y;
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

		pose_2in1._pos._x = dx;
		pose_2in1._pos._y = dy;

		pose_2in1._orient._deg = atan2(sin_angle, cos_angle) * 180 / 3.1415926535;

		return true;
	}

}
