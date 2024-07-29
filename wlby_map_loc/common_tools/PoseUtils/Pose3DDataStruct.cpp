#include "Pose3DDataStruct.h"
#include "angle_math_utils.h"
#include "rot_fuc_deg.h"
#include <math.h>


namespace Localization {

	Pos3D Pos3D::operator+(const Pos3D& pos) const {
		Pos3D result;
		result._xyz[0] = _xyz[0] + pos._xyz[0];
		result._xyz[1] = _xyz[1] + pos._xyz[1];
		result._xyz[2] = _xyz[2] + pos._xyz[2];
		return result;
	}

	Pos3D Pos3D::operator-(const Pos3D& pos) const {
		Pos3D result;
		result._xyz[0] = _xyz[0] - pos._xyz[0];
		result._xyz[1] = _xyz[1] - pos._xyz[1];
		result._xyz[2] = _xyz[2] - pos._xyz[2];
		return result;
	}

	Pos3D Pos3D::operator*(double scale) const {
		Pos3D result;
		result._xyz[0] = _xyz[0] * scale;
		result._xyz[1] = _xyz[1] * scale;
		result._xyz[2] = _xyz[2] * scale;
		return result;
	}

	double Pos3D::getNormal() {
		return sqrt(_xyz[0] * _xyz[0] + _xyz[1] * _xyz[1] + _xyz[2] * _xyz[2]);
	}

	double Pos3D::getEuclideanDis(const Pos3D& pos1, const Pos3D& pos2) {
		double dx = pos1._xyz[0] - pos2._xyz[0];
		double dy = pos1._xyz[1] - pos2._xyz[1];
		double dz = pos1._xyz[2] - pos2._xyz[2];

		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	Orient3D::Orient3D(double yaw_deg, double pitch_deg, double roll_deg) {
		rot_fuc_deg::set_euler(_eulerDeg, yaw_deg, pitch_deg, roll_deg);
		uniformByEuler();
	}

	Orient3D::Orient3D(double quat[4]) {
		rot_fuc_deg::array_copy(quat, _quat, 4);
		uniformByQuat();
	}

	Orient3D::Orient3D(const Orient3D& orient) {
		rot_fuc_deg::array_copy(orient._eulerDeg, _eulerDeg, 3);
		rot_fuc_deg::array_copy(orient._rotmat, _rotmat, 9);
		rot_fuc_deg::array_copy(orient._quat, _quat, 4);
	}

	const Orient3D& Orient3D::operator=(const Orient3D& orient) {
		rot_fuc_deg::array_copy(orient._eulerDeg, _eulerDeg, 3);
		rot_fuc_deg::array_copy(orient._rotmat, _rotmat, 9);
		rot_fuc_deg::array_copy(orient._quat, _quat, 4);
		return *this;
	}

	Orient3D Orient3D::operator*(const Orient3D& delta_orient) const {
		Orient3D result;
		rot_fuc_deg::rotmat_multi(_rotmat, delta_orient._rotmat, result._rotmat);
		result.uniformByRotmat();
		return result;
	}

	Pos3D Orient3D::operator*(const Pos3D& pos) const {
		Pos3D result;
		rot_fuc_deg::rot_vector(_rotmat, pos._xyz, result._xyz);
		return result;
	}

	Orient3D Orient3D::operator / (const Orient3D& orienti) const {
		Orient3D delta;
		double inv_rot_i[9];
		rot_fuc_deg::rotmat_inv(orienti._rotmat, inv_rot_i);
		rot_fuc_deg::rotmat_multi(inv_rot_i, _rotmat, delta._rotmat);
		delta.uniformByRotmat();
		return delta;
	}

	void Orient3D::uniformByEuler() {
		rot_fuc_deg::euler_to_rotmat(_eulerDeg, _rotmat);
		rot_fuc_deg::euler_to_quat(_eulerDeg, _quat);
	}

	void Orient3D::uniformByRotmat() {
		rot_fuc_deg::rotmat_to_euler_new(_rotmat, _eulerDeg);
		rot_fuc_deg::euler_to_quat(_eulerDeg, _quat);
	}
	void Orient3D::uniformByQuat() {
		rot_fuc_deg::quat_to_rotmat(_quat, _rotmat);
		rot_fuc_deg::rotmat_to_euler_new(_rotmat, _eulerDeg);
	}

	void Orient3D::normalize() {
		rot_fuc_deg::normalize_quat(_quat);
		uniformByQuat();
	}

	Orient3D Orient3D::getRecursionOrient(const Orient3D& orienti, const Orient3D& delta_orient) {
		Orient3D result;
		rot_fuc_deg::rotmat_multi(orienti._rotmat, delta_orient._rotmat, result._rotmat);
		result.uniformByRotmat();
		return result;
	}

	Orient3D Orient3D::getRelativeOrient(const Orient3D& orienti, const Orient3D& orientj) {
		Orient3D delta;
		double inv_rot_i[9];
		rot_fuc_deg::rotmat_inv(orienti._rotmat, inv_rot_i);
		rot_fuc_deg::rotmat_multi(inv_rot_i, orientj._rotmat, delta._rotmat);
		delta.uniformByRotmat();
		return delta;
	}

	Orient3D Orient3D::getInvOrient() const {
		Orient3D result;
		rot_fuc_deg::rotmat_inv(_rotmat, result._rotmat);
		rot_fuc_deg::quat_unit_inv(_quat, result._quat);
		rot_fuc_deg::rotmat_to_euler_new(result._rotmat, result._eulerDeg);
		return result;
	}

	void Orient3D::changeYaw(double yaw_deg) {
		_eulerDeg[0] = yaw_deg;
		uniformByEuler();
	}

	void Orient3D::changePitchRoll(double pitch_deg, double roll_deg) {
		_eulerDeg[1] = pitch_deg;   _eulerDeg[2] = roll_deg;
		uniformByEuler();
	}

	Pose3D::Pose3D(double x, double y, double z, double yaw_deg, double pitch_deg, double roll_deg)
		: _pos(x, y, z), _orient(yaw_deg, pitch_deg, roll_deg) {
	}

	Pose3D::Pose3D(double xyz[3], double euler_deg[3]) : _pos(xyz), _orient(euler_deg[0], euler_deg[1], euler_deg[2]) {
	}

	Pose3D::Pose3D(const Pose3D& other) : _pos(other._pos), _orient(other._orient) {
	}

	const Pose3D& Pose3D::operator=(const Pose3D& other) {
		_pos = other._pos;
		_orient = other._orient;
		return *this;
	}

	Pose3D Pose3D::operator*(const Pose3D& pose) const {
		Pose3D result;
		result._pos = _pos + _orient*pose._pos;
		result._orient = _orient*pose._orient;

		return result;
	}

	Pos3D Pose3D::operator*(const Pos3D& pos) const {
		return _pos + _orient*pos;
	}

	Pose3D Pose3D::operator / (const Pose3D& posei) const {
		Pose3D delta;
		Orient3D orienti_inv = posei._orient.getInvOrient();
		delta._pos = orienti_inv*(_pos - posei._pos);
		delta._orient = orienti_inv*_orient;
		return delta;
	}

	Pose2D Pose3D::getPose2D() {
		return Pose2D(_pos._xyz[0], _pos._xyz[1], _orient._eulerDeg[0]);
	}

	Pose3D Pose3D::getPose3D(const Pose2D& pose_2d) {
		return Pose3D(pose_2d._pos._x, pose_2d._pos._y, 0, pose_2d._orient._deg, 0, 0);
	}

	Pose3D Pose3D::getRecursionPose(const Pose3D& posei, const Pose3D& delta_pose) {
		return posei*delta_pose;
	}

	Pose3D Pose3D::getRelativePose(const Pose3D& posei, const Pose3D& posej) {
		return posej / posei;
	}

	double Pose3D::getEuclideanDis(const Pose3D& pose1, const Pose3D& pose2) {
		return Pos3D::getEuclideanDis(pose1._pos, pose2._pos);
	}

	Pose3D Pose3D::getInvPose() const {
		Pose3D pose_inv;
		pose_inv._orient = _orient.getInvOrient();
		pose_inv._pos = (pose_inv._orient*_pos)*-1;
		return pose_inv;
	}

}