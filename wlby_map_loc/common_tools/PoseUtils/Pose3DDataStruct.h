#ifndef _POSE_3D_DATA_STRUCT_H_
#define _POSE_3D_DATA_STRUCT_H_

#include "Pose2DDataStruct.h"


namespace Localization {

	class Pos3D {
	public:
		double _xyz[3];

		Pos3D(double x = 0, double y = 0, double z = 0) {
			_xyz[0] = x;   _xyz[1] = y;   _xyz[2] = z;
		}
		Pos3D(double xyz[3]) {
			_xyz[0] = xyz[0];   _xyz[1] = xyz[1];   _xyz[2] = xyz[2];
		}
		Pos3D(const Pos3D& pos) {
			_xyz[0] = pos._xyz[0];   _xyz[1] = pos._xyz[1];   _xyz[2] = pos._xyz[2];
		}
		const Pos3D& operator=(const Pos3D& pos) {
			_xyz[0] = pos._xyz[0];   _xyz[1] = pos._xyz[1];   _xyz[2] = pos._xyz[2];
			return *this;
		}
		Pos3D operator+(const Pos3D& pos) const;
		Pos3D operator-(const Pos3D& pos) const;
		Pos3D operator*(double scale) const;

		double getNormal();

		static double getEuclideanDis(const Pos3D& pos1, const Pos3D& pos2);
	};

	class Orient3D {
	public:
		double _eulerDeg[3];
		double _rotmat[9];
		double _quat[4];

		Orient3D(double yaw_deg = 0, double pitch_deg = 0, double roll_deg = 0);
		Orient3D(double quat[4]);
		Orient3D(const Orient3D& orient);
		const Orient3D& operator=(const Orient3D& orient);
		Orient3D operator*(const Orient3D& delta_orient) const;
		Pos3D operator*(const Pos3D& pos) const;
		Orient3D operator/(const Orient3D& orienti) const;//this = orienti * result

		void uniformByEuler();
		void uniformByRotmat();
		void uniformByQuat();
		void normalize();

		static Orient3D getRecursionOrient(const Orient3D& orienti, const Orient3D& delta_orient);
		static Orient3D getRelativeOrient(const Orient3D& orienti, const Orient3D& orientj);
		Orient3D getInvOrient() const;
		void changeYaw(double yaw_deg);
		void changePitchRoll(double pitch_deg, double roll_deg);
	};

	class Pose3D {
	public:
		Pos3D _pos;
		Orient3D _orient;

		Pose3D(double x = 0, double y = 0, double z = 0, double yaw_deg = 0, double pitch_deg = 0, double roll_deg = 0);
		Pose3D(double xyz[3], double euler_deg[3]);
		Pose3D(const Pose3D& other);
		const Pose3D& operator=(const Pose3D& other);

		Pose3D operator*(const Pose3D& pose) const;
		Pos3D operator*(const Pos3D& pos) const;
		Pose3D operator/(const Pose3D& posei) const;//this = posei * result
		Pose2D getPose2D();
		static Pose3D getPose3D(const Pose2D& pose_2d);
		static Pose3D getRecursionPose(const Pose3D& posei, const Pose3D& delta_pose);
		static Pose3D getRelativePose(const Pose3D& posei, const Pose3D& posej);
		static double getEuclideanDis(const Pose3D& pose1, const Pose3D& pose2);

		Pose3D getInvPose() const;
	};

    class Pos3Di{
    public:
        int _ix;
        int _iy;
        int _iz;

        Pos3Di(int ix = 0, int iy = 0, int iz = 0) {
            _ix = ix;   _iy = iy;   _iz = iz;
        }

        Pos3Di(const Pos3Di& pos) {
            _ix = pos._ix;   _iy = pos._iy;   _iz = pos._iz;
        }
        const Pos3Di& operator=(const Pos3Di& pos) {
            _ix = pos._ix;   _iy = pos._iy;   _iz = pos._iz;
            return *this;
        }

    };

}

#endif