#ifndef _POSE_2D_DATA_STRUCT_
#define _POSE_2D_DATA_STRUCT_

#include <vector>


namespace Localization {

	class Pos2D;
	class Polar2D {
	public:
		double _deg;
		double _radius;

		Polar2D(double deg = 0, double r = 0) {
			_deg = deg;
			_radius = r;
		}
		Polar2D(const Polar2D& other) {
			_deg = other._deg;
			_radius = other._radius;
		}
		Polar2D& operator=(const Polar2D& other) {
			_deg = other._deg;
			_radius = other._radius;
			return *this;
		}

		Pos2D toPos() const;

		static double getEuclideanDis(const Polar2D& polar1, const Polar2D& polar2);

		static void getPolarErr(const Polar2D& polar1, const Polar2D& polar2, double &radius_err, double &deg_err);

		static std::vector<Pos2D> polarVecToPosVec(const std::vector<Polar2D>& polar_vec);
	};

	class Pos2D {
	public:
		double _x;
		double _y;

		Pos2D(double x = 0, double y = 0) {
			_x = x;   _y = y;
		}
		Pos2D(const Pos2D& pos) {
			_x = pos._x;   _y = pos._y;
		}
		const Pos2D& operator=(const Pos2D& pos);
		Pos2D operator+(const Pos2D& pos) const;
		Pos2D operator-(const Pos2D& pos) const;
		Pos2D operator*(double scale) const;
		static double getEuclideanDis(const Pos2D& pos1, const Pos2D& pos2);
		double norm() const;
		double gradDeg();
		void print();

		Polar2D toPolar() const;
	};

	class Orient2D {
	public:
		double _deg;

		Orient2D(double deg = 0);
		Orient2D(const Orient2D& other);
		const Orient2D& operator=(const Orient2D& other);
		Orient2D operator*(const Orient2D& orient) const;
		Pos2D operator*(const Pos2D& pos) const;//rotate Orient2D, then get pos of new axis in ori axis
		Polar2D operator*(const Polar2D& polar) const;//rotate Orient2D, then get polar of new axis in ori axis
		Orient2D operator/(const Orient2D& orient) const;

		static Pos2D rotatePos2D(const Pos2D& pos, double deg);

		Orient2D getInvOrient() const;
	};

	class Pose2D {
	public:
		Pos2D _pos;
		Orient2D _orient;

		Pose2D(double x = 0, double y = 0, double deg = 0) :_pos(x, y), _orient(deg) {}
		Pose2D(const Pos2D& pos, const Orient2D& orient) : _pos(pos), _orient(orient) {}
		Pose2D(const Pose2D& pose) : _pos(pose._pos), _orient(pose._orient) {}
		const Pose2D& operator=(const Pose2D& other);
		Pose2D operator*(const Pose2D& pose) const;
		Pos2D operator*(const Pos2D& pos) const;
		Pos2D operator*(const Polar2D& polar) const;
		Pose2D operator/(const Pose2D& posei) const;
		Pose2D getInvPose() const;
		Pose2D devideCalibPose(const Pose2D& calib_pose) const;

		void print() const;

		static Pose2D getRelativePoseOfTwoPose(const Pose2D& posei, const Pose2D& posej);//relative defined in i

		static Pose2D getRecursionPose(const Pose2D& posei, const Pose2D& delta_pose);//delta_pose defined in i

		static Pose2D getInvDeltaPose(const Pose2D& delta_pose);

		static Pos2D getRelativePos(const Pose2D& g_pose, const Pos2D& g_pos);//g_pos relative to g_pose

		static Polar2D getRelativePolar(const Pose2D& g_pose, const Pos2D& g_pos);//g_pos relative to g_pose

		static void relativePoseToRotationTranslation(const Pose2D& pose, double rot[9], double trans[3]);

		static void rotationTranslationToRelativePose(double rot[9], double trans[3], Pose2D& pose);

		static double getEuclideanDis(const Pose2D& posei, const Pose2D& posej);

		static double getRelativeDegOfTwoPose(const Pose2D& posei, const Pose2D& posej);

		static bool getDeltaPoseOfTwoAxis(const std::vector<Pos2D>& pos_in1_vec, const std::vector<Pos2D>& pos_in2_vec, Pose2D& pose_2in1);
	};

}

#endif