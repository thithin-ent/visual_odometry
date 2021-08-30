#include "EKF.h"

namespace visual_slam {
	
	EKF::EKF(Vector3d pose_X, Matrix3d C):
		pose_X_(pose_X), C_(C)
		{}

	EKF::~EKF()
		{}

	int EKF::Rows_M(Matrix2d m) {
		int a = m.rows();
		return a;
	}

	int EKF::cols_M(Matrix2d m) {
		int a = m.cols();
		return a;
	}

	int EKF::EKF_pose(const Vector3d& measure_Z,const Matrix3d& R,const Matrix3d& H) {
		Matrix3d K;
		K = H * C_ * H.inverse() + R;
		K = C_ * H.inverse() * K.inverse();
		pose_X_ = pose_X_ + K * (measure_Z - pose_X_);
		C_ = C_ - K * H * C_;

		std::cout << "pose update: " << std::endl;
		std::cout << pose_X_ << std::endl;

		return 0;
	}

}
