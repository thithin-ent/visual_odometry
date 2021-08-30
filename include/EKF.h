#ifndef EKF_H
#define EKF_H
#include "config.h"
using namespace Eigen;

namespace visual_slam {
	class EKF{
		public:

			Vector3d pose_X_;
			Matrix3d C_;
			EKF(Vector3d pose_X, Matrix3d C);
			~EKF();
			
			int Rows_M(Matrix2d m);
			int cols_M(Matrix2d m);
			int EKF_pose(const Vector3d& measure_Z,const Matrix3d& R,const Matrix3d& H);
	};
}
#endif 