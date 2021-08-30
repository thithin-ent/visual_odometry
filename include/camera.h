#ifndef camera_H
#define camera_H
#include "config.h"
using namespace Eigen;
using namespace cv;

namespace visual_slam {

class Camera{
	public:
		double	fx_,fy_,cx_,cy_,depth_;
		Camera(double fx=0, double fy=0, double cx=0, double cy=0, double depth = 0);
		~Camera();

		Vector3d world2camera(const Vector3d& p_w, const Matrix4d& T_c_w);
		Vector3d camera2world(const Vector3d& p_c, const Matrix4d& T_c_w);
		Vector2d camera2pixel(const Vector3d& p_c);
		Vector3d pixel2camera(const Vector2d& p_p, double depth = 1.0);
		Point2d camera2pixel(const Point2d& p_p);
		Vector3d pixel2world(const Vector2d& p_p, const Matrix4d& T_c_w,double depth =1);
		Vector2d world2pixel(const Vector3d& p_w, const Matrix4d& T_c_w);
		

};
}
#endif 