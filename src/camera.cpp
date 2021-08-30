#include "camera.h"


namespace visual_slam {

    Camera::Camera(double fx,double fy,double cx,double cy,double depth):
		fx_(fx),fy_(fy),cx_(cx),cy_(cy),depth_(depth)
		{}

    Camera::~Camera()
        {}

	Vector3d Camera::world2camera(const Vector3d& p_w, const Matrix4d& T_c_w){
        Vector4d p_w_h = p_w.homogeneous();
        return (T_c_w*p_w_h).head<3>();
    }

    Vector3d Camera::camera2world(const Vector3d& p_c, const Matrix4d& T_c_w){
        Vector4d p_c_h = p_c.homogeneous();
        return (T_c_w.inverse()*p_c_h).head<3>();
    }

    Vector2d Camera::camera2pixel(const Vector3d& p_c){
        return Vector2d(
            fx_*p_c(0,0)/p_c(2,0)+cx_,
            fy_*p_c(1,0)/p_c(2,0)+cy_
        );
    }

    Vector3d Camera::pixel2camera(const Vector2d& p_p, double depth){
        return Vector3d(
            (p_p(0,0)-cx_) * depth / fx_,
            (p_p(1,0)-cy_) * depth / fy_,
            depth
        );
    }

    Point2d Camera::camera2pixel ( const Point2d& p)
    {
        return Point2d
           (
               ( p.x - cx_ ) / fx_,
               ( p.y - cy_ ) / fy_
           );
    }

    Vector2d Camera::world2pixel(const Vector3d& p_w, const Matrix4d& T_c_w){
        return camera2pixel( world2camera(p_w, T_c_w) );
    }

    Vector3d Camera::pixel2world(const Vector2d& p_p, const Matrix4d& T_c_w,double depth){
        return camera2world( pixel2camera(p_p, depth), T_c_w );
    }
}


