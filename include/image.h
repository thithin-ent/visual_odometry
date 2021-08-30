#ifndef image_H
#define image_H
#include "config.h"
using namespace Eigen;
using namespace cv;
using namespace std;

namespace visual_slam{
    class frame_image{
        public:
        unsigned int id_;
        cv::Mat image_,depthimage_;
        Matrix4d T_;
        frame_image(unsigned int id,cv::Mat image,cv::Mat depthimage);
        ~frame_image();

        ushort findDepth(const cv::KeyPoint& kp);
        void find_feature_matches ( const Mat& img_1, const Mat& img_2, std::vector<KeyPoint>& keypoints_1, 
                                    std::vector<KeyPoint>& keypoints_2, std::vector< DMatch >& matches );

        void odometry(const Mat& t, const Mat& R, const Matrix4d &T);
    };

}
#endif