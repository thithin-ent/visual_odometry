#include "image.h"

namespace visual_slam
{

    frame_image::frame_image(unsigned int id, cv::Mat image, cv::Mat depthimage) : id_(id), image_(image), depthimage_(depthimage)
    {
    }

    frame_image::~frame_image()
    {
    }

    ushort frame_image::findDepth(const cv::KeyPoint &kp)
    {
        int X = cvRound(kp.pt.x);
        int Y = cvRound(kp.pt.y);
        ushort d = depthimage_.ptr<ushort>(Y)[X];
        return d;
    }

    void frame_image::find_feature_matches(const Mat &img_1, const Mat &img_2, std::vector<KeyPoint> &keypoints_1,
                                           std::vector<KeyPoint> &keypoints_2, std::vector<DMatch> &matches)
    {
        Mat descriptors_1, descriptors_2;

        Ptr<FeatureDetector> detector = ORB::create();
        Ptr<DescriptorExtractor> descriptor = ORB::create();
        // use this if you are in OpenCV2
        // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
        // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        detector->detect(img_1, keypoints_1);
        detector->detect(img_2, keypoints_2);

        descriptor->compute(img_1, keypoints_1, descriptors_1);
        descriptor->compute(img_2, keypoints_2, descriptors_2);

        if (descriptors_1.type() != CV_32F)
        {
            descriptors_1.convertTo(descriptors_1, CV_32F);
        }

        if (descriptors_2.type() != CV_32F)
        {
            descriptors_2.convertTo(descriptors_2, CV_32F);
        }

        vector<vector<DMatch>> match;
        matcher->knnMatch(descriptors_1, descriptors_2, match, 2);

        const double ratio_match = 0.75;
        for (int i = 0; i < match.size(); i++)
        {
            if (match[i][0].distance < ratio_match * match[i][1].distance)
            {
                matches.push_back(match[i][0]);
            }
        }

        vector<Point2f> srcpoints,dstpoints;
        for (int i = 0; i < matches.size(); i++)
        {
            srcpoints.push_back( keypoints_1[ matches[i].queryIdx ].pt );
            dstpoints.push_back( keypoints_2[ matches[i].trainIdx ].pt );
        }

        Mat H = findHomography( srcpoints, dstpoints, CV_RANSAC ); 

        for (int i = 0; i < srcpoints.size(); i++)
        {
            Mat src = (Mat_<double>(3, 1) << srcpoints[i].x, srcpoints[i].y, 1);
            Mat dst = (Mat_<double>(3, 1) << dstpoints[i].x, dstpoints[i].y, 1);
            src = H*src;
            std::cout  << "src: " <<endl << src <<endl << dst <<endl << "dst: " <<endl;
            std::cout << dst.at<double>(0,0) - src.at<double>(0,0) <<endl
                        << dst.at<double>(1,0) - src.at<double>(1,0) <<endl;
        }


        Mat img_match;
        drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
        imshow("feature matching", img_match);
    }

    void frame_image::odometry(const Mat &t, const Mat &R, const Matrix4d &T)
    {
        Matrix4d T_p;
        T_p << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
            0, 0, 0, 1;

        T_ = T * T_p;
    }
}
