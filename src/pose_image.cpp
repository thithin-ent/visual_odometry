#include "config.h"
#include "image.h"
#include "camera.h"

using namespace cv;
using namespace std;

int main()
{
    string str, buffer;
    ifstream file("../config/associations.txt");
    vector<string> dir_dep, dir_img;
    while (getline(file, str))
    {
        vector<string> temp;
        istringstream ss(str);
        while (getline(ss, buffer, ' '))
        {
            temp.push_back(buffer);
        }
        dir_dep.push_back(temp[1]);
        dir_img.push_back(temp[3]);
    }
    file.close();

    vector<visual_slam::frame_image> frame_image;
    //for (int i = 0; i < dir_img.size(); i++)
    for (int i = 0; i < 100; i++)
    {
        Mat image = imread("../config/" + dir_img[i]);
        Mat depth = imread("../config/" + dir_dep[i]);
        visual_slam::frame_image tempspace(i, image, depth);
        frame_image.push_back(tempspace);
    }
    frame_image[0].T_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -2.25,
        0, 0, 0, 1;
    //for (int i = 0; i < dir_img.size() - 1; i++)
    for (int i = 0; i < 100 - 1; i++)
    {
        vector<KeyPoint> keypoint_1, keypoint_2;

        vector<DMatch> matches;
        frame_image[i].find_feature_matches(frame_image[i].image_, frame_image[i + 1].image_, keypoint_1, keypoint_2, matches);
        cout << "매칭 사이즈: " << matches.size() << "개" << endl;

        vector<Point3f> pts_3d;
        vector<Point2f> pts_2d;
        visual_slam::Camera camera(481.20, -480.00, 319.5, 239.5);

        for (DMatch m : matches)
        {
            ushort d = frame_image[i].depthimage_.ptr<unsigned short>(int(keypoint_1[m.queryIdx].pt.y))[int(keypoint_1[m.queryIdx].pt.x)];
            if (d == 0) // bad depth
                continue;
            float dd = d / 5000.0;
            Point2d p1 = camera.camera2pixel(keypoint_1[m.queryIdx].pt);
            pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
            pts_2d.push_back(keypoint_2[m.trainIdx].pt);
        }
        cout << "3d-2d pairs: " << pts_3d.size() << endl;

        Mat r, t;
        Mat K = (Mat_<double>(3, 3) << 481.20, 0, 319.50, 0, -480.00, 239.50, 0, 0, 1);
        solvePnPRansac(pts_3d, pts_2d, K, Mat(), r, t, false, 100, 4.0, 0.99);
        Mat R;
        cv::Rodrigues(r, R);
        cout << dir_img[i] << endl;
        cout << "R=" << endl
             << R << endl;
        cout << "t=" << endl
             << t << endl;

        frame_image[i + 1].odometry(t, R, frame_image[i].T_);
        cout << "T_=" << endl
             << frame_image[i + 1].T_ << endl;
        cout << "----------------------------------------" << endl;
        Mat out_img;
        drawKeypoints(frame_image[i].image_, keypoint_1, out_img, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        imshow("img", out_img);
        waitKey(0);
    }

    return 0;
}