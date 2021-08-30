#include "config.h"
#include "EKF.h"
#include "camera.h"
//#include "frame.h"

using namespace cv;

int main(int argc, char *argv[])
try
{
    rs2::pipeline p;
    p.start();
    rs2::align align_to_color(RS2_STREAM_COLOR);
    int h, w, count = 0;
    std::cout << "start" << std::endl;
    while (count < 5)
    {

        rs2::frameset frames = p.wait_for_frames();
        frames = align_to_color.process(frames);
        rs2::video_frame image = frames.get_color_frame();
        h = image.get_height();
        w = image.get_width();
        Mat img(Size(w, h), CV_8UC3, (void *)image.get_data(), Mat::AUTO_STEP);
        rs2::depth_frame depth = frames.get_depth_frame();
        //h = depth.get_height();
        //w = depth.get_width();
        Mat dep(Size(w, h), CV_16U, (void *)depth.get_data(), Mat::AUTO_STEP);

        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        imshow("depth img", dep);
        imshow("camera img", img);
        if (waitKey(1) == 27)
        {
            
            imwrite("../config/save/depth/" + std::to_string(count) + ".png", dep);
            imwrite("../config/save/rgb/" + std::to_string(count) + ".png", img);
            count++;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}