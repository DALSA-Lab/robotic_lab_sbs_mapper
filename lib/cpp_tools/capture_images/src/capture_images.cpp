#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

namespace
{
    const char *about = "Basic marker detection";

    //! [aruco_detect_markers_keys]
    const char *keys =
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }";
}

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    int camId = parser.get<int>("ci");

    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    cv::VideoCapture inputVideo;
    inputVideo.open(camId);

    // Set resolution to 1920x1080
    inputVideo.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // inputVideo.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    // Optional: set desired frame rate (30 fps)
    inputVideo.set(cv::CAP_PROP_FPS, 60);

    int i = 0;
    int limit = 100;
    int purge_count = 100;
    while (inputVideo.grab() && i < limit + purge_count)
    {
        cv::Mat image;
        inputVideo.retrieve(image);

        // namedWindow("out", WINDOW_NORMAL); // allow resizing
        //  imshow("out", imageCopy);
        if (i >= purge_count)
        {
            std::string filename = "image" + std::to_string(i - purge_count) + ".jpg";
            cv::imwrite(filename, image);
        }
        i++;
    }
    //! [aruco_detect_markers]
    return 0;
}
