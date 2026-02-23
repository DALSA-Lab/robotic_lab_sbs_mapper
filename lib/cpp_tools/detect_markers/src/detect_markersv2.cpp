#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <iostream>
#include "aruco_samples_utility.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>

using namespace std;
using namespace cv;

namespace
{
    const char *about = "Basic marker detection";

    //! [aruco_detect_markers_keys]
    const char *keys =
        "{d        | 0     | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{cd       |       | Input file with custom dictionary }"
        "{v        |       | Input from video or image file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";

    //! [aruco_detect_markers_keys]

    const string refineMethods[4] = {
        "None",
        "Subpixel",
        "Contour",
        "AprilTag"};

}

void draw_plate(Mat img, InputArray rvec, InputArray tvec, InputArray camMatrix, InputArray distCoeffs)
{
    float scale = 1e-3;

    // Define a 3D rectangle in marker coordinate system (Z=0 plane)
    // SBS
    // float w = 86 * scale; // mm
    // float l = 128 * scale; // mm
    // float h = 16 * scale; // mm

    // random box
    float l = 85 * scale;    // mm
    float w = 127.3 * scale; // mm
    float h = 14.5 * scale;  // mm

    // --- Define 8 corners of the cuboid in marker coordinates ---
    vector<Point3f> corners = {
        // bottom face (Z=0)
        {-w / 2, l / 2, 0},  // 0: top-left
        {w / 2, l / 2, 0},   // 1: top-right
        {w / 2, -l / 2, 0},  // 2: bottom-right
        {-w / 2, -l / 2, 0}, // 3: bottom-left
        // top face (Z=-h)
        {-w / 2, l / 2, -h}, // 4
        {w / 2, l / 2, -h},  // 5
        {w / 2, -l / 2, -h}, // 6
        {-w / 2, -l / 2, -h} // 7
    };

    // Project all corners to 2D
    vector<Point2f> imgPoints;
    projectPoints(corners, rvec, tvec, camMatrix, distCoeffs, imgPoints);

    // --- Define faces using indices of corners ---
    vector<vector<int>> faces = {
        {4, 5, 6, 7}, // top
        {0, 1, 5, 4}, // front
        {1, 2, 6, 5}, // right
        {2, 3, 7, 6}, // back
        {3, 0, 4, 7}, // left
        {0, 1, 2, 3}  // bottom
    };

    // Optional: color for each face
    vector<Scalar> faceColors = {
        Scalar(0, 255, 0),  // bottom
        Scalar(0, 200, 0),  // top
        Scalar(0, 150, 0),  // front
        Scalar(255, 0, 0),  // right
        Scalar(0, 0, 255),  // back
        Scalar(255, 255, 0) // left
    };

    // Draw all faces
    for (size_t i = 0; i < faces.size(); i++)
    {
        vector<Point> poly;
        for (int idx : faces[i])
            poly.push_back(imgPoints[idx]);

        vector<vector<Point>> contour{poly};
        // fillPoly(img, contour, faceColors[i]);
        polylines(img, contour, true, Scalar(0, 0, 0), 1);
    }
}

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");

    aruco::DetectorParameters detectorParams = readDetectorParamsFromCommandLine(parser);
    aruco::Dictionary dictionary = readDictionatyFromCommandLine(parser);

    if (parser.has("refine"))
    {
        // override cornerRefinementMethod read from config file
        int user_method = parser.get<aruco::CornerRefineMethod>("refine");
        if (user_method < 0 || user_method >= 4)
        {
            std::cout << "Corner refinement method should be in range 0..3" << std::endl;
            return 0;
        }
        detectorParams.cornerRefinementMethod = user_method;
    }

    std::cout << "Corner refinement method: " << refineMethods[detectorParams.cornerRefinementMethod] << std::endl;

    int camId = parser.get<int>("ci");

    String video;
    if (parser.has("v"))
    {
        video = parser.get<String>("v");
    }

    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    //! [aruco_pose_estimation1]
    Mat camMatrix, distCoeffs;
    if (estimatePose)
    {
        // You can read camera parameters from tutorial_camera_params.yml
        readCameraParamsFromCommandLine(parser, camMatrix, distCoeffs);
    }
    //! [aruco_pose_estimation1]
    //! [aruco_detect_markers]
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    cv::VideoCapture inputVideo;
    int waitTime;
    if (!video.empty())
    {
        inputVideo.open(video);
        waitTime = 0;
    }
    else
    {
        inputVideo.open(camId);
        waitTime = 10;
    }

    // Set resolution to 1920x1080
    inputVideo.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // inputVideo.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    // Optional: set desired frame rate (30 fps)
    inputVideo.set(cv::CAP_PROP_FPS, 60);

    double totalTime = 0;
    int totalIterations = 0;

    //! [aruco_pose_estimation2]
    // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
    //! [aruco_pose_estimation2]

    // prepare a projection matrix for the SBS plate 3D model
    Mat Rt = Mat::eye(3, 4, CV_64F);
    Rt.col(3) = Mat::zeros(3, 1, CV_64F); // explicitly set last column to 0

    // Compute projection matrix
    Mat projMatrix = camMatrix * Rt;

    vector<vector<Vec3d>> tdata(6), rdata(6);

    while (inputVideo.grab())
    {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        //! [aruco_pose_estimation3]
        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;

        // detect markers and estimate pose
        detector.detectMarkers(image, corners, ids, rejected);

        size_t nMarkers = corners.size();
        vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if (estimatePose && !ids.empty())
        {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++)
            {
                // int id = ids[i];
                // if (id < 0 || id > 5)
                    // continue; // only 0–5
// 
                // if (tdata[id].size() >= 1000)
                    // continue;
                // if (rvecs[i][2] < 0 ) {
                //     rvecs[i][2] = rvecs[i][2] + CV_PI;
                //     tvecs[i][2] = -tvecs[i][2];
                // }
                solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i), false, cv::SOLVEPNP_IPPE_SQUARE);
                // solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));

                solvePnPRefineLM(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                // tdata[id].push_back(tvecs[i]);
                // rdata[id].push_back(rvecs[i]);
            }
        }
        //! [aruco_pose_estimation3]
        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if (totalIterations % 30 == 0 && false) // disabled
        {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }
        //! [aruco_draw_pose_estimation]
        // draw results
        image.copyTo(imageCopy);
        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if (estimatePose)
            {
                for (unsigned int i = 0; i < ids.size(); i++)
                {

                    cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
                    cout << "ID: " << i << "Translation (x,y,z): " << tvecs[i] << " Rotation (rx, ry, rz): " << rvecs[i] << std::endl;

                    draw_plate(imageCopy, rvecs[i], tvecs[i], camMatrix, distCoeffs);
                };
            }
        }
        //! [aruco_draw_pose_estimation]

        if (showRejected && !rejected.empty())
            cv::aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        namedWindow("out", WINDOW_NORMAL); // allow resizing
        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if (key == 27)
            break;
    }
    
    return 0;
}
