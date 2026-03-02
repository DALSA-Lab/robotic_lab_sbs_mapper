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
    const char *about = "Compute marker pose using all options";

    const char *keys =
        "{d        | 0     | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }"
        "{dir      |       | input dir }"
        "{iterative|       | Use iterative PNP: RefineLM=0, RefineVSS=1}"
        "{dp       |       | ok}"
        "{cd       |       | ok}";

    const string refineMethods[4] = {"None", "Subpixel", "Contour", "AprilTag"};

    const string iterativeMethods[3] = {"None", "LM", "VSS"};

    const string pnpMethods[10] = {"ITERATIVE", "EPNP", "P3P", "DLS", "UPNP", "AP3P", "IPPE", "IPPE_SQUARE", "SQPNP", "RANSAC"};

}

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    float markerLength = parser.get<float>("l");

    aruco::DetectorParameters detectorParams = readDetectorParamsFromCommandLine(parser);
    aruco::Dictionary dictionary = readDictionatyFromCommandLine(parser);

    bool iterativePnP = parser.has("iterative");
    int iterativeMethod = 0; // default to LM iterative PNP method
    if (iterativePnP)
    {
        iterativeMethod = parser.get<int>("iterative");
        if (iterativeMethod < 0 || iterativeMethod > 1)
        {
            cout << "Iterative PnP methoud should be in range 0..1" << endl;
            return 0;
        }
        cout << "Iterative PnP method: " << iterativeMethods[iterativeMethod] << endl;
    }

    Mat camMatrix, distCoeffs;
    readCameraParamsFromCommandLine(parser, camMatrix, distCoeffs);

    string dir = "images";
    if (parser.has("dir"))
    {
        dir = parser.get<string>("dir");
    }

    // Load images from folder instead of using VideoCapture
    vector<cv::String> imageFiles;
    string src = dir + "/*.jpg";
    cv::glob(src, imageFiles, false);

    if (imageFiles.empty())
    {
        cerr << "No images found in " << src << endl;
        return -1;
    }

    // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    // prepare a projection matrix for the SBS plate 3D model
    Mat Rt = Mat::eye(3, 4, CV_64F);
    Rt.col(3) = Mat::zeros(3, 1, CV_64F); // explicitly set last column to 0

    // Compute projection matrix
    Mat projMatrix = camMatrix * Rt;

    for (int refineMethod = 0; refineMethod < (sizeof(refineMethods) / sizeof(refineMethods[0])); refineMethod++)
    {
        if (refineMethod == 3)
        {
            cout << "Skipping AprilTag refinment" << endl;
            continue;
        }
        detectorParams.cornerRefinementMethod = refineMethod;
        cout << "Corner refinement method: " << refineMethods[detectorParams.cornerRefinementMethod] << endl;
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        
        for (int pnpMethod = 0; pnpMethod < (sizeof(pnpMethods) / sizeof(pnpMethods[0])); pnpMethod++)
        {
            vector<vector<Vec3d>> tdata(6), rdata(6);
            if (pnpMethod == 3 || pnpMethod == 4)
            {
                cout << "Skipping broken PNP methods (DLS, UPNP)" << endl;
                continue;
            }
            cout << "PnP method: " << pnpMethods[pnpMethod] << endl;
            // Loop over images
            for (size_t i = 0; i < imageFiles.size(); ++i)
            {
                cv::Mat image, imageCopy;
                image = cv::imread(imageFiles[i]);
                if (image.empty())
                {
                    cerr << "Could not load " << imageFiles[i] << endl;
                    continue;
                }

                double tick = (double)getTickCount();

                vector<int> ids;
                vector<vector<Point2f>> corners, rejected;

                // detect markers and estimate pose
                detector.detectMarkers(image, corners, ids, rejected);

                size_t nMarkers = corners.size();
                vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

                if (!ids.empty())
                {
                    // Calculate pose for each marker
                    for (size_t i = 0; i < nMarkers; i++)
                    {
                        int id = ids[i];
                        if (id < 0 || id > 5)
                            continue; // only 0–5

                        if (tdata[id].size() >= 1000)
                            continue;

                        if (pnpMethods[pnpMethod] == "RANSAC") {
                            solvePnPRansac(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                        } else {
                            solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i), false, pnpMethod);
                        }
                        if (iterativePnP)
                        {
                            if (iterativeMethod == 0)
                            {
                                solvePnPRefineLM(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                            }
                            else if (iterativeMethod == 1)
                            {
                                solvePnPRefineVVS(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                            }
                        }

                        tdata[id].push_back(tvecs[i]);
                        rdata[id].push_back(rvecs[i]);
                    }
                }
            }

            string f = "markers-" + refineMethods[refineMethod] + "-" + pnpMethods[pnpMethod] + ".csv";
            ofstream out(f);
            out << "id,tx,ty,tz,rx,ry,rz\n";
            for (size_t id = 0; id < tdata.size(); id++)
            {
                if (tdata[id].size() == 0)
                    continue;

                for (size_t k = 0; k < tdata[id].size(); k++)
                {
                    auto t = tdata[id][k];
                    auto r = rdata[id][k];
                    out << id << "," << t[0] << "," << t[1] << "," << t[2] << ","
                        << r[0] << "," << r[1] << "," << r[2] << "\n";
                }
            }
            out.close();
        }
    }
    return 0;
}
