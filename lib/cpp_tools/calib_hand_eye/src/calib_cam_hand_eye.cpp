#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <math.h>

#include <jsoncpp/json/json.h>
#include <fstream>

using namespace std;
using namespace cv;

namespace
{
    const char *about = "Hand-eye calibration";

    //! [aruco_detect_markers_keys]
    const char *keys =
        "{dir      |       | input dir }"
        "{waypoints|       | path to waypoints.json file}"
        "{c        |       | Camera intrinsic parameters. Will skip calibration if provided }";
}

static void readCameraParamsFromCommandLine(CommandLineParser &parser, Mat &camMatrix, Mat &distCoeffs)
{
    string filename = parser.get<string>("c");
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        throw runtime_error("Invalid camera file\n");
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return;
}

int main(int argc, char *argv[])
{

    cout << CV_VERSION << endl;

    // Recipe:
    // poses are stored in the examples/custom_waypoints.json
    // captured images are store in ./images
    // we need to compute the camera intrinsics from the images
    // then we need to compute the object location (using the intrinsics)
    // then use the handeye calib function to calibrate the location by matching movements between robot poses and object locations!
    // happy days!

    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    string waypointsFile;
    if (parser.has("waypoints"))
    {
        waypointsFile = parser.get<string>("waypoints");
    }
    else
    {
        cerr << "Missing waypoints file!" << endl;
        return -1;
    }

    Mat cameraMatrix, distCoeffs;
    Mat globalImg, globalGray;

    Mat coverage =  Mat(1080, 1920, CV_8UC3, Scalar(255, 255, 255));

    string dir = "./images";
    if (parser.has("dir"))
    {
        dir = parser.get<string>("dir");
    }

    string src = dir + "/*.jpg";

    // Open folder with calib images
    vector<String> files;
    glob(src, files, false);

    vector<Mat> R_target2Cam;
    vector<Mat> T_target2Cam;


    bool hasCameraIntrinsics = parser.has("c");
    if (hasCameraIntrinsics)
    {
        readCameraParamsFromCommandLine(parser, cameraMatrix, distCoeffs);
        cout << "Succesfully loaded camera intrinsics from file." << endl;
    }
    else
    {

        // number of inner corners per chessboard pattern and square size
        Size patternSize(5, 8);  // (# vertical, # horizontal)
        float squareSize = 0.03; // 30mm

        // Arrays to store object points and image points from all the images
        vector<vector<Point3f>> objpoints;
        vector<vector<Point3f>> objpoints2;
        vector<vector<Point2f>> imgpoints;


        for (size_t i = 0; i < files.size(); i++)
        {
            // Load image
            // Mat img = imread("c920_checkers.jpg");
            Mat img = imread(files[i]);
            if (img.empty())
            {
                cerr << "Image not found!" << endl;
                return -1;
            }
            cout << "image: " << files[i] << endl;

            Mat img_copy2 = img.clone();
            globalImg = img.clone();
            Mat gray;
            cvtColor(img, gray, COLOR_BGR2GRAY);
            globalGray = gray.clone();

            int nb_vertical = patternSize.width;
            int nb_horizontal = patternSize.height;

            // Prepare object points, like (0,0,0), (1,0,0), ... scaled by squareSize
            vector<Point3f> objp;
            for (int j = 0; j < nb_horizontal; j++)
            {
                for (int k = 0; k < nb_vertical; k++)
                {
                    objp.push_back(Point3f(k * squareSize, j * squareSize, 0.0f));
                }
            }

            // Detect chessboard corners
            vector<Point2f> corners;
            bool ret = findChessboardCorners(gray, patternSize, corners);

            if (ret)
            {
                // Add object points, image points
                objpoints.push_back(objp);
                objpoints2.push_back(objp);
                imgpoints.push_back(corners);

                // Refine corner positions (optional but recommended)
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                             TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

                // Draw the chessboard corners
                drawChessboardCorners(img, patternSize, corners, ret);
                // imshow("Corners", img);
                // waitKey(0);

                for (size_t c = 0; c < corners.size(); c++) {
                    circle(coverage, corners[c], 3, Scalar(255,0,0));
                }
            }
            else {
                cout << "WARN: Failed to calibrate camera for image" << files[i] << endl;
            }
        }
        // imshow("coverage", coverage);
        // waitKey(0);

        string f = "corners.csv";
        ofstream out(f);
        out << "idx, x, y\n";
        for (size_t i = 0; i < imgpoints.size(); i++)
        {

            for (size_t k = 0; k < imgpoints[i].size(); k++)
            {
                auto x = int(imgpoints[i][k].x);
                auto y = int(imgpoints[i][k].y);
                out << i << "," << x << "," << y << "\n";
            }

        }
        out.close();


        // Calibrate camera
        double ret2 = calibrateCamera(objpoints2, imgpoints, globalGray.size(),
                                      cameraMatrix, distCoeffs, R_target2Cam, T_target2Cam);


        int w = globalImg.cols;
        int h = globalImg.rows;
        double alpha2 = 1.2;
        Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs,
                                                        Size(w, h), alpha2,
                                                        Size(w, h));

        cout << "Camera Matrix:" << endl
             << cameraMatrix << endl;
        cout << "Dist coeff:" << endl
             << distCoeffs << endl;
        cout << "New Camera Matrix:" << endl
             << newCameraMatrix << endl;

        cout << "Validating camera matrix and dist. coefficients" << endl;

        double total_error = 0;
        double total_points = 0;

        for (size_t i = 0; i < objpoints.size(); i++)
        {
            vector<Point2f> projected;

            // Reproject 3D object points using the camera model
            projectPoints(objpoints[i], R_target2Cam[i], T_target2Cam[i], cameraMatrix, distCoeffs, projected);

            // Compute per-image error
            double err = 0;
            for (size_t p = 0; p < projected.size(); p++)
            {
                err += norm(imgpoints[i][p] - projected[p]);
            }

            double mean_err = err / projected.size();
            cout << "Image " << i << " reprojection error = " << mean_err << endl;

            total_error += err;
            total_points += projected.size();
        }

        cout << "\nTotal reprojection error = "
             << total_error / total_points << endl;

        // // Save to YAML
        // string calibFilename = "camera_calibration.yml";
        // FileStorage fsOut(calibFilename, FileStorage::WRITE);
        // fsOut << "camera_matrix" << cameraMatrix;
        // fsOut << "distortion_coefficients" << distCoeffs;
        // fsOut << "image_width" << w;
        // fsOut << "image_height" << h;
        // fsOut.release();

        // cout << "intrinsic calibration saved to " << calibFilename << ".\n";
    }

    // now we begin the extrinsic calib (handeye)
    // start by loading the poses from the custom_waypoints.json
    ifstream file(waypointsFile, ifstream::binary);
    if (!file.is_open())
    {
        cerr << "ERROR: could not open JSON file " << waypointsFile << "\n";
        return -1;
    }

    Json::Value data;
    Json::CharReaderBuilder readerBuilder;
    string errs;

    Json::parseFromStream(readerBuilder, file, &data, &errs);
    file.close();

    vector<Mat> R_gripper2Base, T_gripper2Base;
    for (Json::Value::ArrayIndex i = 0; i != data.size(); i++)
    {
        if (data[i].isMember("tool_pose"))
        {
            float x = data[i]["tool_pose"][0].asFloat();
            float y = data[i]["tool_pose"][1].asFloat();
            float z = data[i]["tool_pose"][2].asFloat();

            float rx = data[i]["tool_pose"][3].asFloat();
            float ry = data[i]["tool_pose"][4].asFloat();
            float rz = data[i]["tool_pose"][5].asFloat();

            Mat rvec = (Mat_<double>(3, 1) << rx, ry, rz);
            Mat R;
            Rodrigues(rvec, R);
            R_gripper2Base.push_back(R);

            T_gripper2Base.push_back((Mat_<double>(3, 1) << x, y, z));

            // below is used to print angle between current pose z-axis and XY plane of robot
            // Extract the Z axis of the rotation matrix (equivalent to MATLAB r = r(:,3))
            cv::Vec3d v(R.at<double>(0, 2),
                        R.at<double>(1, 2),
                        R.at<double>(2, 2));

            // Normal of the x-y plane
            cv::Vec3d n(0.0, 0.0, 1.0);

            // Normalize v and n
            double vnorm = cv::norm(v);
            double nnorm = cv::norm(n);

            cv::Vec3d v_norm = v / vnorm;
            cv::Vec3d n_norm = n / nnorm;

            // Angle between v and plane normal
            double dotVal = v_norm.dot(n_norm);
            double theta = acos(dotVal); // radians

            // Angle between vector and plane
            double phi = 90.0 - theta * 180.0 / CV_PI;

            if (abs(phi) <= 45) {
                std::cout << "WARNING: Angle between tool Z-axis and the xy-plane is below 45 degrees, angle: " << phi << " degrees (pose:" << i << ")" << std::endl;
            }

        }
    }

    

    // aruco::DetectorParameters detectorParams;
    // detectorParams.cornerRefinementMethod = 1;

    // aruco::ArucoDetector detector(aruco::getPredefinedDictionary(aruco::DICT_4X4_50), detectorParams);

    // Create ArUco detector once
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    aruco::DetectorParameters detectorParams;
    detectorParams.cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    aruco::ArucoDetector detector(dictionary, detectorParams);

    // detect markers and estimate pose
    // now compute the location of the aruco
    // for (size_t i = 0; i < files.size(); i++)
    // {
    //     Mat image, imageCopy;

    //     // Load image
    //     // Mat img = imread("c920_checkers.jpg");
    //     image = imread(files[i]);
    //     if (image.empty())
    //     {
    //         cerr << "Image not found!" << endl;
    //         return -1;
    //     }

    //     imageCopy = image.clone();

    //     //! [aruco_pose_estimation3]
    //     vector<int> ids;
    //     vector<vector<Point2f>> corners, rejected;

    //     detector.detectMarkers(image, corners, ids, rejected);

    //     if (ids.empty())
    //         continue;

    //     size_t nMarkers = corners.size();
    //     vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    //     float markerLength = 0.04;

    //     // set coordinate system
    //     vector<Point3f> objPoints(4);
    //     objPoints[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    //     objPoints[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    //     objPoints[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    //     objPoints[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    //     // Calculate pose for each marker
    //     for (size_t i = 0; i < nMarkers; i++)
    //     {
    //         // if (rvecs[i][2] < 0 ) {
    //         //     rvecs[i][2] = rvecs[i][2] + CV_PI;
    //         //     tvecs[i][2] = -tvecs[i][2];
    //         // }
    //         Mat rvec, tvec;
    //         solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
    //         // solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));

    //         // Convert rotation vector → rotation matrix
    //         Mat R;
    //         Rodrigues(rvec, R);

    //         // Store
    //         R_target2Cam.push_back(R);
    //         T_target2Cam.push_back(tvec);

    //         //! [aruco_draw_pose_estimation]
    //         // draw results
    //         image.copyTo(imageCopy);
    //         aruco::drawDetectedMarkers(imageCopy, corners, ids);

    //         drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, markerLength * 1.5f, 2);

    //         // imshow("markers", imageCopy);
    //         // waitKey(0);
    //     }
    // }
    cout << "Robot poses: " << T_gripper2Base.size() << "\n";
    cout << "Camera poses: " << T_target2Cam.size() << "\n";

    auto checkVec = [](const vector<Mat> &vec, const string &name)
    {
        cout << name << ": " << vec.size() << " elements\n";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            cout << "  [" << i << "] size = " << vec[i].rows << "x" << vec[i].cols
                 << ", type = " << vec[i].type()
                 << (vec[i].empty() ? " (EMPTY)" : "") << endl;
        }
    };

    // checkVec(R_gripper2Base, "R_gripper2Base");
    // checkVec(T_gripper2Base, "T_gripper2Base");
    // checkVec(R_target2Cam, "R_target2Cam");
    // checkVec(T_target2Cam, "T_target2Cam");

    // for (size_t i = 0; i < R_target2Cam.size(); ++i) {
    //     std::cout << "R_gripper2Base[" << i << "] =\n" << T_gripper2Base[i] << "\n\n";
    // }

    Mat R_cam2Gripper, T_cam2Gripper;
    calibrateHandEye(R_gripper2Base, T_gripper2Base, R_target2Cam, T_target2Cam, R_cam2Gripper, T_cam2Gripper, CALIB_HAND_EYE_TSAI);

    cout << "R = " << endl
         << " " << R_cam2Gripper << endl
         << endl;
    cout << "T = " << endl
         << " " << T_cam2Gripper << endl
         << endl;

    // compute rotations around x,y,z (roll, pitch, yaw)
    float roll, pitch, yaw;

    roll = atan2(R_cam2Gripper.at<double>(2, 1), R_cam2Gripper.at<double>(2, 2));
    pitch = atan2(-R_cam2Gripper.at<double>(2, 0), sqrt(pow(R_cam2Gripper.at<double>(0, 0), 2) + pow(R_cam2Gripper.at<double>(1, 0), 2)));
    yaw = atan2(R_cam2Gripper.at<double>(1, 0), R_cam2Gripper.at<double>(0, 0));

    cout << "roll (rotation around x axis)  rad: " << roll << " deg: " << (180.0 * roll) / M_PI << endl;
    cout << "pitch (rotation around y axis) rad: " << pitch << " deg: " << (180.0 * pitch) / M_PI << endl;
    cout << "yaw (rotation around z axis)   rad: " << yaw << " deg: " << (180.0 * yaw) / M_PI << endl;
    

    // Save to YAML
    string calibFilename = "camera_calibration.yml";
    FileStorage fsOut(calibFilename, FileStorage::WRITE);
    fsOut << "camera_matrix" << cameraMatrix;
    fsOut << "distortion_coefficients" << distCoeffs;
    fsOut << "image_width" << globalImg.cols;
    fsOut << "image_height" << globalImg.rows;
    fsOut << "R_cam2tcp" << R_cam2Gripper;
    fsOut << "T_cam2tcp" << T_cam2Gripper;
    fsOut.release();

    cout << "intrinsic calibration saved to " << calibFilename << ".\n";

    return 0;
}
