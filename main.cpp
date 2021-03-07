////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created Sept 9, 2020 by Craig Hennessey
// Last updated Nov 8, 2020
////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "Robot.h"
#include "CLink.h"

#include <string>
#include <sstream>
#include <iostream>
#include <thread>
#include <conio.h>
#include <stdlib.h>

#include <opencv2/aruco/charuco.hpp>

using namespace std;
using namespace cv;

using namespace dnn;
using namespace aruco;

cv::Vec3d tvec{0,0,0};

bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) //was static bool?
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
};

void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(600, 500), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
};

void detectCharucoBoardWithCalibrationPose()
{
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "calibration.yml";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    }
    else {
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        while (inputVideo.grab()) {
            cv::Mat image, imageCopy;
            inputVideo.retrieve(image);
            image.copyTo(imageCopy);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
            // if at least one marker detected
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0) {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    cv::Vec3d rvec;
                    // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid) {
                        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                    }
                }
            }
           
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(1);
            if (key == 27)
                break;
        }
    }
};

void detectCharucoBoardWithoutCalibration()
{
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    while (inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
        //or
        //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds);
            // if at least one charuco corner detected
            if (charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
        }
        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(30);
        if (key == 27)
            break;
    }
};

void Menu() {
    cout << "Scara Robot\nELEX 7825\n\nMake a seletion:" << endl;
    cout << "(1) Forward Kinematics - Manual" << endl;
    cout << "(2) Forward Kinematics - Automatic" << endl;
    cout << "(3) Inverse Kinematics - Manual" << endl;
    cout << "(4) Inverse Kinematics - Automatic" << endl;
    cout << "(5) Joint Trajectory" << endl;
    cout << "(6) Cartesian Trajectory" << endl;
    cout << "(7) Charuco Robot" << endl;
    cout << "(8) YOLO Robot" << endl;
    cout << "(0) Exit" << endl;
};

int main(int argc, char* argv[]) {

    char select;            //menu selection
    char escape;            //escape to menu
    std::vector <float> v;  //holds xyz+rpy information acquired from fwd kinematics
    float j1, j2, j3, j4;   //joint angles + prismatic length
    float x, y, z, theta;   //coordinates

    while (1) {
        Menu();
        cin >> select;

        if (select < '0' || select > '10') {

            cout << "\nInvalid, try again." << endl;
        }
        else if (select == '0') {

            exit(0);
        }
        else if (select == '1') { //Manual Forward Kinematics

            Size image_size = Size(1000, 600);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");

            cout << "\nSelect joint 1 angle:\t";
            cin >> j1;
            cout << "Select joint 2 angle:\t";
            cin >> j2;
            cout << "Select joint 3 angle:\t";
            cin >> j3;
            cout << "Select prismatic length:\t";
            cin >> j4;

            if (j4 > 200) {
                j4 = 200;
            }
            else if (j4 < 0) {
                j4 = 0;
            }

            CLink L1(200, 50, 50);
            CLink L2(200, 50, 50);
            CLink L3(j4, 50, 50);

            L1.link = L1.createBox(L1.w, L1.h, L1.d);
            L2.link = L2.createBox(L2.w, L2.h, L2.d);
            L3.link = L3.createBox(L3.w, L3.h, L3.d);

            CRobot robot(L1.link, L2.link, L3.link);

            robot.j1 = j1;
            robot.j2 = j2;
            robot.j3 = j3;
            robot.j4 = j4;

            Mat T1 = L1.createHT(0, 0, 0, 0, 0, robot.j1 * CV_PI / 180);                            //create transformation matrix for origin
            L1.transformBox(L1.link, T1);                                                           //transform L1

            Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, robot.j2 * CV_PI / 180);                     //create transformation matrix for L1/L2       
            L2.transformBox(L2.link, T2);                                                           //transform L2

            Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, robot.j3 * CV_PI / 180);     //create transformation matrix for L2/L3
            L3.transformBox(L3.link, T3);                                                           //transform L3

            Mat T4 = T3 * L3.createHT(-robot.j4, 0, 0, 0, 0, 0);                                    //create transformation matrix for prismatic joint

            v = robot.fkin(T4);

            img.release();
            robot.draw_opencv_box(v, robot);
            cv::waitKey(1);

            cout << "Press any key to exit to main menu:";
            cin >> escape;
        }
        else if (select == '2') {   //Automatic Forward Kinematics

            Size image_size = Size(1000, 600);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");

            for (unsigned int i = 0; i <= 1180; i++) {

                j1 = 0;
                j2 = 0;
                j3 = 0;
                j4 = 0;

                if (i <= 360) {
                    j1 = i;
                }
                else if (i > 360 && i <= 720) {
                    j2 = i - 360;
                }
                else if (i > 720 && i <= 1080) {
                    j3 = i - 720;
                }
                else {
                    j4 = i - 1080;
                }

                CLink L1(200, 50, 50);
                CLink L2(200, 50, 50);
                CLink L3(j4, 50, 50);

                L1.link = L1.createBox(L1.w, L1.h, L1.d);
                L2.link = L2.createBox(L2.w, L2.h, L2.d);
                L3.link = L3.createBox(L3.w, L3.h, L3.d);

                CRobot robot(L1.link, L2.link, L3.link);

                Mat T1 = L1.createHT(0, 0, 0, 0, 0, j1 * CV_PI / 180);                          //create transformation matrix for origin
                L1.transformBox(L1.link, T1);                                                   //transform L1

                Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, j2 * CV_PI / 180);                   //create transformation matrix for L1/L2       
                L2.transformBox(L2.link, T2);                                                   //transform L2

                Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, j3 * CV_PI / 180);   //create transformation matrix for L2/L3
                L3.transformBox(L3.link, T3);                                                   //transform L3

                Mat T4 = T3 * L3.createHT(-j4, 0, 0, 0, 0, 0);                                  //create transformation matrix for prismatic joint
                v = robot.fkin(T4);

                robot.j1 = j1;
                robot.j2 = j2;
                robot.j3 = j3;
                robot.j4 = j4;

                img.release();
                robot.draw_opencv_box(v, robot);
                cv::waitKey(1);
            }

            cout << "Press any key to exit to main menu:";
            cin >> escape;
        }
        else if (select == '3') {

            Size image_size = Size(1000, 600);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");

            cout << "\nSelect x coordinate:\t";
            cin >> x;
            cout << "Select y coordinate:\t";
            cin >> y;
            cout << "Select z coordinate:\t";
            cin >> z;
            cout << "Select roll angle:\t";
            cin >> theta;

            if (z < -200) {
                z = -200;
            }
            else if (z > 0) {
                z = 0;
            }

            CLink L1(200, 50, 50);
            CLink L2(200, 50, 50);
            CLink L3(-z, 50, 50);

            L1.link = L1.createBox(L1.w, L1.h, L1.d);
            L2.link = L2.createBox(L2.w, L2.h, L2.d);
            L3.link = L3.createBox(L3.w, L3.h, L3.d);

            CRobot robot(L1.link, L2.link, L3.link);

            robot.ikine(x, y, z, theta, L1.w, L2.w);

            Mat T1 = L1.createHT(0, 0, 0, 0, 0, robot.j1 * CV_PI / 180);                            //create transformation matrix for origin
            L1.transformBox(L1.link, T1);                                                           //transform L1

            Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, robot.j2 * CV_PI / 180);                     //create transformation matrix for L1/L2       
            L2.transformBox(L2.link, T2);                                                            //transform L2

            Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, robot.j3 * CV_PI / 180);     //create transformation matrix for L2/L3
            L3.transformBox(L3.link, T3);                                                           //transform L3

            Mat T4 = T3 * L3.createHT(-robot.j4, 0, 0, 0, 0, 0);                                    //create transformation matrix for prismatic joint
            v = robot.fkin(T4);

            img.release();
            robot.draw_opencv_box(v, robot);
            cv::waitKey(1);
        }
        else if (select == '4') {

            Size image_size = Size(1000, 600);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");

            x = -100;
            y = 50;
            z = -50;
            theta = 0;

            for (unsigned int i = 0; i < 201; i++) {

                CLink L1(200, 50, 50);
                CLink L2(200, 50, 50);
                CLink L3(-z, 50, 50);

                L1.link = L1.createBox(L1.w, L1.h, L1.d);
                L2.link = L2.createBox(L2.w, L2.h, L2.d);
                L3.link = L3.createBox(L3.w, L3.h, L3.d);

                CRobot robot(L1.link, L2.link, L3.link);

                robot.ikine(x, y, z, theta, L1.w, L2.w);

                Mat T1 = L1.createHT(0, 0, 0, 0, 0, robot.j1 * CV_PI / 180);                        //create transformation matrix for origin
                L1.transformBox(L1.link, T1);                                                       //transform L1

                Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, robot.j2 * CV_PI / 180);                 //create transformation matrix for L1/L2       
                L2.transformBox(L2.link, T2);                                                       //transform L2

                Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, robot.j3 * CV_PI / 180); //create transformation matrix for L2/L3
                L3.transformBox(L3.link, T3);                                                       //transform L3

                Mat T4 = T3 * L3.createHT(-robot.j4, 0, 0, 0, 0, 0);                                //create transformation matrix for prismatic joint
                v = robot.fkin(T4);

                x++;

                img.release();
                robot.draw_opencv_box(v, robot);
                cv::waitKey(1);
            }
        }
        else if (select == '5') {

            Size image_size = Size(1000, 600);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");

            float vel1, vel2;
            int steps;
            Mat traj;
            char yn;

            std::vector <float> v1(4);
            std::vector <float> v2(4);

            cout << "\nSelect initial j1 angle:\t";
            cin >> v1[0];
            cout << "Select initial j2 angle:\t";
            cin >> v1[1];
            cout << "Select initial j3 angle:\t";
            cin >> v1[2];
            cout << "Select initial prismatic length:\t";
            cin >> v1[3];

            if (v1[3] < -200) {
                v1[3] = -200;
            }
            else if (v1[3] > 0) {
                v1[3] = 0;
            }

            CLink L1(200, 50, 50);
            CLink L2(200, 50, 50);
            CLink L3(-v1[3], 50, 50);

            L1.link = L1.createBox(L1.w, L1.h, L1.d);
            L2.link = L2.createBox(L2.w, L2.h, L2.d);
            L3.link = L3.createBox(L3.w, L3.h, L3.d);

            CRobot robot(L1.link, L2.link, L3.link);

            cout << "\nSelect final j1 angle:\t";
            cin >> v2[0];
            cout << "Select final j2 angle:\t";
            cin >> v2[1];
            cout << "Select final j3 angle:\t";
            cin >> v2[2];
            cout << "Select final prismatic length:\t";
            cin >> v2[3];

            cout << "\nSelect initial velocity:\t";
            cin >> vel1;
            cout << "\nSelect final velocity:\t";
            cin >> vel2;
            cout << "\nSelect number of time steps:\t";
            cin >> steps;
            cout << "\nReturn to initial position? (y/n):\t";
            cin >> yn;

            if (v2[3] > 200) {
                v2[3] = 200;
            }
            else if (v2[3] < 0) {
                v2[3] = 0;
            }

            traj = robot.traj(v1, v2, vel1, vel2, steps);

            if (yn == 'y') {

                Mat traj2 = robot.traj(v2, v1, vel1, vel2, steps);
                traj.push_back(traj2);
            }

            for (unsigned int i = 0; i < traj.rows; i++) {

                z = traj.at<float>(i, 3);

                if (z > 200) {
                    z = 200;
                }
                else if (z < 0) {
                    z = 0;
                }

                CLink L1(200, 50, 50);
                CLink L2(200, 50, 50);
                CLink L3(z, 50, 50);

                L1.link = L1.createBox(L1.w, L1.h, L1.d);
                L2.link = L2.createBox(L2.w, L2.h, L2.d);
                L3.link = L3.createBox(L3.w, L3.h, L3.d);

                CRobot robot(L1.link, L2.link, L3.link);

                robot.j1 = traj.at<float>(i, 0);
                robot.j2 = traj.at<float>(i, 1);
                robot.j3 = traj.at<float>(i, 2);

                Mat T1 = L1.createHT(0, 0, 0, 0, 0, robot.j1 * CV_PI / 180);                            //create transformation matrix for origin
                L1.transformBox(L1.link, T1);                                                           //transform L1

                Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, robot.j2 * CV_PI / 180);                     //create transformation matrix for L1/L2       
                L2.transformBox(L2.link, T2);                                                            //transform L2

                Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, robot.j3 * CV_PI / 180);     //create transformation matrix for L2/L3
                L3.transformBox(L3.link, T3);                                                           //transform L3

                Mat T4 = T3 * L3.createHT(z, 0, 0, 0, 0, 0);                                    //create transformation matrix for prismatic joint
                v = robot.fkin(T4);

                img.release();
                robot.draw_opencv_box(v, robot);
                cv::waitKey(1);
            }
            img.release();
        }
        else if (select == '6') {

            Size image_size = Size(1000, 600);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");

            float x2, y2, z2, theta2;
            std::vector <float> pose1(4), pose2(4);
            Mat traj;
            int steps;
            char yn;

            cout << "\nSelect initial x coordinate:\t";
            cin >> pose1[0];
            cout << "Select initial y coordinate:\t";
            cin >> pose1[1];
            cout << "Select initial z coordinate:\t";
            cin >> pose1[2];
            cout << "Select initial rotation:\t";
            cin >> pose1[3];

            if (pose1[2] < -200) {
                pose1[2] = -200;
            }
            else if (pose1[2] > 0) {
                pose1[2] = 0;
            }

            CRobot rob;

            cout << "\nSelect final x coordinate:\t";
            cin >> pose2[0];
            cout << "Select final y coordinate:\t";
            cin >> pose2[1];
            cout << "Select final z coordinate:\t";
            cin >> pose2[2];
            cout << "Select final rotation:\t";
            cin >> pose2[3];

            if (pose2[2] < -200) {
                pose2[2] = -200;
            }
            else if (pose2[2] > 0) {
                pose2[2] = 0;
            }

            cout << "\nSelect number of time steps:\t";
            cin >> steps;
            cout << "\nReturn to initial position? (y/n):\t";
            cin >> yn;

            traj = rob.traj(pose1, pose2, 0, 0, steps);

            if (yn == 'y') {
                Mat traj2 = rob.traj(pose2, pose1, 0, 0, steps);
                traj.push_back(traj2);
            }

            for (unsigned int i = 0; i < traj.rows; i++) {

                z = traj.at<float>(i, 2);

                if (z < -200) {
                    z = -200;
                }
                else if (z > 0) {
                    z = 0;
                }

                CLink L1(200, 50, 50);
                CLink L2(200, 50, 50);
                CLink L3(-z, 50, 50);

                L1.link = L1.createBox(L1.w, L1.h, L1.d);
                L2.link = L2.createBox(L2.w, L2.h, L2.d);
                L3.link = L3.createBox(L3.w, L3.h, L3.d);

                CRobot robot(L1.link, L2.link, L3.link);

                robot.ikine(traj.at<float>(i, 0), traj.at<float>(i, 1), z, traj.at<float>(i, 3), L1.w, L2.w);

                Mat T1 = L1.createHT(0, 0, 0, 0, 0, robot.j1 * CV_PI / 180);                            //create transformation matrix for origin
                L1.transformBox(L1.link, T1);                                                           //transform L1

                Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, robot.j2 * CV_PI / 180);                     //create transformation matrix for L1/L2       
                L2.transformBox(L2.link, T2);                                                            //transform L2

                Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, robot.j3 * CV_PI / 180);     //create transformation matrix for L2/L3
                L3.transformBox(L3.link, T3);                                                           //transform L3

                Mat T4 = T3 * L3.createHT(-robot.j4, 0, 0, 0, 0, 0);                                    //create transformation matrix for prismatic joint
                v = robot.fkin(T4);

                img.release();
                robot.draw_opencv_box(v, robot);
                cv::waitKey(1);
            }
        }
        else if (select == '7') {

            Size image_size = Size(1000, 1000);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");

            std::thread cam(detectCharucoBoardWithCalibrationPose);

            x = 0;
            y = 0;
            z = 0;
            float x_old, y_old, z_old;
            std::vector <float> pose1(4), pose2(4);
            Mat traj;

            CLink L1(200, 50, 50);
            CLink L2(200, 50, 50);
            CLink L3(-z, 50, 50);

            L1.link = L1.createBox(L1.w, L1.h, L1.d);
            L2.link = L2.createBox(L2.w, L2.h, L2.d);
            L3.link = L3.createBox(L3.w, L3.h, L3.d);

            CRobot robot(L1.link, L2.link, L3.link);

            while (1) {

                x_old = x;
                y_old = y;
                z_old = z;

                x = 600 * (tvec[0] - 0.29) / 0.34 + 600;
                y = 1000 * (-tvec[2] + 0.28) / 0.89 + 450;
                z = -150 * (tvec[1] + 0.2) / 0.31;

                if (z < -200) {
                    z = -200;
                }
                else if (z > 0) {
                    z = 0;
                }

                if (x > 350) {
                    x = 350;
                }
                else if (x < -350) {
                    x = -350;
                }

                if (y > 400) {
                    y = 400;
                }
                else if (y < -300) {
                    y = -300;
                }
                
                //cout << x << "     " << y << "     " << z << endl;

                pose1 = { x_old, y_old, z_old, 0 };
                pose2 = { x, y, z, 0 };

                traj = robot.traj(pose1, pose2, 0, 0, 1);

                for (unsigned int i = 0; i < traj.rows; i++) {

                    z = traj.at<float>(i, 2);

                    CLink L1(200, 50, 50);
                    CLink L2(200, 50, 50);
                    CLink L3(-z, 50, 50);

                    L1.link = L1.createBox(L1.w, L1.h, L1.d);
                    L2.link = L2.createBox(L2.w, L2.h, L2.d);
                    L3.link = L3.createBox(L3.w, L3.h, L3.d);

                    CRobot robot(L1.link, L2.link, L3.link);

                    robot.ikine(traj.at<float>(i, 0), traj.at<float>(i, 1), z, traj.at<float>(i, 3), L1.w, L2.w);

                    Mat T1 = L1.createHT(0, 0, 0, 0, 0, robot.j1 * CV_PI / 180);                            //create transformation matrix for origin
                    L1.transformBox(L1.link, T1);                                                           //transform L1

                    Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, robot.j2 * CV_PI / 180);                     //create transformation matrix for L1/L2       
                    L2.transformBox(L2.link, T2);                                                            //transform L2

                    Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, robot.j3 * CV_PI / 180);     //create transformation matrix for L2/L3
                    L3.transformBox(L3.link, T3);                                                           //transform L3

                    Mat T4 = T3 * L3.createHT(-robot.j4, 0, 0, 0, 0, 0);                                    //create transformation matrix for prismatic joint
                    v = robot.fkin(T4);

                    img.release();
                    robot.draw_opencv_box(v, robot);
                    cv::waitKey(1);
                }
            }
        }
        else if (select == '8') {

            Size image_size = Size(1000, 1000);                                      //initialize image size
            cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60); //initialize image
            cv::namedWindow("7825 Project");
            x = 0;
            y = 0;
            z = 0;
            float x_old, y_old, z_old;
            std::vector <float> pose1(4), pose2(4);
            Mat traj;

            CLink L1(200, 50, 50);
            CLink L2(200, 50, 50);
            CLink L3(-z, 50, 50);

            L1.link = L1.createBox(L1.w, L1.h, L1.d);
            L2.link = L2.createBox(L2.w, L2.h, L2.d);
            L3.link = L3.createBox(L3.w, L3.h, L3.d);

            CRobot robot(L1.link, L2.link, L3.link);

            cv::Mat im;
            Mat input_blob;
            vector<Mat> network_output;
            vector<int> id;
            vector<float> conf;
            vector<Rect> bbox;
            float confidence_threshold = 0.6;
            float nms_threshold = 0.35;
            int detections_in_layer;
            int output_data_offset;
            Point max_class;
            Mat class_conf;
            double max_conf;
            Point bbox_center;
            Size bbox_size;
            int index;
            string str;
            vector<int> found_index;
            vector<double> layer_proc_time;
            double proc_time;
            char key;

            cv::VideoCapture inputVideo;
            inputVideo.open(0);

            Size network_input_image_size = Size(416, 416);
            vector<string> output_layer_name;

            //Load vector of class names (objects trained to ID)
            vector <string> classnames;
            string load_str;
            ifstream filename("coco.names");

            while (std::getline(filename, load_str)) {
                classnames.push_back(load_str);
            }

            //Load YOLO v3 DNN configuration weights
            cv::dnn::Net net = readNetFromDarknet("yolov3.cfg", "yolov3.weights");
            net.setPreferableBackend(DNN_BACKEND_OPENCV);
            net.setPreferableTarget(DNN_TARGET_CPU);

            //Get layer names and output layer indexes
            vector<string> layer_name = net.getLayerNames();
            vector<int> output_layer_index = net.getUnconnectedOutLayers();

            for (int out_index = 0; out_index < output_layer_index.size(); out_index++) {
                output_layer_name.push_back(layer_name.at(output_layer_index.at(out_index) - 1));
            }

            while (inputVideo.grab()) {

                inputVideo.retrieve(im);

                id.clear();
                conf.clear();
                bbox.clear();
                found_index.clear();
                layer_proc_time.clear();

                //Setup 4D tensor - scale and resize input image to fit network trained data
                blobFromImage(im, input_blob, 1 / 255.0, network_input_image_size, Scalar(0, 0, 0), true, false);

                //Input image to network
                net.setInput(input_blob);

                //runs the forward pass
                net.forward(network_output, output_layer_name);

                //loop over each output layer (x3 Yolov3)
                for (int output_index = 0; output_index < network_output.size(); output_index++) {

                    detections_in_layer = network_output[output_index].rows;
                    output_data_offset = network_output[output_index].cols;

                    //loop over each detection
                    for (int detect_index = 0; detect_index < detections_in_layer; detect_index++) {

                        //network_output = bbox(center(x,y),w,h), bbox confidence, 80x class confidence
                        //index to single result
                        float* data = (float*)network_output[output_index].data + detect_index * output_data_offset;

                        //get all class scores (conf/class) (x80 YOLOv3)
                        class_conf = network_output[output_index].row(detect_index).colRange(5, network_output[output_index].cols);

                        //find class with highest confidence
                        minMaxLoc(class_conf, 0, &max_conf, 0, &max_class);

                        ////if high enough confidence, save
                        //if ((max_conf > confidence_threshold) && (classnames.at(max_class.x) == "cell phone")) {

                        if ((max_conf > confidence_threshold)) {
                            //rescale bbox to image size in pixels
                            bbox_center.x = (int)(data[0] * im.size().width);
                            bbox_center.y = (int)(data[1] * im.size().height);
                            bbox_size.width = (int)(data[2] * im.size().width);
                            bbox_size.height = (int)(data[3] * im.size().height);

                            //save
                            id.push_back(max_class.x);
                            conf.push_back((float)max_conf);
                            bbox.push_back(Rect(bbox_center.x - bbox_size.width / 2, bbox_center.y - bbox_size.height / 2, bbox_size.width, bbox_size.height));
                        }
                    }
                }

                //non maximum suppression - remove overlapping boxes
                dnn::NMSBoxes(bbox, conf, confidence_threshold, nms_threshold, found_index);

                for (int object_index = 0; object_index < found_index.size(); object_index++) {

                    index = found_index.at(object_index);
                    rectangle(im, bbox.at(index), Scalar(255, 255, 0));

                    str = format("%.2f:  ", conf.at(index));
                    str = str + classnames.at(id.at(index));
                    putText(im, str, Point(bbox.at(index).x, bbox.at(index).y - 10), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(255, 255, 255), 2);
                }

                //time per layer and overall time
                //proc_time = net.getPerfProfile(layer_proc_time) / getTickFrequency();
                //putText(im, format("Proc Time: %.2f ", proc_time), Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(255, 255, 255), 2);
                x_old = x;
                y_old = y;
                z_old = z;
                if (bbox_size.width == 0) bbox_size.width = 100;
                //cout << bbox_center.x << "     " << bbox_size.width << "     " << bbox_center.y << endl;
                x = 800 * (bbox_center.x - 80) / 470 - 400;
                y = 800 * (bbox_size.width - 60) / 140 - 400;
                z = -200 * (bbox_center.y - 130) / 270;
                //cout << y << endl;
                if (z < -200) {
                    z = -200;
                }
                else if (z > 0) {
                    z = 0;
                }

                if (x > 350) {
                    x = 350;
                }
                else if (x < -350) {
                    x = -350;
                }

                if (y > 400) {
                    y = 400;
                }
                else if (y < -300) {
                    y = -300;
                }

                pose1 = { x_old, y_old, z_old, 0 };
                pose2 = { x, y, z, 0 };

                traj = robot.traj(pose1, pose2, 0, 0, 1);

                for (unsigned int i = 0; i < traj.rows; i++) {

                    z = traj.at<float>(i, 2);

                    CLink L1(200, 50, 50);
                    CLink L2(200, 50, 50);
                    CLink L3(-z, 50, 50);

                    L1.link = L1.createBox(L1.w, L1.h, L1.d);
                    L2.link = L2.createBox(L2.w, L2.h, L2.d);
                    L3.link = L3.createBox(L3.w, L3.h, L3.d);

                    CRobot robot(L1.link, L2.link, L3.link);

                    robot.ikine(traj.at<float>(i, 0), traj.at<float>(i, 1), z, traj.at<float>(i, 3), L1.w, L2.w);

                    Mat T1 = L1.createHT(0, 0, 0, 0, 0, robot.j1 * CV_PI / 180);                            //create transformation matrix for origin
                    L1.transformBox(L1.link, T1);                                                           //transform L1

                    Mat T2 = T1 * L2.createHT(200, 0, 0, 0, 0, robot.j2 * CV_PI / 180);                     //create transformation matrix for L1/L2       
                    L2.transformBox(L2.link, T2);                                                            //transform L2

                    Mat T3 = T2 * L3.createHT(200, 0, 0, 0, -90 * CV_PI / 180, robot.j3 * CV_PI / 180);     //create transformation matrix for L2/L3
                    L3.transformBox(L3.link, T3);                                                           //transform L3

                    Mat T4 = T3 * L3.createHT(-robot.j4, 0, 0, 0, 0, 0);                                    //create transformation matrix for prismatic joint
                    v = robot.fkin(T4);

                    img.release();
                    robot.draw_opencv_box(v, robot);
                    cv::waitKey(1);
                }

                cv::imshow("out", im);
                key = (char)cv::waitKey(1);
                if (key == 27)
                    break;
            }
        }
        return 0;
    }
}




