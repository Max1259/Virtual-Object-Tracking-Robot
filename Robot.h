#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <thread>
#include <math.h>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class CRobot {
public:
	CRobot() {};
	std::vector <Mat> L1, L2, L3;
	float view_roll = 0.8;
	float view_pitch = 1;
	float j1, j2, j3, j4;

	CRobot(std::vector<Mat> link1, std::vector<Mat> link2, std::vector<Mat> link3) {
		L1 = link1;
		L2 = link2;
		L3 = link3;
	};
	Mat createHT(float tx, float ty, float tz, float rx, float ry, float rz);
	void transformBox(std::vector<Mat>& box, Mat T);

	void drawBox(Mat& im, std::vector<Mat> box, Scalar colour);
	std::vector <float> fkin(Mat end_pose);
	void draw_opencv_box(std::vector <float> end, CRobot rob);
	void ikine(float x, float y, float z, float theta, float a1, float a2);
	Mat traj(std::vector <float> vec1, std::vector <float> vec2, float v1, float v2, int time_steps);
};

