#include "stdafx.h"

#include "Robot.h"
#include <cmath>

Mat CRobot::createHT(float tx, float ty, float tz, float rx, float ry, float rz) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;

	r11 = cos(rz) * cos(ry);
	r12 = cos(rz) * sin(ry) * sin(rx) - sin(rz) * cos(rx);
	r13 = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx);
	r21 = sin(rz) * cos(ry);
	r22 = sin(rz) * sin(ry) * sin(rx) + cos(rz) * cos(rx);
	r23 = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx);
	r31 = -1.0 * sin(ry);
	r32 = cos(ry) * sin(rx);
	r33 = cos(ry) * cos(rx);

	return (Mat1f(4, 4) << r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz, 0, 0, 0, 1);
};

void CRobot::transformBox(std::vector<Mat>& box, Mat T) {

	for (int i = 0; i < box.size(); i++) {
		box.at(i) = T * box.at(i);
	}
};

void CRobot::drawBox(Mat& im, std::vector<Mat> box, Scalar colour) {
	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	// Transform screen coordinate system to middle of screen, fix Y axis
	Mat W = createHT(im.size().width / 2, im.size().height / 2, CV_PI + view_roll, CV_PI + view_pitch, 0, 0);
	transformBox(box, W);

	for (int i = 0; i < 12; i++) {
		Point pt1 = Point2f(box.at(draw_box1[i]).at<float>(0, 0), box.at(draw_box1[i]).at<float>(1, 0));
		Point pt2 = Point2f(box.at(draw_box2[i]).at<float>(0, 0), box.at(draw_box2[i]).at<float>(1, 0));
		cv::line(im, pt1, pt2, colour, 1);
	}
};

std::vector <float> CRobot::fkin(Mat end_pose) {

	float x = end_pose.at<float>(0, 3);
	float y = end_pose.at<float>(1, 3);
	float z = end_pose.at<float>(2, 3);

	float roll, pitch, yaw;

	float sy = sqrt(end_pose.at<float>(0, 0) * end_pose.at<float>(0, 0) + end_pose.at<float>(1, 0) * end_pose.at<float>(1, 0));
	bool singular = sy < 1e-6; // If

	if (!singular)
	{
		roll = atan2(end_pose.at<float>(2, 1), end_pose.at<float>(2, 2));
		pitch = atan2(-end_pose.at<float>(2, 0), sy);
		yaw = atan2(end_pose.at<float>(1, 0), end_pose.at<float>(0, 0));
	}
	else
	{
		roll = atan2(-end_pose.at<float>(1, 2), end_pose.at<float>(1, 1));
		pitch = atan2(-end_pose.at<float>(2, 0), sy);
		yaw = 0;
	}

	std::vector <float> v{ x,y,z,roll,pitch,yaw };
	return v;
};

void CRobot::draw_opencv_box(std::vector <float> end, CRobot rob) {

	Size image_size = Size(1000, 600);
	cv::Mat img = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60);
	cv::namedWindow("7825 Project");

	std::ostringstream str;
	str << "X: " << setprecision(3) << end[0] << "   Y: " << setprecision(3) << end[1] << "   Z: " << setprecision(3) << end[2];
	std::string var = str.str();
	
	std::ostringstream str2;
	str2	 << "Roll: " << setprecision(3) << end[3] << "   Pitch: " << setprecision(3) << end[4] << "   Yaw: " << setprecision(3) << end[5];
	std::string var2 = str2.str();

	std::ostringstream str3;
	str3 << "Joint 1: " << setprecision(3) << rob.j1 << "   Joint 2: " << setprecision(3) << rob.j2 << "   Joint 3: " << setprecision(3) << rob.j3;
	std::string var3 = str3.str();

	putText(img, var, Point(10, 30), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(img, var2, Point(10, 50), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(img, var3, Point(10, 70), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 255, 255), 1);

	drawBox(img, rob.L1, CV_RGB(255, 0, 0));
	drawBox(img, rob.L2, CV_RGB(0, 255, 0));
	drawBox(img, rob.L3, CV_RGB(0, 0, 255));

	imshow("7825 Project", img);
};

void CRobot::ikine(float x, float y, float z, float theta, float a1, float a2) {

	float a1_2 = a1 * a1;
	float a2_2 = a2 * a2;
	float x2 = x * x;
	float y2 = y * y;
	float a1_2_sq = (a1 + a2) * (a1 + a2);

	while ((x2 + y2) > a1_2_sq) {
		if (x > y) { x -= 1; }
		else { y -= 1; }

		x2 = x * x;
		y2 = y * y;
	}

	j1 = (180/CV_PI) * 2 * atan2(2*a1*y - sqrt(-1*a1_2*a1_2 + 2*a1_2*a2_2 + 2*a1_2*x2 + 2*a1_2*y2 - a2_2*a2_2 + 2*a2_2*x2 + 2*a2_2*y2 - x2*x2 - 2*x2*y2 - y2*y2), a1_2 + 2*a1*x - a2_2 + x2 + y2);
	j2 = (180/CV_PI) * 2 * atan2(sqrt((-a1_2 + 2*a1*a2 - a2_2 + x2 + y2)*(a1_2 + 2*a1*a2 + a2_2 - x2 - y2)),-a1_2 + 2*a1*a2 - a2_2 + x2 + y2);
	j3 = theta;
	j4 = -z;

	if (x == -400) {
		j1 += 180;
	}
};

Mat CRobot::traj(std::vector <float> vec1, std::vector <float> vec2, float v1, float v2, int time_steps) {

	std::vector <float> t;
	Size s = Size(6, time_steps);
	Mat tt;
	Mat c;
	std::vector <float> z{ 0,0,0,0 };
	Mat z_ = Mat(z);
	cv::transpose(z_, z_);

	std::vector <float> vel1{ v1,v1,v1,v1 };
	Mat vel1_ = Mat(vel1);

	for (float i = 0; i <= time_steps; i++) {
		t.push_back(i / time_steps);
	}

	Mat vec1_m = Mat(vec1);
	Mat vec2_m = Mat(vec2);
	
	Mat A = 6 * (vec2_m-vec1_m) - 3 * (v2 - v1);
	cv::transpose(A,A);
	Mat B = -15 * (vec2_m - vec1_m) + (8 * v1 + 7 * v2);
	cv::transpose(B, B);
	Mat C = 10 * (vec2_m - vec1_m) - (6 * v1 + 4 * v2);
	cv::transpose(C, C);
	cv::transpose(vel1_, vel1_);
	cv::transpose(vec1_m, vec1_m);

	for (int i = 0; i <= time_steps; i++) {
		std::vector <float> temp;
		temp.push_back(pow(t[i], 5));
		temp.push_back(pow(t[i], 4));
		temp.push_back(pow(t[i], 3));
		temp.push_back(pow(t[i], 2));
		temp.push_back(t[i]);
		temp.push_back(1);

		Mat temp2 = Mat(temp);
		cv::transpose(temp2, temp2);
		tt.push_back(temp2);
	}

	c.push_back(A);
	c.push_back(B);
	c.push_back(C);
	c.push_back(z_);
	c.push_back(vel1_);
	c.push_back(vec1_m);

	return tt * c;
};


