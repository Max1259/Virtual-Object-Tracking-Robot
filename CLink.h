#pragma once
#include "stdafx.h"
#include "Robot.h"

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <thread>

using namespace std;
using namespace cv;

class CLink : public CRobot {
public:
	float w, h, d;

	CLink(float width, float height, float depth) {
		w = width;
		h = height;
		d = depth;
	};

	std::vector<Mat> link;
	std::vector<Mat> createBox(float w, float h, float d);
};