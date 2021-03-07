#include "stdafx.h"
#include "CLink.h"

std::vector <Mat> CLink::createBox(float w, float h, float d) {
    std::vector <Mat> box;

    // The 8 vertexes, origin at the middle of the box
    box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
    box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
    box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
    box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
    box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
    box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
    box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
    box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

    // Move origin to middle of the the left hand face
    Mat T = createHT(w / 2, 0, 0, CV_PI, 0, 0);
    for (int i = 0; i < box.size(); i++) {
        box.at(i) = T * box.at(i);
    }

    return box;
};

